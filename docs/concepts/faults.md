# Faults and safety

A fault only stops things. Faulted hardware comes back through `set_hardware_component_state` or an
e-stop release, never by itself.

## Hardware faults

A component faults when `read` or `write` returns ERROR or throws, when a mode switch fails after
validation, or when it finishes no `read` or `write` for `hardware_hang_timeout_ms` (100 ms) while a
controller uses it. In the same cycle RT stops every controller that claims the component, and what
is chained to them, and asks the hardware to hold their interfaces, `prepare_command_mode_switch`
and `perform_command_mode_switch` with an empty start list. The supervisor then deactivates the
controllers and runs `on_error`, or `on_shutdown` if that fails too. A `read` or `write` that
returns DEACTIVATE deactivates the component instead.

The first cause is kept until the next lifecycle transition and appears in the log and in the answer
of the request it hit, for example `write returned ERROR` or a hang with its measured stall.

Mode switches do not count toward the hang timeout, a bus may take most of a second to change modes.
Each gets `mode_switch_timeout_ms` instead.

A plugin call that outlives its deadline quarantines its controller or component until it returns,
see [Quarantine](reconciler.md#quarantine).

## Released interfaces hold NaN

Every command interface without an active claimer, and every reference without an active writer,
holds NaN. The hardware plugin must hold a joint whose command is NaN, so a joint stops instead of
keeping the last command of a controller that ended.

## E-stop

`estop_topics` lists `std_msgs/Bool` topics, reliable and transient local, true is engaged. A topic
is engaged until its first message, while its last message is true, once it had no publisher for
`estop_publisher_grace_ms`, and, with `estop_deadline_ms` set, once no message arrived for that
long. While any topic is engaged, RT stops every unprotected controller that claims command
interfaces in the next cycle, and switches that would start one are rejected. Broadcasters keep
running. hcm does not tell a hard from a soft e-stop. If power is cut, the hardware faults. A soft
e-stop needs a cycling control loop, a loop stuck in a plugin call stops nothing. The "Controller
Manager Activity" diagnostic reports ERROR once a cycle stays open longer than
`hardware_hang_timeout_ms`.

Use one aggregating publisher per topic. With several publishers the last message wins.

A controller with `<controller>.protected: true`, such as one that keeps an arm limp, is never
stopped implicitly by the e-stop or a conflicting switch. It still stops through a switch that names
it, its own failure, a fault of its hardware, or the stop of a controller it is chained to.

## Recovery

When every topic is released, hcm waits `autostart_cooldown_ms`, brings the startup hardware that is
not active up again, up to `autostart_attempts` times, and then activates the inactive
`startup.controllers_active` with one AUTO switch. It skips, with the reason in the log, protected
controllers and controllers whose joints are in use or whose hardware is still down. A faulted
component also comes back with `ros2 control set_hardware_component_state arm_system active`, which
then runs the same startup switch. If the hardware stays down after the last attempt, no controller
is activated and the log says "Autostart gave up on the hardware".
