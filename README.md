# hector_controller_manager

A controller manager for ros2_control robots. It loads the standard controller and hardware
plugins through pluginlib and serves the `controller_manager_msgs` services, so the spawner,
`ros2 control` and existing launch files work unchanged. It replaces `controller_manager` where a
robot needs predictable behaviour under faults, hangs and e-stops.

Every request and every fault changes a desired state. A pure reconciler compares it with the
actual state and takes one safe step at a time. Each controller and each hardware component has an
owner thread that makes all of its non realtime plugin calls, so a hanging callback blocks only its
own plugin and is given up at its deadline. The control loop never allocates and never blocks.
[doc/design.md](doc/design.md) describes the internals.

## Running it

```bash
ros2 run hector_controller_manager ros2_control_node --ros-args --params-file controllers.yaml
```

In a launch file it takes the place of `controller_manager`'s node:

```python
Node(
    package="hector_controller_manager",
    executable="ros2_control_node",
    parameters=[{"robot_description": robot_description}, "controllers.yaml"],
)
```

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    estop_topics: [/e_stop_manager/estop]
    startup:
      hardware_active: [arm, base]
      controllers_loaded: [joint_state_broadcaster, arm_controller]
      controllers_active: [joint_state_broadcaster, arm_controller]
    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController
      params_file: arm_controller.yaml
```

Without the `robot_description` parameter the node waits for the latched `robot_description` topic.
The robot description and the semantic description are read once, a description that changes at
runtime is not supported, and a failed initialization needs a restart. The controller services
appear only after a successful initialization, diagnostics and `~/activity` tell waiting from
failed. The library class
`hector_controller_manager::ControllerManager` can also be embedded, for example by gz_ros2_control,
which passes its own executor and a hardware factory.

On SIGINT the manager first waits for the request in flight, such as a hardware activation spending
bus timeouts, and then deactivates and shuts down the hardware. ros2 launch sends SIGKILL after its
`sigterm_timeout` plus `sigkill_timeout`, 10 s by default. Set `sigterm_timeout` on the node above
the longest hardware call, so the hardware always gets its `on_deactivate`.
`ros2_control_node` exits with 1 after a shutdown it logged as "Shutdown incomplete", else with 0.

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `update_rate` | 100 Hz | Rate of the control loop |
| `executor_threads` | 2 | Threads of the node's executor, ignored when the caller passes an executor |
| `thread_priority` | 50 | SCHED_FIFO priority of the control loop on a realtime kernel, read only by `ros2_control_node` |
| `default_timeout_ms` | 5000 | Deadline of requests that give no timeout, of fault handling and of the shutdown |
| `hardware_state_timeout_ms` | 30000 | Deadline of hardware state requests and of the bring up after loading |
| `hardware_fault_timeout_ms` | 5000 | Time of one hardware lifecycle call during fault handling |
| `hardware_hang_timeout_ms` | 100 | A used component without a finished cycle for this long faults |
| `estop_topics` | [] | `std_msgs/Bool` topics, true is engaged, reliable and transient local |
| `estop_publisher_grace_ms` | 1000 | How long a topic may have no publisher before it counts as engaged |
| `estop_deadline_ms` | 0 | A released topic without a message for this long engages, 0 is off, else at least 100 |
| `autostart_cooldown_ms` | 2000 | Wait after an e-stop release before the hardware is brought up |
| `autostart_attempts` | 3 | Attempts to bring the hardware up after a release |
| `startup.hardware_active` | [] | Components brought to active at startup, all others too unless listed below |
| `hardware_components_initial_state.unconfigured`, `.inactive` | [] | Components left in that state at startup, as upstream |
| `startup.controllers_loaded` | [] | Controllers loaded and configured at startup |
| `startup.controllers_active` | [] | Controllers activated at startup by one AUTO switch |
| `<controller>.type` | | Plugin class of the controller |
| `<controller>.params_file` | | One or more parameter files of the controller |
| `<controller>.node_options_args` | [] | Extra ROS arguments for the controller's node |
| `<controller>.protected` | false | Never stop the controller implicitly, see below |
| `robot_description`, `robot_description_semantic` | "" | When empty, the latched topic of the same name is used |
| `use_sim_time` | false | Pace the control loop on `/clock`, passed on to the controllers |
| `capabilities` | [force_auto] | Read only, tells clients such as hector_controller_orchestrator what the manager supports |

`update_rate`, `executor_threads`, `autostart_attempts` and the durations must be at least 1, except
`autostart_cooldown_ms`, which may be 0, and `estop_deadline_ms`. A value below its minimum stops
the node at startup. `update_rate` must also stay below 1e9, `thread_priority` is left to the
kernel.

The startup parameters together are the startup configuration. A few limits are fixed, a
component gets 60 s to load, a failed switch gets 400 ms on top of its deadline to roll back, and a
switch needs at least 100 ms left before its deadline to change a hardware mode.

## Safety behaviour

A released command interface holds NaN, and so does every reference without an active writer. The
hardware is expected to hold a joint whose command is NaN, so a joint stops instead of keeping the
last command of a controller that ended. When one controller takes a joint over from another, the
hardware sees at most one mode switch and never a command of the old controller after the new one
started.

A fault only stops things. A failing controller stops together with everything chained to it, and a
hardware fault stops every controller that uses the component. Nothing is reactivated because of a
fault. A component that does not finish a cycle within the hang timeout faults, and a plugin call
that outlives its deadline is given up and quarantines its controller or component until it
returns.

While an e-stop topic is engaged, the control loop stops every controller that claims command
interfaces and is not protected, in the next cycle, and switches that would start one are
rejected. Broadcasters keep running. The e-stop starts engaged until every topic said false. A
protected controller, for example one that keeps an arm limp, is stopped only by a switch that
names it, by its own failure, by a fault of what it uses or, as a provider, together with an
unprotected controller that writes it and that the e-stop stops.

When the e-stop is released, the manager brings the startup hardware up again and activates the
startup controllers that are not active. The same happens when a hardware state request brings a
component back to active. Protected controllers, controllers whose joints are already in use and
controllers whose hardware is still down are skipped, and each skip is logged with the reason.

The e-stop topics expect one aggregating publisher per topic, such as the e_stop_manager. If several
nodes publish on one topic, the last message wins, so a release from one of them releases the topic.
A publisher that disappears counts as engaged after `estop_publisher_grace_ms`, a crashed one only
after the RMW's liveliness lease. A hung publisher that is still alive is caught only through
`estop_deadline_ms`.

## Differences from upstream ros2_control

At configure or startup the manager rejects asynchronous controllers, a per controller
`update_rate` that differs from the manager's, the interface configurations ALL and REGEX,
INDIVIDUAL_BEST_EFFORT for command interfaces, `fallback_controllers`, `enforce_command_limits:
true` and a hardware `rw_rate` other than 0 or `update_rate`.

`reload_controller_libraries` always fails. A BEST_EFFORT switch runs as STRICT and logs a warning.
STRICT accepts redundant requests, activating an active controller and deactivating an inactive one
do nothing, so a spawner that repeats a request does not fail. Naming an active controller in both
lists restarts it. AUTO also starts the inactive providers a started controller needs and restarts
providers whose chained mode changes. FORCE_AUTO additionally stops the active controllers that
claim an interface the started ones need, and the users of controllers it stops. The
`capabilities` parameter advertises FORCE_AUTO, so a client can send one FORCE_AUTO request instead
of planning a STRICT switch itself. `set_hardware_component_state` accepts the targets active and
inactive.

The e-stop topics, protected controllers and the `startup.*` parameters are additions. A
controller or component with a call that still runs after its deadline is reported in a transition
state, such as deactivating, on `~/activity` and in the list services. The execution times on
`~/statistics` are in microseconds like upstream. The controllers' introspection goes to
`/ros2_control/introspection_data` of the process-wide node `/ros2_control`, which outlives every
manager of the process, where upstream publishes it under the manager's name. It is published
every 50 ms of wall time, stamped with wall time.

## Diagnostics

The "Controller Manager Activity" diagnostic reports the state of every controller and component,
the skipped cycles of each component and the duration of the last control cycle as
`cycle_time_s`. While the e-stop is engaged its level is WARN and its message names the topic and
why it engaged, for example "it has no publisher any more". Refused switches and skipped startup
controllers name the same cause. Every failed request logs the reason and returns it in the service
response where the message has a field for it.

## Build and test

```bash
colcon build --packages-select hector_controller_manager --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select hector_controller_manager && colcon test-result --verbose
```

The tests need `cm_test_suite`. `test_explorer` enumerates the core's state space and takes several
minutes, far longer under sanitizers.
