# hector_controller_manager

A controller manager for ros2_control robots. It loads the standard controller and hardware
plugins through pluginlib and serves the `controller_manager_msgs` services, so the spawner,
`ros2 control` and existing launch files work unchanged. It replaces `controller_manager` where a
robot needs predictable behaviour under faults, hangs and e-stops.

Every request and every fault changes a desired state. A pure reconciler compares it with the
actual state and takes one safe step at a time. Each controller and each hardware component has an
owner thread that makes all of its non realtime plugin calls, so a hanging callback blocks only its
own plugin and is given up at its deadline. The control loop never allocates and never blocks.
[docs/design.md](docs/design.md) describes the internals, [docs/](docs/index.md) is the user
documentation, built with MkDocs. Preview it with
`pip install -r docs/requirements.txt && mkdocs serve`.

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
| `thread_priority` | 50 | SCHED_FIFO priority of the control loop on a realtime kernel, which only `ros2_control_node` applies, see the asynchronous owners below |
| `lock_memory` | true on a realtime kernel | Lock the memory of the process before the control loop starts, as upstream, which only `ros2_control_node` applies. A failure warns and the "Realtime Scheduling" diagnostic reports ERROR |
| `cpu_affinity` | | One core or a list of cores of the control loop, as upstream, which only `ros2_control_node` applies. A refused core warns |
| `default_timeout_ms` | 5000 | Deadline of requests that give no timeout, of fault handling and of the shutdown |
| `hardware_state_timeout_ms` | 30000 | Deadline of hardware state requests and of the bring up after loading |
| `hardware_fault_timeout_ms` | 5000 | Time of one hardware lifecycle call during fault handling |
| `hardware_hang_timeout_ms` | 100 | A used component without a finished read or write for this long faults, a mode switch has its own deadline |
| `mode_switch_timeout_ms` | 1000 | Time of every mode switch from its start, whoever started it, a request deadline or the grace never cuts it short |
| `estop_topics` | [] | `std_msgs/Bool` topics, true is engaged, reliable and transient local |
| `estop_publisher_grace_ms` | 1000 | How long a topic may have no publisher before it counts as engaged |
| `estop_deadline_ms` | 0 | A released topic without a message for this long engages, 0 is off, else at least 100 |
| `autostart_cooldown_ms` | 2000 | Wait after an e-stop release before the hardware is brought up |
| `autostart_attempts` | 3 | Attempts to bring the hardware up after a release |
| `enforce_command_limits` | true | Clamp the hardware commands to the joint limits, see below, read at startup |
| `command_limits_excluded_hardware` | [] | Components whose commands are not limited |
| `joint_limits.<joint>.*` | | Acceleration limits and tighter limits on top of the URDF, in the format of joint_limits |
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
the node at startup, and so does a `startup.*` or `hardware_components_initial_state.*` parameter
that is not a list of strings. That check runs only at startup, a later `ros2 param set` with
another type on a parameter given at launch makes the next autostart fail with an error and
activate nothing. `update_rate` must also be at most 65535, the list services report it as a 16 bit
number. `thread_priority` is left to the kernel.

The startup parameters together are the startup configuration. A few limits are fixed, a
component gets 60 s to load, a failed switch gets 400 ms on top of its deadline to roll back, and a
switch needs at least 100 ms left before its deadline to validate a hardware mode change. A request
answers within its timeout plus the longer of `mode_switch_timeout_ms` and those 400 ms, as a mode
switch that runs at the deadline finishes first.

A controller in `startup.controllers_active` that cannot start is skipped with the reason, together
with every controller that needs it, so a chain never runs with a link missing. The rest activate.
If one of them is not loaded at all, for example because its load timed out, no startup controller
is activated. The hardware still comes up and the log names the controller.

## Safety behaviour

A released command interface holds NaN, and so does every reference without an active writer. The
hardware is expected to hold a joint whose command is NaN, so a joint stops instead of keeping the
last command of a controller that ended. When one controller takes a joint over from another, the
hardware sees at most one mode switch and never a command of the old controller after the new one
started.

A fault only stops things. A failing controller stops together with everything chained to it, and a
hardware fault stops every controller that uses the component. Nothing is reactivated because of a
fault. A component that does not finish a read or write within the hang timeout faults, and a
plugin call that outlives its deadline is given up and quarantines its controller or component
until it returns. A mode switch, also the hold after an e-stop, does not count toward the hang
timeout. Every mode switch gets `mode_switch_timeout_ms` from its start, whether a request, fault
handling, the e-stop, a rollback or the shutdown started it. No deadline and no waiting request cuts
it short, since giving it up would fault a healthy component. Only a switch stuck longer is given
up. A request whose deadline passes meanwhile is answered as timed out once the switch ended, and
what it left is finished as idle work.

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

## Command limits

Between the controller update and the hardware write, the manager clamps every position, velocity
and effort command of a component to the limits of its joint, like upstream
`enforce_command_limits`. The limits come from the URDF `<limit>` and the `<ros2_control>` tags,
and the `joint_limits` parameters of the manager may tighten them and add an acceleration limit.

```yaml
controller_manager:
  ros__parameters:
    joint_limits:
      arm_joint_1: {has_acceleration_limits: true, max_acceleration: 5.0}
```

- A position command moves at most the velocity limit per period from the previous limited command,
  and its speed grows at most by the acceleration limit per period. Only the first write after an
  update moves the joints, by the time since the last write that did, at most `1 / update_rate`.
  So an overrun allows no longer step, a skipped write, for example during a mode switch, adds
  none, and further writes before the next update, as gz makes one per physics step, and the extra
  write after a hand over hold the joints.
  Slowing down is never limited, so a goal is not overshot.
- A revolute or prismatic goal is clamped to the joint's range, a continuous joint has none. A joint
  that is outside its range, for example after power on, holds there and its commands only move it
  back.
- A velocity command is clamped to the velocity limit and its speed grows at most by the
  acceleration limit per period, an effort command is clamped to the effort limit. Slowing down is
  not limited here either, a velocity command that reverses stops at once and then accelerates.
  A velocity command toward a position limit is slowed so the next period ends at the limit at
  most, and zeroed once the actual position reached it. With an acceleration limit it is slowed
  earlier, so the joint brakes at that limit and stops there. A write without time, as gz makes one
  per physics step, keeps the speed of the write before. An async component reads right after its
  write, so its position lags the last command by one period, which the limits add to it. It needs
  a position state, the command is NaN while the position is not finite, and while that state is
  busy the write skips, so the hardware keeps its last command one more period.
- Deceleration and jerk limits are not enforced. The `joint_limits` parameters reject them, the
  `<ros2_control>` tags may set them and they are ignored.
- NaN, the hold of a released interface, passes unchanged. The previous command restarts from the
  actual position and velocity after a NaN and whenever the claim of a command interface of the
  joint changes, so a new claimer, a reactivation after an e-stop or a fault, and the release of a
  torque off interface never make the joint jump. Until the actual position is known the command
  is NaN.
- `<limits enable="false"/>` on a joint or a command interface disables its limits. A limited
  position command and a velocity command with position limits need a position state, and a limit
  that a command uses must be positive, else the component fails to load. An unused `effort="0"`
  of a position joint is fine.
- RT applies the limits when it copies the commands to the plugin, see the hardware handles below.
  They count the cycles that clamped a command and the cycles that command a joint more than 5 mm
  or mrad outside its range, which sensor noise at a limit does not reach. The diagnostics report
  both per component as `clamped_cycles` and `out_of_range_cycles`, and the manager logs a warning
  once per diagnostics period while they grow. A cycle in which a handle is busy skips the write and
  counts as skipped.

The default is on, as upstream, so a controller that commands a jump cannot move a joint faster
than its URDF allows. Only the copy for the plugin is limited, so a goal that a controller writes
once still converges.

## Hardware handles

Controllers never share a handle with a hardware plugin. Every double interface of a component has
a shadow that the controllers read and write. RT copies the plugin's states into the shadows after
a read and the commands from the shadows, limited, into the plugin before a write. A command
interface that no running controller claims gets NaN, also while a stopping controller's last
command stays in its shadow until its release. For an asynchronous component RT copies the states
at its read and its write and the commands at its write, each only while the owner thread is
free, so in no plugin call and done with its last cycle. So a controller sees all joints of one
read, the hardware gets all joints of one update, and no command changes while the plugin reads it.
A cycle without a copy keeps the last snapshot and counts as skipped. An asynchronous component may
only export double interfaces, a synchronous one passes its other interfaces to the controllers
uncopied. A command that a plugin writes into its own handles, for example in
`perform_command_mode_switch`, lasts until the next write copies the shadow over it, NaN for an
interface without a writer.

The owner thread of an asynchronous component takes the `thread_priority` of its `<ros2_control>`
tag and the `affinity`, `print_warnings` and `thread_priority` of
`<properties><async .../></properties>`, where `thread_priority` overrides the tag's. On a
realtime kernel it runs with SCHED_FIFO at that priority, or one below the control loop's
`thread_priority` when the URDF sets none. The parser cannot tell an explicit 50 from none, so 50
also runs one below the control loop, 89 for a control loop at 90. Without realtime privileges it
only warns with the kernel's reason, as the control loop does. Off a realtime kernel the manager
logs once at startup that the owners and the control loop of `ros2_control_node` run without
realtime scheduling. Neither stops the manager, the diagnostics report it as an error. Threads a
plugin starts on its owner, such as an executor, keep the owner's cores but run with SCHED_OTHER, so
they cannot preempt a FIFO owner. Threads started on the control loop run with SCHED_OTHER too. An
embedding process such as gz
runs its control loop without SCHED_FIFO, so there the owners may run above it.
With `print_warnings`, the default, the manager logs once per diagnostics period while the
component skips cycles.

## Differences from upstream ros2_control

At configure or startup the manager rejects asynchronous controllers, a per controller
`update_rate` that differs from the manager's, the interface configurations ALL and REGEX,
INDIVIDUAL_BEST_EFFORT for command interfaces, `fallback_controllers`, a hardware `rw_rate`
other than 0 or `update_rate` and an asynchronous component whose `scheduling_policy` is not
`synchronized`.

`reload_controller_libraries` always fails, plugin libraries stay loaded until the process exits.
Hardware that an embedding process creates through its own factory, as gz_ros2_control does, is
not pinned, the manager does not own that loader and cannot know the library path. That is
harmless today, GazeboSimSystem registers no pre-shutdown callback.
A BEST_EFFORT switch runs as STRICT and logs a warning.
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
state, such as deactivating, on `~/activity` and in the list services. So is an active controller
that RT stopped, by a failure, a fault or the e-stop, until its `on_deactivate` ran, and an active
component with a fault, such as a hang, until its `on_error` ran. The execution times on
`~/statistics` are in microseconds like upstream.

`~/introspection_data` carries the values that active controllers and hardware plugins register,
as upstream, but not the values of the interfaces. RT samples them once per cycle and only while no
thread that writes them runs, so a cycle in which a controller is in a lifecycle call, a component
in a plugin call or an asynchronous component still in its cycle publishes no sample. A controller
or component hung in a plugin call stops all samples until it is gone. Values a plugin writes from
threads of its own or from callbacks of its node, which run on the manager's executor, are not
excluded. The registry is one per process, so it serves one controller manager per process.

## Diagnostics

The "Controller Manager Activity" diagnostic reports the state of every controller and component,
the skipped cycles of each component and the duration of the last control cycle as
`cycle_time_s`. While the e-stop is engaged its level is WARN and its message names the topic and
why it engaged, for example "it has no publisher any more". Refused switches and skipped startup
controllers name the same cause. Every failed request logs the reason and returns it in the service
response where the message has a field for it.

The "Realtime Scheduling" diagnostic reports the policy and priority in effect of the control loop
as `control_loop.scheduling` and of each asynchronous owner as `<component>.scheduling`, for
example "SCHED_FIFO 49". While one of them runs without SCHED_FIFO its level is ERROR and its
message names the thread and why, "no realtime kernel" or the kernel's refusal with its errno
text, else it is OK. The manager keeps running either way. In an embedding process such as gz the
control loop is not the manager's, it is reported as "run by the embedding process, not reported"
and only the owners count.

## Build and test

```bash
colcon build --packages-select hector_controller_manager --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select hector_controller_manager && colcon test-result --verbose
```

The tests need `cm_test_suite`. `test_explorer` enumerates the core's state space and takes about
two minutes, under sanitizers about seven on a sample of it.
