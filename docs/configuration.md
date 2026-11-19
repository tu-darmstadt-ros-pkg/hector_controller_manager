# Configuration

hcm reads its parameters once at startup. `startup.controllers_active` is read again at every
autostart, the per controller parameters when the controller is loaded.

| Parameter | Default | Meaning |
|---|---|---|
| `update_rate` | 100 | Control loop rate in Hz, 1 to 65535 |
| `executor_threads` | 2 | Threads of the node's executor, ignored when an embedding process passes its own |
| `thread_priority` | 50 | SCHED_FIFO priority of the control loop on a realtime kernel. Async owners without a priority of their own run one below it |
| `lock_memory` | true on a realtime kernel | `ros2_control_node` locks the memory of the process before its control loop starts, as upstream. A failure warns and the "Realtime Scheduling" diagnostic reports ERROR |
| `cpu_affinity` | | One core or a list of cores the control loop of `ros2_control_node` runs on, as upstream. A refused core warns |
| `use_sim_time` | false | Pace the loop on `/clock`, passed on to the controllers |
| `robot_description`, `robot_description_semantic` | "" | When empty, the first message on the latched topic of the same name. The SRDF is forwarded to every controller that declares it |
| `default_timeout_ms` | 5000 | Deadline of a request without a timeout, of fault handling and of the shutdown |
| `hardware_state_timeout_ms` | 30000 | Deadline of `set_hardware_component_state` and of the bring up after loading |
| `hardware_fault_timeout_ms` | 5000 | Time of one hardware lifecycle call during fault handling |
| `hardware_hang_timeout_ms` | 100 | A component in use without a finished `read` or `write` for this long faults |
| `mode_switch_timeout_ms` | 1000 | Time of every mode switch from its start |
| `estop_topics` | [] | `std_msgs/Bool` e-stop topics, see [Faults and safety](concepts/faults.md#e-stop) |
| `estop_publisher_grace_ms` | 1000 | How long a topic may have no publisher before it counts as engaged |
| `estop_deadline_ms` | 0 | A released topic without a message for this long engages. 0 is off, else at least 100 |
| `autostart_cooldown_ms` | 2000 | Wait after a release before the hardware is brought up, may be 0 |
| `autostart_attempts` | 3 | Attempts to bring the hardware up after a release |
| `hardware_components_initial_state.unconfigured`, `.inactive` | [] | Components left in that state, as upstream, also after a release. All others go active at startup and after every release |
| `startup.hardware_active` | [] | Components that go active even if listed above |
| `startup.controllers_loaded` | [] | Controllers loaded and configured at startup |
| `startup.controllers_active` | [] | Controllers activated by one AUTO switch at startup and after every release |
| `enforce_command_limits` | true | Clamp position, velocity and effort commands to the joint limits |
| `command_limits_excluded_hardware` | [] | Components whose commands are not limited |
| `joint_limits.<joint>.*` | | Acceleration limits and tighter limits on top of the URDF, in the `joint_limits` format |
| `<controller>.type` | | Plugin class |
| `<controller>.params_file` | | One file or a list. The controller node reads only these, not the manager's arguments |
| `<controller>.node_options_args` | [] | Extra ROS arguments for the controller node, such as remaps |
| `<controller>.protected` | false | Never stopped implicitly, see [Faults and safety](concepts/faults.md#e-stop) |

Loading one component has a fixed limit of 60 s.

A position command moves at most the velocity limit per period, so a controller that commands a jump
cannot move a joint faster than the URDF allows. Deceleration and jerk limits are not enforced.

On top of the [quickstart](index.md#quickstart) configuration, a robot whose e-stop node republishes
its state every 200 ms and whose arm joint gets an acceleration limit adds

```yaml
controller_manager:
  ros__parameters:
    estop_deadline_ms: 1000
    hardware_components_initial_state:
      inactive: [servo_bus]
    joint_limits:
      arm_joint_1: {has_acceleration_limits: true, max_acceleration: 5.0}
```

## Startup checks

A value below its minimum, or a startup list that is not a list of strings, stops the node. The
robot description is rejected when a component sets `rw_rate` other than 0 or `update_rate`, an
async `scheduling_policy` other than `synchronized`, when `startup.hardware_active`,
`hardware_components_initial_state.*` or `command_limits_excluded_hardware` names an unknown
component, or when `joint_limits` names no limited joint while `enforce_command_limits` is on.

!!! warning
    A rejected description fails the initialization. No hardware comes up and the node needs a
    restart. The "Controller Manager Activity" diagnostic reports "Initialization failed, the node
    needs a restart" and the log names the cause.
