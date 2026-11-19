# hector_controller_manager

hector_controller_manager (hcm) is a replacement for the ros2_control `controller_manager` node. It
loads the same controller and hardware plugins and serves the same `controller_manager_msgs`
services, so `ros2 control`, the spawner and existing clients keep working.

## Why it exists

hcm builds on the ros2_control controller manager and its plugin interfaces. It was written for
robots that combine chained controllers with async hardware and switch between them at runtime.
That combination needs a few things the upstream controller manager does not offer yet, or handles
in ways that are hard to make robust within its current design.

- **Switches as one plan.** Activating a chain often takes several steps, providers before their
  users and mode switches in between. hcm validates the whole switch first and only then carries it
  out, instead of applying it step by step. If a step fails, it rolls back to the state before.
- **AUTO and FORCE_AUTO.** A request may name only the controller you want, hcm starts the
  providers it needs and, with FORCE_AUTO, stops what claims the same interfaces. See
  [Switches and chains](concepts/switches.md).
- **Async hardware.** Lifecycle, mode switch and async read and write calls run on a thread owned by
  each component, so a slow or hanging bus blocks only itself.
- **Hardware error paths.** A fault stops exactly what depends on it, in the same cycle, and the
  state stays consistent afterwards. E-stop topics and a startup configuration are built in.
- **Status you can trust.** `~/activity` and the list services report every state change,
  including those caused by faults and e-stops.

To keep this manageable, hcm deliberately leaves out some upstream features, such as async
controllers and `fallback_controllers`. The full list is under [Migration](migration.md#rejected).

## Quickstart

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
    estop_topics: [/estop]
    startup:
      controllers_loaded: [joint_state_broadcaster, arm_controller]
      controllers_active: [joint_state_broadcaster, arm_controller]
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController
      params_file: arm_controller.yaml
```

hcm activates every component of the URDF, loads `startup.controllers_loaded` and activates
`startup.controllers_active`. The e-stop starts engaged, so `arm_controller`, which claims command
interfaces, starts only once `/estop` publishes false, and again after every release. Without a
`robot_description` parameter it waits for the latched `robot_description` topic. The services
appear once initialization succeeded. Check with `ros2 control list_controllers`.

The e-stop subscription is reliable and transient local, and a release must come from a publisher
that stays. For a test this releases it.

```bash
ros2 topic pub -r 5 --qos-reliability reliable --qos-durability transient_local \
  /estop std_msgs/msg/Bool "{data: false}"
```
