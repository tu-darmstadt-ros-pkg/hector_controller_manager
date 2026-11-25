# Migrating from ros2_control

Change the package of the node to `hector_controller_manager`. The executable stays
`ros2_control_node`, and controller `type` and `params_file` entries stay as they are.

## Replacing the spawner

The upstream spawner and `ros2 control` work against hcm. To bring the robot up without a spawner,
and again after every e-stop release, move the spawner arguments into the startup parameters.

```python
# before
Node(package="controller_manager", executable="spawner",
     arguments=["arm_controller", "--param-file", "arm_controller.yaml"])
```

```yaml
# after
controller_manager:
  ros__parameters:
    startup:
      controllers_loaded: [arm_controller]
      controllers_active: [arm_controller]
    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController
      params_file: arm_controller.yaml
```

A spawner with `--inactive` maps to `startup.controllers_loaded` only. `--activate-as-group` is not
needed, the startup switch activates the whole list at once and starts the providers it needs. Do
not run a spawner for the same controllers next to `startup.*` or `estop_topics`, both would act
after a release.

## Differences

- A released command interface holds NaN. The hardware must hold a joint whose command is NaN.
- STRICT accepts redundant requests, BEST_EFFORT and strictness 0 run as STRICT with a warning,
  FORCE_AUTO is supported.
- The `activate_asap` flag of a switch request is ignored.
- `list_controllers` lists the controllers in name order.
- Controllers get no `on_shutdown` when the process exits.
- The introspection is on `ros2_control/introspection_data` in the namespace of the first
  controller manager of the process, not on `~/introspection_data` of the manager.
- At shutdown a controller hung in `on_deactivate` leaves the hardware only the 400 ms grace to
  deactivate and shut down.
- Deactivating hardware is refused while an active controller commands it, where upstream refuses
  nothing.
- Readers keep running on deactivated hardware.
- A velocity command toward a position limit is slowed to the speed that reaches the limit within
  one period, as upstream. With an acceleration limit it also brakes before the limit, so the joint
  stops there.
- A velocity command interface of a joint with position limits needs a position state of that joint,
  else its component fails to load. `<limits enable="false"/>` on the joint also drops its velocity
  and acceleration clamp, so give it a position state or make it continuous in the URDF.
- Each command interface of a hardware plugin is a copy that RT refreshes every write. A value the
  plugin writes into its own command handle, for example in `perform_command_mode_switch`, lasts
  only until the next write.
- A plugin may see a new claimer's command in `write` before `perform_command_mode_switch` starts
  its interface, so it must act only on started interfaces.
- On SIGINT or SIGTERM hcm finishes the request in flight, then deactivates and shuts down the
  hardware within `default_timeout_ms` plus `mode_switch_timeout_ms` plus 400 ms. Set the launch `sigterm_timeout` above the longest hardware call,
  ros2 launch sends SIGKILL after 10 s by default.

## Rejected

- Async controllers and a per controller `update_rate` other than the manager's.
- The interface configurations ALL and REGEX, and INDIVIDUAL_BEST_EFFORT for command interfaces.
- `fallback_controllers`.
- An async component that exports a non double interface, and the hardware settings listed under
  [Startup checks](configuration.md#startup-checks).
- A robot description that changes at runtime. Only the first one is used.
