# Interfaces

Names are relative to the node, `controller_manager` by default. Services and messages are those of
`controller_manager_msgs`, and the services appear once initialization succeeded.

## Services

The list services (`~/list_controllers`, `~/list_controller_types`, `~/list_hardware_components`,
`~/list_hardware_interfaces`) never wait for a running request. The request services queue to the
supervisor in FIFO order. Where the response has a `message`, it carries the reason of a failure.

| Service | Differs from upstream |
|---|---|
| `~/switch_controller` | See [Switches](concepts/switches.md) |
| `~/set_hardware_component_state` | Targets active and inactive only. Activating a component that was not active also runs the startup switch |
| `~/reload_controller_libraries` | Always fails, libraries stay loaded until the process exits |

A controller or component whose call outlived its deadline is reported in a transition state, such
as deactivating, until the call returns. So is an active controller that RT stopped, until its
`on_deactivate` ran, and an active component with a fault, until its `on_error` ran.

## Topics

**Changed:** the introspection moved from `~/introspection_data` of the manager, for example
`/controller_manager/introspection_data`, to `ros2_control/introspection_data` in the namespace of
the first manager of the process, `/ros2_control/introspection_data` in the root namespace.
Subscribers of the old topic must change it.

| Topic | Content |
|---|---|
| `~/activity` | `ControllerManagerActivity` with the state of every controller and component, on every change, reliable and transient local |
| `~/statistics/names`, `~/statistics/values` | Execution times in µs every 50 ms, the cycle as `<node>.total_time`, each controller update and each component read and write |
| `<ns>/ros2_control/introspection_data/names`, `.../values` | Values that controllers and plugins register, without the interface values. Published by the process-wide node `ros2_control` in the namespace `<ns>` of the first controller manager of the process, sampled only while one manager lives in it |

## Diagnostics

**Controller Manager Activity** is WARN while the e-stop is engaged, with the topic and why it
engaged, or while the node waits for the robot description, and ERROR after a failed initialization.
It lists every state, `cycle_time_s`, and per component `skipped_cycles`, `clamped_cycles` and
`out_of_range_cycles`. Growing skipped cycles mean an async owner does not finish within a period or
a plugin call holds the component. Growing clamped cycles mean a controller commands beyond the
joint limits.

**Realtime Scheduling** is ERROR while the control loop or an async owner runs without SCHED_FIFO,
for example "no realtime kernel". In an embedding process only the async owners count.
