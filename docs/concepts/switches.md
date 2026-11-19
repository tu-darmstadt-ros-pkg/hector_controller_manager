# Switches and chained controllers

## Strictness

- **STRICT** starts and stops exactly what the request names. Every provider a started controller
  needs must be active or started with it. Activating an active controller or deactivating an
  inactive one does nothing, so a repeated request does not fail.
- **BEST_EFFORT** runs as STRICT and logs a warning.
- **AUTO** also starts the inactive providers a started controller needs, transitively.
- **FORCE_AUTO** is AUTO that also stops the active controllers claiming an interface the started
  ones need, together with their users.

Naming an active controller in both lists restarts it. In every strictness, a running provider whose
chained mode changes restarts. Deactivating a provider whose users stay active is rejected unless
the request is FORCE_AUTO. A switch that would stop a protected controller it does not name is
rejected. The read only `capabilities` parameter lists `force_auto`, so a client can send one
FORCE_AUTO request instead of planning a STRICT one.

For example, with `diff_drive_controller` writing the references of a chainable `wheel_pid`
controller, one AUTO request for `diff_drive_controller` starts `wheel_pid` first, then
`diff_drive_controller`.

## How a switch runs

1. Leftover work from an earlier request or fault is finished first.
2. The switch is planned or rejected, for example for two claimers of one interface, two modes on
   one joint, a missing provider, a chain cycle, or a start while the e-stop is engaged.
3. `prepare_command_mode_switch` runs with the full start and stop lists on every touched component.
   An error rejects the switch before anything changed. `prepare` runs again right before the
   `perform`, so a plugin sees two prepares per mode change.
4. The steps run. A component normally gets one `perform_command_mode_switch` per switch, also when
   one controller takes a joint over from another. A new claimer's commands reach the interface from
   its first `update` on, which can be before `perform_command_mode_switch` starts it. Until then
   the interface holds NaN.
5. If a step fails, or a started controller fails in its first update, the switch rolls back.

A request answers within its `timeout` (0 means `default_timeout_ms`) plus the longer of
`mode_switch_timeout_ms` and 400 ms, since a mode switch that runs at the deadline finishes first.

## Chains

A provider is in chained mode exactly while an active controller writes its references, and it
restarts when its last writer leaves. A writer updates before the provider it writes. A failure
stops the failed controller, every writer and reader of a stopped controller, and every provider a
stopped controller writes.
