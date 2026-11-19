# hector_controller_manager design

## Overview

Every request (load, configure, cleanup, unload, switch, hardware state, startup, shutdown) and
every fault becomes a change of the **desired** state. The pure function `core::next_step` compares
desired with the **actual** state and returns one step that is safe on its own. The supervisor runs
that step on the owner threads of what it touches and applies the outcome to actual through
`core::apply`. Desired only shrinks on failures, so the loop terminates.

The README lists the parameters and the differences from upstream ros2_control. The startup
configuration named below is the parameters `startup.hardware_active`,
`startup.controllers_loaded`, `startup.controllers_active` and
`hardware_components_initial_state.*`. The grace is a fixed 400 ms that a request gets on top of
its deadline to roll back or, for a shutdown, to stop the hardware. The robot description is read
once, from the parameter or the first message on its topic, and a changing description is not
supported.

| Part | Responsibility | Thread |
|---|---|---|
| `core` (model, planner, chain rules, reconciler, rollback) | pure logic without ROS or threads | caller's |
| `Supervisor` | the only writer of the model, runs requests in FIFO order with the shutdown first, absorbs faults | supervisor |
| `StepExecutor`, `ControllerLifecycle` | run steps and lifecycle requests on the owners | supervisor |
| `runtime::OwnerThread` | one per controller and per hardware component, makes every plugin call except RT's | one each |
| `runtime::RtLoop` | `read`, `update`, `write` over the adopted `Plan` | RT |
| `runtime::Plan`, `PlanSlot` | immutable, preallocated, handed to RT through one atomic | built by supervisor |
| `ControllerManager` | ROS node, services, topics, parameters, status snapshot | executor threads |
| `EStopMonitor` | e-stop topics, their publishers and deadline, the cause | e-stop thread |

## Threads

| Call | Thread | Guard |
|---|---|---|
| `read`, `write` of sync hardware | RT | `IoGate`, RT skips the component while its owner runs a call |
| `read`, `write` of async hardware | its owner, triggered at the end of RT `write` | the owner serializes every call |
| `prepare`, `perform`, lifecycle, `on_error`, init, export | the component's owner | for sync hardware the owner holds the `IoGate` |
| `update` | RT | only controllers in the adopted plan that did not fail in this activation |
| controller `init`, lifecycle, `set_chained_mode`, assign and release | the controller's owner | the controller is in no adopted plan |
| executor callbacks of plugin nodes | executor threads | the plugin's own business |
| list services, activity topic, diagnostics | executor threads | read the supervisor's immutable snapshot |

At runtime there is one RT thread, the supervisor, one owner thread per controller and per
component, the executor threads (`executor_threads`), a startup thread for the bring up and, with
e-stop topics, one e-stop thread.
Each call of a request service runs on a thread of its own until it is answered, so an executor
thread never blocks on the supervisor. These request threads are not bounded, so a client that
floods calls gets one waiting thread per call, until each is answered or busy at its timeout plus
the grace. The supervisor runs one request at a time in FIFO order, only the shutdown goes ahead of
the waiting ones. A late return of a call is applied through the inbox, never by the owner itself.

Mutable state and its one writer:

| State | Writer | Readers |
|---|---|---|
| model, desired, request queue | supervisor | none, snapshots are copies |
| published plan slot | supervisor | RT |
| adopted and updated plan generation, RT in-call flag | RT | supervisor |
| controller `failed_activation` | RT | supervisor, owners |
| controller `failed_update`, the activation whose own update failed | RT | supervisor |
| sync hardware IO fault, hang fault, stop requests | RT | supervisor, owner |
| async hardware IO fault, progress | its owner | RT, supervisor |
| hardware lifecycle word, live interfaces, stop acknowledgements | its owner | RT, supervisor |
| timing statistics | RT and owners | diagnostics, sampled from atomics |

`runtime::ThreadRole` names the supervisor, the owners and RT. Checked builds (Debug and sanitizer
builds) assert the role on every entry point. RT holds its role per call, so `read`, `update` and
`write` may come from changing threads (gz_ros2_control) but never overlap.

`ros2_control_node` paces RT like upstream, on the steady clock or under `use_sim_time` on the ROS
clock. Each cycle gets the measured period, and a late cycle does not catch up in a burst. On the
ROS clock the first cycle waits until the clock started and a backward jump restarts the schedule.
The schedule restarts from the observed time, so the cycle rate is rounded to the /clock step (21 ms
for a 20 ms period and a 3 ms step), and switching `use_sim_time` at runtime gives one cycle with a
very long measured period, as upstream. Every deadline, the hang check, the e-stop topics and the
executor barrier stay on the steady clock, so they keep working while the simulation is paused. A
request that needs an RT cycle, such as a switch, then gives up at its wall deadline.

Wakeups use a futex `Doorbell`. Ringing is one atomic increment, and the wake syscall runs only when
a thread waits. RT never allocates, takes a blocking lock or lets an exception escape, which a
malloc hook in the tests checks.

### Async hardware

An async component's owner makes every call into the plugin, so the plugin needs no lock and two of
its calls never overlap. RT talks to the owner through words only RT writes.

1. **Trigger.** At the end of RT `write`, RT stores the cycle's time and period, bumps a counter and
   wakes the owner. The owner runs one cycle per wakeup, with pending stops, `write` and `read` in
   that order. Triggers that arrive while the owner is busy merge into one cycle, and RT counts the
   skip.
2. **Stop requests.** When a controller fails, RT bumps a counter per command interface of its
   failure scope. Before its next `write` the owner holds them with `perform(∅, stop)`.
3. **Hang check.** If a triggered cycle stays unserved longer than the hang timeout
   (`hardware_hang_timeout_ms`, 100 ms) while running controllers use the component, RT faults it
   and stops its scope.

The owner answers through the lifecycle word (state and a generation every transition bumps), the
IO fault word and the call fault word. RT reads them at its next `read` and stops the scope of a
fault once per generation.

## Invariants

- **Commands only from the current activation.** Hardware applies a command interface only after
  its claimer updated once in the current activation, and holds it before the claimer leaves the
  running set. A switch holds (`perform(∅, stop)`), detaches, deactivates, activates, attaches,
  waits for one update and starts (`perform(start, ∅)`). When a successor takes the joint over,
  the component is taken instead of held, so it gets no writes until one combined perform after
  that update (section "Atomic stop and start of command interfaces").
- **Released handles hold NaN.** After `on_deactivate`, also a refused one, and after a failed
  activation, the controller's owner writes NaN into every double command handle it released. A
  next claimer that writes nothing in its first updates then leaves NaN, never the old command.
- **Unclaimed references are NaN.** Before `on_activate` of a chainable controller its owner writes
  NaN into all its references, so a reference without an active writer never carries a stale value.
- **Chained mode** of a provider equals "an active controller writes it". It is set only on a
  controller about to start. A provider whose writer leaves restarts. This holds in settled states.
  During a teardown a provider stays chained for up to one supervisor step after its last writer
  was deactivated, while it is out of the plan, gets no update and its references hold NaN.
- **Update order.** A writer updates before the provider it writes, a provider before the readers
  of its exported states. A switch whose providers form a cycle, following written and read
  providers, is rejected. A writer that reads a provider down its own chain updates first and reads
  that state from the last cycle.
- **Failure closure.** The closure of a failed controller stops it, every writer and reader of a
  stopped controller and every provider a stopped controller writes. A hardware fault seeds every
  running claimer of the component. A failing pure reader stops only itself and its users.
- **Faults never reactivate.** A fault only shrinks desired. Nothing comes back up because of a
  fault, and a provider below a failed writer stops. A switch keeps the hardware targets of
  desired, so one accepted while a fault is still handled keeps the fault's target. Only a hardware
  state request raises a faulted component again. An activation by that request that brings the
  component up from not active or faulted runs the startup switch of the autostart on release, so
  only that request and a release bring the startup set back. The answer of the request waits for
  that switch, its `ok` reports the hardware only. A request in the short window between an RT
  fault and its absorption answers ok without restoring the startup set. The core selects the
  startup set (`select_startup`) from a model the shell copies from the status, and only the end to
  end tests of the startup triggers cover that copy.
- **Quarantine.** A call given up at its deadline still runs on its owner. Its entity is
  quarantined until every given up call returned. Nothing claims what a quarantined controller
  holds, and the providers it writes do not start. A component taken for a quarantined claimer
  counts as touched, so nothing new claims it. A switch that does not touch a quarantined entity
  succeeds once its desired state is reached apart from it. An activation given up at its deadline
  has failed, so a late SUCCESS of `on_activate` is followed at once by `on_deactivate` and the NaN
  of its handles on the same owner, before the return is reported. A late activation therefore
  never counts as active, also not over a provider the rollback stopped.

## Protected controllers

A controller loaded with the parameter `<controller>.protected: true` is never stopped implicitly. A
switch whose plan stops it without naming it in its deactivate list, by AUTO or FORCE_AUTO conflict
resolution or by a restart of a chain, is rejected with a reason that names it, and the e-stop does
not stop it either. The README lists what does. So a torque off controller that keeps an arm limp
gives the torque back only through a deliberate deactivation. The explorer checks every plan that
stops a controller implicitly once more with that controller protected and requires a rejection.

## E-stop

The parameter `estop_topics` names std_msgs/Bool topics, true is engaged, QoS reliable and
transient local. The controller manager makes no difference between a hard and a soft e-stop, the
hardware does. A hard e-stop cuts power, the hardware faults, and the failure closure of that fault
also stops protected controllers on it. A soft e-stop leaves the hardware active. Each topic is
expected to have one aggregating publisher, such as the e_stop_manager. With several publishers the
last message wins.

A topic counts as engaged while its last message says true, until its first message, and once it
had no publisher for longer than `estop_publisher_grace_ms` (1 s) since its last message or the last
time a publisher was counted. In each case it stays engaged until a message says false. With
`estop_deadline_ms` set, a released topic that receives no message for longer than the deadline
engages as well, also while its publisher is still counted. That suits a source that republishes
its state periodically, such as the aggregates of the e_stop_manager. A topic engages by whichever
rule fires first. The e-stop is engaged while any topic is.

A vanished publisher is found through the subscription's publisher count, which every RMW reports,
checked every 100 ms on the steady clock. A publisher that exits cleanly is noticed after the grace,
one that crashes or loses the network only after the RMW's liveliness lease plus the grace (about
10 s with Cyclone and zenoh by default), and a hung but alive one only through the deadline. The
grace keeps a latched false that rmw_zenoh delivers before it counts the publisher, and it catches
a publisher that releases and leaves before a check counted it, such as `ros2 topic pub --once`.
With rmw_zenoh a publisher counts as gone while the router connection is down, so a reconnect that
takes longer than the grace engages the e-stop. A missing or QoS incompatible publisher never
releases. The deadline is bookkept on the subscription side, so it needs no deadline QoS support of
the RMW.

The e-stop node must start engaged and publish true before any false, because a restarted node that
publishes false releases the e-stop and starts the autostart without an operator. A release must
come from a publisher that stays, a one shot false releases only for the grace.

- **Stop.** The e-stop subscriptions and their publisher check run in a callback group that a single
  threaded executor of its own spins on a thread of its own until shutdown, so no slow callback of
  the node and no executor size delays an engage. The controller manager sets a flag RT reads at
  every update. While it is set, RT stops every running controller of the adopted plan that claims
  command interfaces and is not protected, through the failure path of an update error. Its failure
  scope is marked failed in that activation and its hardware interfaces are held by their owners. So
  the stop happens in the next cycle, also while the supervisor works on a request, and the
  supervisor absorbs it like a fault. Desired shrinks, the controllers are deactivated, their
  handles get NaN. The failure scope follows the chain rules, a protected controller chained to a
  stopped one stops with it, so protection does not hold there, and a switch that activates such a
  chain logs a warning that names it. Broadcasters without command interfaces keep running.
- **Latch.** While engaged, a switch that would start a controller that claims command interfaces
  and is not protected is rejected with the reason. Starting a protected controller is allowed, for
  example a torque off controller during a soft e-stop. A switch accepted just before the flag is
  set loses its starts to the RT stop and rolls back like a start that fails in its first update.
- **Cause.** While engaged, the diagnostics status of the controller manager is WARN, and its
  message, a refused switch and a startup skip name the topic and why it engaged, which is that it
  says engaged, has no publisher any more or sent no message within `estop_deadline_ms`. The
  cause is the topic that engaged the e-stop, or the first topic at startup. When that topic
  releases while another keeps the e-stop engaged, the cause becomes an engaged one.

## Autostart on release

When every e-stop topic is released, the controller manager applies its startup configuration
again, on a thread of its own like a request.

1. It waits `autostart_cooldown_ms` (2 s) so that hardware that lost power can boot.
2. Every component that the startup configuration brings to active and that is not active, has a
   fault or a call still running goes active through the path of a hardware state request, up to
   `autostart_attempts` (3) times with the cooldown in between. A request that gave way to another
   one ("try again") is asked again within `hardware_state_timeout_ms` and uses up no attempt.
3. One AUTO switch activates the startup controllers that are not active. It leaves out every
   controller that cannot start now, so one of them never rejects the whole set. A controller is
   left out, with the reason in the log, if
   - it is protected, since only an explicit request brings a protected controller back after a
     fault;
   - the e-stop latch would reject it;
   - it claims a joint or a reference that a running controller claims, while writers of different
     references of one provider do not conflict;
   - the planner would reject its claims (`core::resolve_claims`), because hardware it commands is
     not active, has a fault or is taken for a quarantined claimer, or hardware it reads strictly is
     neither active nor inactive or has a fault;
   - its AUTO expansion (`core::auto_start_set`) needs a provider that is left out, such as a
     trajectory controller that writes only the references of a position controller on a protected
     joint;
   - it cannot start itself, being unconfigured after a failed configure, finalized or quarantined,
     or its expansion is rejected.

A startup controller the operator stopped on purpose comes back with the set, after every release
and after an explicit hardware activation. A left out controller is no failure of the autostart.

At the initial startup the same switch runs, without the controllers the latch would reject while
engaged. An autostart ends early when the e-stop engages again or the controller manager shuts
down, and a newer release supersedes an older autostart. Every outcome is logged.

## Failure handling

- **Validation.** Before any step, `prepare_command_mode_switch` with the full lists runs on every
  touched component. An error only rejects the switch. An error or exception in a later prepare or
  perform faults the component. What a switch finds left over is reconciled until 100 ms before
  its deadline. If that does not settle (a block by a quarantine counts as settled), or a switch
  that needs validation has less time left, the switch is rejected as busy, so it is never planned
  on an unsettled model and an expired deadline never quarantines a component that did not answer.
- **A failed step** shrinks desired by its failure closure, so the reconciler never retries it. The
  supervisor then sets desired to the state before the request minus what cannot run any more or
  a fault or the e-stop stopped meanwhile, and reconciles again, a rollback through the same steps.
  A controller that refused a chained mode keeps working in its old mode.
- **Reasons.** Every failed call reports in its `core::Outcome` the result or exception text of the
  callback and, for a controller, the state it ended in. rclcpp_lifecycle logs and swallows the
  exception of a controller callback, so its text appears only in the controller's log. A hardware
  fault keeps the reason of its first cause since the last transition, such as the result or
  exception of a read or write, a failed prepare or perform, RT not leaving a read or write in time,
  or a hang with the measured stall and the plugin call the owner was in. The writer of the fault
  records the reason into a fixed buffer without allocating before it publishes the fault. The
  supervisor logs each reason once with its step or fault and adds it to the answer of the request
  it hit.
- **Controllers.** `on_activate` FAILURE leaves the controller inactive, ERROR or an exception
  goes through the lifecycle error path. A started controller that fails in its first update fails
  the switch like a failed activation, and so does one that a hardware fault stops before the
  switch answers. An update that throws fails like one that returns ERROR. RT copies the
  exception's text into a fixed buffer of the controller without allocating (up to 255
  characters), and the supervisor logs it once with the failure and adds it to the answer of a
  switch that started the controller. Each controller the failure closure stops is named with the
  faulted component and its claim on it, or with the stopped controller through which the closure
  reached it. The supervisor builds that text from desired, RT only sets its flags. A controller
  that refuses to stop counts as stopped, it is shut down and reported finalized. A controller is
  reported active from its activation on, before its first update, so the status may lag RT
  during a switch. It settles once the switch answered.
- **Hardware.** ERROR or an exception of a lifecycle callback runs `on_error`. If that fails too,
  `on_shutdown` runs so the component releases its drive. A component that does not deactivate goes
  through `on_error`. A failed transition that leaves the component below what its claimers need
  stops them like a fault. Hardware state requests and the bring up after loading at
  initialization run under `hardware_state_timeout_ms` (30 s), because a bus driver's configure and
  activate take longer than the 5 s of `default_timeout_ms`.
- **Hang rule.** A component with running claimers that does not finish a cycle within the hang
  timeout (`hardware_hang_timeout_ms`) faults. A request starts a hold or start only with at least
  100 ms left before its deadline, whatever the hang timeout is, otherwise it is rejected without
  faulting the hardware. A call on the owner of a sync component waits for RT to leave a read or
  write for ten hang timeouts (1 s by default), then the component faults.
- **Calls in idle work.** Idle work (fault handling, what a request left over) gives each step the
  hang timeout, so a callback there that hangs or is merely slower is quarantined early until it
  returns, and a waiting request starts. A hardware lifecycle call in idle work (on_error,
  deactivate, cleanup, shutdown of a faulted component) gets `hardware_fault_timeout_ms` (5 s)
  instead, because a bus driver spends bus timeouts there on a dead bus. Its wait still ends within
  the hang timeout once a request is waiting, so the call is quarantined and the request starts.
  Controller calls, holds, takes, mode switches, detaches and the wait for an update keep the hang
  timeout, since they hold a joint or keep RT from a plan. A hardware state request for a component
  whose call still runs waits within its deadline until the call returned, but gives way to a
  request queued meanwhile and is answered "try again", so a hung call never blocks a waiting
  request.
- **Deadlines.** At a request's deadline desired becomes "what is active now and was active before",
  which only stops. The hardware start of activated controllers then follows without the request.
- **Shutdown.** Desired becomes empty and every component finalized. It goes ahead of the waiting
  requests and starts after the request in flight, so it is never dropped and the hardware always
  gets its `on_deactivate` and `on_shutdown`. From its start it returns within its timeout plus the
  grace. A shutdown that waited behind a slow call, such as fault handling under a long hang
  timeout, still gets 200 ms of the grace to hold and stop the controllers, the hardware gets the
  rest. After shutdown and after a broken invariant, switches and hardware state requests are
  rejected. A controller hung in `on_deactivate` keeps its providers active, every
  other component is deactivated and shut down, its interfaces held. Once a plugin was leaked,
  `ros2_control_node` ends with `std::quick_exit`, so no static destructor races the leaked plugin.
- **Signals.** rclcpp's handler of SIGINT and SIGTERM shuts the context down, and a controller
  cannot transition after that (its lifecycle cannot publish). The controller manager therefore
  shuts itself down in a pre-shutdown callback of its node's context, while the control loop still
  runs, also in a process that embeds it and shuts the context down itself. The process ends within
  `default_timeout_ms` plus the grace once the request in flight returned. Only that request's own
  deadline bounds the wait for it, which is 30 s for a hardware state request, up to 60 s per
  component plus the bring up for `initialize`, and the client's timeout for a switch. A controller
  whose stop is refused and whose shutdown fails too, as after a context that ended without its
  pre-shutdown callbacks, counts as finalized and is never used again, so a stop is never retried
  forever. The callback runs on rclcpp's signal thread, so while it waits a second SIGINT or a
  SIGTERM changes nothing. The README says how long ros2 launch waits before its SIGKILL.

## Atomic stop and start of command interfaces

Every control mode change drops Dynamixel torque briefly and costs bus time. A switch therefore
calls `perform_command_mode_switch(start, stop)` at most once per component, and not at all when a
restart changes neither interfaces nor modes. Below, A is the claimer of a live interface that stops
in the switch and B its successor, which may be A again or claim another mode.

**NaN write.** Once the last claimer of a taken component that stops in the switch returned from
`on_deactivate` and its NaN is in, the component's owner calls `write()` once, bypassing the block.
Every live interface of the component then holds NaN, and the hardware holds a joint whose command
is NaN, so a velocity joint stops instead of keeping the last speed of the ended activation. Writes
stay blocked afterwards until SwitchModes, because B may write its handles in `on_activate` and a
command applies only after its claimer updated (section "Invariants"). Between Detach and the end of
`on_deactivate` the component gets no writes, because the controller may write its handles there. A
NaN write that does not return by the step's deadline faults and quarantines the component like a
Take that timed out, so no claimer starts on it while that write may still read handles. Reads go on
during the handover. Dynamixel's bus watchdog (4 cycles by default) counts any instruction packet,
reads included, according to the Robotis e-Manual.

A hung `on_deactivate` gets no NaN write. NaN written beside the still running callback could be
overwritten by it before the write reads the handle, and the hardware would apply a command of
the ended activation. Instead its live interfaces of the taken component are held with
`perform(∅, stop)` once the call is given up, a second mode change accepted for this error case,
so the joint stops instead of carrying the last command. The component stays taken with writes
blocked until the callback returned and its job wrote the NaN, also for the held interfaces.

### Sequence

A component is *taken* while a live interface of it changes its claimer, only if a controller
desired after the switch commands that joint, and only if every running claimer of its live
interfaces stops in this switch. A take saves no perform for a plain stop, a deactivation without
a successor or a shutdown, so these hold before Detach. Lifecycle callbacks run inside the take and
may block until the deadline, so a claimer that keeps running on another joint of the
component would lose its writes for that long. Such a component holds and starts. A switch that
stops A and starts B runs these steps.

| Step | Thread | Effect |
|---|---|---|
| Take | supervisor, then the component's owner | writes are blocked, then a no op job on the owner proves for async hardware that no write that began earlier still runs |
| Detach | supervisor, RT | plan without A, wait until RT runs no older plan |
| Deactivate | A's owner, then the component's owner | `on_deactivate`, release, then NaN into A's hardware command handles. After the last stopping claimer of the component, one `write()` of the NaN |
| Activate | B's owner | assign, `on_activate` |
| Attach, AwaitUpdate | supervisor, RT | plan with B, wait until RT finished an update with it |
| SwitchModes | the component's owner | at most one `prepare` and `perform(start, stop)`, then writes are unblocked. With both lists empty the supervisor unblocks them without a job |

For sync hardware the no op job proves nothing, because RT makes the writes. Detach provides the
guarantee there. It waits until RT runs no older plan, and RT checks the block before each write.

`start` lists the interfaces whose claimer is updated and that are not live, `stop` the live ones
without an updated claimer. A taken component also stops in it the joints whose claimer stops
without a successor. The first write after the handback carries a command of B's current
activation, or NaN. An activation that fails also leaves NaN in its hardware command handles.

Interfaces that only start (no live interface changes its claimer) are not taken. They start in
SwitchModes after AwaitUpdate, which is one perform. Interfaces that cannot be neutralized (handles
that do not hold a double), interfaces of a faulted component and interfaces of a claimer that
failed are held before Detach, so they may cost a second mode change.

### NaN on every release

Every release of a hardware command interface writes NaN after the controller's callback returned,
for a planned stop, a restart, a handover, the hold and start fallback, the stop after a fault, a
refused stop, a shutdown and a failed activation. All of them go through the controller's
Deactivate (or a failed Activate) on its owner. A first update without a write is legitimate
controller behaviour, for example a controller that has no command yet. The hardware then sees
NaN and must hold the joint. Without the NaN it would apply the last command of the ended
activation.

### Why the NaN is written after `on_deactivate`, off RT

The component is taken before Detach, so RT applies no command of A while the NaN is pending. The
NaN goes in after `on_deactivate`, because a controller may write its handles there. RT is the
wrong writer for it, because RT need not adopt the detach plan at all (a loop between cycles is
quiescent at once, and the next plan it sees is the attach plan), and RT may only try_lock a
handle. A's owner writes with a blocking lock, which cannot fail for a double handle, after RT is
quiescent and before B is assigned, so no thread writes the handle concurrently. The plugin's own
reset on activation stays the primary mechanism.

### Invariants

- A live interface of a component that is not taken has a claimer that is running and updated in its
  current activation (`check_invariants`).
- Taken implies writes blocked in the runtime, for sync (RT write) and async (owner write) alike.
  The model's `taken` and the runtime's write block change together. Take sets both, a SwitchModes
  that returned in time clears both, and every lifecycle transition clears both (the runtime block
  is tied to the lifecycle generation).
- A settled state has no taken component, so `next_step` gives a taken component back in the
  SwitchModes slot, after every activation that can still run. A desired controller that is blocked
  (quarantine) therefore does not keep a component taken. A claimer of a live interface that is
  detached but not stopped (active and quarantined) does, because its handles may still be written.

### Failures and timeouts

- **perform fails.** The component faults, writes stay unapplied because of the fault, and the fault
  path stops its claimers and runs `on_error`.
- **perform outlives the deadline.** The component faults and is quarantined, its writes stay
  blocked until its next lifecycle transition, so a late perform never applies anything. A
  SwitchModes, like Hold and Take, starts only with 100 ms left before the deadline, unless it only
  gives components back without a mode change.
- **Take outlives the deadline** (an async owner stuck in a cycle). The component faults and stays
  blocked, like after a mode switch that timed out.
- **A callback hangs.** A hung `on_activate` quarantines its controller, which is not an updated
  claimer, so SwitchModes stops its interfaces and gives the component back. A hung `on_deactivate`
  holds its live interfaces through `perform(∅, stop)` and keeps the component taken until it
  returned and wrote its NaN in the same job, because the plugin may still write its handles. One
  that never returns keeps it taken for good, without writes, until shutdown.
- **RT stuck in a call during Detach.** The detached controllers stay quarantined until RT left the
  call. Writes stay blocked until A's Deactivate and its NaN, which follow only after RT left, so a
  command A computed in that late call is never applied, also not by the old plan's write after the
  call.
- **The deadline passes before SwitchModes.** Only RT progress is missing, so the request is
  answered and the idle reconcile hands the component back when RT cycles.
- **A fault during the handover.** RT stops the failure scope and asks the owner for an urgent hold.
  SwitchModes skips the start of an interface whose claimer failed, and the urgent hold lands before
  the next write because writes skip while a stop request is pending.
- **Async hardware.** The owner checks the block before every write. The no op job of Take makes a
  write that began before the block finish before Detach, so it can only carry A's commands from
  while A still ran.
- **A rollback** runs through the same steps. A restarted A on the same interfaces keeps them live
  and costs no perform. A known gap is a handover to another mode whose successor fails, which stops
  the joint in SwitchModes and the rollback starts A's mode again, two performs.
