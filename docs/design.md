# hector_controller_manager design

## Overview

Every request (load, configure, cleanup, unload, switch, hardware state, startup, shutdown) and
every fault becomes a change of the **desired** state. The pure function `core::next_step` compares
desired with the **actual** state and returns one step that is safe on its own. The supervisor runs
that step on the owner threads of what it touches and applies the outcome to actual through
`core::apply`. Desired only shrinks on failures, so the loop terminates.

[Configuration](configuration.md) lists the parameters and [Migrating](migration.md) the differences
from upstream ros2_control. The startup configuration named below is the parameters
`startup.hardware_active`, `startup.controllers_loaded`, `startup.controllers_active` and
`hardware_components_initial_state.*`. The grace is a fixed 400 ms that a request gets on top of its
deadline to roll back or, for a shutdown, to stop the hardware. The robot description is read once,
from the parameter or the first message on its topic, and a changing description is not supported.

| Part | Responsibility | Thread |
|---|---|---|
| `core` (model, planner, chain rules, reconciler, rollback) | pure logic without ROS or threads | caller's |
| `Supervisor` | the only writer of the model, runs requests in FIFO order with the shutdown first, absorbs faults | supervisor |
| `prepare_hardware` | parses and checks the components of the description before any plugin exists | supervisor |
| `StepExecutor`, `ControllerLifecycle` | run steps and lifecycle requests on the owners | supervisor |
| `Entities`, `Inbox` | the runtimes by name, late returns and owner events for the supervisor | supervisor, owners |
| `RequestQueue`, `RequestThreads` | the requests waiting for the supervisor, one thread per request call | request threads |
| `runtime::OwnerThread` | one per controller and per hardware component, makes every plugin call except RT's | one each |
| `runtime::HardwareRuntime` | a component's plugin, its owner and what RT and the owner share | RT, owner |
| `runtime::ControllerRuntime` | a controller's plugin, its owner and the failure words RT writes | RT, owner |
| `runtime::ShadowHandles`, `CommandLimiter` | the handles controllers use, the copies to and from the plugin and the command limits | RT, owner |
| `runtime::RtLoop` | `read`, `update`, `write` over the adopted `Plan` | RT |
| `runtime::Plan`, `PlanSlot`, `build_plan` | immutable, preallocated, handed to RT through one atomic | built by supervisor |
| `runtime::IoGate`, `Doorbell`, `FaultLatch`, `FixedText`, `WriteBlock`, `StopBatch`, `OwnerCycle`, `CycleStamp`, `HangWatch`, `HardwareStats`, `ThreadRole` | the primitives RT shares with the other threads, see "Atomics" | RT and the others |
| `ControllerManager` | ROS node, services, topics, parameters, status snapshot | executor threads |
| `Startup` | the startup configuration at the bring up and again after an e-stop release | startup thread, request threads |
| `Diagnostics` | the activity and scheduling diagnostics with the cycle counters of each component | executor threads |
| `ExecutionTimeRegistry` | the execution times registered on `~/statistics`, a registry per manager | executor threads |
| `Introspection` | the process-wide introspection registry, its node and the process gate | constructors, RT |
| `ExecutorBarrier` | confirms that the executor dropped a removed controller node | controller owners |
| `PluginLoader` | pluginlib loaders of controllers and hardware, pins their libraries | owners, executor threads |
| `EStopMonitor` | e-stop topics, their publishers and deadline, the cause | e-stop thread |

## Threads

| Call | Thread | Guard |
|---|---|---|
| `read`, `write` of sync hardware | RT | `IoGate`, RT skips the component while its owner runs a call |
| `read`, `write` of async hardware | its owner, triggered at the end of RT `write` | the owner serializes every call |
| copies between the shadow handles and the plugin's handles | RT, the write through a block on the owner | `IoGate`, and for async hardware an owner that served every trigger |
| `prepare`, `perform`, lifecycle, `on_error`, init, export | the component's owner | the owner holds the `IoGate` |
| `update` | RT | only controllers in the adopted plan that did not fail in this activation |
| controller `init`, lifecycle, `set_chained_mode`, assign and release | the controller's owner | the controller is in no adopted plan |
| executor callbacks of plugin nodes | executor threads | the plugin's own business |
| list services, activity topic, diagnostics | executor threads | read the supervisor's immutable snapshot |

At runtime there is one RT thread, the supervisor, one owner thread per controller and per
component, the executor threads (`executor_threads`), a startup thread for the bring up and, with
e-stop topics, one e-stop thread.
Each call of a request service runs on a thread of its own (`RequestThreads`) until it is answered,
so an executor thread never blocks on the supervisor. These request threads are not bounded, so a
client that floods calls gets one waiting thread per call, until each is answered or busy at its
timeout plus the grace. The supervisor runs one request at a time in FIFO order, only the shutdown
goes ahead of the waiting ones. A load, configure, cleanup or unload gets `default_timeout_ms` from
its start, because a plugin call given up at its deadline is leaked. It answers within twice that
timeout plus the grace. A late return of a call is applied through the inbox, never by the owner
itself.

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
| values registered for introspection | RT and the owner, for async hardware only the owner | RT, see "Introspection" |
| scheduling of the control loop, of an async owner | RT before its first cycle, the owner before its load returns | the "Realtime Scheduling" diagnostic |

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

RT copies every double interface between the plugin's handles and the shadow handles, under the try
locks of each handle. The whole cycle of `bench_cycle` grows by about 36 ns per handle on a Ryzen 9
9950X, from the slope over 8 to 128 joints, an upper bound for the copy of a handle. Athena's 79
double handles on its two buses cost at most about 3 µs per cycle, 0.015 % of its 20 ms period, so
the copies are not optimised.

### Atomics

Every atomic of the package with its order. Its header states who writes and reads it and what it
protects. A change of an order updates its row and states why the new order suffices, since x86
hides a missing acquire or release and only a review or a weakly ordered CPU catches it. Orders are
rel release, acq acquire, acq_rel for read modify writes, sc sequentially consistent and rlx
relaxed.

| Atomic | Class | Order |
|---|---|---|
| `bits_` | `IoGate` | acq_rel, acq |
| `sequence_`, `waiters_` | `Doorbell` | sc |
| `text_` | `FixedText` | rlx, published by its owner's release |
| `word_` | `FaultLatch` | rel, acq, rlx for the writer's own check |
| `generation_` | `WriteBlock` | rel, acq |
| `requests_[]` | `StopBatch` | rel RMW, acq |
| `begun_` | `StopBatch` | rlx, the counters publish it |
| `total_`, `done_` | `StopBatch` | rel, acq |
| `kind_` | `OwnerCycle` | rel, acq, rlx CAS |
| `time_ns_`, `period_ns_` | `CycleStamp` | rlx, an async cycle publishes them |
| `call_` | `HangWatch` | rlx |
| `switching_` | `HangWatch` | rel, acq |
| `read_seconds_`, `write_seconds_`, `skipped_`, `out_of_range_` | `HardwareStats` | rlx |
| `clamped_`, `last_clamped_` | `HardwareStats` | rel, acq for the count, rlx index |
| `lifecycle_` | `HardwareRuntime` | rel, acq |
| `claim_words_[]`, `updates_` | `ShadowHandles` | rlx, the gate orders an owner copy |
| `published_`, `rt_state_`, `rt_updated_`, `rt_cycle_open_`, `waiting_` | `PlanSlot` | sc |
| `failed_activation` | `ControllerRuntime` | rel, acq |
| `failed_update`, `estop_activation` | `ControllerRuntime` | rlx, after `failed_activation` |
| `update_seconds_` | `ControllerRuntime` | rlx |
| `cycle_seconds_`, `cycle_start_` | `RtLoop` | rlx |
| `estop_` | `RtLoop` | rel RMW, acq RMW by RT, acq |
| `engaged_` | `EStopMonitor` | acq_rel exchange, acq |
| `state_` | `OwnerThread::Job` | acq_rel CAS, acq |
| `id` | `OwnerThread` | sc |
| `id_` | `ThreadRole` | sc |
| `initialization_failed_`, `stopped_cleanly_` | `ControllerManager` | sc |
| `fired` (local) | `ExecutorBarrier` | sc |
| `done` | `RequestThreads` | sc |
| `leaked` | `leak.cpp` | sc |

### Async hardware

An async component's owner makes every call into the plugin, so the plugin needs no lock and two of
its calls never overlap. RT talks to the owner through words only RT writes. Controllers use shadow
handles, a copy of every double interface of the component, and RT copies between them and the
plugin's handles only while the owner is free. RT enters the `IoGate`, which the owner holds for
every plugin call outside its cycle, and the owner served every trigger. RT copies the states in
`read` and again in `write`, so an owner whose read ends after RT's read still delivers its states,
and the commands in `write`. The states a controller sees come from one plugin read and a plugin
write applies one update.
Near a position limit a drive that lags its velocity command makes the limiter's cap alternate,
which stays conservative and bounded.

1. **Trigger.** At the end of RT `write`, if the owner is free, RT stores the cycle's time and
   period, copies the commands if a write may run, sets the cycle word to Write or Read and wakes
   the owner. The owner runs the cycle, with pending stops, `write` if RT copied and `read` in that
   order, then clears the word. So at most one cycle is outstanding.
   While the owner is busy RT neither copies nor triggers and counts the skip, so two copies never
   go into one write. A copy the owner does not write, because a block or a stop came after it, is
   undone in the limits at the next copy, so it adds no step.
2. **Stop requests.** When a controller fails, RT bumps a counter per command interface of its
   failure scope and publishes them together, so a plugin never holds part of a controller's
   claims. Before its next `write` the owner holds them with `perform(∅, stop)`.
3. **Hang check.** RT checks every component in its `read` and its `write`. The window starts at
   the first check that finds the owner busy while the component is in use. It ends at a check
   that finds it free, out of use or in a mode switch, whose step's deadline bounds it, and at a
   trigger, since an owner may finish every cycle between RT's check and its trigger. If it lasts
   longer than the hang timeout (`hardware_hang_timeout_ms`, 100 ms), RT faults the component and
   stops its scope. A cycle that stays unserved and a plugin call that holds the gate fault alike,
   as on sync hardware, where the owner is busy while it holds the gate. A stall of RT itself ends
   at a check that finds the owner free, it counts only if the owner is busy at both ends.

The owner answers through the lifecycle word (state and a generation every transition bumps), the
IO fault word and the call fault word. RT reads them at its next `read` and stops the scope of a
fault once per generation.

### Introspection

Controllers and hardware register their values in the registry under the key that
hardware_interface fixes, so `Introspection` holds one registry per process. Its node
`ros2_control`, created in the namespace of the first manager, publishes it on
`ros2_control/introspection_data`. Neither is ever destroyed, so no manager's end leaves the others
without the publisher. The process therefore counts as leaking, and `exit_process` skips the static
destructors that would race the publish thread. Each manager has a statistics registry of its own
instead, keyed by its resolved `~/statistics`. It outlives its manager until a later manager of the
same name replaces it.

RT samples the registry in `write`, after the sync writes and before the triggers, with
`publishAsync`, which only try locks. It samples only if it enters the process gate of
`Introspection` and the `IoGate` of every component, finds every async owner free as before a copy,
and enters the introspection gate, which every controller owner holds for its lifecycle calls and
for creating and destroying its plugin. The `IoGate` counts its owners, so all controller owners
share one. Otherwise the cycle publishes no sample. The plugins enable and disable their values
inside these calls, so a sample never sees a value while its writer runs. Hardware plugins are
created before the first plan and destroyed after the last, when RT samples nothing.

A sample reads the values of every manager of the process, and the RT loop of one manager cannot
hold off the RT loop or the owners of another without waiting. The process gate is therefore held
as owner while more than one manager lives, and several managers in one process publish no sample.
Sampling every 50 ms off RT would publish their values, but read them while their writers run. A
manager joins before it creates a plugin, waiting for a sample in flight, and leaves after its
plugins are gone. The process gate has a single RT bit, so RT tries it only while no owner holds it,
which leaves the sole manager's RT and, before its first plan, its flush as the only parties. A
plugin leaked by a manager that is gone may still write an enabled value while the remaining
manager samples, which this rule does not cover. The node takes `use_sim_time` and the context of
the first manager. pal_statistics ends the publish thread once the default context is shut down, so
an embedding process that returns from `main` must shut it down first, as static destructors would
race the thread otherwise, and a manager created after that publishes no introspection.

Registrations raise the capacity of the registry's queue of enables and disables to the most
values registered at once, and the registry fills it with its own values when it is created, so
until a sample drains it every enable is lost. The supervisor therefore flushes it before the bring
up, while no owner runs a call, under the process gate, so not while another manager lives. More of
them between two samples than that capacity lose the excess, which leaves a value shown or hidden,
but never read while it is written.

## Invariants

- **Commands only from the current activation.** Hardware applies a command interface only after
  its claimer updated once in the current activation, and holds it before the claimer leaves the
  running set. A switch holds (`perform(∅, stop)`), detaches, deactivates, activates, attaches,
  waits for one update and starts (`perform(start, ∅)`). When a successor takes the joint over,
  the component is taken instead of held, so it gets no writes until one combined perform after
  that update (section "Atomic stop and start of command interfaces").
- **Released handles hold NaN.** After `on_deactivate`, also a refused one, and after a failed
  activation, the controller's owner writes NaN into every double command handle it released. Those
  are shadows that only controllers write, so the NaN stays until the next claimer writes, and RT
  copies it to the plugin. A next claimer that writes nothing in its first updates then leaves NaN,
  never the old command. Until the NaN is in, the shadow still holds the last command, so RT copies
  NaN for every double command interface that the plan of its last update does not claim.
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
  fault and its absorption answers ok without restoring the startup set. The supervisor selects the
  startup set (`select_startup`) from its model once it reconciled the stops RT made meanwhile, so
  a supervisor busy past the cooldown does not leave out a controller that a short engage stopped.
- **Quarantine.** A call given up at its deadline still runs on its owner. Its entity is quarantined
  until every given up call returned. Nothing claims what a quarantined controller holds, and the
  providers it writes do not start. While a component is taken for a hung claimer (active and
  quarantined), nothing new claims any of its interfaces, a rollback included, until the claimer
  returned. `core::hung_claimer` is that one rule for the planner and the reconciler. A switch that
  does not touch a quarantined entity succeeds once its desired state is reached apart from it. An
  activation given up at its deadline has failed, so a late SUCCESS of `on_activate` is followed at
  once by `on_deactivate` and the NaN of its handles on the same owner, before the return is
  reported. A late activation therefore never counts as active, also not over a provider the
  rollback stopped.

## Protected controllers

A controller loaded with the parameter `<controller>.protected: true` is never stopped implicitly. A
switch whose plan stops it without naming it in its deactivate list, by AUTO or FORCE_AUTO conflict
resolution or by a restart of a chain, is rejected with a reason that names it, and the e-stop does
not stop it either. [Faults and safety](concepts/faults.md#e-stop) lists what does. So a torque off
controller that keeps an arm limp gives the torque back only through a deliberate deactivation. The
explorer checks every plan that stops a controller implicitly once more with that controller
protected and requires a rejection.

## E-stop

The parameter `estop_topics` names std_msgs/Bool topics, true is engaged, QoS reliable and
transient local. The controller manager makes no difference between a hard and a soft e-stop, the
hardware does. A hard e-stop cuts power, the hardware faults, and the failure closure of that fault
also stops protected controllers on it. A soft e-stop leaves the hardware active. Each topic is
expected to have one aggregating publisher, such as an e-stop node that combines its sources. With
several publishers the last message wins.

A topic counts as engaged while its last message says true, until its first message, and once it
had no publisher for longer than `estop_publisher_grace_ms` (1 s) since its last message or the last
time a publisher was counted. In each case it stays engaged until a message says false. With
`estop_deadline_ms` set, a released topic that receives no message for longer than the deadline
engages as well, also while its publisher is still counted. That suits a source that republishes
its state periodically, such as an aggregating e-stop node. A topic engages by whichever
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
  the node and no executor size delays an engage. That thread runs at normal priority, so no busy
  SCHED_FIFO thread may share a core with it, a FIFO hog delayed an engage by up to about 900 ms in
  a measurement. An engage sets an engaged and a pending bit, which
  RT reads at every update. A release clears only the engaged bit. The update that reads the pending
  bit clears it, so an engage shorter than a cycle stops as well. An engage while no plan is
  published sets only the engaged bit, as no controller ran that it could miss, so a controller
  started after the release at startup keeps running. While either bit is set, RT stops every
  running controller of the adopted plan that claims command interfaces and is not protected. It
  stops them through the failure path of an update error. Their failure scope is marked failed in
  that activation, and their hardware interfaces are held by their owners. So the stop happens in
  the next cycle, also while the supervisor works on a request. The supervisor absorbs it like a
  fault. Desired shrinks, the controllers are deactivated and their handles get NaN. The failure
  scope follows the chain rules. A protected controller chained to a stopped one stops with it, so
  protection does not hold there. A switch that activates such a chain logs a warning that names it.
  Broadcasters without command interfaces keep running. Since RT stops, a soft e-stop needs a
  cycling loop, and RT stuck in a plugin call stops nothing. Once a cycle of the loop of
  ros2_control_node stays open longer than the hang timeout the "Controller Manager Activity"
  diagnostic reports ERROR. An embedded loop is not reported, gz writes in the step after its read
  and update, so its cycles stay open across a whole gz step.
- **Latch.** While engaged, a switch that would start a controller that claims command interfaces
  and is not protected is rejected with the reason. Starting a protected controller is allowed, for
  example a torque off controller during a soft e-stop. A switch accepted just before an engage
  loses its starts to the RT stop and rolls back like a start that fails in its first update.
- **Cause.** While engaged, the diagnostics status of the controller manager is WARN, and its
  message, a refused switch and a startup skip name the topic and why it engaged, which is that it
  says engaged, has no publisher any more or sent no message within `estop_deadline_ms`. The
  cause is the topic that engaged the e-stop, or the first topic at startup. When that topic
  releases while another keeps the e-stop engaged, the cause becomes an engaged one.

## Autostart on release

When every e-stop topic is released, `Startup` applies the startup configuration again, on a
thread of its own like a request.

1. It waits `autostart_cooldown_ms` (2 s) so that hardware that lost power can boot.
2. Every component that the startup configuration brings to active goes active through the path
   of a hardware state request, a no op for one that is up, so a fault the status does not show yet
   is recovered too. It tries up to `autostart_attempts` (3) times with the cooldown in between. A
   request that gave way to another one ("try again") is asked again within
   `hardware_state_timeout_ms` and uses up no attempt.
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
  on an unsettled model. An expired deadline quarantines a component that did not answer only in
  the validation edge under "Rare availability edges".
- **A failed step** shrinks desired by its failure closure, so the reconciler never retries it. The
  supervisor then sets desired to the state before the request minus what cannot run any more or a
  fault or the e-stop stopped meanwhile, and reconciles again, a rollback through the same steps. It
  ends at the request's deadline plus the grace, also when a mode switch ran past the deadline.
  Past the deadline it starts no step that switches modes. Idle work finishes the holds and starts
  of what still runs, and a controller not yet restored stays stopped and the answer names it. It
  restores nothing on a component taken for a hung claimer, and the answer reports the rollback
  incomplete. Only a new request claims the component again once the claimer returned. A controller
  that refused a chained mode keeps working in its old mode.
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
- **Hang rule.** A component with running claimers that does not finish a read or write within the
  hang timeout (`hardware_hang_timeout_ms`) faults. The `prepare` and `perform` of a mode switch,
  also of the owner's urgent hold, do not count. Every step that switches modes (Hold, Take,
  SwitchModes) gets `mode_switch_timeout_ms` (1 s) from its start, whoever started it, so a healthy
  slow mode switch of a bus always finishes. No request deadline, grace or waiting request
  cuts it short. Only a switch past that time is given up as in "Failures and timeouts". A request
  may start a switch with any time left. Once its deadline has passed, the request takes no further
  step and is answered as timed out, its leftover is reconciled as idle work. Its rollback starts
  no mode switch past the deadline either. So a request answers within its deadline plus the longer
  of `mode_switch_timeout_ms`, for a mode switch started just before it, and the grace. A call on
  the owner of a sync component waits for RT to leave a read or write for ten hang timeouts (1 s by
  default), then the component faults.
- **Calls in idle work.** Idle work (fault handling, what a request left over) gives each step the
  hang timeout, so a callback there that hangs or is merely slower is quarantined early until it
  returns, and a waiting request starts. A hardware lifecycle call in idle work (on_error,
  deactivate, cleanup, shutdown of a faulted component) gets `hardware_fault_timeout_ms` (5 s)
  instead, because a bus driver spends bus timeouts there on a dead bus. Its wait still ends within
  the hang timeout once a request is waiting, so the call is quarantined and the request starts.
  Holds, takes and mode switches get `mode_switch_timeout_ms` as everywhere. While a hold runs on a
  component, a protected claimer on another joint of it gets no reads or writes for up to
  `mode_switch_timeout_ms`, as a bus servo mode change blocks the whole bus. Controller calls,
  detaches and the wait for an update keep the hang timeout, since they hold a joint or keep RT from
  a plan. A hardware state request for a component whose call still runs waits within its deadline
  until the call returned, but gives way to a request queued meanwhile and is answered "try again",
  so a hung call never blocks a waiting request.
- **Rare availability edges.** Two rare cases cost availability, neither is unsafe. A request with a
  short deadline whose validation waits behind an urgent hold can reach its deadline in the
  validation, which quarantines the component. A NaN write step behind an urgent hold has only the
  hang timeout (100 ms) in idle work, so it can fault and quarantine the component the same way.
- **Deadlines.** At a request's deadline desired becomes "what is active now and was active before",
  which only stops. The hardware start of activated controllers then follows without the request.
- **Holding everything.** A broken invariant or an exception that escapes a request, fault
  handling or idle work holds everything. Desired keeps no controller, RT stops every unprotected
  command claimer in every later update as an engage does, also in a plan published after the hold,
  and every request but the shutdown is rejected until a restart. The log names holding everything
  as the cause of these stops, an e-stop engaged meanwhile included.
  Work that threw is retried only at the next wake of the supervisor, so a throw that repeats does
  not spin. Controllers detached under a stuck RT call are then released only at that wake too, so
  the writes of their hardware stay blocked until it. That is safe, since everything is held.
  Fault handling takes every inbox event before it applies them, so a throw drops the
  events after it. That is safe, since their entities stay quarantined until a restart.
- **Shutdown.** Desired becomes empty and every component finalized. It goes ahead of the waiting
  requests and starts after the request in flight, so it is never dropped and the hardware gets its
  `on_deactivate` and `on_shutdown`. A component whose quarantined call still hangs when the process
  exits is never shut down. From its start it returns within its timeout plus
  `mode_switch_timeout_ms` plus the grace, a hold running at the deadline finishes and the hardware
  keeps its share of the grace after it. A shutdown that waited behind a slow call, such as fault
  handling under a long hang timeout, still gets 200 ms of the grace to hold and stop the
  controllers, the hardware gets the rest. After shutdown and after a broken invariant, switches and
  hardware state requests are rejected. A controller hung in `on_deactivate` keeps its providers
  active, every other component is deactivated and shut down, its interfaces held. Once a plugin was
  leaked, `ros2_control_node` ends with `std::quick_exit`, so no static destructor races the leaked
  plugin.
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
  SIGTERM changes nothing. [Migrating](migration.md#differences) says how long ros2 launch waits
  before its SIGKILL.

## Atomic stop and start of command interfaces

On bus servos a control mode change typically drops torque briefly and costs bus time. A switch
therefore calls `perform_command_mode_switch(start, stop)` at most once per component, and not at
all when a restart changes neither interfaces nor modes. Below, A is the claimer of a live interface
that stops in the switch and B its successor, which may be A again or claim another mode.

**NaN write.** Once the last claimer of a taken component that stops in the switch returned from
`on_deactivate` and its NaN is in, the component's owner calls `write()` once, bypassing the block.
Every live interface of the component then holds NaN, and the hardware holds a joint whose command
is NaN, so a velocity joint stops instead of keeping the last speed of the ended activation. Writes
stay blocked afterwards until SwitchModes, because B may write its handles in `on_activate` and a
command applies only after its claimer updated (section "Invariants"). Between Detach and the end of
`on_deactivate` the component gets no writes, because the controller may write its handles there. A
NaN write that does not return by the step's deadline faults and quarantines the component like a
Take that timed out, so no claimer starts on it while that write may still read handles. Reads go on
during the handover.

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

The component is taken or held before Detach, and from the first update with the detach plan RT
copies NaN for A's interfaces, so RT applies no command of A while the NaN is pending. The
NaN goes in after `on_deactivate`, because a controller may write its handles there. RT is the
wrong writer for it, because RT need not adopt the detach plan at all (a loop between cycles is
quiescent at once, and the next plan it sees is the attach plan), and RT may only try_lock a
handle. A's owner writes with a blocking lock, which cannot fail for a double handle, after RT is
quiescent and before B is assigned, so no thread writes the handle concurrently. The plugin's own
reset on activation stays the primary mechanism.

### Invariants

- A live interface of a component that is not taken, quarantined or faulted has a claimer that is
  running and updated in its current activation (`check_invariants`). A faulted component writes
  no command until its next lifecycle transition, at most the NaN write of a timed out switch. Its
  claimers may stop while it is quarantined, and their hold follows only once its call returned.
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
- **perform outlives mode_switch_timeout_ms.** The component faults and is quarantined, its writes
  stay blocked until its next lifecycle transition, so a late perform never applies anything. That
  deadline is `mode_switch_timeout_ms` from the start of the step, not the request's.
- **Take outlives mode_switch_timeout_ms** (an async owner stuck in a cycle). The component faults
  and stays blocked, like after a mode switch that timed out.
- **A callback hangs.** A hung `on_activate` quarantines its controller, which is not an updated
  claimer, so SwitchModes stops its interfaces and gives the component back. A hung `on_deactivate`
  holds its live interfaces through `perform(∅, stop)` and keeps the component taken until it
  returned and wrote its NaN in the same job, because the plugin may still write its handles. One
  that never returns keeps it taken for good, without writes, until shutdown.
- **RT stuck in a call during Detach.** The detached controllers stay quarantined until no RT write
  can use the claims of a plan that ran them, that is until RT updated with a newer plan or is
  outside a call with no write left after an older update. Writes stay blocked until A's Deactivate
  and its NaN, which follow only then, so a command A computed in that late call is never applied,
  also not by the old plan's write after the call.
- **The deadline passes before SwitchModes.** Only RT progress is missing, so the request is
  answered and the idle reconcile hands the component back when RT cycles.
- **A fault during the handover.** RT stops the failure scope and asks the owner for an urgent hold.
  SwitchModes skips the start of an interface whose claimer failed, and the urgent hold lands before
  the next write because writes skip while a stop request is pending.
- **perform hangs inside the urgent hold.** The owner keeps `switching_` set, so the hang rule never
  faults the component. Only the deadline of the Hold step the supervisor queued behind the urgent
  hold bounds it, that step then faults and quarantines the component.
- **Async hardware.** A write begins when RT copies its commands. RT checks the block before it
  copies and the owner again before every write. A trigger that lands after the owner last polled
  and before the block is served after the job, so every owner call under the gate drops the write
  of such a pending copy and the cycle only reads, and the next copy undoes its step in the limits.
  The no op job of Take makes a write that began before the block finish before Detach, so it can
  only carry A's commands from while A still ran.
- **A rollback** runs through the same steps. A restarted A on the same interfaces keeps them live
  and costs no perform. A known gap is a handover to another mode whose successor fails, which stops
  the joint in SwitchModes and the rollback starts A's mode again, two performs.

## Tests

- `test_core` and `test_explorer` test the pure planner and reconciler. The explorer runs every
  controller topology up to three controllers on two components with every single failure against
  a brute force oracle, about 2 minutes in a release build. Under sanitizers it explores a seeded
  eighth of the topologies of three controllers, and its random test of four controllers runs 100
  topologies instead of 300.
- `test_runtime`, `test_supervisor*`, `test_fault_handling` and `test_handover` test the threads,
  faults, hangs and handovers. `test_supervisor` and `test_command_limiter` fail if the control loop
  allocates.
- `test_controller_manager`, `test_services`, `test_autostart` and `test_signals` test the node,
  its services and its shutdown on SIGINT and SIGTERM.
- `test_suite_cm` runs the common test suite `cm_test_suite`, and `test_suite_cm_all_async` runs it
  with every component async. `replay_<script>` replays each recorded failure in `hector_controller_manager/test/scripts/`.

Debug, ASan and TSan builds are checked builds (`runtime::CHECKED_BUILD`). They assert the thread
role of every entry point and the preconditions stated in the code, and abort on a broken invariant
instead of holding everything. RelWithDebInfo is not a checked build.
