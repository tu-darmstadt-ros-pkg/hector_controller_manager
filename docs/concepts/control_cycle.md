# The control cycle

Each cycle the control loop (RT) runs three phases.

1. `read` of every sync component, then the hang check of every component.
2. `update` of every active controller, a writer before the provider it writes. A controller whose
   update fails or throws is stopped in this cycle together with what is chained to it.
3. `write` of every sync component with the commands clamped to the joint limits, then the hang
   check again and the trigger of every async component.

While a component's owner thread is inside a plugin call, RT skips that component for the cycle and
counts it in the `skipped_cycles` diagnostic.

## Timing

`ros2_control_node` paces the loop at `update_rate` on the steady clock, or on `/clock` with
`use_sim_time`. A late cycle does not catch up in a burst. The missed cycles are skipped and logged,
so the hardware never gets several writes at once. Deadlines, the hang check and the e-stop stay on
the steady clock and keep working while a simulation is paused.

On a realtime kernel the loop runs SCHED_FIFO at `thread_priority`. Without one, or without the
privilege, hcm keeps running and the "Realtime Scheduling" diagnostic reports ERROR. On a realtime
kernel `ros2_control_node` also locks the memory of the process (`lock_memory`), so no page fault
delays a cycle, and it pins the loop to `cpu_affinity` if set. A process that embeds the
`ControllerManager` class, such as a simulator plugin, runs its own loop and calls `read`, `update`
and `write` itself.
