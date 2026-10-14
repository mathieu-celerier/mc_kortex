# Control loop timing

Overview and log entries: [README](../README.md#control-loop-and-latency-logs).
This page shows how one tick behaves, and how `feedback_wait_us`,
`refresh_timeout_ms`, `max_missed_refresh` and `low_level_type` change it.
The durations in the diagrams are illustrative, not measurements.

## How to read the diagrams

Each diagram is a few milliseconds of the loop, one row per signal:

| Row | Shows |
|---|---|
| 1 kHz schedule | A slot every 1000 µs. ▼ marks when a tick actually started (orange with its delay when late), ✕ a slot that was skipped |
| PC → robot | The command frame `CMD n` sent at the start of tick `n` |
| robot → PC | Its feedback `FB n`, linked to the command by a dashed line. Green: used by the tick that sent it. Orange: used by a later tick. Red ✕: given up |
| Control thread | Send (blue), spin-wait for the feedback (hatched), `controller.run()` (purple), then a spin until the next slot (grey) |
| Controller runs on | The feedback the controller used during that tick, and its age in ticks (`kortex_feedback_age`) |

The dashed vertical line in each tick is the `feedback_wait_us` deadline.
It turns red when this tick's feedback has not arrived by then. Frame boxes
have a minimum width so their labels fit: their left edge, the send or
arrival time, is to scale.

## The tick

One thread runs every tick in this order:

1. Send the command built at the previous tick.
2. Spin until this tick's feedback is in, or until `feedback_wait_us` after
   the start of the tick, whichever comes first. With several arms, the
   shortest `feedback_wait_us` is used, and it is capped at the period.
3. Collect every reply that has arrived, this tick's or a late one, and keep
   the most recent.
4. Update the sensors and run the controller.
5. Build the command sent at the next tick.

Everything from the send to the built command has to fit in the period:

```
send + min(round trip, feedback_wait_us) + sensors + controller + build ≤ 1000 µs
```

## Reply before the deadline

![Reply before the deadline](control-loop/tick-reply-in-time.svg)

The wait ends as soon as the reply arrives, so the controller runs on this
tick's feedback (age 0). The rest of the period is spent spinning until the
next slot.

## Reply after the deadline

![Reply after the deadline](control-loop/tick-reply-late.svg)

The wait stops at the deadline. The controller runs on the newest feedback
it already has, the previous tick's (age 1). The late reply stays queued and
is collected at the next tick. Commands still leave every 1 ms: the send
rate never depends on the reply.

## Bypass

![Bypass](control-loop/tick-bypass.svg)

With `low_level_type: bypass` a tick sends one frame per actuator and one to
the interconnect, joint 7 first because it is the farthest along the chain.
The replies come back staggered, joint 1 first. The wait ends when all of
them are in, or at the deadline. Each joint's feedback is updated on its
own, so a late joint only makes its own feedback older. The round trip is
longer than through the base, so `feedback_wait_us` must leave room for the
slowest joint (see [Low level bypass](../README.md#low-level-bypass)).

## Tick over budget

![Tick over budget](control-loop/tick-overrun.svg)

When the wait plus the controller take longer than the period, the next tick
starts late (`perf_Tick_startDelay`). The delay builds up from tick to tick.
Once a tick would start more than half a period late, the loop skips that
slot (`perf_LoopOverruns`) rather than sending commands in a burst to catch
up. Commands then leave 1 and 2 ms apart (`kortex_LoopPerf`), the actuators
get uneven position steps, and the controller runs slower than real time.

## No wait

![No wait](control-loop/tick-no-wait.svg)

The same tick with `feedback_wait_us: 0`. The thread no longer waits: the
controller always runs on the previous tick's feedback (age 1), which arrived
while the last tick was computing. The tick now fits in the period, as long
as the controller alone does. This trades one tick of feedback delay for a
regular 1 kHz command rate.

## Replies time out

![Replies time out](control-loop/tick-timeout.svg)

A request still without a reply `refresh_timeout_ms` after it was sent is
given up and logged ("no reply after ...ms"). A reply that arrives before
then is still used, however many ticks late. While no new feedback comes in,
the controller keeps running on the last one and the actuators hold the last
command. After `max_missed_refresh` ticks in a row with no new feedback, the
controller is stopped. The diagram uses `refresh_timeout_ms: 1` to show this
in a few ticks.
