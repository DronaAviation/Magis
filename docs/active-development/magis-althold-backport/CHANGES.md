# magis-althold-backport - Changes

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [DESIGN](DESIGN.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

Each code change, with the MagisV2 commit it ports and why. Links use paths relative to this folder;
line numbers are those after the task that made the change.

## Task 2: prerequisites from MagisV2 `4657cbf` ( 2026-11-20 )

| File | Change | Ported from | Why |
|---|---|---|---|
| [mw.cpp](../../../src/main/mw.cpp) after `isBlackboxOn` | `static bool throttleRaisedSinceArm = false;` | `4657cbf` mw.cpp | Distinguishes airborne from on-the-ground while armed. Its only reader is the `a7f398d` I-term freeze in `processRx` ( task 5 ). |
| [mw.cpp](../../../src/main/mw.cpp) `annexCode`, after the `isLanding` throttle override, before the throttle-curve lookup | set when `ARMING_FLAG ( ARMED ) && rcData [ THROTTLE ] > masterConfig.rxConfig.mincheck` | `4657cbf` mw.cpp | Same condition and order as MagisV2. `rcData` is the pilot input in Magis ( no override path ). |
| [mw.cpp](../../../src/main/mw.cpp) `mwDisarm` | `throttleRaisedSinceArm = false;` | `4657cbf` mw.cpp | Cleared on every disarm through `mwDisarm`. |
| [config.cpp](../../../src/main/config/config.cpp) | `EEPROM_CONF_VERSION` 106 -> 107 | `4657cbf` config.cpp | Saved profiles keep the old ALT gain; the bump resets settings to defaults on the first boot ( accelerometer and compass need recalibrating ). |
| [config.cpp](../../../src/main/config/config.cpp) `resetPidProfile` | `P8 [ PIDALT ]` 100 -> 128 | `4657cbf` config.cpp | Unity gain: `altitudehold.cpp` computes `P8 [ PIDALT ] * error / 128`, so a 10 cm error asks for 10 cm/s. Needed by the shaped loop ( task 3 ), where velocity demand = profile rate + P_ALT x error. |
| [Control.cpp](../../../src/main/API/Control.cpp) `PIDProfile_P::setDefault` | `P8 [ PIDALT ]` 100 -> 128 | review finding, task 2 | The public `PIDProfile.setDefault ( )` kept its own hard-coded 100. After the EEPROM reset gives 128, a user-code call to it would silently drop the gain back to 0.78 of the intended value and weaken the shaped loop. MagisV2 hit the same split defaults and later gave them one source ( `ebf4e46` ); here both sites carry a cross-reference comment instead. |
| [altitudehold.cpp](../../../src/main/flight/altitudehold.cpp) includes | `#include "drivers/system.h"` | `c569c0f` uses `millis ( )` | For the flip-exit timers of task 4. |

**Deviation from the plan:** the flag is not exported through `mw.h`. In the three commits ported
here nothing outside `mw.cpp` reads it; MagisV2's accessor `throttleRaisedInFlight ( )` came later
( `73022c3`, `setDesiredAngle` roll / pitch ), which this port does not include.

**Not ported from `4657cbf`:** the `mwArm` guard on `baroResetGroundLevel ( )` ( Magis has no arm-time
re-zero ), the barometer throttle / temperature compensation and ICP-10111 changes ( Magis flies an
MS5611 ), the estimator tilt-threshold units fix and `ALT_CEILING_MARGIN_CM` ( not in scope; see the
decisions log ). The deadband removal and `offloadHoverTrim` are part of task 3.

Reviewed by `pluto-reviewer`: one SHOULD-FIX ( the `Control.cpp` default above ), fixed; two NOTEs,
both resolved in place ( the unity-gain comment now says it holds once the deadband is gone; a landing
started on the ground setting the latch is harmless, since every `land ( )` exit calls `mwDisarm ( )` ).

Build: PRIMUSX clean build exit 0, no new `src/` warnings. Size unchanged ( text 76 872, data 1 140,
bss 8 116 ): with no reader yet, LTO removes the flag; the image differs from 2.0.1 by the two
constants.

## Task 3: setpoint shaping, MagisV2 `2a8d59a` + the `4657cbf` altitude-hold part ( 2026-11-20 )

Hand-ported with the `cpp-pro` agent against the spec
`git -C <MagisV2> diff 4657cbf^ 2a8d59a -- src/main/flight/altitudehold.cpp src/main/flight/altitudehold.h`
( nothing else touched those files between the two commits ). Only
[altitudehold.cpp](../../../src/main/flight/altitudehold.cpp) and
[altitudehold.h](../../../src/main/flight/altitudehold.h) changed.

### From `4657cbf`

| What | Magis file:line | Note |
|---|---|---|
| `ALT_CEILING_MARGIN_CM 25` replaces the literal 50 in `limitAltitude ( )` | cpp:58, :1176 | Decided in the task 3 mini-grill. |
| `offloadHoverTrim ( )` + call from the centred-stick branch | cpp:248, call :332 | Moves the hover trim out of `errorVelocityI` into `initialThrottleHold` one count per 20 ms while settled, so hover throttle is no longer capped at 1800 as the battery sags. |
| `errorVelocityI` kept across setpoint changes | cpp:303 ( comment ) | **Magis never had the two resets MagisV2 removed**: its four resets are all entry / disarm resets ( BARO cpp:409, SONAR :430, the I-term `else` :563, `AltRst ( )` :1120 ) and all stay. |
| `applyDeadband ( error, 5 )` removed from the position error | cpp:503 | A 5 cm band subtracted 5 cm from every error ( a 10 cm sag read as 5 ) and caused a limit cycle. |
| Estimator tilt threshold `> 30` -> `> 300` deci-degrees | cpp:1028 | Decided in the task 3 mini-grill: 30 meant 3 deg, so the slow ~15 s filter ran in all normal flight and the estimate lagged high in descents. Magis's `max_angle_inclination` is 200 ( 20 deg ), so this now trips only in aggressive flight. |
| `extern int32_t errorVelocityI;` | h:77 | With MagisV2's comment. |

### From `2a8d59a`

| What | Magis file:line | Note |
|---|---|---|
| Stick moves the setpoint: 0 at the deadband edge to `ALT_MAX_CLIMB_CMS` 40 / `ALT_MAX_DESCENT_CMS` 30 cm/s at full stick, profile rate ramped at `ALT_STICK_ACCEL_CMSS` 100 cm/s^2, target leashed `ALT_TARGET_LEASH_CM` 50 cm from `EstAlt`; centring coasts to a stop | request cpp:325-331, shaping :456-517 | The old raw `( thr - 1500 ) / 4` clamped -100 / +120 and the `AltHold = EstAlt` snap on centring are gone. |
| External `AltHold` writes become a goal, flown at `ALT_CMD_MAX_CLIMB_CMS` 60 / `ALT_CMD_MAX_DESCENT_CMS` 30 cm/s, braking at `ALT_GOAL_DECEL_CMSS` 80 cm/s^2 to stop on it; a stick move cancels it | cpp:462-497 | Detected as `AltHold != lrintf ( altTarget )`, as in MagisV2, so no writer needed an edit: `setAltitude` / `setRelativeAltitude` ( `API/Control.cpp:209/232` ), MSP ( `serial_msp.cpp:1728/1733` ) and take-off ( `command.cpp:135` ) all become goals. |
| Velocity demand = profile rate + `P8 [ PIDALT ] * error / 128`, clamped per source, slewed at `ALT_VEL_ACCEL_CMSS` 150 cm/s^2 | cpp:509-516 | Replaces the +/-300 cm/s clamp. Needs task 2's P_ALT 128. |
| Landing keeps its own descent, `( landThrottle - 1500 ) / 4` with `ALT_LAND_MAX_DESCENT_CMS` 100 cm/s | cpp:322-325, :511 | The default `landThrottle` is 1200, so -75 cm/s; `Command.land ( landSpeed )` ( `API/User.cpp:324`, `1305 - landSpeed` ) gives -48 to -112 cm/s, clamped here to -100. Every value is faster than the 30 cm/s stick cap, so the landing invariant holds. MagisV2's 1300 -> 1150 ramp is deliberately not ported. |
| Controller held in reset while disarmed or stick-armed at idle | cpp:534-538; `altHoldGroundIdle` set / cleared :359, :362, :398 | Stops the velocity integrator winding down while waiting on the ground. |
| `velocityControl` removed; `calculateAltHoldThrottleAdjustment` takes `float ctrlDt` ( clamped 0 .. 0.05 s ) | cpp:520, :540; callers :691, :871 | `posControl.cpp`'s `velocityControlX / Y` are unrelated and untouched. |

New file-static state: `altRate`, `altVelTarget`, `altTarget`, `altGoal` ( float, cm and cm/s ),
`altGoalActive`, `altHoldGroundIdle` ( bool ), `lastOffload` ( ms ), plus `resetAltSetpoint ( )` and
`shapedVelocitySetpoint ( )`. The shaping lives in its own function, the name `c569c0f` later uses, so
task 4 is a transcription.

**Deviation corrected during the task:** the first draft moved the stick / landing request into a new
`altStickRate` and put the final demand in `setVelocity`, which would have changed what the public
`getSetVelocity ( )` ( `DesiredVelocity.get ( Z )` ) returns, in a port where the API stays 1.0.1, and
would have made the `c569c0f` port a translation rather than a transcription. `setVelocity` now holds
the request, as in MagisV2 `2a8d59a` and as in Magis before the port; the demand lives in the return
value and `altVelTarget`.

**Also changed for consistency:** the SONAR entry uses `resetAltSetpoint ( )` like BARO ( cpp:428 ), so
its bare `AltHold = EstAlt` is not read as a goal. SONAR is off on both Magis targets, so this is not
compiled.

Build: PRIMUSX clean, exit 0, `no new warnings in src/`; text 77 720, data 1 140, bss 8 140
( +848 B flash, +24 B RAM over task 2 ). Flash 77.0 KB of 252 KB, RAM 9.1 KB of 40 KB.

### Review fixes ( `pluto-reviewer`, 0 BLOCKING, 1 SHOULD-FIX, 7 NOTE )

| Finding | Action | Magis file:line |
|---|---|---|
| SHOULD-FIX: in the `alt_hold_fast_change` branch ( off by default ) the centring `AltHold = EstAlt` was read as an external write, so the recentred height was flown back to as a goal from a stale target: the craft sank about a third of the climb, roughly 50 cm after a 1.5 m stick climb | `resetAltSetpoint ( )` there instead, dropping the target and rate with it. A deliberate improvement on MagisV2 `2a8d59a`, which has the same defect in a branch it never flies; it cannot affect the default path | cpp:299-305 |
| NOTE: the landing comment claimed a fixed `landThrottle` 1200 | Corrected in the comment and in the table above: `Command.land ( )` writes `1305 - landSpeed` | cpp:221-226 |
| NOTE: `altholdDebug3` carried `isAltHoldChanged`, always 0 in the shaped branch | Repurposed to `altGoalActive`, which the task 7 bench log needs | cpp:368 |

Accepted as-is, all matching MagisV2 behaviour, recorded for the task 7 / 8 tests and the README:

- `DesiredPosition.get ( Z )` returns the moving target while a goal is flown, not the commanded
  height, so `set ( Z, get ( Z ) + 50 )` from user code adds to an intermediate value.
- A one-shot user altitude write made while the stick-arm ground idle holds the motors is overwritten
  every tick by the reset branch. The app take-off is unaffected ( it arms itself, so
  `isThrottleStickArmed` is false ). MagisV2 later added `altHoldReleaseGroundIdle ( )` for this
  ( `5e26823` ), outside this port.
- The offloaded hover trim stays in `initialThrottleHold` across a disarm, so removing a payload
  between flights without a BARO toggle or reboot starts the next take-off hot until P and I absorb
  it. Same as MagisV2.

Verified clean by the reviewer against the reference, with line references for each: all 12 items; the
take-off goal path ( no one-tick race, no stale ground idle ); the landing descent, which is never
limited by the stick cap and which its simulation showed starts about 0.5 s later than before with
touchdown still detected; the leash on the ground and on a craft that cannot climb; the hover-trim
arithmetic ( baseline + I / 8192 + adjustment conserved, unsigned `millis` wrap safe ); units and
rounding at every boundary; single-precision maths only, one `sqrtf` per 100 Hz tick; the uncompiled
SONAR branch; and that no `velocityControl` reference remains.

## Task 4: flip bypass and return to height, MagisV2 `c569c0f` ( 2026-11-22 )

Hand-ported with the `cpp-pro` agent against `git -C <MagisV2> show c569c0f -- src/main/flight/altitudehold.cpp`
and that commit's `flip-althold-regression/CHANGES.md`. `c569c0f` sits directly on `2a8d59a`, the base
task 3 ported, and `setVelocity` means the same thing in both trees, so this transcribes. Only
[altitudehold.cpp](../../../src/main/flight/altitudehold.cpp) changed; `acrobats.h` already declares
`extern uint32_t flipState`.

| What | Magis file:line | Note |
|---|---|---|
| `#include "flight/acrobats.h"` | cpp:53 | No baseline warnings in that header and none in the build log, so the gate is unaffected. |
| `flipActive ( )`: `flipState != 0` under `#ifdef ENABLE_ACROBAT`, `false` otherwise | cpp:267 | PRIMUSX defines `ENABLE_ACROBAT` ( `target.h:183` ). |
| `flipVelocitySetpoint ( )`: the flip's throttle flown as `( throttle - 1500 ) / 4` straight to the velocity loop, no ramp, slew or leash; `altTarget` / `altRate` / `altGoalActive` / `altVelTarget` synced so the hand-back starts clean | cpp:280-299 | Exact transcription. Inside the deadband it runs the old P loop clamped to +/-300. |
| Stick branch while a flip runs: `constrain ( deflection / 4, -ALT_FLIP_MAX_DESCENT_CMS 100, ALT_FLIP_MAX_CLIMB_CMS 120 )` | cpp:398-401 | The pilot's sticks outside a flip stay shaped at 40 / 30. |
| Controller selection `flipActive ( ) ? flipVelocitySetpoint ( ) : shapedVelocitySetpoint ( ctrlDt )` | cpp:655 | |
| Flip entry edge: snapshot `errorVelocityI` ( 0 if BARO_MODE was off ), the pre-flip `AltHold`, and whether a return is allowed ( `BARO_MODE && !altHoldGroundIdle && AltHold >= ALT_FLIP_RETURN_MIN_CM` 20 ) | cpp:617-627 | Checked ahead of the tilt test, so a flip that ends tilted is still handed back. |
| Flip exit edge: `resetAltSetpoint ( )`, restore `errorVelocityI`, stamp `altFlipExitAtMs`, and write the pre-flip `AltHold` as a return goal | cpp:628-634 | The flip's last 120 cm/s setpoint left to slew down with a wound-up integrator drove a flyaway into the ceiling on MagisV2, so the restore is not optional. |
| 500 ms integrator hold after exit ( `ALT_FLIP_EXIT_I_HOLD_MS` ), clamp still applied | cpp:674-680 | Braking the ~120 cm/s exit climb otherwise banked about -20 counts of trim and dropped the craft 15-46 cm below the target. |
| 100 ms throttle grace after a flip ( `ALT_FLIP_EXIT_IGNORE_MS` ), ordered after `isLanding` and before the flip branch | cpp:382-397 | The flip's last 2000 stays in `rcData` until the next RX frame and would read as full stick, cancelling the return goal. |

Deviations from `c569c0f`, neither of them behavioural: `shapedVelocitySetpoint ( )` already sits above
the controller, so MagisV2's forward declaration is not needed; and MagisV2's `log-N.txt` citations in
the comments became "MagisV2 flip-althold-regression", since those logs are not in this tree.

Build: PRIMUSX clean, exit 0, `no new warnings in src/`; text 77 936, data 1 140, bss 8 180
( +216 B flash, +40 B RAM over task 3 ). Flash 77.2 KB of 252 KB, RAM 9.1 KB of 40 KB.

### Review fixes ( `pluto-reviewer`, 0 BLOCKING, 1 SHOULD-FIX, 3 NOTE )

**SHOULD-FIX, a Magis-specific defect in the ported design.** The guard that stops a flip returning
the craft to the floor was `AltHold >= ALT_FLIP_RETURN_MIN_CM` ( 20 cm ). That works in MagisV2
because its baro datum is re-zeroed on arm, so `AltHold` is height above the arm point. **Magis never
re-zeroes the baro** ( verified: no `baroResetGroundLevel ( )` call anywhere outside `barometer.cpp`,
and none in `mwArm ( )` ), so `EstAlt` and `AltHold` are measured from wherever the craft booted. A
craft armed on any surface reading 20 cm or more against that datum - booted on the floor and flown
from a table, or after session baro drift, which Magis does not temperature-compensate - would store
that surface as the return goal, fly back down to it after the flip and sit there at hover thrust,
with the ground-effect baro bias masking the descent so touchdown is never detected. The second half
of the guard, `!altHoldGroundIdle`, is also a race: `applyMultirotorAltHold ( )` clears that flag
within one 3.5 ms loop of the flip starting, while the entry edge runs from the 100 Hz task.

Fix ( cpp:615, :628-636, :658 ): a new `altGroundAlt` records `EstAlt` in the branch that already runs
whenever the craft is on the ground ( disarmed, or armed on the stick with the motors idle ), and the
test is now `( preFlipTarget - altGroundAlt ) >= ALT_FLIP_RETURN_MIN_CM`. `altHoldGroundIdle` is kept
as a second, earlier guard. If the ground branch has never run since boot the reference stays 0 and
the test degrades to MagisV2's.

**NOTE, taken as well** ( cpp:633 ): while a goal is being flown `AltHold` is the *moving* target, so a
flip sent during a take-off climb or during the return descent from a previous flip stored an
intermediate height; repeated flips ratcheted the altitude up with no ceiling ( `max_altitude` is -1
by default ). The stored target is now `altGoalActive ? lrintf ( altGoal ) : AltHold`. Task 8 flies
five or more flips, so this matters there.

Both are deliberate deviations from `c569c0f`, recorded here and in the decisions log.

**NOTEs accepted as-is:** with `alt_hold_fast_change` on ( off by default ) the return to height
depends on timing and usually degrades to holding the exit height, as master did - the I restore and
the 500 ms hold still apply; and a flip that starts with BARO on and ends after BARO is switched off
leaves a spurious 100 ms throttle grace at the next BARO entry, which is harmless.

**Confirmed by the reviewer against the code:** the bypass is reached ( `updateActivatedModes ( )`
rebuilds the AUX mask before `updateAltHoldState ( )` reads it, and `applyAltHold ( )` is gated on
`FLIGHT_MODE ( BARO_MODE )`, which `acrobats.cpp` never touches, so it runs every 3.5 ms through the
flip ); `getEstVelocity ( )` returns the same `VelocityZ` the bypass drives, and the demand is
identical to master's 120 cm/s, so the flip's 100 cm/s threshold is reachable by construction ( thrust
margin is for task 8 ); the exit hand-over cannot split across ticks or run twice, and is correctly
overridden by a disarm; `millis ( )` arithmetic is wrap-safe; the hover-trim offload cannot run during
a flip or the grace.

Build after the fix: PRIMUSX clean, exit 0, `no new warnings in src/`; text 78 000, data 1 140,
bss 8 188 ( +280 B flash, +48 B RAM over task 3 ).

## Task 5: airborne stick-low I-term freeze, MagisV2 `a7f398d` ( 2026-11-22 )

Ported by hand ( no agent ) against `git -C <MagisV2> show a7f398d -- src/main/flight/pid.cpp src/main/flight/pid.h src/main/mw.cpp`.
Magis `pid.cpp` was byte-equivalent to MagisV2's before the change apart from whitespace, so the pid
hunks are a straight transcription, re-typed to keep CRLF.

| File | Change | Why |
|---|---|---|
| [pid.cpp](../../../src/main/flight/pid.cpp):72 | `static bool gyroITermFrozen = false;` | Main-loop only state. |
| [pid.cpp](../../../src/main/flight/pid.cpp):108 | `pidSetGyroITermFreeze ( bool )` | Setter used by `mw.cpp`. |
| [pid.cpp](../../../src/main/flight/pid.cpp):842 | `pidRewrite`: the `errorGyroI` accumulation is skipped while frozen; the clamp and the ITerm still use the held value | The rate I-term is the only integrator holding an off-centre CG in ANGLE mode. |
| [pid.h](../../../src/main/flight/pid.h):110 | declaration | |
| [mw.cpp](../../../src/main/mw.cpp):804-812 `processRx ( )` | `freezeGyroITerm = THROTTLE_LOW && ARMED && throttleRaisedSinceArm && !isThrottleStickArmed && pidController == PID_CONTROLLER_REWRITE`; the stick-low reset of `errorAngleI` / `errorGyroI` runs only when not frozen | Below mincheck 1100 the reset zeroed the trim on every RX frame and the craft leaned 2-7 deg toward a payload ( 2-3 deg with none ). `throttleStatus` itself is unchanged, so stick-arm, switch disarm and the beeper behave as before. |
| [mw.cpp](../../../src/main/mw.cpp):507 `mwDisarm ( )` | `pidSetGyroITermFreeze ( false );` | The freeze never survives a disarm. |

Still resets in flight, as in MagisV2: THROTTLE mode after an ARM-switch arm ( `isThrottleStickArmed`
stays set ), and any controller other than `pidRewrite` ( Magis's default `pidController` is 1 =
REWRITE, `config.cpp:164` ). Known residual, as in MagisV2: a wind-up that builds on the floor before
the stick goes low is held rather than cleared by a later full stick-down.

This is the first reader of task 2's `throttleRaisedSinceArm`, so the flag is now in the image
( it was removed by LTO while nothing read it ).

Build: PRIMUSX clean, exit 0, `no new warnings in src/`; text 78 128, data 1 140, bss 8 188 ( +128 B
flash, RAM unchanged: the two bools fit existing padding ).

### Review ( `pluto-reviewer`, 0 BLOCKING, 0 SHOULD-FIX, 2 NOTE ): no code change

The port was confirmed a faithful copy of `a7f398d` ( pid hunks byte for byte; the `processRx`
condition, the guard and the `mwDisarm` clear identical apart from comment text ), and these paths
were checked in the Magis tree and found correct:

- The other two `THROTTLE_LOW` uses in `processRx` are the `FEATURE_MOTOR_STOP` auto-disarm / beeper
  block and the argument to `processRcStickPositions ( )`; none resets an I-term, and all correctly
  keep reading the unchanged `throttleStatus`.
- `processRx ( )` runs on every RX frame and at least every 20 ms ( `rx/rx.cpp:393` ), and the raw
  throttle only enters `rcData` there, so the flag and the stick state change together. The flag can
  only be stale in safe directions: a mid-cycle arm by user code or take-off leaves it false, and
  every disarm clears it in `mwDisarm`.
- A fast `Command.land ( landSpeed > 205 )` puts `landThrottle` below mincheck, but
  `calculateThrottleStatus ( )` reads `rcData` after the raw RX value has overwritten it, so it does
  not normally read as `THROTTLE_LOW`. In the windows where it does ( the 600 ms invalid-pulse hold,
  the `skipRxSamples` suspend ) the I-term is held rather than zeroed, which cannot grow it, and every
  landing exit calls `mwDisarm ( )`. Same as MagisV2.
- A stale `isThrottleStickArmed` can only *block* the freeze ( master's reset behaviour ), never engage
  it wrongly.
- `pidRewrite` never reads `errorAngleI`, so skipping `pidResetErrorAngle ( )` while frozen changes
  nothing for it; any other selected controller resets as before.
- `gyroITermFrozen` is touched only from the main loop ( `pid_controller` at `mw.cpp:1225`, `processRx`
  and `mwDisarm` ), so no `volatile` is needed; the new `pid.h` declaration sits inside the existing
  `extern "C"` block, which the C includers need.

**NOTE, recorded as a known limit:** on the frame where an ARM-switch disarm happens with the stick
low while frozen, that one frame skips the stick-low reset; `mwDisarm ( )` then clears the freeze and
the latch, and `errorGyroI` keeps its flight value for at most 20 ms with the motors off. Every arm
path on PRIMUSX resets before arming, so nothing carries into the next flight. Same as MagisV2.

**NOTE, a correction to a premise used earlier in this topic:** Magis *does* have a user RC override
path, so the payload fix does nothing for a flight flown on user throttle - see the decisions log.

## Task 14: barometer ground zero, tracked while disarmed and frozen on arm ( 2026-11-22 )

Ported from MagisV2 `4657cbf` ( the `baroUpdateZero ( )` / `baroAltitudeOffset` part only ). Before this,
Magis set its altitude reference once, at boot: log-3 started at -46 cm on the floor it booted on, log-2
at -5 cm. Now altitude is measured from where the craft armed, so the take-off goal, `limitAltitude ( )`
and the flip-return floor guard ( task 4 ) all mean what they say.

| File | Change | Why |
|---|---|---|
| [barometer.cpp](../../../src/main/sensors/barometer.cpp):33 | `#include "config/runtime_config.h"` | `ARMING_FLAG ( ARMED )` |
| [barometer.cpp](../../../src/main/sensors/barometer.cpp):367-408 | `baroUpdateZero ( )`: an IIR ( alpha 0.05 per sample, about 2 s at 10 Hz ) tracks the raw altitude into `baroAltitudeOffset` while disarmed; returns immediately while `ARMED`; `getBaroZeroOffset ( )` reads it | MagisV2's mechanism: the boot reference goes stale while the board sits on the ground |
| [barometer.cpp](../../../src/main/sensors/barometer.cpp):418-422 | `apmBaroCalculateAltitude ( )`: `rawAltitude` is the existing conversion ( unchanged ); `BaroAlt = rawAltitude - baroAltitudeOffset` | The function PRIMUSX uses ( `baroCalculateAltitude ( )` takes it when `PRIMUSX2` is not defined ) |
| [barometer.h](../../../src/main/sensors/barometer.h):61 | `float getBaroZeroOffset ( void );` | For the log; inside the existing `extern "C"` / `#ifdef BARO` block |

**Not ported, on purpose:** the throttle and temperature compensation ( measured and dropped, task 15 ),
and the ISA `pressureToAltitude ( )` rewrite ( Magis already converts at a fixed `baroGroundTemperature`,
not the live die temperature, which was the point of that rewrite ).

**Departures from MagisV2, each because the offset is state:**

- **A settle hold of 3 s after every disarm** ( `BARO_ZERO_SETTLE_MS` ), from the review's BLOCKING
  finding: see below.
- **A non-finite or absurd reading ( beyond 1 km ) is not fed to the IIR.** A single NaN would otherwise
  latch in the offset for the rest of the power cycle and make every later altitude NaN; before this
  task one bad sample cost one bad altitude, and `BaroAlt` for that sample is still computed as before.
- **At most once per baro sample**, keyed on `getBaroLastUpdate ( )`. This is a guard rather than a fix:
  on PRIMUSX `checkReading ( )` is compiled out ( it needs `LASER_ALT` ) and `checkBaro ( )` already
  gates on the same timestamp, so the function is reached once per sample today. It keeps the time
  constant right if a second caller is ever added.

**The guard against an in-flight re-zero is `ARMED` plus the settle hold.** This corrects how the plan
first described the task: `throttleRaisedSinceArm` guards a different MagisV2 call,
`baroResetGroundLevel ( )` inside `mwArm ( )`, which Magis does not have.

Log: `C` ( temperature, no longer needed with task 15 dropped ) is swapped for `O`, the offset in cm, so a
log shows it tracking while disarmed and frozen while armed. Budget unchanged.

### Review ( `pluto-reviewer`: 1 BLOCKING, 2 SHOULD-FIX, 3 NOTE )

**BLOCKING, fixed: "disarmed" does not mean "on the ground" in Magis.** A flip that times out
( `acrobats.cpp:165`, `:188` ) and the ARM switch turned off in the air ( `rc_controls.cpp:197-199` )
both call `mwDisarm ( )` mid-flight **without setting `Crash` or `PREVENT_ARMING`**, and the craft can
re-arm during the fall that follows: the chuck-arm branch fires on `netAccMagnitude < 11`
( `rc_controls.cpp:183` ) and `mwArm ( )` passes on `netAccMagnitude < 2` ( `mw.cpp:552` ). I verified
all four of these directly. Tracking during that window pulls the datum up toward the height the craft
fell from, at 5 % per baro sample: about 45-80 cm after a 0.5-1 s ARM toggle at 2 m, and the next
flight's ceiling and absolute heights would be measured from there. **MagisV2 gates on `ARMED` alone
and has the same gap.**

Fix ( `barometer.cpp` ): `baroUpdateZero ( )` watches the ARMED state every sample, stamps the falling
edge, and skips the IIR for `BARO_ZERO_SETTLE_MS` 3000 ms afterwards, which is far longer than any fall
a chuck-arm can catch. Across a disarm and re-arm in the air the datum therefore stays exactly where
the original arm froze it. A time hold was chosen over a stillness test to keep `barometer.cpp` free of
IMU state.

**SHOULD-FIX, fixed:** the re-seed on ground recalibration was dead code presented as a feature -
`baroGroundPressure` is written only by the boot `baroCalibrate ( )`, and the gyro-calibration stick
command reaches only `performBaroCalibrationCycle ( )`, which nothing calls on this target. The branch
and its static are gone. **SHOULD-FIX, fixed:** the once-per-sample key was justified by a double call
that does not happen on PRIMUSX; it is kept as a guard with an honest comment.

**NOTE, accepted and recorded:** the datum freezes at the **arm** instant, not at take-off, so baro
drift during a long armed ground-idle wait ( a few cm per minute ) stays in the flight frame. MagisV2
re-zeroes through that window instead. Extending tracking into armed ground idle was considered and
**rejected**: its guard would have to be "armed but throttle never raised", which is exactly the state
an in-air re-arm leaves behind, reopening the BLOCKING path in a worse form. Control is unaffected
either way, because the controller's reset branch pins `AltHold` and `altGroundAlt` to `EstAlt` while
idling.

**NOTE, for task 9:** this is a public API behaviour change. `getEstAltitude ( )`
( `API/Estimate.cpp:141` ), the MSP altitude and the `BaroAlt` telemetry are now relative to the arm
point rather than to boot, and while disarmed they are high-passed with the IIR's ~2 s constant, so a
drone held 1 m up reads about 0 within ~6 s. Magis has no API wiki, so it goes in CHANGES and the
CHANGELOG.

**Verified clean by the reviewer:** the continuity walkthrough found **no step** at any transition
( first sample after boot calibration seeds to exactly 0; disarmed tracking; the arm instant, where the
offset simply stops moving; disarm; re-arm on the ground ), and none of the earlier tasks depends on the
boot datum ( `AltRst ( )`, the hover-trim offload and the landing detection have no datum dependence;
`altGroundAlt` and the flip floor are consistent with the new frame ). Also clean: main-loop-only access
with no ISR sharing, single-precision maths only, constant-initialised statics, and the exact float
compare on `baroGroundPressure` being correct given it is written once.

Build after the fix: PRIMUSX clean, exit 0, `no new warnings in src/`; text 78 984 / data 1 140 /
bss 8 220 ( +128 B flash, +24 B RAM for the port; the log line is `TEMP-BENCH` ).

**Re-checked after the fix** ( same reviewer, 0 BLOCKING ): the settle hold closes the path, including a
second disarm inside the same window ( each disarm restamps ) and a `millis ( )` wrap ( the subtraction
is unsigned ). The boot path is unaffected: there is no edge and no hold at power-on, the first sample
seeds `BaroAlt` to exactly 0, and that happens before `isCalibrated` lets the craft arm. Nothing can
latch the offset permanently; repeated disarms on the ground only delay tracking by seconds, worth
about 1 cm of pressure drift.

Two residual notes, both accepted:

- **A re-arm within ~10 s of a landing inherits that flight's baro error.** At touchdown the floor reads
  some error `e` ( flight drift plus ground effect, typically 10-50 cm ) against the frozen datum.
  Tracking resumes 3 s after the disarm and then removes it with the IIR's ~2 s constant: about 60 % of
  `e` is left at 4 s, 22 % at 6 s, and 5 % by ~9 s. Control is unaffected, since the decay is smooth and
  every controller target is relative ( the reset branch pins `AltHold` / `altTarget` / `altGroundAlt` to
  `EstAlt`, take-off is `EstAlt + 120`, and landing detection uses velocity ), but the absolute consumers
  - the ceiling and absolute user / MSP heights - start offset by the residual. **The flight pattern
  already ends with 10 s at rest, which covers this; the task 8 tests keep that gap before any re-arm.**
- **A sub-sample armed spell can be missed**, because the ARMED state is sampled once per baro sample
  ( ~101.5 ms ). To matter it would need a disarm, a re-arm of under 100 ms in the air, and a second
  disarm about 3 s later, i.e. ~44 m of free fall. Not reachable; stamping from `mwDisarm ( )` instead
  would make it exact, at the cost of coupling `barometer.cpp` to `mw.cpp`.

## Task 9: instrumentation removed, final state ( 2026-11-22 )

**The `TEMP-BENCH` test logging is gone.** `API/PlutoPilot.cpp` carried no permanent change - stripping its
tagged lines left it identical to the committed file apart from two blank lines - so it was restored
from git exactly. In `flight/pid.cpp` and `mw.cpp` the tagged accessors were stripped, and the
remaining diffs match their pre-instrumentation sizes to the line ( `pid.cpp` 15, `mw.cpp` 22 ). A search
finds no `TEMP-BENCH`, `benchFlags`, `benchPitch` or `benchGyroITerm` left anywhere under `src/main`.

**Final build**, PRIMUSX, clean tree: exit 0, `no new warnings in src/`, and the warning totals are still
**exactly** those of the untouched 2.0.1 ( 2 361; `src/` 899, `lib/` 1 462 ). text 78 256 / data 1 140 /
bss 8 212.

| | 2.0.1 baseline | this port | change |
|---|---|---|---|
| flash ( text + data ) | 78 012 B | 79 396 B | **+1 384 B** |
| RAM ( data + bss ) | 9 256 B | 9 352 B | **+96 B** |
| of capacity | 30.2 % of 252 KB / 22.6 % of 40 KB | 30.8 % / 22.8 % | |

### File index: the whole port

| File | Lines | What | Task |
|---|---|---|---|
| [flight/altitudehold.cpp](../../../src/main/flight/altitudehold.cpp) | +374 / -63 | setpoint shaping, goal profile, landing descent, reset at idle, hover-trim offload, deadband removal, tilt-threshold units, ceiling margin, flip bypass and exit hand-over, flip ground guard and goal snapshot | 2, 3, 4 |
| [flight/altitudehold.h](../../../src/main/flight/altitudehold.h) | +3 | `extern errorVelocityI` | 3 |
| [sensors/barometer.cpp](../../../src/main/sensors/barometer.cpp) | +98 / -1 | ground zero tracked while disarmed, frozen on arm, 3 s settle hold, NaN guard | 14 |
| [sensors/barometer.h](../../../src/main/sensors/barometer.h) | +2 | `getBaroZeroOffset ( )` | 14 |
| [flight/pid.cpp](../../../src/main/flight/pid.cpp) | +14 / -1 | `pidSetGyroITermFreeze ( )`, guarded I accumulation | 5 |
| [flight/pid.h](../../../src/main/flight/pid.h) | +1 | its declaration | 5 |
| [mw.cpp](../../../src/main/mw.cpp) | +21 / -1 | `throttleRaisedSinceArm`, the freeze condition in `processRx`, both cleared in `mwDisarm` | 2, 5 |
| [config/config.cpp](../../../src/main/config/config.cpp) | +9 / -2 | P_ALT 128, `EEPROM_CONF_VERSION` 107 | 2 |
| [API/Control.cpp](../../../src/main/API/Control.cpp) | +3 / -1 | `PIDProfile.setDefault ( )` P_ALT 128 | 2 |
| [CHANGELOG.md](../../../CHANGELOG.md) | new | v2.1.0 | 9 |

9 source files, **+525 / -69** ( the task 9 move re-indented a block, and task 10 corrected comments ).
Not touched: `API/PlutoPilot.cpp` ( restored ), the Makefile ( the version bump is task 11 ), `lib/`.

### Compliance check against the MagisV2 pipeline docs

`Altitude_Hold_Estimator.md`, `Core_PID_Controller.md` and `FLIGHT_INVARIANTS.md` were read as the
specification and each requirement they state about the four ported commits was hunted in the Magis
code. **49 requirements: 0 contradictions, 1 absent, 7 intended deviations, 3 undocumented extras.**
The checker also compared `shapedVelocitySetpoint ( )`, `flipVelocitySetpoint ( )`,
`offloadHoverTrim ( )`, the velocity PID, the stick branch and the flip hand-back statement for
statement against `c569c0f`, and found them the same apart from the listed deviations.

**ABSENT, fixed ( `altitudehold.cpp` ):** the ground-idle / stick-arm block sat **inside the
slow-change branch**, where the reference has it after both branches. With `alt_hold_fast_change = 1`
( CLI-settable, off by default ) it therefore never ran, so that mode neither held the controller in
reset on the ground nor cleared `isThrottleStickArmed` - and because the airborne I-term freeze is
gated on `! isThrottleStickArmed`, an ARM-switch arm in fast mode would have left **the payload fix
switched off for the entire flight**. The block now sits after the `if / else`, as in `c569c0f`.

*This does not invalidate the flight validation:* nothing ran between the end of the slow branch and
the end of the function, so on the default path the move is behaviour-identical, and the rebuilt image
is the same size to the byte. Fast mode was never flown.

**Comment corrected:** the flip-return guard's comment still said "Magis never re-zeroes the baro on
arm", which task 14 made untrue. `altGroundAlt` is now normally about 0 and the guard matches
MagisV2's 20 cm test; it is stricter only after a disarm and re-arm in the air.

**Noted, not changed:** `AltRst ( )` sets `initialThrottleHold` to 1600 where MagisV2 uses 1500. It is
pre-existing Magis code outside the ported diff, and runs only while disarmed after a large vertical
velocity, but it can overwrite an offloaded hover baseline before the next take-off if BARO stays on.
Recorded in the CHANGELOG's known issues.

**Verified as holding, among the 49:** the stick never replaces the position loop and there is no snap
on centring; the 0-to-40 / 30 cm/s stick map, the 100 cm/s^2 ramp and the 50 cm leash; the goal
profile with its `sqrt ( 2 a s )` braking, per-source clamps and 150 cm/s^2 slew; the landing keeping
its own descent and never passing through the stick limits; the ground reset; the flip bypassing
altitude hold rather than switching it off, with the BOXBARO toggle rebuilt every RX frame; the whole
flip hand-back including the 500 ms integrator hold and the 100 ms throttle grace; the barometer datum
never re-zeroing in flight; the freeze condition and that `throttleStatus` is untouched; and that every
arming path resets the attitude I-terms.

## Task 10: whole-topic review ( 2026-11-22 )

`pluto-reviewer` over the complete diff, looking for what the per-piece reviews could not see: **no
BLOCKING findings, and no cross-feature interaction where two individually-correct pieces produce a
wrong result.** Six SHOULD-FIX, five NOTE; all fixed or recorded below.

**The correction that matters: a stalled flip does not disarm in the air.** The ASCEND timeout
( `flight/acrobats.cpp:93-99` ) does `ACTIVATE_RC_MODE ( BOXBARO ); flipState = 0;
ENABLE_FLIGHT_MODE ( ANGLE_MODE ); return;` - it abandons the flip and hands the craft back to altitude
hold, level. Only the **PITCHING** and **SLOWDOWNANDEXIT** timeouts call `mwDisarm ( )`
( `:165`, `:188` ), i.e. a flip whose *rotation* stalls, not one whose *climb* never gets going. So the
regression `c569c0f` fixes is "the flip silently does not happen, after 2.2 s of full throttle" - still
serious off the ground, since that is a climb, but not an in-air disarm. Corrected in the CHANGELOG,
`TESTING.md` and `TASKS.md`, which all repeated the wrong version.

**Other SHOULD-FIX, all in the CHANGELOG:**

- The I-term fix was described as working whenever airborne. It does **not** apply in THROTTLE mode after
  an ARM-switch arm, because only altitude hold clears `isThrottleStickArmed` - MagisV2 has the same
  limitation. Now stated.
- `DesiredVelocity.get ( Z )` was listed as unchanged. Its meaning is, but its **range** is now
  +40 / -30 instead of +120 / -100, and it reads 0 with ALT_HOLD off where it used to hold its last
  value. Also added: a `DesiredPosition.set ( Z )` issued while stick-armed at ground idle is now
  **discarded**, where 2.0.1 kept it.
- "MSP altitude writes" and "`BaroAlt` telemetry" were listed as affected. On PRIMUSX neither exists:
  the MSP waypoint writes to the target are under `#ifdef GPS` and PRIMUSX has `//#define GPS`
  ( `target/PRIMUSX/target.h:152` ), and `BLACKBOX` is off too. The MSP altitude *reply* does exist and
  is affected. Corrected.
- `altitudehold.h`: the comment on the new `extern int32_t errorVelocityI` called it "the sole integrator
  carrying the whole hover trim", which stopped being true once `offloadHoverTrim ( )` began moving the
  trim into the baseline. Reworded. The export itself is kept: it is part of what `4657cbf` adds, and
  Magis's header now matches MagisV2's.

**NOTEs recorded rather than changed:** the fast-mode claim is now narrower ( centring no longer flies
back to the old target, and the mode now idles on the ground like the default, but its climb still winds
the integrator down and its landing is open-loop - both pre-existing ); the learned hover baseline
survives a battery change, which is now an operating note; and `AltRst ( )` also fires once on the first
estimator tick after boot, which is now in the known issues.

**The task 9 move was verified in both directions:** on the default path the block runs in the same place
it always did, with nothing in between, so behaviour is unchanged - and in fast mode it now forces idle
over whatever that branch computed, clears `isThrottleStickArmed` above 1500 and sets the ground-idle
flag, so the freeze and the controller reset work there too.

**Cross-feature paths traced clean:** the flip-timeout disarm against the ground-zero settle hold and the
exit hand-over ( they agree: the exit runs first, the disarmed reset wipes it, and the datum is held
throughout ); the return goal, `altGroundAlt` and the arm-relative datum ( one frame ); the hover-trim
offload against the flip's integrator snapshot ( the offload cannot run while flipping, in the exit grace
or during a goal, and the baseline does not move during a flip, so the total trim is preserved ); and the
landing against the shaping, the goal profile and the leash ( at touchdown the leash's -50 cm plus the
-75 feed-forward keeps the descent demand at its -100 clamp ).

Build after the comment and doc edits: PRIMUSX clean, gate clean, text 78 256 / data 1 140 / bss 8 212 -
unchanged, as comment-only edits should be.

### API version: 1.0.1 -> 1.0.2 at commit

No signature changed, but user code can observe: `getEstAltitude ( )` and the MSP altitude reply now read
from the arm point; `DesiredPosition.get ( Z )` returns the moving target during a goal;
`DesiredPosition.set ( Z )` is profiled, and dropped during ground idle; `DesiredVelocity.get ( Z )` has a
new range and zeroes with ALT_HOLD off; `PIDProfile.setDefault ( )` writes 128. `pluto-rules` item 15 asks
for a bump on a behaviour change, and MagisV2 set the precedent in this exact situation: `2a8d59a` moved
1.3.1 -> 1.3.2, "behaviour only, no signature change". Magis has no API wiki for these calls, so the
CHANGELOG's API section is the record.
