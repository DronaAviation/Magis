# magis-althold-backport - Tasks

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [DESIGN](DESIGN.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

**Topic path for `/pluto-task`:** `Magis/docs/active-development/magis-althold-backport/` ( the Magis
repo is nested in the MagisV2 working copy; the skill's default topic root is MagisV2's ).
**Build recipe** ( from the Magis root, Git Bash ):
`PATH="/c/PlutoIDE/tools/ARM GNU ToolChain/bin:/c/PlutoIDE/tools/CYGWIN 64/bin:$PATH" make TARGET=PRIMUSX 2>&1 | tee build.log`
then `python -I ../tools/warnings.py --baseline docs/active-development/magis-althold-backport/warnings-baseline.json check build.log` ( new `src/` warnings fail ) and the `arm-none-eabi-size` line for flash / RAM ( see [TESTING.md](TESTING.md) ). Hex:
`Build/PRIMUSX/PRIMUSX-<FW_Version>.hex`. `make TARGET=PRIMUSX clean` before a full rebuild.

## Resume

Read this block first in a new session; read only the task it points to.

| | |
|---|---|
| **Current task** | **Closed - pending commit** ( 2026-11-22 ): every task done or dropped |
| **Next step** | The user commits with `git commit -F .git/MAGISV2_COMMIT_MSG.txt`; the next session fills the commit hash into this topic's README and the index. |
| **Open questions** | none |
| **Blocked on** | the user's commit |
| **Last updated** | 2026-11-22 |

## Summary

| | |
|---|---|
| **Mode** | feature ( backport; design chosen in [DESIGN.md](DESIGN.md): hand-port ) |
| **Goal** | Magis flies MagisV2's altitude setpoint shaping ( `2a8d59a` ), flip bypass with return to height ( `c569c0f` ) and airborne I-term freeze ( `a7f398d` ), with the `4657cbf` pieces they rely on. |
| **Done when** | On a PRIMUSX with PlutoMonitor logs: a shaped climb / descent starts and stops smoothly and coasts to a stop on centring; an app take-off flies the goal profile; an app flip completes and returns to its pre-flip altitude within about +10 cm; the pitch mean stays within 0.3 deg through a 6.5 s full stick-down with a front payload; landing, stick arm / disarm and RC-loss landing behave as before. Build: no new warnings against the task-1 baseline, flash / RAM fit. |
| **Target** | PRIMUSX ( PRIMUSV3R out of scope ) |
| **Analysis** | Patch apply fails on about 90 % of hunks ( CRLF, +454 / -253 base drift ); wholesale file replacement drags in estimator code Magis lacks. Hand-port, function by function; `pid.cpp` / `pid.h` taken as-is. About +1-1.5 KB flash, +40 B RAM. |
| **Pipeline docs affected** | none ( Magis has none ). Reference: MagisV2 `subsystems/Altitude_Hold_Estimator.md`, `Core_PID_Controller.md`. New `Magis/CHANGELOG.md`. |
| **Order** | 1 → 2 → 3 → 4 → 5 → 6 → 12 → 7 → 13 → 14 → 8 → 16 → 9 → 10 → 11 ( 15 dropped ) |

## Index

| # | Title | Status | Depends on | Skills / agent |
|---|---|---|---|---|
| 1 | Branch and baseline build | done 2026-11-20 | - | pluto-build ( recipe only, manual make ) |
| 2 | Prerequisites: `throttleRaisedSinceArm`, P_ALT 128, EEPROM 107 | done 2026-11-20 | 1 | pluto-rules |
| 3 | Altitude setpoint shaping ( `2a8d59a` + deadband removal + hover-trim offload ) | done 2026-11-20 | 2 | pluto-rules, cpp-pro agent |
| 4 | Flip bypass and return to height ( `c569c0f` ) | done 2026-11-22 | 3 | pluto-rules, cpp-pro agent |
| 5 | Airborne stick-low I-term freeze ( `a7f398d` ) | done 2026-11-22 | 2 | pluto-rules |
| 6 | Build gate | done 2026-11-22 | 3, 4, 5 | pluto-build ( manual make ), warning diff vs task 1 |
| 7 | Bench check, props off | done 2026-11-22 | 6, 12 | pluto-flighttest |
| 8 | Flight validation | done 2026-11-22 | 7, 13, 14 | pluto-flighttest, pluto-log-analyst agent |
| 9 | Docs: CHANGES, CHANGELOG, compliance against the MagisV2 subsystem docs | done 2026-11-22 | 8 | spec-to-code-compliance |
| 10 | Topic review | done 2026-11-22 | 9 | pluto-reviewer agent |
| 11 | Version bump and commit preparation | done 2026-11-22 | 10 | pluto-commit ( adapted: no all-target build, no pipeline promotion ) |
| 12 | Make the Magis hex flashable by PlutoIDE | done 2026-11-22 | 6 | pluto-build ( method ) |
| 13 | Estimator drift: master baseline with baro fields | done 2026-11-22 | 7 | pluto-flighttest, pluto-log-analyst agent |
| 14 | Baro ground zero: track disarmed, freeze on arm | done 2026-11-22 | 13 | pluto-rules, pluto-reviewer agent |
| 15 | Baro throttle + temperature compensation ( fitted ) | dropped (measured: no effect to correct) | 13, 14 | - |
| 16 | Flip validation by eye ( `c569c0f` ) | done 2026-11-22 ( small sample ) | 8 | pluto-flighttest |

Status values: `todo`, `in-progress`, `done YYYY-MM-DD`, `blocked (<why>)`, `dropped (<why>)`.
Serial numbers are never reused or renumbered.

Dropped closing tasks: *Architecture & pipeline docs* ( Magis has no pipeline folder; its content is
task 9 ) and *Graph refresh* ( Magis has no graphify graph; nothing in MagisV2 changes ).

## Tasks

### 1. Branch and baseline build

**Description.** In the Magis repo create `althold-backport` off `master` ( `git checkout -b`; no
commit ). Build the untouched PRIMUSX with the recipe above and record the baseline: the full list of
`warning:` lines ( file, line, message ) saved as `BASELINE_WARNINGS.txt` in this folder, and the
`arm-none-eabi-size` text / data / bss for the 2.0.1 image. Keep the produced
`Build/PRIMUSX/PRIMUSX-2.0.1.hex` as the rollback image ( copy it next to the baseline as
`PRIMUSX-2.0.1-rollback.hex` is acceptable since `Build/` is cleaned ). Note the git line-ending
setting ( `core.autocrlf` ) so every edited file stays CRLF.

- **Depends on:** -
- **Skills / agent:** pluto-build ( for the warning-diff method only; its driver does not run here )
- **Files:** none edited
- **Safety impact:** none
- **Done when:** the build succeeds, `BASELINE_WARNINGS.txt` and the size line are in `TESTING.md`, the rollback hex is kept.
- **Rollback:** n/a
- **Status:** done 2026-11-20
- **Result:** Branch `althold-backport` off `2e55152`; clean PRIMUSX build in 43 s; flash 76.2 KB / 252 KB, RAM 9.0 KB / 40 KB; 899 `src/` + 1 462 `lib/` warnings baselined in `warnings-baseline.json` ( list in `BASELINE_WARNINGS.txt` ); rollback hex kept; tree is CRLF via `core.autocrlf=true`, index LF.

### 2. Prerequisites: `throttleRaisedSinceArm`, P_ALT 128, EEPROM 107

**Description.** Bring in the `4657cbf` pieces the later tasks depend on. In
[mw.cpp](../../../src/main/mw.cpp) add `bool throttleRaisedSinceArm`, set it in `annexCode` when
`ARMED && rcData [ THROTTLE ] > masterConfig.rxConfig.mincheck`, clear it in `mwDisarm` ( `:473` ) and
export it through [mw.h](../../../src/main/mw.h) for `altitudehold.cpp`. In
[config.cpp](../../../src/main/config/config.cpp) raise the ALT P default 100 -> 128 in
`resetPidProfile` and bump `EEPROM_CONF_VERSION` 106 -> 107 ( wipes saved settings on first boot ).
Add `#include "drivers/system.h"` to [altitudehold.cpp](../../../src/main/flight/altitudehold.cpp)
for `millis ( )`. Compare each hunk against MagisV2 `git show 4657cbf -- src/main/mw.cpp src/main/config/config.cpp`.

- **Depends on:** 1
- **Skills / agent:** pluto-rules
- **Files:** [mw.cpp](../../../src/main/mw.cpp), [mw.h](../../../src/main/mw.h), [config.cpp](../../../src/main/config/config.cpp), [altitudehold.cpp](../../../src/main/flight/altitudehold.cpp)
- **Safety impact:** arming state tracking only; no control change yet. The EEPROM bump forces acc / compass recalibration.
- **Done when:** PRIMUSX builds with no new warning against the baseline; `throttleRaisedSinceArm` is true after the first throttle raise after arming and false after disarm ( checked on the bench in task 7 ).
- **Rollback:** revert the branch commit or reflash `PRIMUSX-2.0.1-rollback.hex`.
- **Status:** done 2026-11-20
- **Result:** `throttleRaisedSinceArm` added to mw.cpp ( `static`, set in `annexCode`, cleared in `mwDisarm` ); `EEPROM_CONF_VERSION` 106 -> 107 and `P8 [ PIDALT ]` 100 -> 128 in config.cpp, plus the same default in the public `PIDProfile::setDefault` ( review finding ); `drivers/system.h` included in altitudehold.cpp. PRIMUSX builds clean, no new warnings, size unchanged ( LTO drops the flag until task 5 reads it ). Reviewer: 1 SHOULD-FIX fixed, 2 NOTEs resolved.

### 3. Altitude setpoint shaping ( `2a8d59a` + deadband removal + hover-trim offload )

**Description.** Hand-port MagisV2 `2a8d59a` into
[altitudehold.cpp](../../../src/main/flight/altitudehold.cpp), using MagisV2's
`althold-setpoint-shaping/CHANGES.md` as the line-by-line spec and the current MagisV2
`altitudehold.cpp` as the reference text: `shapedVelocitySetpoint ( )` ( 0 .. 40 up / 30 down cm/s
from the deadband edge, ramp 100 cm/s^2, 50 cm leash ); external `AltHold` writes ( `setAltitude`,
`setRelativeAltitude` in `API/Control.cpp:209/232`, MSP `serial_msp.cpp:1728/1733`, take-off
`command.cpp:135` ) become a goal flown at 60 / 30 cm/s braking at 80 cm/s^2, cancelled by a stick
move; velocity demand = profile rate + P_ALT x error, clamped per source, slewed at 150 cm/s^2,
replacing the +/-300 cm/s clamp; the stick branch at `:205-228` and the snap-to-EstAlt on centring go;
`applyDeadband ( error, 5 )` at `:360` is removed; `offloadHoverTrim ( )` added as in `4657cbf`;
controller held in reset while disarmed or stick-armed at idle ( `:251-256` ); `velocityControl`
removed and `calculateAltHoldThrottleAdjustment` takes the loop dt. Landing: `isLanding` with
`landThrottle` ( default 1200: `mw.cpp:348`, `command.cpp:90`; `Command.land ( landSpeed )` writes
`1305 - landSpeed` at `API/User.cpp:324` ) keeps flying `( landThrottle - 1500 ) / 4`, i.e. -75 cm/s by
default and -48 to -112 clamped to -100, under a 100 cm/s clamp, never through the stick limits. Keep
`setVelocity` holding the stick / landing request as MagisV2 does, so `getSetVelocity ( )`
( `API/Control.cpp:259` ) keeps its meaning. Files stay CRLF; Magis's unspaced Cleanflight style;
`float` maths only.

- **Depends on:** 2
- **Skills / agent:** pluto-rules; cpp-pro agent ( self-contained C++ module; give it the MagisV2 reference file, CHANGES.md and this task text )
- **Files:** [altitudehold.cpp](../../../src/main/flight/altitudehold.cpp), [altitudehold.h](../../../src/main/flight/altitudehold.h)
- **Safety impact:** ALT_HOLD, take-off and landing behaviour. **Breaks the app flip until task 4 is done: do not flash this build for a flip.**
- **Done when:** PRIMUSX builds with no new warning; a desk review against the MagisV2 CHANGES.md ticks every bullet of `2a8d59a` plus the deadband and hover-trim items; `pluto-reviewer` on the task diff ( Magis paths given explicitly ) has no BLOCKING finding.
- **Rollback:** reflash `PRIMUSX-2.0.1-rollback.hex`.
- **Status:** done 2026-11-20
- **Result:** All 12 items ported by `cpp-pro` and verified against `4657cbf^..2a8d59a`; shaping in its own `shapedVelocitySetpoint ( )`. PRIMUSX clean, no new warnings, +848 B flash / +24 B RAM. `setVelocity` kept as the request after a first draft changed it. Reviewer: 0 BLOCKING, 1 SHOULD-FIX ( fast-change branch ) and 2 NOTEs fixed, 3 accepted as MagisV2 parity. Not flyable until task 4.

### 4. Flip bypass and return to height ( `c569c0f` )

**Description.** Hand-port MagisV2 `c569c0f` into
[altitudehold.cpp](../../../src/main/flight/altitudehold.cpp) using
`flip-althold-regression/CHANGES.md` as the spec: `flipActive ( )` from `flipState`
( [acrobats.cpp](../../../src/main/flight/acrobats.cpp), gated on `ENABLE_ACROBAT` );
`flipVelocitySetpoint ( )` flies `( throttle - 1500 ) / 4` clamped 120 up / 100 down straight to the
velocity loop with no ramp, slew or leash; on flip exit reset the setpoint to zero, restore
`errorVelocityI` to its pre-flip value, hold the velocity integrator 500 ms, ignore the throttle
100 ms, and set the pre-flip `AltHold` as a return goal on the 60 / 30 profile unless ALT_HOLD was
off, the craft was idle-held on the ground, or the target is below 20 cm. Check how Magis's flip
treats BOXBARO ( `acrobats.cpp:104` ) so the bypass sees ALT_HOLD the way the app flow leaves it.

- **Depends on:** 3
- **Skills / agent:** pluto-rules; cpp-pro agent
- **Files:** [altitudehold.cpp](../../../src/main/flight/altitudehold.cpp), possibly [acrobats.h](../../../src/main/flight/acrobats.h) for a `flipState` accessor
- **Safety impact:** flip under ALT_HOLD; flip exit climb and return. A wrong exit hand-over caused a flyaway into the ceiling on MagisV2 ( CHANGES.md ), so the integrator restore and the 500 ms hold are not optional.
- **Done when:** PRIMUSX builds with no new warning; every bullet of `c569c0f`'s CHANGES.md is ticked; `pluto-reviewer` on the task diff has no BLOCKING finding.
- **Rollback:** reflash `PRIMUSX-2.0.1-rollback.hex`.
- **Status:** done 2026-11-22
- **Result:** All five hunks transcribed from `c569c0f` ( verified against the reference myself ); `acrobats.h` already declared `flipState`. PRIMUSX clean, no new warnings, +280 B flash / +48 B RAM over task 3. Reviewer: 0 BLOCKING; the one SHOULD-FIX was a real Magis-specific defect ( the 20 cm floor guard assumed an arm-relative baro datum Magis does not have ) and is fixed with a ground reference, together with the goal-target snapshot that stops repeated flips ratcheting the height up.

### 5. Airborne stick-low I-term freeze ( `a7f398d` )

**Description.** Port `a7f398d`: in [pid.h](../../../src/main/flight/pid.h) /
[pid.cpp](../../../src/main/flight/pid.cpp) add `pidSetGyroITermFreeze ( bool )`; while set,
`pidRewrite` ( `:735`, accumulation at `:831` ) skips the `errorGyroI` accumulation on all three axes
while the clamp and the ITerm still use the held value ( the MagisV2 hunk applies as-is, re-typed to
keep CRLF ). In [mw.cpp](../../../src/main/mw.cpp) `processRx` ( `:789-792` ): when
`THROTTLE_LOW && ARMED && throttleRaisedSinceArm && !isThrottleStickArmed ( )` and the controller is
`pidRewrite` ( `pidController == 1`, `config.cpp:164` ) freeze instead of calling
`pidResetErrorGyro ( )`; every other case resets as before and clears the freeze; `mwDisarm` clears it.
`throttleStatus` and `calculateThrottleStatus` are unchanged so stick arm, switch disarm and the
beeper behave as before.

- **Depends on:** 2
- **Skills / agent:** pluto-rules
- **Files:** [pid.h](../../../src/main/flight/pid.h), [pid.cpp](../../../src/main/flight/pid.cpp), [mw.cpp](../../../src/main/mw.cpp)
- **Safety impact:** attitude I-term handling in flight; ground, arming, idle and disarm resets unchanged. Known residual ( as on MagisV2 ): a wind-up built on the floor before the stick goes low is held, not cleared.
- **Done when:** PRIMUSX builds with no new warning; the freeze condition matches MagisV2 `mw.cpp` after `a7f398d` line for line; `pluto-reviewer` on the task diff has no BLOCKING finding.
- **Rollback:** reflash `PRIMUSX-2.0.1-rollback.hex`.
- **Status:** done 2026-11-22
- **Result:** Ported by hand: `gyroITermFrozen` + `pidSetGyroITermFreeze ( )` and the guarded accumulation in `pid.cpp` / `pid.h`, the freeze condition and guarded reset in `processRx`, the clear in `mwDisarm`. Every identifier was already visible in `mw.cpp`. PRIMUSX clean, no new warnings, +128 B flash, RAM unchanged; `throttleRaisedSinceArm` is now in the image ( first reader ). Reviewer: 0 BLOCKING, 0 SHOULD-FIX, 2 NOTEs recorded, no code change.

### 6. Build gate

**Description.** Clean build of PRIMUSX with the recipe; diff the `warning:` lines against
`BASELINE_WARNINGS.txt` ( any new warning under `src/` is fixed, never baselined away ); record
text / data / bss and the delta against the task-1 numbers in `TESTING.md`. Confirm the hex name and
that `make TARGET=PRIMUSX -n` lists no stale object.

- **Depends on:** 3, 4, 5
- **Skills / agent:** pluto-build ( method ), manual make
- **Files:** none edited unless a warning must be fixed
- **Safety impact:** none
- **Done when:** zero new `src/` warnings; flash and RAM within the F303's 252 K / 40 K with the delta recorded ( expected about +1-1.5 KB / +40 B ).
- **Rollback:** n/a
- **Status:** done 2026-11-22
- **Result:** Clean PRIMUSX build, exit 0; gate clean, and the warning totals are identical to the baseline ( 2 361; `src/` 899 ). Flash 77.4 KB / 252 KB ( +1 256 B ), RAM 9.1 KB / 40 KB ( +72 B ), both within plan; no stale objects. Found that PlutoIDE cannot flash the Magis hex name ( pre-existing, master `2e55152` ): task 12.

### 7. Bench check, props off

**Description.** With `pluto-flighttest`, add a temporary PlutoMonitor print to Magis's
`API/PlutoPilot.cpp` ( fields: `EstAlt`, `AltHold`, the shaped setpoint, the velocity demand,
`errorVelocityI`, `throttleRaisedSinceArm`, the freeze flag, `flipState`, roll / pitch I-term ) and,
props off: arm with the stick and confirm the controller stays in reset at idle; raise the throttle and
confirm `throttleRaisedSinceArm` latches and clears on disarm; with ALT_HOLD on, move the stick and
watch the setpoint ramp and leash rather than step; pull the stick fully down while "airborne" is faked
( after a throttle raise ) and confirm the I-term holds; disarm and confirm the reset. Keep the print
under the MSP budget. Record in `TESTING.md`.

- **Depends on:** 6, 12
- **Skills / agent:** pluto-flighttest
- **Files:** [PlutoPilot.cpp](../../../src/main/API/PlutoPilot.cpp) ( temporary print, removed or neutralised before commit )
- **Safety impact:** props off
- **Done when:** each bench item has a logged line in `TESTING.md` showing the expected behaviour; no unexpected motor command.
- **Rollback:** n/a
- **Status:** done 2026-11-22
- **Result:** Superseded by a flight: the user flew the instrumented build ( log-1 ). Passed in flight: arming resets the rate I-term, the take-off goal profile ( +120 cm in 2.5 s ), the 40 cm/s stick climb, centring without a snap, the I-term freeze ( held exactly through 2.5 s and 5.8 s stick-downs ), the 30 cm/s stick descent, and the landing invariant ( -75 cm/s request, touched down and disarmed itself ). The app keeps `Palt` 128. Not exercised: B2, B7, B8 and the leash ( carried to task 8 ). **New finding: the altitude estimate drifted about 1.9 m low over the 80 s flight, with a -55 cm/s velocity bias still present at rest after the disarm ( task 13 ).**

### 8. Flight validation

**Description.** Flight plan from `pluto-flighttest`, mirroring the MagisV2 TESTING.md of the three
topics, on the PRIMUSX ( brushed, MPU6500 + MS5611 ) with PlutoMonitor logs: ( A ) ALT_HOLD stick
climb / descent: rate ramps to about 40 / 30 cm/s, coasts to a stop on centring, no overshoot step;
( B ) app take-off to 120 cm: profile flown, time to target recorded; ( C ) at least 5 app flips with
ALT_HOLD on: rotation completes ( no 2.2 s timeout ), climb reaches 100 cm/s, return to the pre-flip
altitude within about +10 cm, no flyaway on exit; ( D ) front payload, full stick-down 5-6 s in hover:
pitch mean within 0.3 deg, I-term held; ( E ) regression: stick arm then take-off, landing from the app
( touchdown detected ), RC-loss landing, disarm. Large logs go to `pluto-log-analyst`. If the MagisV2
constants misbehave on this sensor set, record the numbers and open a tuning decision before changing
any constant.

- **Depends on:** 7, 13, 14
- **Skills / agent:** pluto-flighttest; pluto-log-analyst agent
- **Files:** none ( logs and `TESTING.md` )
- **Safety impact:** flights; the rollback hex is at hand. Fly ( C ) only after ( A ) and ( B ) pass.
- **Done when:** log-4 ( one combined flight ) passes its nine steps in TESTING.md, above all the payload lean ( pitch mean within 0.3 deg of the hover baseline through a 5-6 s full stick-down, with the pitch I-term constant ) and the ground freeze ( `O` exactly constant from arm to disarm ); and the flips are flown separately and judged by eye, with the result written into TESTING.md.
- **Rollback:** reflash `PRIMUSX-2.0.1-rollback.hex`
- **Status:** done 2026-11-22
- **Result:** log-4, one combined flight ( 1 225 ticks, no dropped records ). Passed: the freeze **not** engaging before a throttle raise ( 86 ticks, `Z` 28 throughout - the case log-1 missed ); the I-term freeze in flight, with `Q` exactly constant through 18.6 s, 5.5 s and 6.7 s stick-downs; the payload lean at -0.31 / -0.41 / -0.19 deg against the 2-7 deg of the bug; the ground freeze, `O` exactly constant through each armed run; the stick caps ( 39 up, -29 down ); the landing at -75 cm/s; and the leash, one 3 cm overshoot on one tick of 1 225. Caveat: the lean baseline was not a settled hover, so those figures carry about +/-0.2 deg. The flips remain unflown: task 16.

### 9. Docs: CHANGES, CHANGELOG, compliance against the MagisV2 subsystem docs

**Description.** Write `CHANGES.md` ( each change with file:line links and a file index ), create
`Magis/CHANGELOG.md` with a `v2.1.0` section ( the three ported changes and the `4657cbf` extras, with
the MagisV2 hashes as provenance, the EEPROM wipe note ), finish `TESTING.md`, update this topic's
README and the `active-development/README.md` row. Run `spec-to-code-compliance` with MagisV2's
`Altitude_Hold_Estimator.md` ( setpoint shaping and flip interaction sections ) and
`Core_PID_Controller.md` ( stick-low reset / freeze ) as the spec and the Magis files as the code;
record every deliberate deviation ( fixed `landThrottle`, no Kalman, no laser ) in `CHANGES.md`.
Remove the temporary print of task 7 if still present.

- **Depends on:** 8
- **Skills / agent:** spec-to-code-compliance
- **Files:** this folder, `Magis/CHANGELOG.md`, `Magis/docs/active-development/README.md`
- **Safety impact:** none
- **Done when:** compliance check lists no unexplained deviation; CHANGES / TESTING / CHANGELOG complete.
- **Rollback:** n/a
- **Status:** done 2026-11-22
- **Result:** `TEMP-BENCH` instrumentation removed ( `PlutoPilot.cpp` restored from git exactly; `pid.cpp` and `mw.cpp` stripped back to their port-only diffs ) and the clean tree re-gated: no new warnings, totals identical to the 2.0.1 baseline, flash +1 384 B and RAM +96 B. `CHANGELOG.md` created with a v2.1.0 entry. Compliance against the MagisV2 pipeline docs: 49 requirements, **0 contradictions**, 7 intended deviations confirmed, 1 absent and fixed ( the ground-idle block ran only in the non-default fast-change mode, which left the payload fix off there ), 1 stale comment corrected.

### 10. Topic review

**Description.** `pluto-reviewer` on the whole branch diff ( `git -C Magis diff master...althold-backport`,
paths given explicitly, told that the tree is Magis: no `rcDataPilot`, CRLF, `-Wundef` ), with the
`pluto-rules` checklist and the MagisV2 CHANGES.md files as the spec. Fix BLOCKING findings and re-run
task 6's gate.

- **Depends on:** 9
- **Skills / agent:** pluto-reviewer agent ( always a subagent )
- **Files:** as found
- **Safety impact:** as found
- **Done when:** no BLOCKING finding open; the gate re-run is clean.
- **Rollback:** n/a
- **Status:** done 2026-11-22
- **Result:** `pluto-reviewer` over the whole diff: **0 BLOCKING**, and no cross-feature interaction producing a wrong result ( the flip-timeout disarm against the settle hold, the return goal against the arm-relative datum, the offload against the flip snapshot, and the landing against the shaping and leash were all traced clean ). 6 SHOULD-FIX fixed, including a factual error repeated through the docs: **a stalled flip climb does not disarm in the air** - the ASCEND timeout abandons the flip and hands back to level altitude hold; only a stalled *rotation* disarms. The task 9 move was confirmed behaviour-identical on the default path and correct in fast mode. Recommendation adopted: **API 1.0.1 -> 1.0.2** at commit.

### 11. Version bump and commit preparation

**Description.** Set `FW_Version = 2.1.0` **and `API_Version = 1.0.2`** in the Magis Makefile ( the API bump was decided in task 10: public behaviour changed without a signature change, as MagisV2 did in `2a8d59a` ), rebuild so
`Build/PRIMUSX/PRIMUSX-2.1.0.hex` exists, set the topic **Closed - pending commit** here and in the
index, stage the Magis changes and draft the commit message from
`MagisV2/.claude/skills/pluto-commit/COMMIT_TEMPLATE.md` ( summary, details, versions, build numbers,
flight results ). **The user runs `git commit`.** The `pluto-commit` skill is used for its template and
bump steps only: no all-target build ( PRIMUSV3R out of scope ), no pipeline promotion, no graph refresh.

- **Depends on:** 10
- **Skills / agent:** pluto-commit ( adapted )
- **Files:** [Makefile](../../../Makefile), this folder
- **Safety impact:** none
- **Done when:** the 2.1.0 hex builds, everything is staged, the drafted message is shown and the user has committed.
- **Rollback:** n/a
- **Status:** done 2026-11-22
- **Result:** `FW_Version` 2.1.0 and `API_Version` 1.0.2; PRIMUSX clean-built ( exit 0, gate clean, warning totals identical to the 2.0.1 baseline ) into `Build/PRIMUSX/PRIMUSX-2.1.0.hex`, with the PlutoIDE copy `PRIMUSX_2.1.0.hex`. CHANGELOG version line added, topic closed pending commit, change staged by path and the commit message drafted. The user commits.


### 12. Make the Magis hex flashable by PlutoIDE

**Description.** PlutoIDE 3.0.2 flashes only `Build/<TARGET>/<project>_<TARGET>_<FW_Version>.hex`
or the legacy `Build/<TARGET>/<TARGET>_<FW_Version>.hex` ( `bootstrap.js:72-82` in the installed
extension ), while the Magis Makefile writes `Build/<TARGET>/<TARGET>-<FW_Version>.hex` with a hyphen,
since master `2e55152`. So no current Magis build can be flashed from PlutoIDE, and the task 7 bench
check needs one. Found during task 6; pre-existing, not caused by the port. The fix is the user's
choice ( decisions log ): a Makefile change matching MagisV2's `PROJECT` naming, or a rename of the
built hex for testing only.

- **Depends on:** 6
- **Skills / agent:** pluto-build ( method )
- **Files:** [Makefile](../../../Makefile) if the Makefile route is chosen
- **Safety impact:** none ( build output name only )
- **Done when:** PlutoIDE finds and flashes the PRIMUSX build of this branch.
- **Rollback:** revert the Makefile line, or delete the renamed copy
- **Status:** done 2026-11-22
- **Result:** Testing-only route, chosen by the user: the Makefile is untouched and the built hex is copied from `PRIMUSX-<V>.hex` to the legacy `PRIMUSX_<V>.hex` that PlutoIDE accepts. Done for the current build ( byte-identical, `cmp` clean ); `Build/` is git-ignored so nothing enters the repo. The command and the stale-copy check are in TESTING.md.


### 13. Estimator drift in flight: isolate the cause

**Description.** log-1 ( the first flight of the port ) shows the altitude estimate about 1.9 m below
the floor it took off from at the disarm, building mainly during the two long descents, while the pilot
saw the craft clearly in the air when it read 0.5-0.8 m below the take-off point; after the disarm the
estimator's velocity still read -55 cm/s at rest, against about 0 at rest before arming. Before the
task 8 flights, which measure heights against this estimate, attribute it. **The user chose a master
baseline flight, with no independent height reference** ( heights judged by eye ), and **"measure
first" for the barometer compensation** ( decisions log ): the same flight therefore carries the
barometer fields, so one pair of flights both attributes the drift and fits the coefficients tasks 14
and 15 need.

Log change ( all `TEMP-BENCH` ): add `C` = die temperature x100 ( `getBaroTemperature ( )`, printed as
an integer because Magis still has the broken double print ), `P` = raw pressure in Pa
( `getBaroPressure ( )` ) and `H` = `rcData [ THROTTLE ]`; drop `R`, `Q`, `G` and `F`, whose checks
passed in log-1 and which no case here uses. Budget: 124 B worst case, inside the ~130 B limit.

Build untouched master `2e55152` in a **separate git worktree** so the branch is never edited, with the
same tags for the variables master has ( it has `AltHold`, `EstAlt`, `VelocityZ`, `setVelocity`,
`errorVelocityI`, the baro getters and `rcData`; it needs its own small flags helper ). Fly both builds
on the same pattern: **10 s at rest disarmed, take-off, 60 s hover, a 5 s full stick-down, hover, land,
10 s at rest disarmed**, after a careful level accelerometer calibration. Each build's `START` line
identifies it ( `Palt` 128 branch, 100 master ).

What the pair of logs settles:

- `E` falling while `P` is steady -> the accelerometer path of the estimator, not the barometer.
- `P` rising with `C` -> die-temperature drift: fit Pa/degC for task 15.
- `P` tracking `H` -> rotor inflow: fit Pa/count for task 15.
- master drifting the same way -> pre-existing Magis behaviour, and task 8 can proceed in parallel
  with 14 / 15; master clean -> the port caused it.

- **Depends on:** 7
- **Skills / agent:** pluto-flighttest; pluto-log-analyst agent for the two logs; pluto-rules for the master instrumentation
- **Files:** [PlutoPilot.cpp](../../../src/main/API/PlutoPilot.cpp) ( the log change, temporary ); a master worktree outside the branch
- **Safety impact:** flights; the estimate is off by up to 2 m by the end of a flight, so keep them short, low and over a clear floor, and be ready to land manually. Master flies the old controller ( raw stick rate up to 120 / 100 cm/s ).
- **Done when:** both logs are in `logs/` and `TESTING.md` records which hypothesis they support, with the fitted Pa/degC and Pa/count slopes ( or a statement that the barometer is not the cause ).
- **Rollback:** the log change is `TEMP-BENCH` only; after the master flight restore the branch's flashable hex ( `cp PRIMUSX-2.0.1.hex PRIMUSX_2.0.1.hex`, then `cmp` ) and recalibrate after flashing.
- **Status:** done 2026-11-22
- **Result:** Flown on both builds. **The 1.9 m drift did not reproduce**: the estimate closed to -15 cm on the branch and -12 cm on master over 78 / 75 s, and the at-rest velocity decayed to about 0 within 4.5 s of the motors stopping. **The barometer is ruled out**: pressure closed within 2 Pa while the die warmed 10-11 C, and the fitted slopes are weak and opposite in sign between the flights ( -0.48 and +0.56 Pa/degC ), where MagisV2 measured -2.2. log-1 is attributed to its accelerometer calibration, the first boot after the settings wipe. Recommendation recorded: drop task 15.


### 14. Barometer ground zero: track while disarmed, freeze on arm

**Description.** Port the ground-reference half of MagisV2 `4657cbf`: the barometer ground zero follows
the pressure while the craft is disarmed and freezes at the arm instant, so altitudes are relative to
where the craft armed. Magis sets `baroGroundPressure` only at the boot calibration and at the
gyro-calibration stick command ( `sensors/barometer.cpp`, `io/rc_controls.cpp:250` ), which is why every
height in log-1 was relative to boot. With this in, the flip's 20 cm floor guard and `limitAltitude ( )`
mean what they say, and `altGroundAlt` from task 4 becomes a second line of defence rather than the only
one. **It must never re-zero in flight.** The guard is `ARMED` itself: while armed the offset cannot move
( MagisV2's own `baroUpdateZero ( )` gates the same way ). *Corrected 2026-11-22: this section first named
`throttleRaisedSinceArm` as the guard; in MagisV2 that flag guards a different call, `baroResetGroundLevel ( )`
inside `mwArm ( )`, which Magis does not have.*

- **Depends on:** 13
- **Skills / agent:** pluto-rules; pluto-reviewer agent
- **Files:** [barometer.cpp](../../../src/main/sensors/barometer.cpp), [barometer.h](../../../src/main/sensors/barometer.h), [mw.cpp](../../../src/main/mw.cpp)
- **Safety impact:** changes the altitude frame: every target, the ceiling and the flip return are measured from the arm point instead of boot. A re-zero in flight would silently redefine altitude, so the offset is frozen while `ARMED` **and** for 3 s after any disarm ( a disarm in the air can be followed by a re-arm in the air ).
- **Done when:** PRIMUSX builds with no new warning; `pluto-reviewer` has no BLOCKING finding; in a task 8 log, `O` settles while disarmed, stops dead at the arm tick and never moves while armed, and `E` starts the flight near 0 **given at least 10 s on the ground since the previous landing** ( within that gap the previous flight's error is only partly worked off ).
- **Rollback:** reflash `PRIMUSX-2.0.1-rollback.hex`.
- **Status:** done 2026-11-22
- **Result:** Ported from `4657cbf`: an IIR tracks the ground zero while disarmed and it freezes on arm, so altitude is measured from the arm point instead of boot ( log-3 started at -46 cm on its own floor ). The review found a BLOCKING hole that MagisV2 shares - a flip timeout or an ARM-switch-off disarms in the air without setting `Crash`, and the chuck-arm re-arms during the fall - so tracking is now also held for 3 s after any disarm. A dead re-seed branch was removed and two over-stated comments corrected. PRIMUSX clean, +128 B flash / +24 B RAM, re-checked clean.

### 15. Barometer throttle and temperature compensation, coefficients fitted to this airframe

**Description.** Port the compensation half of MagisV2 `4657cbf` - throttle ( rotor inflow ) and die
temperature terms, both relative to the arm instant and clamped together - **with the slopes fitted
from the task 13 logs**, not MagisV2's. MagisV2 measured -2.17 to -2.42 Pa/degC on an ICP-10111 and
0.0086 Pa per throttle count; Magis has an MS5611 on a different airframe, so its numbers have to come
from its own flights. Magis already converts pressure to altitude at a fixed ground temperature
( `apmBaroCalculateAltitude ( )` ), so that third part of `4657cbf` needs no port. Only do this if
task 13 shows the barometer is actually a cause; if it is not, close this task as dropped and record
why.

- **Depends on:** 13, 14
- **Skills / agent:** pluto-rules; pluto-flighttest for the validation hover; pluto-reviewer agent
- **Files:** [barometer.cpp](../../../src/main/sensors/barometer.cpp), [barometer.h](../../../src/main/sensors/barometer.h)
- **Safety impact:** alters the altitude every control loop reads. A wrong sign or an unclamped term would drive the craft into the floor or the ceiling, so keep MagisV2's clamp and prove the sign on the bench before flying.
- **Done when:** a 3 min hover holds height within about +/-20 cm with no steady trend, and the ground closure ( pressure after landing minus before arming ) is small; PRIMUSX builds clean with no new warning; reviewer clean.
- **Rollback:** set both coefficients to 0, which makes the chain inert, or reflash `PRIMUSX-2.0.1-rollback.hex`.
- **Status:** dropped 2026-11-22 ( measured: there is no effect to correct )
- **Result:** Dropped on the task 13 measurement, the user's decision. Over a 10-11 C die warm-up the raw pressure closed within 2 Pa ( branch ) and 1 Pa ( master ), bounding the whole flight's barometric drift under about 17 cm; MagisV2's -2.2 Pa/degC would have moved it 22-27 Pa, about 2 m. The per-flight fits ( -0.48 and +0.56 Pa/degC, r -0.18 and +0.20 ) are opposite in sign and consistent with zero, and a flying log cannot fit this anyway: pressure there is dominated by the craft's own 3 m of altitude change, which MagisV2 could subtract only because it had a laser reference. A bench soak ( stationary, motors off, from cold ) is the experiment that would give a usable coefficient if the drift ever returns.


### 16. Flip validation by eye ( `c569c0f` )

**Description.** The flip bypass and return-to-height are the whole point of `c569c0f`, and the RC link
that carries the log cannot survive a flip, so this is the one piece of hardware validation done by
observation. Five or more app flips with ALT_HOLD on, settled between them, over a clear floor with
about 3 m of headroom; then, if it can be done safely, one flip sent while the previous one's return
descent is still running. What matters is that **each flip completes its rotation** - the regression
`c569c0f` fixes is the flip stalling at the shaped 40 cm/s cap, so its climb state times out after 2.2 s
and the flip is abandoned - and that the craft comes back to roughly the height it was sent from rather than
climbing away or sinking. Record the count and what was seen in `TESTING.md`.

- **Depends on:** 8
- **Skills / agent:** pluto-flighttest
- **Files:** none ( `TESTING.md` )
- **Safety impact:** flips at height. A flip whose climb stalls is abandoned at 2.2 s of full throttle, which off the ground is a climb; a flip whose rotation stalls disarms in the air ( `acrobats.cpp:165`, `:188` ). Keep the floor and the airspace clear, and do not fly over people.
- **Done when:** flips complete their rotation with no timeout and no in-air disarm, and return to roughly their starting height; recorded in `TESTING.md`. ( Met with one or two flips rather than the five asked for: the failure mode is deterministic, so completion proves the bypass, but repeatability is not established. )
- **Rollback:** reflash `PRIMUSX-2.0.1-rollback.hex` ( master behaviour ) if a flip misbehaves.
- **Status:** done 2026-11-22
- **Result:** Flown by eye. All flips completed their rotation and returned to height, and a flip sent during the previous one's return descent went back to the **original** height, confirming the goal snapshot added beyond MagisV2 in task 4. Sample was one or two flips, not the five asked for: the bypass and the exit hand-over are proven ( the failure without them is a deterministic 2.2 s timeout and an in-air disarm ), repeatability across batteries, heights and directions is not. Logged as a commit-time limitation.

## Decisions log

Newest last. One line each: `YYYY-MM-DD [decision|assumption|out-of-scope|risk] text`.

- 2026-11-20 [decision] Mode feature: a backport of settled MagisV2 designs; the design choice is the port strategy only.
- 2026-11-20 [decision] Target PRIMUSX only. Hand-port ( DESIGN.md option b ) confirmed by the user.
- 2026-11-20 [decision] All three commits go in, in one build; `2a8d59a` and `c569c0f` are never flashed apart ( the shaping alone breaks the flip ).
- 2026-11-20 [decision] `4657cbf` extras included: `throttleRaisedSinceArm` ( required ), 5 cm position deadband removed, `offloadHoverTrim`, P_ALT 100 -> 128 with `EEPROM_CONF_VERSION` 106 -> 107.
- 2026-11-20 [decision] Planning and evidence live inside Magis ( `docs/active-development/` ); MagisV2 is not edited. Code on a new Magis branch `althold-backport` off `master`; the user makes every commit.
- 2026-11-20 [decision] FW 2.0.1 -> 2.1.0 at commit; API 1.0.1 unchanged. A `Magis/CHANGELOG.md` is started.
- 2026-11-20 [decision] Done when flight-validated on the PRIMUSX ( tests A-E of task 8 ).
- 2026-11-20 [assumption] MagisV2 constants ( 40 / 30, 60 / 30, 120 / 100 cm/s; 100 / 80 / 150 cm/s^2; 50 cm leash; 500 ms, 100 ms, 20 cm ) used as-is on MPU6500 + MS5611; tuned only if task 8 demands, as a logged decision.
- 2026-11-20 [assumption] Magis's fixed `landThrottle` 1200 ( about -75 cm/s ) stays the landing descent under the 100 cm/s clamp; the MagisV2 confirm-throttle landing ramp is not ported.
- 2026-11-20 [assumption] Brushed motors; the +/-300 `errorVelocityI` clamp is kept.
- 2026-11-20 [assumption] `pluto-rules` flight / MCU rules apply; the `rcDataPilot` rule does not ( Magis has no RC override path ).
- 2026-11-20 [out-of-scope] PRIMUSV3R ( F103 built with hard-FPU M4 flags, Makefile:169; never built here ), Kalman VelocityZ / EstAlt filtering, laser fusion, any API signature change, the MagisV2 landing ramp.
- 2026-11-20 [risk] The EEPROM bump wipes saved settings: recalibrate acc and compass on first boot.
- 2026-11-20 [risk] `pluto-build`'s driver, `pluto-commit`'s all-target build and the graphify graph are MagisV2-only; Magis uses the manual recipe at the top of this file and a hand-captured warning baseline.
- 2026-11-20 [risk] The Magis repo is an untracked nested folder ( `?? Magis/` ) inside the MagisV2 working copy; do not stage it from MagisV2.
- 2026-11-20 [decision] Magis gate uses MagisV2's `tools/warnings.py` with `--baseline` pointing at this topic's `warnings-baseline.json` ( recorded on untouched `2e55152`, PRIMUSX, gcc 14.2.1 ); MagisV2's baseline is not touched.
- 2026-11-20 [decision] The 2.0.1 rollback hex is kept in the topic folder, excluded from git by the folder's `.gitignore`.
- 2026-11-20 [assumption] Line endings: index LF, working tree CRLF through global `core.autocrlf=true`; commits normalise either way, edits keep CRLF.
- 2026-11-20 [risk] PlutoIDE wrote `Magis/plutoide.ini` ( project `Magis`, target `PRIMUSX`, firmware `magis` 2.0.1 ). The Magis Makefile names the hex `PRIMUSX-<ver>.hex` and has no `PROJECT`; whether PlutoIDE flashes that name is unconfirmed. Settle before task 7.
- 2026-11-20 [risk] `flight/pid.h` carries 36 baseline warnings that repeat per including file: a new `#include` of it fails the gate without new code.
- 2026-11-20 [decision] Task 2: `throttleRaisedSinceArm` stays `static` in mw.cpp and is not exported through mw.h. In `2a8d59a` / `c569c0f` / `a7f398d` only mw.cpp `processRx` reads it; the MagisV2 accessor `throttleRaisedInFlight ( )` came with `73022c3` ( `setDesiredAngle` ), outside this port.
- 2026-11-20 [decision] Task 2 review fix: the public `PIDProfile::setDefault ( )` ( `API/Control.cpp` ) also sets `P8 [ PIDALT ]`; both it and `resetPidProfile ( )` now say 128, each with a cross-reference comment. Giving them one shared source ( as MagisV2 did in `ebf4e46` ) is out of scope here.
- 2026-11-20 [assumption] Unconfirmed: whether the Pluto app pushes its own PID set over `MSP_SET_PID` on connect, which would override the 128 default. Watch `P_ALT` in the task 7 bench log.
- 2026-11-20 [risk] 33 of 315 `src/main` files are stored CRLF in the index ( `API/Control.cpp` among them ), so `git diff --check` flags every added line in them as trailing whitespace. Expected; do not "fix" it by stripping CRs.
- 2026-11-20 [decision] Task 3 mini-grill: the estimator tilt-threshold units fix from `4657cbf` goes into task 3 ( `altitudehold.cpp:857` `> 30` deci-degrees = 3 deg -> `> 300` ), since the shaped loop flies against `EstAlt` and the slow filter otherwise runs in all normal flight. Flight-checked in task 8.
- 2026-11-20 [decision] Task 3 mini-grill: `ALT_CEILING_MARGIN_CM` 25 ( from `4657cbf`; Magis has a literal 50 at `altitudehold.cpp:1005` ) is ported too.
- 2026-11-20 [decision] Task 3: `4657cbf`'s "keep `errorVelocityI` across setpoint changes" ( no reset when the stick moves the setpoint ) is part of the hover-trim offload item and goes in with it.
- 2026-11-20 [decision] Task 3: `setVelocity` keeps MagisV2's meaning ( the stick / landing request ), so the public `getSetVelocity ( )` / `DesiredVelocity.get ( Z )` is unchanged and the `c569c0f` port stays a transcription. A first draft had it hold the final demand.
- 2026-11-20 [decision] Task 3 review fix: the `alt_hold_fast_change` branch calls `resetAltSetpoint ( )` on centring instead of only `AltHold = EstAlt`. MagisV2 has the same defect in that branch; fixing it here cannot affect the default path, which never runs it.
- 2026-11-20 [decision] Task 3: `altholdDebug3` now logs `altGoalActive` instead of the always-zero `isAltHoldChanged`.
- 2026-11-20 [assumption] Corrected: `landThrottle` is not fixed. Default 1200 ( -75 cm/s ); `Command.land ( landSpeed )` writes `1305 - landSpeed` ( `API/User.cpp:324` ), i.e. -48 to -112 cm/s clamped to -100. All faster than the 30 cm/s stick cap, so the landing invariant holds.
- 2026-11-20 [risk] Pre-existing Magis defect found in review, out of this port's scope and needing its own topic: `isThrottleStickArmed` ( `io/rc_controls.cpp:175` ) is cleared only inside BARO / SONAR mode and in `mwDisarm`, so arming on the ARM switch in throttle mode and then switching ALT_HOLD on with the stick at or below 1500 drives `rcCommand [ THROTTLE ] = 1000` and the craft drops. A stick-arm that `mwArm ( )` refuses also leaves it set, so a later app take-off sits at idle.
- 2026-11-20 [risk] Accepted MagisV2-parity behaviours, to watch in tasks 7 and 8: `DesiredPosition.get ( Z )` returns the moving target while a goal is flown; a one-shot user altitude write during the stick-arm ground idle is overwritten; the offloaded hover trim survives a disarm in `initialThrottleHold`.
- 2026-11-22 [decision] Task 4 review fix, a deliberate deviation from `c569c0f`: the flip return test measures against a new `altGroundAlt` ( `EstAlt` recorded whenever the controller runs its on-ground reset ) instead of against a bare 20 cm. Magis never re-zeroes the baro on arm, so `AltHold` is relative to boot, and the MagisV2 test would pass on any surface 20 cm above the boot height, flying the craft back down onto it at hover thrust with no touchdown detection.
- 2026-11-22 [decision] Task 4: the pre-flip target stored for the return is `altGoalActive ? lrintf ( altGoal ) : AltHold`, not bare `AltHold`, so a flip sent while a goal is being flown ( take-off climb, or the return descent from a previous flip ) does not store an intermediate height. Without it repeated flips ratchet the altitude up unbounded, and task 8 flies five or more.
- 2026-11-22 [assumption] Accepted, matching MagisV2: with `alt_hold_fast_change` on ( off by default ) the post-flip return usually degrades to holding the exit height; a flip ending after BARO is switched off leaves a harmless 100 ms throttle grace at the next BARO entry.
- 2026-11-22 [risk] Pre-existing and out of scope, found in the task 4 review: `acrobats.cpp` SLOWDOWNANDEXIT has no timeout on its fall-through branch, so a stuck flip state would keep the bypass active. Predates this port; worth its own topic with the `isThrottleStickArmed` item.
- 2026-11-22 [assumption] **Corrected, and it was stated wrongly earlier in this topic and to the reviewers:** Magis *does* have a user RC override path. `RcCommand_Set` / `RcCommand_SetChannel` ( `API/User.cpp:143-182` ) write `RC_ARRAY [ ]` and latch `userRCflag [ ]`, and `mw.cpp` `userCode ( )` applies them between user-loop ticks ( channel 3 into `rcData [ THROTTLE ]`, roll / pitch / yaw into `rcCommand [ ]` ). What Magis does not have is MagisV2's separate `rcDataPilot [ ]` pilot snapshot. No consequence for this port: MagisV2's own shaping deflection and `throttleRaisedSinceArm` latch also read `rcData [ THROTTLE ]`, so the behaviour is identical.
- 2026-11-22 [risk] Pre-existing, out of scope, needs its own topic: **a Magis user RC override never expires.** `resetUserRCflag ( )` ( `API/API-Utils.cpp:155` ) exists but has no caller anywhere in the tree, where MagisV2 calls it every loop ( `mw.cpp:1219` ) to drop an override after `max ( 250 ms, 2 x userLoopFrequency )`. In Magis a one-shot `RcCommand_Set ( THROTTLE, x )` keeps being re-applied for as long as Developer Mode runs, so with setpoint shaping it moves the altitude setpoint indefinitely, and a stale value is re-applied immediately if Dev Mode resumes after a gap.
- 2026-11-22 [risk] Known limit of the freeze, as in MagisV2: a wind-up built on the floor with the stick at or above mincheck is held, not cleared, by a later full stick-down. Magis paths that can hold it on the ground: a THROTTLE-mode stick landing after a chuck arm or `Command.arm`, and an ALT_HOLD spin-up aborted by a full stick-down before lift-off. Neither is new.
- 2026-11-22 [decision] Task 6 passed: clean PRIMUSX build, warning totals identical to the task-1 baseline, flash +1 256 B and RAM +72 B against 2.0.1, both within plan.
- 2026-11-22 [risk] Settled the open question from task 1, and it is a blocker for task 7: PlutoIDE 3.0.2 flashes only `<project>_<TARGET>_<ver>.hex` or `<TARGET>_<ver>.hex` ( underscore ), and the Magis Makefile writes `<TARGET>-<ver>.hex` ( hyphen ) since master `2e55152`, so no current Magis build is flashable from PlutoIDE. Pre-existing; added as task 12, ahead of task 7.
- 2026-11-22 [decision] Task 12, the user's choice: fix the PlutoIDE hex name by **renaming the built file for testing only**, not by changing the Makefile. `cp Build/PRIMUSX/PRIMUSX-<V>.hex Build/PRIMUSX/PRIMUSX_<V>.hex` after every build; `Build/` is git-ignored, so the repo is untouched and `PRIMUSX-<V>.hex` stays the Makefile's output name.
- 2026-11-22 [risk] The renamed copy is not refreshed by `make`, so a rebuild leaves it stale and PlutoIDE would silently flash the old image. `cmp` the two files before every flash, and do not use PlutoIDE's Build button between the copy and the flash.
- 2026-11-22 [decision] Task 7 instrumentation: Magis has no `Monitor_Print`, so the bench log uses `Monitor.print / println ( tag, int )` ( `API/Utils.h` ), integers only - Magis still carries the pre-`1f8e78e` double-print bug. Three read-only taps were added for file-static state: `benchGyroITerm ( )` and `benchGyroITermFrozen ( )` in `flight/pid.cpp`, `benchFlags ( )` in `mw.cpp`. The Cleanflight `debug[]` array was not reused because the altitude estimator and the ADC driver already write all four slots.
- 2026-11-22 [decision] Every bench line is tagged `TEMP-BENCH` and removal is by stripping those lines; verified that stripping restores `PlutoPilot.cpp` to its committed text. Task 9 removes them and must re-run the gate afterwards.
- 2026-11-22 [risk] On a still, level bench a frozen rate I-term is indistinguishable from a reset one ( both read 0 ): case B3 must wind it up by holding about 10 deg of tilt with the throttle raised before B5 pulls the stick down. Recorded in TESTING.md.
- 2026-11-22 [decision] The user flew the task 7 build with the props on instead of the props-off bench, so log-1 is the first flight. The pilot reported: a 20-30 cm sag after the take-off climb, then recovery; clearly in the air at 33-60 s; the landing touched down and disarmed itself.
- 2026-11-22 [decision] log-1 passed in flight: arming resets the rate I-term ( -256 to -1 ), the take-off goal profile ( +120 cm at 60 cm/s in 2.5 s, matching MagisV2 ), the 40 cm/s stick climb, centring without a snap ( at most 6 cm overshoot ), the I-term freeze ( held exactly through 2.5 s and 5.8 s full stick-downs at non-zero values ), the 30 cm/s stick descent and the landing invariant ( -75 cm/s request ). The app keeps `Palt` 128. Task 7 closed; B2, B7, B8 and the leash carried to task 8.
- 2026-11-22 [assumption] log-1 read 0 cm/s at rest before arming and -55 cm/s at rest after the disarm, so the leading hypothesis is a bias that appears only with the motors running ( vibration or throttle ), not a constant accelerometer calibration offset, which the filter absorbs at rest.
- 2026-11-22 [risk] **The altitude estimate drifted about 1.9 m low over the 80 s flight** ( -8 cm on the floor at take-off, -195 cm on the same floor at the disarm; about -0.8 m or worse while the pilot saw the craft clearly airborne ), building mainly during the two long descents. After the disarm, at rest with the motors off, the estimator's velocity still read -55 cm/s and `E` kept falling to -235: the bias sits in the filter's accelerometer path, not only in an in-air barometer effect. Leading hypotheses: a vertical accelerometer bias ( the accelerometer was recalibrated just before this flight, after the settings wipe ), then the task 3 tilt-threshold change altering how it is corrected. Added task 13 ahead of task 8 to isolate it.
- 2026-11-22 [risk] In the last 0.6 s before the disarm `V` read -89 .. -101 cm/s, so Magis's velocity touchdown test ( `vel > -8` after 2 s ) could not have fired; the landing ended by another path, most likely the impact check. A gentler touchdown might fall through to the 30 s timeout with the motors running.
- 2026-11-22 [assumption] Pre-existing, not caused by the port: with the throttle stick centred while disarmed the rate I-term free-runs to its clamp; arming resets it.
- 2026-11-22 [decision] Task 13, the user's choice: attribute the drift with a **master baseline flight** rather than an A/B of the tilt threshold, and with **no independent height reference** ( heights judged by eye against the floor ). Same flight pattern on master and on the branch, each with 10 s at rest before arming and after landing.
- 2026-11-22 [risk] Switching between master and the branch resets every setting each time ( `EEPROM_CONF_VERSION` 106 vs 107 ): recalibrate the accelerometer and compass after every flash. Matters here, since an accelerometer bias is the leading suspect.
- 2026-11-22 [risk] PlutoIDE flashes the fixed path `Build/PRIMUSX/PRIMUSX_2.0.1.hex` ( version from the open Makefile ), so the master hex must sit there for its flight and the branch copy must be restored afterwards. Each build's `START` line names it ( `Palt` 128 branch, 100 master ), so a log cannot be attributed to the wrong build.
- 2026-11-22 [decision] The user's call on the barometer: **measure first, then port with fitted coefficients.** MagisV2's slopes ( -2.17 to -2.42 Pa/degC, 0.0086 Pa/count ) were measured on an ICP-10111 and a different airframe; Magis has an MS5611 and log-1 carries no temperature, pressure or throttle field, so applying them blind could add as much error as it removes. Task 13's flights now carry those fields and fit the slopes; task 15 ports the compensation with them.
- 2026-11-22 [decision] The user's call: **port the arm-time ground freeze** ( task 14 ), so altitudes are relative to the arm point as in MagisV2 instead of to boot. `throttleRaisedSinceArm` from task 2 is the guard that keeps it from ever re-zeroing in flight.
- 2026-11-22 [assumption] The third part of `4657cbf`, converting pressure to altitude at a fixed temperature, needs no port: Magis's `apmBaroCalculateAltitude ( )` already uses `baroGroundTemperature`, captured at calibration, not the live reading.
- 2026-11-22 [decision] Task 13's log drops `R`, `Q`, `G` and `F` ( their checks passed in log-1 ) to fit `C`, `P` and `H` inside the byte budget: 124 B worst case. Task 8 restores the I-term fields for the payload test.
- 2026-11-22 [decision] Task 13 review fix: `H` logs `althold_throttle` ( the throttle handed to the mixer ), not `rcData [ THROTTLE ]`. In ALT_HOLD the stick only sets a climb rate, so the stick sits near 1500 through a hover and could not have fitted the rotor-inflow coefficient at all. `I` was dropped with it, being incomparable between the builds ( the branch offloads the trim into the baseline ), and the tick counter now wraps at 9999: worst case 120 B per tick instead of 128 B with 1-2 B of margin.
- 2026-11-22 [assumption] `C` and `P` are each a roughly 100 ms mean of the MS5611 conversions in that window, not instantaneous samples, and the baro and user-loop clocks drift against each other, so repeated or skipped values between ticks are expected. A single absurd `P` is the known uninitialised-sample path in `apmBaroRead ( )`, pre-existing on both builds.
- 2026-11-22 [decision] Task 13 answered both of its questions. The 1.9 m drift **did not reproduce**: estimate closure -15 cm ( branch ) and -12 cm ( master ) over 78 / 75 s, and the at-rest velocity decayed to about 0 within 4.5 s of the motors stopping, where log-1 held -55 cm/s. The branch does not drift, and not differently from master.
- 2026-11-22 [assumption] log-1's drift is attributed to its accelerometer calibration ( the first boot after the `EEPROM_CONF_VERSION` 107 wipe ), not to the firmware: log-2 and log-3 were flown after a deliberate level recalibration. An inference from two clean flights, not a proof, so the 10 s at-rest windows stay in every future flight: `V` about 0 at rest, decaying to about 0 after the motors stop.
- 2026-11-22 [decision] **The barometer is ruled out as a drift source on this hardware.** Pressure closed within 2 Pa over 75-78 s on both builds while the die warmed 10-11 C; the fitted slopes are weak and opposite in sign between the flights ( -0.48 and +0.56 Pa/degC, r -0.18 and +0.20 ), against MagisV2's -2.17 to -2.42 Pa/degC on its ICP-10111. At MagisV2's coefficient the observed warm-up would have moved the pressure 22-27 Pa, about 2 m: it did not. The throttle fit is equally null.
- 2026-11-22 [risk] Recommendation on task 15: drop it. Porting MagisV2's compensation with its own coefficients would inject roughly 2 m of error across a 10 C warm-up on this board; fitting Magis coefficients is pointless when the measured effect is indistinguishable from noise. Awaiting the user's decision.
- 2026-11-22 [decision] **Task 15 dropped**, the user's decision on the measurement. The flight logs can rule out a large temperature effect but cannot fit a small one: pressure in a flying log is dominated by the craft's own altitude change ( about 3 m, against the 2 Pa the warm-up moved it ), which is why MagisV2 needed a laser reference across seven flights. The end-to-end bound stands on its own: 2 Pa of closure over a 10-11 C warm-up is under about 17 cm of drift per flight, so a compensation could not help by more than that and a wrong-signed one would add as much. If the drift returns, the experiment to run first is a bench soak ( stationary, motors off, from cold, logging only pressure, temperature and millis ), not another flight.
- 2026-11-22 [decision] Task 14: the in-flight guard for the ground freeze is `ARMED`, not `throttleRaisedSinceArm` as the plan first said. MagisV2's `baroUpdateZero ( )` itself gates on `ARMED`; its `throttleRaisedSinceArm` guard protects `baroResetGroundLevel ( )` in `mwArm ( )`, a call Magis does not have. `ARMED` is the stronger guard: the offset cannot move at any point while armed.
- 2026-11-22 [decision] Task 14 departures from MagisV2, all because the tracked offset is state: updated once per baro sample ( Magis calls `baroCalculateAltitude ( )` twice per sample ); non-finite or absurd readings are kept out of the IIR so a single NaN cannot latch for the whole power cycle; re-seeded when the ground reference is recalibrated so that does not step the estimate.
- 2026-11-22 [risk] Task 14 review, BLOCKING and fixed: "disarmed" does not mean "on the ground" in Magis. The flip timeout and an ARM-switch-off in the air both `mwDisarm ( )` mid-flight without setting `Crash`, and the chuck-arm re-arms during the fall ( `netAccMagnitude < 11`, and `mwArm ( )` passes on `< 2` ) - all four verified in the source. Tracking the ground zero through that window would have carried 45-80 cm of the fall into the next flight's datum. MagisV2 gates on `ARMED` alone and has the same gap; Magis now also holds tracking for `BARO_ZERO_SETTLE_MS` 3000 ms after every disarm.
- 2026-11-22 [decision] Task 14: the settle hold is time-based rather than a stillness test ( `netAccMagnitude` / SMALL_ANGLE ), to keep `barometer.cpp` free of IMU state; 3 s is far longer than any fall the chuck-arm can catch.
- 2026-11-22 [decision] Task 14: tracking is **not** extended into armed ground idle, where MagisV2 still re-zeroes. Its guard would have to be "armed but throttle never raised", precisely the state an in-air re-arm leaves, which would reopen the blocking path in a worse form. The cost is that baro drift during a long armed wait stays in the flight frame ( a few cm per minute ); control is unaffected, since the reset branch pins `AltHold` to `EstAlt` while idling.
- 2026-11-22 [decision] Task 14 removed a dead re-seed branch: `baroGroundPressure` is written only by the boot calibration, and the gyro-calibration stick command reaches only `performBaroCalibrationCycle ( )`, which nothing calls on this target. The once-per-sample key is kept as a guard, with its comment corrected: `checkReading ( )` needs `LASER_ALT` and is compiled out, so there is no double call today.
- 2026-11-22 [risk] Task 14 is a public API behaviour change for task 9 to record: `getEstAltitude ( )`, the MSP altitude and the `BaroAlt` telemetry now read relative to the arm point rather than boot, and while disarmed they are high-passed with a ~2 s constant, so a drone held 1 m up reads about 0 within ~6 s.
- 2026-11-22 [decision] Task 14 re-checked clean after the settle-hold fix: the path is closed, each disarm restamps the window, the `millis ( )` subtraction is unsigned so the wrap is safe, and the boot seed still gives `BaroAlt` exactly 0 before the craft can arm.
- 2026-11-22 [assumption] Accepted: a re-arm within about 10 s of a landing inherits part of that flight's baro error ( the hold is 3 s, then the IIR's ~2 s constant works it off: about 60 % left at 4 s, 5 % by 9 s ). Only the absolute consumers care - the ceiling and absolute user / MSP heights - and the standard flight pattern's closing 10 s at rest already covers it. Task 14's Done-when was reworded accordingly.
- 2026-11-22 [assumption] Accepted: the ARMED state is sampled once per baro sample, so an armed spell shorter than ~100 ms is invisible to the edge detector. Reaching a wrong datum through it needs a disarm, a sub-100 ms re-arm in the air and a second disarm 3 s later, about 44 m of free fall. Stamping from `mwDisarm ( )` would make it exact but couples `barometer.cpp` to `mw.cpp`; not done.
- 2026-11-22 [decision] Task 8 log: `Z` packs the flags, the goal flag and `flipState` ( `Z / 64` ) into one field so that pitch `Y` and the ground offset `O` fit; reviewed at 117 B worst case. `R` / `Q` are back for the payload test.
- 2026-11-22 [risk] Reading the task 8 logs: `Y` saturates at +/-900 and folds back past vertical, so flips are found with `F`, never with `Y`; **roll is not logged, so the payload must be mounted front or back, not to one side**; `G` is only valid while `Z` has bit 8; and `E` sliding toward 0 after a landing is the ground-zero tracker resuming, not a fault.
- 2026-11-22 [decision] The flip ground guard ( a flip while armed on the floor must not set a return goal ) is deliberately not flown: with props on it would launch at full throttle for 2.2 s before giving up, which off the ground is a climb to the ceiling. Verified by review; a props-off bench check is the safe way to see it.
- 2026-11-22 [decision] The user cannot log while flipping ( `plutoLoop ( )` only runs while the RC link is live ), so task 8 becomes **one combined logged flight** ( the old flight 1 plus the payload lean test, log-4 ) and the flips are flown separately and judged by eye. log-5 and log-6 are not used.
- 2026-11-22 [risk] The flip bypass and return-to-height of `c569c0f` therefore get **eye-only hardware validation**: that each flip completes its rotation rather than stalling into the 2.2 s timeout, and comes back to roughly its starting height. The numbers MagisV2 measured ( about +10 cm of settling, 100 cm/s reached in 0.4 s ) cannot be reproduced here. To be recorded as a limitation at commit.
- 2026-11-22 [decision] Task 8 closed on log-4. The port is flight-validated: the I-term freeze holds the rate I-term bit-for-bit through stick-downs of 18.6 s, 5.5 s and 6.7 s and does **not** engage before a throttle raise; the payload lean stays at 0.19-0.41 deg where the bug gave 2-7; the ground freeze holds the offset exactly constant through each armed run; the stick caps, the -75 cm/s landing and the leash all behave.
- 2026-11-22 [risk] The lean figures were measured against a moving baseline ( the craft was being flown, not held in the settled 20 s hover the plan asked for: pitch spans 31 deg in the 5 s before the first freeze ), so they carry about +/-0.2 deg. The mechanism is not in doubt; the numbers are softer than MagisV2's.
- 2026-11-22 [decision] The pilot confirms the 3.2 s disarm inside log-4 was **on the ground**, a normal turnaround between two flights, not an in-air disarm. It incidentally confirmed the task 14 limitation in the wild: the re-arm came 3.2 s after the disarm, so the ground zero had tracked for only 0.2 s and the second flight started 14 cm off in the absolute frame.
- 2026-11-22 [decision] The flips are split out as task 16, flown and judged by eye, since the RC link that carries the log cannot survive them. They are the only hardware validation `c569c0f` will get.
- 2026-11-22 [decision] Task 16 done: the flips completed and returned to height, and the back-to-back case returned to the original height, confirming the task 4 goal snapshot that MagisV2 does not have. Hardware validation of `c569c0f` is therefore positive.
- 2026-11-22 [risk] The flip sample was one or two, not the five planned or the seven MagisV2 flew. Because the failure without the bypass is deterministic and total ( the climb never reaches 100 cm/s, so the climb state times out at 2.2 s and the flip is abandoned ), completion proves the mechanism; what is **not** proven is repeatability across battery states, heights and flip directions, and the exit transient MagisV2 measured ( a 16-42 cm dip with velocity biased about -13 cm/s for a few seconds ) was not looked for. To be stated as a limitation in the commit message.
- 2026-11-22 [decision] Task 9 compliance check against the MagisV2 pipeline docs: 49 in-scope requirements, **0 contradictions**, all 7 intended deviations present and correctly reasoned.
- 2026-11-22 [risk] Found and fixed in task 9: the ground-idle / stick-arm block sat inside the slow-change branch instead of after both branches as `c569c0f` has it, so with `alt_hold_fast_change = 1` it never ran - leaving the controller un-reset on the ground and `isThrottleStickArmed` never cleared, which would have kept the airborne I-term freeze off for a whole flight in that mode. Default-path behaviour is unchanged by the move ( nothing ran between the end of the branch and the end of the function; the rebuilt image is the same size ), so the flight validation stands.
- 2026-11-22 [assumption] Noted, not changed: `AltRst ( )` sets `initialThrottleHold` to 1600 where MagisV2 uses 1500. Pre-existing Magis code outside the ported diff; it runs only while disarmed after a large vertical velocity, but can overwrite an offloaded hover baseline before the next take-off if BARO stays on. In the CHANGELOG known issues.
- 2026-11-22 [decision] Task 10 whole-topic review: 0 BLOCKING, no bad cross-feature interaction. 6 SHOULD-FIX fixed, 5 NOTEs recorded.
- 2026-11-22 [assumption] **Corrected, and it had been repeated in the CHANGELOG, TESTING.md, TASKS.md and to the user: a flip whose climb stalls does not disarm in the air.** The ASCEND timeout ( `acrobats.cpp:93-99` ) re-activates BOXBARO, clears `flipState` and enables ANGLE_MODE, abandoning the flip with the craft level in altitude hold. Only the PITCHING and SLOWDOWNANDEXIT timeouts call `mwDisarm ( )` ( `:165`, `:188` ), i.e. a stalled rotation. The regression `c569c0f` fixes is therefore "2.2 s of full throttle and then no flip", which off the ground is a climb - serious, but not an in-air disarm.
- 2026-11-22 [decision] **API_Version 1.0.1 -> 1.0.2** at commit. No signature changed, but `getEstAltitude ( )` and the MSP altitude reply are now arm-relative, `DesiredPosition.get ( Z )` returns the moving target during a goal, `DesiredPosition.set ( Z )` is profiled and dropped during ground idle, `DesiredVelocity.get ( Z )` has a new range and zeroes with ALT_HOLD off, and `PIDProfile.setDefault ( )` writes 128. `pluto-rules` item 15 plus MagisV2's own precedent ( `2a8d59a`: 1.3.1 -> 1.3.2, behaviour only ).
- 2026-11-22 [assumption] Corrected in the CHANGELOG: on PRIMUSX there are **no** MSP writes to the altitude target ( they are under `#ifdef GPS`, which PRIMUSX does not define ) and no `BaroAlt` telemetry ( no `TELEMETRY`, `BLACKBOX` off ). The MSP altitude reply does exist and is arm-relative now.
- 2026-11-22 [decision] Task 11: FW 2.0.1 -> 2.1.0 ( minor: new user-visible behaviour ) and API 1.0.1 -> 1.0.2 ( patch: behaviour only ). Built PRIMUSX only, as planned; PRIMUSV3R stays out of scope and unbuilt.
- 2026-11-22 [decision] Staged by path: the 9 source files, the Makefile, `CHANGELOG.md`, the Magis active-development index, and the topic folder's documents, `.gitignore` and warning baseline. Deliberately **not** staged: the topic's `logs/` ( raw flight logs, about 1 MB ), `plutoide.ini` ( PlutoIDE's local project file ) and `Build/`. The commit message carries no AI attribution, per the project's commit rules.
