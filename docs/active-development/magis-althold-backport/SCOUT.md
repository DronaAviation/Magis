# magis-althold-backport - Scout brief

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [DESIGN](DESIGN.md)

`pluto-scout` brief from the planning session of 20 Nov 2026, kept verbatim ( line numbers are those
of Magis `master` HEAD `2e55152` and MagisV2 `BugFix-June26` HEAD `2ed03fc` ).

---

## Scout brief: port MagisV2 2a8d59a + c569c0f + a7f398d (alt-hold setpoint shaping, flip bypass, airborne stick-low I-term freeze) into Magis
Graph: stale (built c344387, HEAD 2ed03fc). It covers MagisV2 only. Magis was read directly.

Subsystems: altitude hold, PID, RC input, flip, commands. Pipeline docs: MagisV2 `subsystems/Altitude_Hold_Estimator.md`, `Core_PID_Controller.md`, `RC_Input_Pipeline.md`.

Key code (Magis):
- `Magis:src/main/flight/altitudehold.cpp:172` applyMultirotorAltHold: the stick branch is at 205-228, the stick-arm idle at 251-256.
- `:286` updateAltHoldState, and `:309` updateSonarAltHoldState (SONAR is off on both targets).
- `:341` calculateAltHoldThrottleAdjustment: `:352` sets AltHold=EstAlt while disarmed, `:357` checks velocityControl, `:360` has `applyDeadband(error,5)`.
- `:604` apmCalculateEstimatedAltitude, which calls `:702` calculateAltHoldThrottleAdjustment. It is called from `mw.cpp:705`.
- `Magis:src/main/mw.cpp:789-792` resets the I-terms on THROTTLE_LOW. `:473` mwDisarm. `:348` isLanding writes `rcData=landThrottle`.
- `Magis:src/main/flight/pid.cpp:831` accumulates errorGyroI in pidRewrite (735).
- `Magis:src/main/command/command.cpp:95` takeOff, `:141` land, `:226` B_FLIP (needs MAG_MODE).
- `Magis:src/main/flight/acrobats.cpp:69` desiredVelocity 100, `:93` 2200 ms timeout, `:104` DEACTIVATE BOXBARO, `:107` throttle 2000.

Mode: feature (backport)

Current behaviour: Magis uses the pre-shaping controller. Outside the deadband the stick sets a raw rate `(thr-1500)/4`, clamped -100/+120. On centring, AltHold snaps to EstAlt. The position loop has a 5 cm subtracting deadband. A THROTTLE_LOW stick resets the attitude I-terms on every RX frame.

Analysis:

1. Change-to-counterpart map
- velocityControl: present at :54. Only altitudehold uses it.
- setVelocity: present at :61. `getSetVelocity` reads it at `API/Control.cpp:259`.
- errorVelocityI: present at :57, and it is the same code.
- altHoldThrottleAdjustment: present at :58.
- AltHold:
  - Present at :59.
  - External writers: `API/Control.cpp:209/232` (setAltitude, setRelativeAltitude) and `io/serial_msp.cpp:1728/1733` (MSP).
  - Take-off writes it through `DesiredPosition.setRelative(Z,120)` at `command.cpp:135`.
- offloadHoverTrim: absent (it came in with 4657cbf).
- isLanding: present (`command.h:44`).
- landThrottle: different. It is a fixed 1200 (`command.cpp:90`, `mw.cpp:499`), which flies -75 cm/s. MagisV2 ramps 1300 to 1150 (f60c9ad).
- Landing detection: different. Magis lands on a 30 s timeout, accADC>8500, or vel>-8 after 2 s. MagisV2 has the confirm-throttle ramp.
- flipState and ENABLE_ACROBAT: present on both targets (target.h PRIMUSX:183, PRIMUSV3R:115). The flip logic matches MagisV2 at 2a8d59a^ apart from formatting.
- isThrottleStickArmed: present (`rc_controls.cpp:87,175`).
- throttleRaisedSinceArm: absent.
- landingInProgress: absent.
- pidSetGyroITermFreeze: absent.
- mincheck: 1100 (`config.cpp:442`).
- pidController: 1, which is REWRITE (`config.cpp:164`).
- Altitude estimator: the same 3rd-order complementary filter as MagisV2, but without MagisV2's Kalman filtering of VelocityZ/EstAlt (98c0be3).

2. Base divergence (`-w` diff stats; the patches were dry-run, not applied)
- altitudehold.cpp vs 2a8d59a^: +454/-253.
- mw.cpp: +123/-271.
- command.cpp: +60/-86.
- acrobats.cpp: formatting only.
- pid.cpp and pid.h: identical apart from whitespace and CRLF line endings.
- Magis files are CRLF. All hunks fail on line endings alone.
- With the patches converted to CRLF (`patch --dry-run`):
  - 2a8d59a: 7 of 8 hunks fail.
  - c569c0f: 4 of 7 fail. The 3 that pass do so at fuzz 3, so they are unreliable.
  - a7f398d: pid.cpp and pid.h apply cleanly (`git apply --3way --check`). mw.cpp fails 1 of 4; the other 3 pass only at fuzz 2-3 because the `throttleRaisedSinceArm` context is missing.
- `--3way` cannot help on mw.cpp: the repo lacks the base blobs.
- `git -C Magis status --short` is unchanged (clean).

3. Hidden prerequisites
- **Required:** `throttleRaisedSinceArm`. Set it in annexCode when `ARMED && rcData[THR] > mincheck`, and clear it in mwDisarm (the 4657cbf mw.cpp hunk).
- **Required:** `#include "drivers/system.h"` in altitudehold.cpp, for millis().
- Magis has no arm-time baro reset. grep found only the boot calibrations, apmBaroCalibrate and icp10111BaroCalibrate (barometer.cpp ~438/476). So the in-flight re-zero bug is absent, and the temperature/throttle compensation is absent too.
- **Optional:** offloadHoverTrim, the removal of the 5 cm deadband, and PIDALT 128 (Magis has P 100, EEPROM 106; MagisV2 bumped to 128 and EEPROM 107).

4. Build path
- The toolchain is not on PATH. It is at `C:/PlutoIDE/tools/ARM GNU ToolChain/bin` (gcc 14.2.1), with make under `CYGWIN 64/bin`.
- `make TARGET=PRIMUSX -n` succeeds (408 commands). The Vagrantfile still pins gcc 4.9.3.
- Flags: the same warning set as MagisV2, plus `-Wundef` and `-Wunsafe-loop-optimizations`. gnu17 / gnu++17, `-Os -flto`.
- PRIMUSX is F303 (M4, hard FPU, 252K / 40K).
- **PRIMUSV3R is an F103 (Cortex-M3, 126K / 20K) but is compiled with `-mfpu=fpv4-sp-d16 -mfloat-abi=hard` (Makefile:169).** It has never been built here, and this is suspect.
- There is no `memory` target. `arm-none-eabi-size` runs after the link (Makefile:509).
- The hex is `Build/<T>/<T>-2.0.1.hex`. There is no PROJECT variable and no plutoide.ini.
- driver.sh does `cd` to the MagisV2 root (driver.sh:22) and hard-codes the MagisV2 targets, so Magis needs its own build step. A warning baseline must be captured before the change.

5. Hardware and behaviour differences
- Sensors: MPU6500 + MS5611 + AK8963 on Magis, against ICM20948 + ICP10111 on PRIMUS_X2_v1.
- Defaults that match MagisV2: looptime 3500, deadband 40, fast_change 0, PIDVEL 120/45/1.
- ALT_HOLD is set through `Mode_BARO` configAUX (`API/RxConfig.cpp:91`). The default AUX channel was not checked.
- No target.h constant overrides the hard-coded numbers. They are tuned for MagisV2 hardware.
- Today the Magis flip gets 120 cm/s. If 2a8d59a is ported without c569c0f, the flip breaks, so these two must go together.

6. Constraints and conflicts
- Flash/RAM headroom: unknown until a baseline build.
- Magis `docs/` is upstream Cleanflight. The only related files are `development/PID Internals.md` and `Modes.md`. There is no CHANGELOG.
- `src/test/unit/altitude_hold_unittest.cc` exists. Do not use it.

7. Options
- **(a) Patch-apply** commit by commit: 2a8d59a has about 11% clean hunks. Not viable except for the pid.cpp/pid.h part of a7f398d.
- **(b) Hand-port, recommended:**
  - Files: altitudehold.cpp (about 250 lines: shaping, flip bypass, hand-back), mw.cpp (throttleRaisedSinceArm plus the freeze), pid.cpp/pid.h (taken as-is).
  - Cost: RAM about 40 B. Flash about 1-1.5 KB. Loop cost negligible (100 Hz).
  - API signatures are unchanged. Altitude and take-off behaviour change.
- **(c) Replace altitudehold.cpp wholesale** with the MagisV2 version: rejected. It drags in Kalman filtering, laser fusion and the VL53L1X code, which Magis does not have.

Constraints / invariants that apply:
- FLIGHT_INVARIANTS "landing must not go through the stick limits", "flip bypasses alt-hold" and "stick-down does not zero the I-term" all carry over.
- The rcDataPilot invariant does not apply. Magis has no override path, and rcData is the pilot.
- No float doubles. Use sqrtf and fabsf.

Resource conflicts: none.

Already answered by the code: both Magis targets have the flip. The controller in use is pidRewrite. pid.cpp is byte-equivalent to MagisV2. Landing already flies -75 cm/s. Take-off arms through `mwArm`, so altHoldGroundIdle stays false and the goal profile runs.

Questions for the user:
1. Targets: PRIMUSX only, or PRIMUSV3R too? Does V3R build or fly at all?
2. Port 4657cbf's hover-trim offload and the deadband removal as well?
3. Raise PIDALT 100 to 128, with an EEPROM bump that wipes saved settings?
4. Do the Magis targets still fly brushed motors with these hover throttles? This affects the +/-300 errorVelocityI clamp.
5. Where should this be recorded: a MagisV2 topic folder name, or a commit and branch in Magis?
6. New Magis FW_Version (2.0.1 to ?) and API_Version?
7. Is there a flight-test path on Magis: PlutoMonitor logs, or the same app flip and AUX3 flow?

Doc drift: not applicable. MagisV2 code is unchanged.

Prior work: MagisV2 topics `althold-setpoint-shaping/`, `flip-althold-regression/`, `payload-tilt-on-descent/`, `altitude-hold/`. There is no topic for Magis.

Suggested skills:
- pluto-rules: its MCU and flight rules apply. Its file paths do not.
- cpp-pro, for the altitudehold port.
- pluto-reviewer, given the Magis diff and paths explicitly.
- pluto-flighttest.
- pluto-build does not apply. Use a manual make with the PATH set as above.

Pipeline docs to update at the end: none in MagisV2. Record the result in the new topic and the Magis commit message.

---

Answers given in the interview ( 20 Nov 2026 ): 1 PRIMUSX only. 2 yes to both. 3 yes, EEPROM 106 -> 107.
4 brushed, clamp kept. 5 everything inside Magis, branch `althold-backport`. 6 FW 2.1.0, API 1.0.1.
7 PlutoMonitor logs, app flip and ALT_HOLD switch, off-centre payload all available.
