# magis-althold-backport

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [DESIGN](DESIGN.md) · [TESTING](TESTING.md)

| | |
|---|---|
| **Status** | **Closed - pending commit** ( 22 Nov 2026 ) |
| **Mode** | feature ( backport of settled MagisV2 changes ) |
| **Repo / branch** | Magis, `althold-backport` off `master` ( HEAD `2e55152` ). The user makes every commit. |
| **Target** | PRIMUSX ( F303, 252 K / 40 K ). PRIMUSV3R out of scope. |
| **Versions** | FW 2.0.1 -> **2.1.0**, API 1.0.1 -> **1.0.2** ( behaviour only, no signature change; decided in task 10 ). |
| **Reference** | MagisV2 commits `2a8d59a`, `c569c0f`, `a7f398d` ( and `4657cbf` ); MagisV2 topics `althold-setpoint-shaping/`, `flip-althold-regression/`, `payload-tilt-on-descent/`, `altitude-hold/`; MagisV2 `fw-architecture-pipeline/subsystems/Altitude_Hold_Estimator.md`, `Core_PID_Controller.md`. |

## Summary

Magis still runs the pre-shaping altitude hold: outside the deadband the throttle stick sets a raw
climb rate `( thr - 1500 ) / 4` clamped to -100 / +120 cm/s, centring snaps `AltHold` to `EstAlt`,
the position loop has a 5 cm subtracting deadband, and a full stick-down resets the attitude
I-terms on every RX frame ( `mw.cpp:789-792` ). MagisV2 fixed all of this in three commits. This
topic hand-ports them, function by function ( patch apply fails on about 90 % of hunks: the Magis
files are CRLF and `altitudehold.cpp` has drifted +454 / -253 lines from the MagisV2 base ).

What goes in:

1. `4657cbf` prerequisites: `throttleRaisedSinceArm` ( set in `annexCode` while `ARMED` and the
   throttle is above `mincheck`, cleared in `mwDisarm` ), removal of the 5 cm position deadband,
   `offloadHoverTrim`, P_ALT 100 -> 128 with an `EEPROM_CONF_VERSION` bump ( 106 -> 107 ).
2. `2a8d59a` setpoint shaping: stick moves the setpoint at 0 .. 40 up / 30 down cm/s ramped at
   100 cm/s^2, leashed 50 cm from `EstAlt`; external `AltHold` writes ( take-off, `setAltitude`,
   `setRelativeAltitude`, MSP ) become a goal flown at 60 / 30 cm/s braking at 80 cm/s^2; velocity
   demand = profile rate + P_ALT x error, slewed at 150 cm/s^2; landing keeps its own descent;
   controller held in reset while disarmed or stick-armed at idle; `velocityControl` removed.
3. `c569c0f` flip: while `flipState` is set the flip throttle is flown as a raw rate ( 120 / 100
   cm/s clamp, no ramp / slew / leash ); on exit the setpoint resets, `errorVelocityI` is restored,
   the integrator is held 500 ms, the throttle ignored 100 ms, and the pre-flip `AltHold` ( >= 20 cm )
   is flown back to on the goal profile.
4. `a7f398d` I-term: below `mincheck` while airborne ( `ARMED && throttleRaisedSinceArm &&
   !isThrottleStickArmed`, controller `pidRewrite` ) the rate I-term is frozen, not zeroed.

## Gotchas

- `2a8d59a` without `c569c0f` breaks the app flip ( the flip's climb waits for 100 cm/s and the
  shaped cap is 40 ). Both ship in the same build; never flash a build with only task 3 done.
- The EEPROM bump wipes every saved setting on first boot: recalibrate the accelerometer and compass.
- Magis files are CRLF; keep them CRLF. Magis's `landThrottle` is a fixed 1200 ( about -75 cm/s ),
  which stays the landing descent under the new 100 cm/s clamp.
- `API/Control.cpp:259` `getSetVelocity` reads `setVelocity`, which holds the stick / landing request
  as it does in MagisV2, so `DesiredVelocity.get ( Z )` keeps its meaning under API 1.0.1. The final
  clamped and slewed demand lives in `altVelTarget` and the controller's return value.
- Build: toolchain is not on PATH. `PATH="/c/PlutoIDE/tools/ARM GNU ToolChain/bin:/c/PlutoIDE/tools/CYGWIN 64/bin:$PATH" make TARGET=PRIMUSX`
  from the Magis root. Hex: `Build/PRIMUSX/PRIMUSX-<FW_Version>.hex`. No `memory` target: read
  `arm-none-eabi-size` output after the link. The MagisV2 `pluto-build` driver does not apply.
- Gate: `python -I ../tools/warnings.py --baseline docs/active-development/magis-althold-backport/warnings-baseline.json check build.log`
  from the Magis root ( baseline from task 1: 899 `src/` warnings, flash 76.2 KB, RAM 9.0 KB ). Rollback image
  `PRIMUSX-2.0.1-rollback.hex` sits in this folder ( git-ignored ).

## Open items

- PlutoIDE flashing of a Magis build: the Makefile names the hex `PRIMUSX-<ver>.hex`; confirm PlutoIDE
  accepts it ( `plutoide.ini` project `Magis` ) before the bench check of task 7.
- Constants are the MagisV2 values ( tuned on ICM20948 + ICP10111 ); Magis has MPU6500 + MS5611.
  Tune only if the flights of task 8 demand it.
- PRIMUSV3R: F103 ( M3 ) compiled with hard-FPU M4 flags ( Makefile:169 ); never built here.
  Out of scope; a later topic.

- Pre-existing defect for a separate topic ( found in the task 3 review ): a stale
  `isThrottleStickArmed` cuts the motors if ALT_HOLD is switched on in flight after an ARM-switch
  arm with the stick low ( `io/rc_controls.cpp:175`, cleared only in BARO / SONAR mode and
  `mwDisarm` ). Not caused by this port.

- Pre-existing, for a separate topic ( found in the task 5 review ): a user RC override never expires
  in Magis. `resetUserRCflag ( )` ( `API/API-Utils.cpp:155` ) has no caller, where MagisV2 calls it
  every loop. With setpoint shaping a one-shot `RcCommand_Set ( THROTTLE, x )` therefore moves the
  altitude setpoint for as long as Developer Mode runs.
- Also pre-existing, same topic ( task 4 review ): `acrobats.cpp` SLOWDOWNANDEXIT has no timeout on
  its fall-through branch, so a stuck flip state would keep the altitude-hold flip bypass active.

- **Flashing**: PlutoIDE cannot see the Makefile's `PRIMUSX-<ver>.hex` ( it wants an underscore, and
  that hyphen name is master's own, from `2e55152` ). By the user's choice the Makefile is unchanged;
  copy the hex to `PRIMUSX_<ver>.hex` after every build and `cmp` the two before flashing, or a stale
  image gets flashed. Commands in [TESTING.md](TESTING.md).

- **Barometer measured and ruled out** ( task 13, log-2 / log-3 ): pressure closed within 2 Pa over
  75-78 s on both the branch and untouched master while the die warmed 10-11 C, with fitted slopes of
  -0.48 and +0.56 Pa/degC, i.e. noise, against MagisV2's -2.2 on its ICP-10111. Porting `4657cbf`'s
  compensation is **not** justified here and task 15 is **dropped**; the ground freeze ( task 14 ) still
  goes in. If drift ever returns, the experiment is a bench soak ( stationary, motors off, from cold ),
  not another flight: in a flying log the pressure signal is dominated by the craft's own altitude.
- **The 1.9 m drift of log-1 did not reproduce** on either build ( closure -15 cm branch, -12 cm
  master ). Attributed to that flight's accelerometer calibration, the first boot after the settings
  wipe. Every flight keeps 10 s at rest before arming and after landing to watch for its return.

- **Leave about 10 s between a landing and the next arm** ( task 14 ): the ground zero is held for 3 s
  after a disarm and then works the previous flight's error off with a ~2 s constant, so an immediate
  re-arm starts with a residual in the absolute heights.

## Next actions

`/pluto-task next` with this folder's path ( `Magis/docs/active-development/magis-althold-backport/` ).
