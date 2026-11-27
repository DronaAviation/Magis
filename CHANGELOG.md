# Changelog

All notable changes to Magis firmware are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [v2.1.0] - 2026-11-22

Brings the MagisV2 altitude-hold work to Magis ( PRIMUSX ). The throttle stick now moves the altitude
target smoothly instead of switching to raw climb-rate control; commanded heights such as the app
take-off are flown on a profile; the app flip, which this shaping would otherwise stall, bypasses it and
returns to the height it was sent from; a full stick-down in flight no longer drops the attitude trim
that holds an off-centre payload; and altitude is measured from where the craft armed rather than from
where it booted. Ported by hand from MagisV2 `4657cbf`, `2a8d59a`, `c569c0f` and `a7f398d`; flight-tested
on PRIMUSX.

**After flashing, every setting is reset** ( `EEPROM_CONF_VERSION` 106 -> 107 ): recalibrate the
accelerometer and the compass, on a level surface, before flying.

### Changed

- **Firmware version** 2.0.1 -> 2.1.0; **API version** 1.0.1 -> 1.0.2 ( behaviour only, no signature
  change: see *API behaviour changes* below ). Flash +1 384 B, RAM +96 B on PRIMUSX
  ( 77.5 KB of 252 KB, 9.1 KB of 40 KB ).
- **Altitude hold: the throttle stick moves the target, it no longer replaces the position loop**
  ( MagisV2 `2a8d59a` ). The climb rate rises from 0 at the edge of the deadband to 40 cm/s up and
  30 cm/s down at full stick, ramped at 100 cm/s^2, and the target is held within 50 cm of the estimate
  so it cannot run away from a craft that is not keeping up. Centring the stick lets the target coast to
  a stop instead of snapping it to the current estimate, which overshot. Previously the stick commanded a
  raw rate of up to +120 / -100 cm/s.
- **Commanded heights are flown on a profile**: the app take-off and
  `DesiredPosition.set / setRelative ( Z )` climb at up to 60 cm/s and descend at up to 30 cm/s, braking at
  80 cm/s^2 to stop on the target. ( The MSP waypoint writes to the altitude target are behind `GPS`,
  which PRIMUSX does not define, so they are not in this build. ) A 120 cm take-off takes about 2.5 s. Moving the stick cancels the goal.
- **The velocity demand** is the profile rate plus the position correction, clamped per source and
  slewed at 150 cm/s^2, replacing the fixed +/-300 cm/s clamp.
- **Landing keeps its own descent**: `( landThrottle - 1500 ) / 4` with a 100 cm/s clamp, -75 cm/s by
  default, never limited by the 30 cm/s stick cap. Routed through the stick limits, a slow descent
  hovers just above the floor and touchdown is never detected.
- **The altitude controller is held in reset on the ground** ( disarmed, or armed on the stick with the
  motors at idle ), so waiting on the ground no longer winds the velocity integrator down and delays the
  take-off.
- **Altitude is now measured from where the craft armed**, not from where it booted ( MagisV2 `4657cbf` ).
  The barometer ground reference tracks the reading while the craft is disarmed and freezes on arm.
  Previously a craft that had sat on the ground for a while could start a flight 0.5 m off zero.
- **Altitude P gain 100 -> 128** ( unity: a 10 cm error asks for 10 cm/s ), in both the stored defaults and
  `PIDProfile.setDefault ( )`. This is why the settings reset.
- **The 5 cm position deadband is removed**: it subtracted 5 cm from every error, so a 10 cm sag read as
  5 and caused a limit cycle.
- **Hover-trim offload**: while settled, the hover trim moves from the velocity integrator into the
  baseline throttle one count at a time, so the sustainable hover throttle is no longer capped as the
  battery sags.
- **The altitude estimator's tilt test uses the right units** ( `> 300` deci-degrees, i.e. 30 deg ). It
  read `> 30`, i.e. 3 deg, so the slow estimator filter ran in almost all normal flight.
- **The altitude ceiling margin** is 25 cm below `max_altitude` instead of 50.

### Fixed

- **The app flip under ALT_HOLD** ( MagisV2 `c569c0f` ). With the stick shaping above, the flip's
  full-throttle climb would have been capped at 40 cm/s, never reached the 100 cm/s the flip waits for,
  and been abandoned when its climb state timed out after 2.2 s, leaving the craft level in altitude hold
  with no flip. ( A flip whose *rotation* stalls is the case that disarms in the air, from the later
  timeouts in `flight/acrobats.cpp`. ) While a flip runs, altitude hold now flies its
  throttle as a raw rate ( up to 120 / 100 cm/s ) with no shaping; on exit it restarts the setpoint,
  restores the pre-flip hover trim, holds the integrator 500 ms while the exit climb brakes, ignores the
  stale throttle for 100 ms, and flies back to the height the flip was sent from.
- **A full stick-down in flight no longer leans the craft toward a payload** ( MagisV2 `a7f398d` ). Below
  `mincheck` the attitude I-terms were zeroed on every RX frame, and in ANGLE mode the rate I-term is the
  only thing holding an off-centre centre of gravity, so the craft leaned 2-7 deg toward it. While
  airborne, a low stick now **freezes** the rate I-term instead; on the ground, at arming, in ground idle
  and when disarmed it is reset as before.
  **Where it applies:** in ALT_HOLD, and after a chuck-arm or an app arm. After an ARM-switch arm in
  THROTTLE mode the stick-down still resets the I-term, because only altitude hold clears
  `isThrottleStickArmed`. MagisV2 has the same limitation.

### Safety: where this release goes beyond MagisV2

Each was found while porting or in review, and each is a deliberate difference from the MagisV2 code:

- **The barometer ground reference also stays frozen for 3 s after any disarm**, not only while armed.
  Magis can disarm in the air without flagging a crash ( a flip whose rotation stalls, or the ARM switch
  turned off ) and then re-arm during the fall through the chuck-arm; tracking during that fall would carry up to
  about 80 cm of it into the next flight's datum. MagisV2 gates on armed alone.
- **A flip sent during a goal remembers the goal**, not the moving target, so repeated flips cannot
  ratchet the height upward. MagisV2 stores the moving target.
- **The flip's return-to-height floor guard** measures against the ground reference rather than a bare
  20 cm, so a flip sent while armed on a raised surface cannot fly the craft back down onto it.
- **A NaN or absurd barometer reading can no longer latch** in the ground reference for the rest of the
  power cycle.
- **The `alt_hold_fast_change` mode** ( off by default ) no longer flies the craft back down to the old
  target after a stick climb, which MagisV2 still does in that mode, and it now idles the motors and holds
  the controller in reset while stick-armed on the ground as the default mode does. That mode remains
  otherwise unimproved: its climb still winds the velocity integrator down, so the craft sinks after
  centring until it recovers, and its landing is open-loop.

### API behaviour changes ( no signature change )

- `getEstAltitude ( )` and the MSP altitude reply now read relative to the **arm point** rather than boot.
  While disarmed they settle toward 0 over a few seconds.
- `DesiredPosition.get ( Z )` returns the **moving target** while a commanded height is being flown, not
  the commanded height itself.
- `DesiredPosition.set / setRelative ( Z )` is flown on a profile rather than stepped, and a height set
  while the craft is stick-armed at ground idle is **discarded** ( the controller pins the target to the
  estimate until the throttle is raised ). Previously it was kept.
- `DesiredVelocity.get ( Z )` still returns the stick or landing climb-rate request, but its range is now
  +40 / -30 cm/s at full stick instead of +120 / -100, and it reads 0 whenever ALT_HOLD is off rather than
  holding its last value.
- `PIDProfile.setDefault ( )` sets the altitude P gain to 128 instead of 100.

### Operating notes

- **Leave about 10 s between a landing and the next arm.** The ground reference is held for 3 s after a
  disarm and then settles over a few seconds; an immediate re-arm starts with part of the previous
  flight's error in the absolute heights ( the ceiling, absolute user heights ). Control is unaffected.
- A user RC throttle override now drives the shaped target. Note that in Magis a user RC override does
  not expire ( see Known issues ).
- **The learned hover baseline survives a disarm and a battery change.** `offloadHoverTrim ( )` moves the
  hover trim into the baseline throttle, which is only reset when ALT_HOLD is switched on again ( to 1500 )
  or by `AltRst ( )`. After swapping to a fresh pack, switch ALT_HOLD off and on again, or expect the first
  take-off to start on the old pack's sagged baseline.

### Validation

Flown on PRIMUSX, logs in `docs/active-development/magis-althold-backport/`:

- The take-off profile ( +120 cm at 60 cm/s in 2.5 s ), the 40 / 30 cm/s stick caps, centring without a
  snap, and the -75 cm/s landing.
- The I-term freeze held the rate I-term **exactly constant** through full stick-downs of 18.6 s, 5.5 s and
  6.7 s, and correctly stayed off before a throttle raise; the payload lean was 0.19-0.41 deg.
- The ground reference stayed exactly constant through each armed flight.
- The app flips completed their rotation and returned to height, including a flip sent during the
  previous one's return.
- A barometer temperature / throttle compensation, as MagisV2 has, was **measured and not adopted**: over a
  10-11 C warm-up the pressure closed within 2 Pa, so on this sensor there is no effect to correct.

Limitations: the flips were judged by eye ( the telemetry link does not survive a flip ) and on a small
sample of one or two; the lean figures were measured against a moving rather than settled baseline and
carry about +/-0.2 deg; PRIMUSV3R was not built or flown.

### Known issues ( pre-existing, not addressed in this release )

- If the craft is armed with the ARM switch in throttle mode and ALT_HOLD is then switched on with the
  stick at or below centre, the throttle drops to idle ( `isThrottleStickArmed` is only cleared inside
  altitude hold ).
- A user RC override ( `RcCommand_Set` ) never expires: `resetUserRCflag ( )` is never called.
- The flip's SLOWDOWNANDEXIT state has no timeout on one branch.
- Landing touchdown is detected by impact or the 30 s timeout, not by the velocity test, which the velocity
  estimate does not satisfy during a landing.
- The double overload of `Monitor.print` prints wrong digits after the first decimal.
- `AltRst ( )` sets the baseline throttle to 1600 rather than 1500, so it can overwrite a learned hover
  baseline. It runs while disarmed after a large vertical velocity, and once on the first estimator tick
  after boot, so a first flight with ALT_HOLD already on starts from a 1600 baseline.
- PlutoIDE 3.0.2 cannot flash the `PRIMUSX-<version>.hex` this Makefile writes; copy it to
  `PRIMUSX_<version>.hex` first.

## [v2.0.1]

Earlier releases are recorded in the git history.
