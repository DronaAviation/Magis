# magis-althold-backport - Design

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [DESIGN](DESIGN.md)

The behaviour is settled by the MagisV2 work ( `2a8d59a`, `c569c0f`, `a7f398d`, their topics and
`Altitude_Hold_Estimator.md` ). The design choice here is only **how to carry it into Magis**.

## Starting point ( scout, 20 Nov 2026 )

- `Magis:src/main/flight/pid.cpp` / `pid.h` are byte-equivalent to MagisV2 apart from whitespace
  and CRLF line endings. `acrobats.cpp` matches MagisV2 at `2a8d59a^` apart from formatting.
- `altitudehold.cpp` differs from MagisV2 at `2a8d59a^` by +454 / -253 lines ( `-w` ), `mw.cpp` by
  +123 / -271, `command.cpp` by +60 / -86.
- All Magis sources are CRLF. A raw `git apply --check` fails on every hunk; after converting the
  patches to CRLF, `2a8d59a` fails 7 of 8 hunks, `c569c0f` 4 of 7 ( the 3 passing only at fuzz 3 ),
  `a7f398d` applies cleanly to `pid.cpp` / `pid.h` and fails 1 of 4 in `mw.cpp` ( the rest at fuzz
  2-3, since the `throttleRaisedSinceArm` context is missing ). `--3way` cannot help: the repo lacks
  the base blobs.
- Magis lacks `throttleRaisedSinceArm`, `offloadHoverTrim`, `landingInProgress`,
  `pidSetGyroITermFreeze`, the Kalman VelocityZ / EstAlt filtering ( `98c0be3` ), the laser fusion
  and the confirm-throttle landing ramp. Its `landThrottle` is a fixed 1200.

## Options

| | (a) Patch-apply commit by commit | (b) Hand-port, function by function | (c) Replace `altitudehold.cpp` wholesale |
|---|---|---|---|
| How | Convert each patch to CRLF, `git apply --3way`, resolve conflicts | Re-implement each change against Magis's own functions, keeping its names and CRLF; take `pid.cpp` / `pid.h` as-is | Copy MagisV2's current `altitudehold.cpp` and adapt its includes |
| Carries over cleanly | ~11 % of `2a8d59a`, ~40 % of `c569c0f` ( unreliable fuzz ), `pid.*` of `a7f398d` | n/a: everything is re-typed against the MagisV2 diff and docs | File compiles only after removing Kalman, laser fusion, VL53L1X, landing-ramp code |
| Files | `altitudehold.cpp`, `mw.cpp`, `pid.cpp`, `pid.h`, `config.cpp` | same | same plus every header the new file pulls in |
| Flash / RAM / loop | about +1-1.5 KB / +40 B / negligible ( 100 Hz ) | same | larger, unknown until stripped |
| Safety | Conflict resolution by hand in the altitude loop: high chance of a silently wrong merge | Each function re-read against the MagisV2 CHANGES.md; reviewable step by step | Carries code paths never flown on Magis hardware |
| API | `getSetVelocity` keeps reading `setVelocity`; no signature change | same | same |
| Risk | High: fuzz-matched hunks land in the wrong place | Medium: transcription error, caught by review and flight | High: scope creep into estimator changes |

## Choice

**(b) Hand-port.** Confirmed by the user on 20 Nov 2026. The `pid.cpp` / `pid.h` part of `a7f398d`
is the one piece that could be patch-applied, but it is 15 lines and is typed in with the rest so the
whole diff has one provenance.

Port order ( each step builds ): prerequisites ( `throttleRaisedSinceArm`, P_ALT, EEPROM, deadband,
hover trim ) -> setpoint shaping -> flip bypass and return -> I-term freeze. The shaping and the flip
bypass are never flashed apart.
