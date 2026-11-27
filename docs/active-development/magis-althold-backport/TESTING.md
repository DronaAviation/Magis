# magis-althold-backport - Testing

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [DESIGN](DESIGN.md) · [TESTING](TESTING.md)

## Build gate method ( all builds in this topic )

From the Magis root, Git Bash:

```bash
export PATH="/c/PlutoIDE/tools/ARM GNU ToolChain/bin:/c/PlutoIDE/tools/CYGWIN 64/bin:$PATH"
make TARGET=PRIMUSX clean && make TARGET=PRIMUSX > build.log 2>&1
python -I ../tools/warnings.py --baseline docs/active-development/magis-althold-backport/warnings-baseline.json check build.log
```

`check` exits 1 on any new `src/` warning ( fingerprint file + flag + message with a count, line
numbers ignored ). The baseline is this topic's `warnings-baseline.json`, never MagisV2's
`tools/warnings-baseline.json`, and is never regenerated to hide a new warning. `-I` keeps
`tools/warnings.py` from shadowing the standard `warnings` module.

Warnings in a header count once per file that includes it ( `flight/pid.h` carries 36 ): a new
`#include` of such a header raises the count with no new code and must be justified or avoided.

## Task 1: baseline ( 2026-11-20 )

| | |
|---|---|
| Tree | Magis `master` `2e55152`, untouched, branch `althold-backport` created from it |
| Toolchain | Arm GNU Toolchain 14.2.Rel1 ( gcc 14.2.1 ), GNU Make 4.4.1 ( PlutoIDE Cygwin ) |
| Build | `make TARGET=PRIMUSX`, clean tree, exit 0, 43 s |
| Image | `Build/PRIMUSX/PRIMUSX-2.0.1.hex` ( 219 493 B ), ELF `MAGIS_PRIMUSX.elf` |
| Size | text 76 872, data 1 140, bss 8 116 |
| Flash | 78 012 B = 76.2 KB of 252 KB ( 30.2 % ) |
| RAM | 9 256 B = 9.0 KB of 40 KB ( 22.6 % ) |
| Warnings | 2 361 parsed: `src/` 899, `lib/` 1 462 ( 2 369 raw lines incl. linker / LTO notes ) |
| Top flags | `-Wconversion` 944, `-Wsign-conversion` 805, `-Wregister` 130, `-Wcomment` 97, `-Wfloat-conversion` 60, `-Wshadow` 57 |
| Rollback | `PRIMUSX-2.0.1-rollback.hex` in this folder, sha256 `c25c450e...bcea46`, identical to the build; ignored by this folder's `.gitignore` |

Baseline `src/` warnings in the files the port touches:

| File | Warnings |
|---|---|
| `flight/altitudehold.cpp` | 36 |
| `flight/altitudehold.h` | 0 |
| `mw.cpp` | 25 |
| `mw.h` | 0 |
| `flight/pid.cpp` | 54 |
| `flight/pid.h` | 36 |
| `config/config.cpp` | 2 |
| `flight/acrobats.cpp` / `.h` | 0 |

Full list: [BASELINE_WARNINGS.txt](BASELINE_WARNINGS.txt). Machine baseline: `warnings-baseline.json`.

Line endings: the index stores LF, the working tree is CRLF through the global `core.autocrlf=true`
( `C:/DevTools/Git/etc/gitconfig` ); there is no `.gitattributes`. Commits normalise either way;
edits keep CRLF so the working-tree diff stays readable.

## Task 2: prerequisites build ( 2026-11-20 )

| | |
|---|---|
| Build | `make TARGET=PRIMUSX`, clean tree, exit 0, 38 s |
| Gate | `no new warnings in src/` against the task-1 baseline |
| Size | text 76 872, data 1 140, bss 8 116: unchanged ( LTO removes `throttleRaisedSinceArm`, written but not yet read; task 5 adds the reader ) |
| Image | `PRIMUSX-2.0.1.hex` sha256 `f0862599...` differs from the rollback `c25c450e...` ( P_ALT 128, EEPROM 107 ) |

Re-build after the review fix ( `Control.cpp` ALT P default, comment wording ): exit 0, gate clean,
size again unchanged at 76 872 / 1 140 / 8 116.

`git diff --check` reports "trailing whitespace" for every added line in
[Control.cpp](../../../src/main/API/Control.cpp): that file is stored **CRLF in the index** ( 33 of
315 `src/main` files are, the rest LF ), so the `CR` shows up in the raw diff on context and added
lines alike. The added lines match the file's own endings; the warning is a property of the file, not
of the change.

## Task 3: setpoint shaping build ( 2026-11-20 )

| | |
|---|---|
| Build | `make TARGET=PRIMUSX` after `clean`, exit 0 |
| Gate | `no new warnings in src/` |
| Size | text 77 720, data 1 140, bss 8 140 |
| Delta vs task 2 | flash +848 B, RAM +24 B |
| Flash | 78 860 B = 77.0 KB of 252 KB ( 30.6 % ) |
| RAM | 9 280 B = 9.1 KB of 40 KB ( 22.7 % ) |
| Line endings | both changed files pure CRLF; `git diff --check` clean on `src/main/flight/` |

**Not flyable yet.** The shaped 40 cm/s stick cap stalls the app back-flip, which waits for 100 cm/s
of climb and times out at 2.2 s ( `flight/acrobats.cpp` ). Task 4 ( MagisV2 `c569c0f` ) fixes that; do
not flash this build for a flip test.

Re-build after the three review fixes: exit 0, gate clean, size unchanged at 77 720 / 1 140 / 8 140.

New log field for the bench and flight tests: `altholdDebug3` is now 1 while the goal profile is
running ( take-off, `setAltitude`, MSP ) and 0 otherwise. `altholdDebug` is `AltHold`, `altholdDebug8`
the position error, `altholdDebug7` the throttle adjustment.

## Task 4: flip bypass build ( 2026-11-22 )

| | |
|---|---|
| Build | `make TARGET=PRIMUSX` after `clean`, exit 0 |
| Gate | `no new warnings in src/` |
| Size | text 77 936, data 1 140, bss 8 180 |
| Delta vs task 3 | flash +216 B, RAM +40 B |
| Delta vs the 2.0.1 baseline | flash +1 064 B, RAM +64 B |
| Flash | 79 076 B = 77.2 KB of 252 KB ( 30.7 % ) |
| RAM | 9 320 B = 9.1 KB of 40 KB ( 22.8 % ) |

The shaping and the flip bypass are now both in, so the build is flyable again in principle; task 6
is the formal gate and task 7 the bench check before anything is flown.

Re-build after the review fix ( flip-return ground reference and the goal-target snapshot ): exit 0,
gate clean, text 78 000 / data 1 140 / bss 8 188. Flash 79 140 B = 77.3 KB of 252 KB; RAM 9 328 B =
9.1 KB of 40 KB. Against the untouched 2.0.1 baseline: +1 128 B flash, +72 B RAM.

**Task 8 note for the flip tests:** send the five-plus flips with the craft settled between them, and
also deliberately test one flip sent during the return descent from the previous one ( the goal-target
snapshot above is what stops that ratcheting the height up ), and one flip sent while armed on the
ground before take-off ( the craft must hold where the flip ends, not fly back down to the surface ).

## Task 5: I-term freeze build ( 2026-11-22 )

| | |
|---|---|
| Build | `make TARGET=PRIMUSX` after `clean`, exit 0 |
| Gate | `no new warnings in src/` |
| Size | text 78 128, data 1 140, bss 8 188 |
| Delta vs task 4 | flash +128 B, RAM 0 |
| Delta vs the 2.0.1 baseline | flash +1 256 B, RAM +72 B |
| Flash | 79 268 B = 77.4 KB of 252 KB ( 30.7 % ) |
| RAM | 9 328 B = 9.1 KB of 40 KB ( 22.8 % ) |
| Symbols | `gyroITermFrozen` and `throttleRaisedSinceArm` both present in the ELF |

**Correction for the task 7 bench check:** Magis has a user RC override path
( `RcCommand_Set` -> `RC_ARRAY` / `userRCflag`, applied in `mw.cpp` `userCode ( )`: channel 3 into
`rcData [ THROTTLE ]`, roll / pitch / yaw into `rcCommand [ ]` ). So the bench test must read the
pilot stick through `rcData [ THROTTLE ]` knowing user code can write it, and should include one case
with no `RcCommand_Set` active at all, which is how the flight tests of task 8 are flown.

## Task 6: build gate ( 2026-11-22 )

The formal gate on the finished code ( tasks 2-5 ), from a clean `Build/`.

| | |
|---|---|
| Build | `make TARGET=PRIMUSX` after `clean`, exit 0, 48 s |
| Gate | `no new warnings in src/` ( `check -v`: nothing new in `lib/` either ) |
| Warnings | 2 361 total, `src/` 899, `lib/` 1 462: **identical to the task-1 baseline**. About 400 changed lines added no warning and removed none. |
| Size | text 78 128, data 1 140, bss 8 188 |
| Flash | 79 268 B = 77.4 KB of 252 KB ( 30.7 % ) |
| RAM | 9 328 B = 9.1 KB of 40 KB ( 22.8 % ); 31.6 KB left for the stack |
| Delta vs the 2.0.1 baseline | flash +1 256 B ( planned +1-1.5 KB ), RAM +72 B ( planned +40 B ) |
| Stale objects | `make TARGET=PRIMUSX -n` after the build compiles nothing |
| Image | `Build/PRIMUSX/PRIMUSX-2.0.1.hex` ( the version moves to 2.1.0 in task 11 ) |

Per task: task 2 +0 ( LTO dropped the unread flag ), task 3 +848 / +24, task 4 +280 / +48, task 5 +128 / 0
( flash / RAM, bytes ).

### PlutoIDE cannot flash this hex ( pre-existing, blocks task 7 )

Read from the installed extension, `drona-aviation.pluto-ide-3.0.2`, `src/services/bootstrap.js:72-82`:
PlutoIDE builds with `make PROJECT=<project_name> TARGET=<target>` ( :450, :582 ) and then flashes
`Build/<TARGET>/<project_name>_<TARGET>_<FW_Version>.hex`, falling back to the legacy
`Build/<TARGET>/<TARGET>_<FW_Version>.hex`. With `plutoide.ini` project `Magis` those are
`Build/PRIMUSX/Magis_PRIMUSX_2.0.1.hex` and `Build/PRIMUSX/PRIMUSX_2.0.1.hex`.

The Magis Makefile has no `PROJECT` variable and writes `Build/PRIMUSX/PRIMUSX-2.0.1.hex`, with a
**hyphen**, which matches neither, so PlutoIDE stops with `Firmware not found: PRIMUSX_2.0.1.hex`.
That name came in with master's own latest commit `2e55152` ( it replaced `Experience_PLUTOX.hex` ),
so this affects every current Magis build, not just this branch. MagisV2's Makefile has
`PROJECT ?= DEFAULT` and `$(PROJECT)_$(TARGET)_$(FW_Version).hex`. Recorded as task 12.

## Task 12: a PlutoIDE-flashable hex, by renaming only ( 2026-11-22 )

The user chose the testing-only route, so **the Makefile is unchanged**. After every build, copy the
built hex to the legacy name PlutoIDE accepts:

```bash
cd <Magis root>
V=$(grep -m1 '^FW_Version' Makefile | sed 's/.*=[[:space:]]*//')
cp "Build/PRIMUSX/PRIMUSX-$V.hex" "Build/PRIMUSX/PRIMUSX_$V.hex"   # hyphen -> underscore
```

`Build/` is git-ignored ( `.gitignore:2` ), so the copy never enters the repo. The legacy name is used
rather than `Magis_PRIMUSX_<V>.hex` because it does not depend on `plutoide.ini`'s `project_name`;
PlutoIDE tries the project-name form first and falls back to this one
( `bootstrap.js:72-82` ).

Done for the current build: `Build/PRIMUSX/PRIMUSX_2.0.1.hex`, 223 032 B, byte-identical to
`PRIMUSX-2.0.1.hex` ( `cmp` clean ).

### Flash the right file

**The copy does not update itself.** `make` only rewrites the hyphenated file, so a rebuild leaves the
underscore copy stale and PlutoIDE would flash the old image without complaining. Before every flash:

```bash
cmp "Build/PRIMUSX/PRIMUSX-$V.hex" "Build/PRIMUSX/PRIMUSX_$V.hex" && echo "copy is current"
```

Re-run the `cp` whenever that fails. Do not press PlutoIDE's own Build button between the copy and the
flash, for the same reason. After task 11 bumps the version the name becomes `PRIMUSX_2.1.0.hex`;
delete the 2.0.1 copies then, so a stale one cannot be picked.

## Task 7: bench check, props off ( 2026-11-22 )

### Instrumentation ( temporary, every line tagged `TEMP-BENCH` )

Magis user code prints with `Monitor.print ( tag, int )` / `Monitor.println` ( `API/Utils.h` ), the
equivalent of MagisV2's `Monitor_Print`. Everything is printed as an integer: Magis's double print is
the pre-`1f8e78e` code ( precision clamp 0..7, the digit-subtraction bug MagisV2 fixed ).

Three values the bench needs are file-static, so two small read-only accessors were added:
`benchGyroITerm ( axis )` and `benchGyroITermFrozen ( )` in `flight/pid.cpp`, and `benchFlags ( )`
in `mw.cpp`. The Cleanflight `debug[]` array was not used: the altitude estimator writes `debug[1..3]`
and the ADC driver all four, so it would collide.

Log line, one per `plutoLoop ( )` tick ( 100 ms ), about 120 B:

| Tag | Value | Unit |
|---|---|---|
| `n` | tick counter: steps by 2 across a lost record, by 1 across a stall | - |
| `t` | `millis ( )` | ms |
| `A` | `AltHold`, the altitude target | cm |
| `E` | `getEstAltitude ( )`, the estimate | cm |
| `V` | `VelocityZ`, estimated climb rate | cm/s |
| `S` | `setVelocity`, the stick / landing / flip request | cm/s |
| `I` | `errorVelocityI / 8192`, the hover trim in the velocity integrator | throttle counts |
| `G` | 1 while a goal is flown ( take-off, `setAltitude`, MSP, flip return ). **Only valid while `Z` bit 3 is set**: nothing writes it with BARO off, so it can read stale | - |
| `F` | `flipState`, 0 = no flip | - |
| `Z` | flags: 1 rate I-term frozen, 2 `throttleRaisedSinceArm`, 4 ARMED, 8 BARO_MODE, 16 stick-armed | bits |
| `R` / `Q` | roll / pitch rate ITerm ( `errorGyroI >> 13`, the value `pidRewrite` applies ) | - |

On the first tick after Developer Mode comes on, a `START Palt <p> t <ms>` line instead: the stored
ALT P gain, to settle whether the app overwrites the new default 128 when it connects.

Build: PRIMUSX clean, exit 0, gate clean; text 78 860 / data 1 140 / bss 8 204 ( the instrumentation
costs +732 B flash, +16 B RAM, all removed in task 9 ).

### Before the bench

1. **Props off.** The motors will spin, up to full throttle in case B8.
2. Flash `Build/PRIMUSX/PRIMUSX_2.0.1.hex` from PlutoIDE after `cmp`-checking it against
   `PRIMUSX-2.0.1.hex` ( task 12 ). sha256 of this build's image starts `275ad2b221c3c461`.
3. **First boot after flashing resets every setting** ( `EEPROM_CONF_VERSION` 107 ): recalibrate the
   accelerometer and the compass before anything else.
4. App connected over Wi-Fi, Developer Mode on ( `plutoLoop ( )` only runs then ), PlutoMonitor
   logging. Save the log as `logs/log-1.txt` in this folder.
5. Board level and still unless a case says to move it.

### Cases and what passes

| # | Do | Pass if |
|---|---|---|
| B1 | Disarmed, ALT_HOLD on, throttle stick fully down, 10 s untouched | `START` shows `Palt 128`; `Z` 8; `A` = `E`; `I` 0; `G` 0; `R` `Q` 0 |
| B2 | Arm from the app with the throttle stick low; 5 s | `Z` 28 ( armed + baro + stick-armed; if it reads 12 the app arm does not set stick-armed - note it ); motors at idle; `A` = `E`; `I` 0; `R` `Q` 0 |
| B3 | Raise the throttle stick to about three-quarters and **hold the board tilted about 10 deg** ( or hold roll stick ) for 2 s | `Z` 14 ( stick-armed clears, the latch sets ); `S` steps to about 18 ( 40 at full stick, never more ); `A` climbs gradually - the ramp - and stops at `E` + 50 at most, never jumps; **`R` or `Q` wind up to tens of counts, or to the +/-256 clamp** |
| B4 | Centre the throttle stick; 5 s | `S` 0; `A` stays where B3 left it, does **not** snap back to `E` |
| B5 | **Straight from B3, with the I-term still wound up**: keep the board tilted and pull the throttle fully down for 6 s, then back to centre | while down: `Z` 15 ( the frozen flag, 1, is set ); **`R` / `Q` stay exactly constant** at the wound-up value, not drifting and not dropping to 0; `S` -30; after centring: `Z` 14 and `R` `Q` move with the tilt again |
| B6 | Disarm from the app | `Z` 8; `R` `Q` 0; `I` 0; `A` = `E` |
| B7 | Arm again with the stick low, do **not** raise it, then tilt the board and pull the stick fully down for 5 s | `Z` 28 ( or 12 ), never odd: no latch, so the frozen flag 1 never sets; `R` `Q` fall back to 0 / -1 instead of holding - the old reset behaviour, which is correct on the ground |
| B8 | *Optional.* Armed with ALT_HOLD and MAG on, stick centred: send a flip from the app; wait for it to time out ( about 2.2 s ); then disarm | while `F` is not 0: `S` 120 during the climb; after `F` returns to 0: **`G` stays 0** ( a flip sent on the ground must not set a return goal: the task 4 ground guard ) and `I` returns to its pre-flip value |

**What the bench cannot show:** that the shaped climb is smooth in the air, the take-off profile
timing, a flip completing its rotation, or the payload holding level. Those are task 8.

### Why B3 must wind the I-term up first

`pidRewrite` integrates every 3.5 ms loop but `processRx ( )` resets at 50 Hz, so on a still, level
bench only about six loops' worth accumulates between resets: with the default `I8` 10 that is far
less than one count, and `errorGyroI >> 13` reads 0 either way. **Frozen and reset look identical
unless the I-term is first wound up.** Held at about 10 deg with the throttle up, the ANGLE
`desiredRate` is about 250, which integrates at roughly 148 counts/s and reaches tens of counts or the
+/-256 clamp ( `GYRO_I_MAX` ) within a second or two. Then the difference is unmistakable: frozen holds
the value exactly, reset drops it to 0 or -1. `Z` bit 0 shows the flag itself in either case.

### Review of the instrumentation ( `pluto-reviewer`, 0 BLOCKING, 1 SHOULD-FIX, 2 NOTE )

- **SHOULD-FIX, fixed:** some added lines carried no `TEMP-BENCH` tag ( the `extern "C"` declarations,
  the print block, two comments ), so tag-stripping would have left orphan comments behind. Every added
  line is now tagged, and stripping them restores `PlutoPilot.cpp` to its committed text exactly
  ( verified by diff; two blank lines remain, which task 9 drops ).
- **NOTE, fixed:** `setVelocity` and `VelocityZ` appear in no header, so their definitions have C++
  linkage; declaring them inside `extern "C"` was ill-formed, though it happens to link because GCC
  does not mangle globals. They are now declared outside the block, and the block's comment is correct
  for the six that really are C linkage.
- **NOTE, acted on in the plan above:** the wind-up requirement for B3 / B5.
- **Confirmed by the reviewer:** worst case **125 B** per tick against the ~130 B limit ( typical
  115-120 B; the START tick is 36 B ), with the ring also 256 B; every accessor and
  `PIDProfile.get ( )` read-only, nothing a controller reads is written; all casts explicit and no
  field able to reach `INT_MIN`; `plutoLoop ( )` runs from the main loop, not an ISR, so there is no
  race on the flags; and the two accessors link with consistent C++ linkage on both sides
  ( checked in the object files ).

### log-1 results: flown, not benched ( 2026-11-22, PRIMUSX, Wi-Fi, 82.6 s )

[logs/log-1.txt](logs/log-1.txt), parsed into a tick table ( one row per 100 ms ). **The user flew
this build with the props on** rather than running the props-off bench, so log-1 is the first flight
of the port. Pilot's observations, asked after reading the log: the craft sank 20-30 cm after the
take-off climb and came back; it was **clearly in the air** during 33-60 s; the landing touched down and
disarmed by itself.

**Log health.** 812 ticks, every one of the 12 fields present 812 times: nothing was cut and the app
stayed connected, so the 125 B worst-case line is within budget. The tick counter never skips: no record
lost. Three `START` markers: the beginning, and two Developer Mode restarts at 59.2 s and 61.3 s, each a
single 200 ms gap - the usual Wi-Fi RC-frame hiccup.

**Stored ALT P gain: `Palt 128` at all three starts.** The app does not overwrite the new default.
Settles the open assumption from task 2.

#### What passed, in flight

| Check | Log | Result |
|---|---|---|
| Arming resets the rate I-term | Disarmed with the stick centred, `R` / `Q` sat at the -256 clamp ( no stick-low reset runs while disarmed with the stick centred: pre-existing, untouched by the port ); on the arm tick they went to -1 | **Pass**: the disarmed wind-up never reaches flight. |
| Take-off goal profile ( task 3 ) | App take-off at 1.2 s: `A` from -8 to 112, exactly +120 cm, at a steady 6 cm per tick ( 60 cm/s ), braking in steps 6, 4, 3, 2, 2, 0 and stopping exactly on the goal; `G` 1 throughout, 0 on arrival; `E` followed to about 100 | **Pass**: 120 cm in **2.5 s**, matching MagisV2's measured 2.7 s ( 4.3 s before shaping ). |
| Post-take-off sag | For about 5 s after arrival `E` sat 20-35 cm below the 112 cm target, then recovered; the pilot saw a 20-30 cm dip | **Observation, not a failure**: the same class as MagisV2's braking sag at a flip exit. A tuning candidate for task 8. |
| Hover-trim offload | After arrival, `I` drained 26 -> 2 into the baseline | Ran as designed ( throttle-neutral by construction: it moves each count from the integrator to the baseline ). |
| Stick climb ( B3 ) | Near-full stick at 9.6 s: `S` 39, never above 40; `A` 112 -> 128 -> 167 -> 201, i.e. 40 cm/s, no jump; latch set | **Pass.** |
| Centring coasts to a stop ( B4 ) | Stick centred at 11.8 s with `A` 201, `E` 192: `A` stayed at 201 and `E` settled there with at most 6 cm of overshoot | **Pass**: the old code would have snapped the target to 192. |
| **I-term freeze in flight ( task 5 )** | Full stick-down 13.6-16.1 s: `Z` 15, **`R` 8 and `Q` -30 exactly constant for 2.5 s**. Again 26.4-32.2 s: **`R` 19 and `Q` -16 constant for 5.8 s**. Both resumed moving once the stick was centred | **Pass**, and both held a non-zero, wound-up value, so they cannot be mistaken for a reset. 5.8 s is close to the 6.5 s of MagisV2's payload flight. No payload was carried and attitude is not logged, so the lean itself is task 8. |
| Stick descent | `S` -30 / -29 throughout both stick-downs; `A` down at 30 cm/s | **Pass.** |
| **Landing invariant** | App land at 79.0 s: `S` **-75**, `A` descending at about 76 cm/s; touched down and disarmed itself ( pilot ) | **Pass**: the request is `( 1200 - 1500 ) / 4`, well past the 30 cm/s stick cap, so the landing never goes through the stick limits. |
| Bounds | The freeze engaged only with latch + armed + stick full down; `S` never exceeded its cap in any mode ( 40 up, 30 down, 75 landing ); no goal started without a command | No unexpected motor command or state. |

**Not exercised by log-1:** B2 ( arm with the stick low: the ground-idle reset ), B7 ( the freeze must
not engage before a throttle raise ), B8 / flips ( `F` 0 throughout ), and the leash ( `E` followed `A`,
so `A - E` never reached 50 cm ). Carried into task 8.

#### Finding: the altitude estimate drifted about 1.9 m low during the flight

The pilot's observations give the log a ground truth it otherwise lacks:

| When | `E` ( estimate ) | `V` | Where the craft really was | Estimate error |
|---|---|---|---|---|
| 0-1.1 s, on the floor, disarmed | -8 cm | -1 .. 1 | on the floor | 0 ( reference ) |
| 33-60 s, after the 5.8 s stick-down, stick centred | -38 .. -79 cm, holding `A` -54 | about 0 | **clearly in the air** ( pilot ) | at least about -0.8 m |
| 81.69 s, the disarm tick, on the same floor | **-195 cm** | -53 | on the floor, touched down ( pilot ) | **about -1.9 m** |
| 81.7-82.6 s, disarmed, motors stopped | -195 -> **-235 cm** | **-53 .. -67** | at rest on the floor | still growing |

The controller held the estimate on target throughout ( `A - E` stayed within -28 .. +38 cm ), so it
flew correctly *in its own frame*: the frame itself moved about 2 m in 80 s. The error built mainly
during the two long descents ( 26-32 s, the 5.8 s stick-down: `A` 120 -> -54; and the landing,
79-81.7 s: `A` -10 -> -174 ), not during the holds, where `E` stayed on target.

**The last row is the most telling.** With the motors stopped and the craft at rest there is no
airflow, so the barometer cannot be the whole story: the estimator's *own velocity state* carried
-55 cm/s, before the flight it read 0 at rest, and it was still integrating `E` downward a full second
after the disarm. The bias was built in flight and sits in the accelerometer path of the filter, which
the barometer corrects only slowly.

Ranked hypotheses ( none proven yet ):

1. **A vibration- or throttle-dependent vertical accelerometer bias feeding the velocity estimate.**
   Evidence for: the velocity state stayed at -55 cm/s with the craft at rest and the motors off, yet
   read about 0 at rest *before* arming. A constant offset ( for example from the accelerometer
   recalibration after the settings wipe ) does not fit that: the filter's bias state absorbs a
   constant offset at rest. A bias that appears only while the motors run does: it shifts with
   throttle, so it moves in descents, and once the motors stop the filter's learned bias is wrong in
   the other direction and decays slowly. Test: log 10 s at rest before arming and 10 s after landing
   on every flight; `V` must read about 0 at rest.
2. **The estimator tilt-threshold fix of task 3 ( `> 30` -> `> 300` deci-degrees ).** It is the one
   estimator change the port made. Before it the estimator ran its slow filter ( time constant about
   5 s ) in almost all flight; now it runs the normal one ( about 2 s ), with different correction gains
   between the accelerometer path and the barometer. MagisV2 made the same change on an ICP-10111 with
   throttle and temperature compensation and a Kalman-filtered `VelocityZ`, none of which Magis has.
   This would not create a bias, but it changes how it is corrected. Test: A/B against `> 30`.
3. **Barometer drift with no compensation**: Magis has neither the temperature nor the throttle
   compensation of `4657cbf`, and this was likely a cold start. Evidence against: the error came in steps
   during descents and kept growing with the motors off, which slow temperature drift does not explain.
4. **Pre-existing on master**: no master log with these fields exists to compare.

**Why it matters before more flying:** every altitude in the port is relative to this estimate. The
flip return goal and its 20 cm ground guard ( task 4 ), `limitAltitude ( )`, and Magis's velocity
touchdown test ( `vel > -8` after 2 s, `command.cpp` ) all assume it is roughly right. In the last 0.6 s
before the disarm `V` read -89 .. -101 cm/s, so the velocity test could not have detected this
touchdown; it disarmed by another path, most likely the impact check ( `accADC > 8500` ), which a gentler
touchdown might not trigger, leaving the 30 s timeout with the motors running on the floor. Recorded as
task 13.

## Task 13: drift attribution, branch vs master ( 2026-11-22 )

### The two builds

| Flight | Build | Hex ( in this folder, git-ignored ) | sha256 starts | `START` line reads |
|---|---|---|---|---|
| **log-2** | branch `althold-backport`, drift log | `hex-log2-branch.hex` | `188d9d3a5a2e3aa5` | `Palt128` |
| **log-3** | untouched master `2e55152`, same drift log | `hex-log3-master.hex` | `545c62781f3f8ed1` | `Palt100` |

Master was extracted with `git archive` into the session scratchpad and built there, so the branch's
working tree and history were never touched. Both builds link and run the same 11-field log; the
branch build passed its warning gate ( text 78 856 / data 1 140 / bss 8 196 ), master built clean with
no warning in the two files instrumented ( text 77 532 / bss 8 124 ).

Log fields, identical on both builds:

| Tag | Value | Unit |
|---|---|---|
| `n`, `t` | tick counter, `millis ( )` | -, ms |
| `A`, `E` | altitude target, estimate | cm |
| `V` | estimated climb rate | cm/s |
| `S` | stick / landing request | cm/s |
| `Z` | flags: 4 armed, 8 BARO_MODE, 16 stick-armed; 1 frozen and 2 latch exist only on the branch | bits |
| **`C`** | barometer die temperature, straight from the MS5611 driver | **centi-degC** ( 2534 = 25.34 C ) |
| **`P`** | raw pressure | **Pa** |
| **`H`** | `althold_throttle`: the throttle sent to the mixer | 1000-2000 |

`G`, `F`, `R` and `Q` were dropped to make room ( their checks passed in log-1 ), and `I` with them
( see below ). Worst case 120 B per tick, typical about 110 B. The tick counter wraps at 9999 so the
line cannot grow past the budget during a long session.

**`H` is the motor throttle, not the stick.** The review caught this: in ALT_HOLD the stick only sets
a climb rate, so `rcData [ THROTTLE ]` sits near 1500 through a whole hover whatever the real thrust
is, and a throttle-vs-pressure fit on it would measure nothing. `althold_throttle`
( `flight/altitudehold.h:33`, `extern "C"` on both builds ) is `initialThrottleHold +
altHoldThrottleAdjustment` as handed to the mixer, which is the signal the rotor-inflow coefficient
needs. It is only written while BARO_MODE runs, and reads 1000 during ground idle.

`I` was dropped because it is **not comparable between the two builds**: the branch offloads the hover
trim into `initialThrottleHold`, so `I` is only the residual there, while on master it carries the
whole trim. `H` gives the hover-thrust figure that can be compared.

### Before each flight

1. **Put that flight's hex at the path PlutoIDE flashes** ( it always flashes
   `Build/PRIMUSX/PRIMUSX_2.0.1.hex`, taking 2.0.1 from the open Makefile ). From the Magis root:

   ```bash
   T=docs/active-development/magis-althold-backport
   cp "$T/hex-log2-branch.hex" Build/PRIMUSX/PRIMUSX_2.0.1.hex   # log-2 ( done already )
   cp "$T/hex-log3-master.hex" Build/PRIMUSX/PRIMUSX_2.0.1.hex   # log-3
   sha256sum Build/PRIMUSX/PRIMUSX_2.0.1.hex                      # check against the table
   ```

2. **Flash, then recalibrate the accelerometer and the compass.** Every switch between the two builds
   resets all settings ( `EEPROM_CONF_VERSION` 107 on the branch, 106 on master ). Calibrate on a surface
   you have checked is level: the leading suspect is an accelerometer bias.
3. **Start each flight cold-ish and the same way**: battery in, wait the same time ( say 2 min ) before
   logging, so the two flights start at comparable temperatures.
4. Props on, app on Wi-Fi, Developer Mode on, PlutoMonitor logging into `logs/log-2.txt` or
   `logs/log-3.txt`. Check the first line reads the `Palt` value of the build you meant to fly.

### The flight ( identical on both builds )

| Phase | Do | For |
|---|---|---|
| 1 | **10 s on the floor, disarmed, untouched** | `V` at rest before the motors run; the starting `P` and `C` |
| 2 | Arm and take off with the app ( ALT_HOLD on ) | same take-off on both builds |
| 3 | **Hover 60 s, sticks centred** | drift with no stick input: `E` vs `P` vs `C` |
| 4 | **Throttle fully down for 5 s**, then centre | a descent, where log-1's error grew |
| 5 | Hover 20 s, sticks centred | whether the error stays after the descent |
| 6 | Land with the app | the touchdown error: `E` at touchdown vs at take-off, same floor |
| 7 | **10 s on the floor, disarmed, untouched** | `V` at rest after the motors stop ( log-1: -55 cm/s ) |

Keep it low and over a clear floor; if the craft climbs or sinks on its own by more than about 50 cm,
land with the app. Master flies the old controller: its stick climbs and descends at up to 120 / 100 cm/s
and its take-off is a step, not a profile, so phase 4 will drop faster on master; that is expected. Note
anything you see that the log cannot ( height by eye at each phase, a bump, a draught ).

### What the logs will settle

| If the logs show | It points to |
|---|---|
| `E` falls while `P` stays flat | the estimator's accelerometer path, not the barometer |
| `P` rises steadily with `C` | die-temperature drift: fit Pa per degC for task 15 |
| `P` steps with `H` ( the mixer throttle, phase 4 ) | rotor inflow: fit Pa per throttle count for task 15 |
| `V` about 0 at rest in phase 1 but not in phase 7 | a bias that only exists while the motors run |
| master and branch drift alike | pre-existing in Magis, not the port |
| master clean, branch drifts | the port caused it: next, A/B the task 3 tilt threshold |

### After both flights

Restore the branch build at the flash path, so the next flash cannot be master by accident:

```bash
cp Build/PRIMUSX/PRIMUSX-2.0.1.hex Build/PRIMUSX/PRIMUSX_2.0.1.hex && cmp Build/PRIMUSX/PRIMUSX-2.0.1.hex Build/PRIMUSX/PRIMUSX_2.0.1.hex
```

### Review of the drift log ( `pluto-reviewer`, 1 BLOCKING, 6 NOTE )

- **BLOCKING, fixed:** `H` logged `rcData [ 3 ]`, the stick, which is near-constant in ALT_HOLD, so the
  rotor-inflow fit these flights exist for would have been impossible. It now logs `althold_throttle`.
  `I` was dropped in the same edit as not comparable across the builds. Both rebuilt, branch re-gated.
- **NOTE, acted on:** the real worst case was 128 B, not the 125 B the source comment claimed, with
  only 1-2 B of margin and growing past 130 B once the tick counter passed four digits or `A` / `E`
  reached -1000 cm, which this very drift could cause. Dropping `I` and wrapping the counter at 9999
  brings it to 120 B.
- **NOTE, affects how the logs are read:** `C` and `P` are **not single samples**. `apmBaroRead ( )`
  runs about every 101.5 ms and averages the conversions in that window ( roughly 6-8 pressure, 1-2
  temperature ), so each is a ~100 ms mean. The baro clock and the 100 ms user loop drift against each
  other, so an occasional repeated or skipped `P` / `C` between ticks is normal. `P` is absolute Pa and
  `C` is centi-degC, neither ground-referenced; `E` is relative to the **boot** reference and uses the
  boot temperature, not `C`.
- **NOTE, pre-existing on both builds:** if no pressure conversion completes inside a window,
  `apmBaroRead ( )` passes uninitialised values to the formula ( `barometer.cpp:233` ), so a single
  absurd `P` sample should be treated as that, not as real pressure.
- **NOTE, comparability:** directly comparable across the two builds are `n t E V C P H` and `Z` bits
  4 / 8 / 16 ( the estimator is identical between them: the only controller difference is the `dt`
  argument ). `S` is cm/s on both but capped differently ( master -100 / +120 raw, branch -30 / +40
  shaped ), and `A` moves continuously on the branch while master freezes it and snaps it to the
  estimate, so compare `A - E` only in hold segments. `Z` bits 1 and 2 never set on master.
- **NOTE:** the master copy differs from pristine `2e55152` only by the log block and `benchFlags ( )`.

### log-2 and log-3 results ( 2026-11-22 ): the drift did not reproduce, and the barometer is not a cause

[logs/log-2.txt](logs/log-2.txt) branch ( `Palt128`, 767 ticks, 78.1 s, armed 70.7 s ) and
[logs/log-3.txt](logs/log-3.txt) master ( `Palt100`, 732 ticks, 74.7 s, armed 63.3 s ). Both flown on
the task 13 pattern after a level accelerometer calibration.

#### 1. The 1.9 m drift of log-1 did not happen on either build

Error is the estimate on the floor at the end against the estimate on the floor at the start, the same
measurement that gave -1.9 m in log-1:

| | start, at rest | end, at rest | **closure** | max in flight |
|---|---|---|---|---|
| **branch** ( log-2 ) | `E` -5 cm | `E` -20 cm | **-15 cm** | 292 cm |
| **master** ( log-3 ) | `E` -46 cm | `E` -58 cm | **-12 cm** | 221 cm |

Both are within the drone's own +/-8 cm wobble plus a few cm. The velocity estimate also settled: after
the motors stopped it decayed from about -55 cm/s to -1 cm/s within 4.5 s on the branch, and the same
on master after its touchdown transient. In log-1 it **stayed** at -55 cm/s with `E` still falling a
second after the disarm.

So the branch does not drift, and it does not drift differently from master. The difference between
log-1 and log-2 is not the firmware: it is the calibration. log-1 was the first boot after the settings
wipe, and log-2 and log-3 were flown after deliberately recalibrating the accelerometer on a surface
checked level. That matches a Z-accelerometer offset, which feeds the estimator's velocity directly.

#### 2. The barometer is stable: no temperature or throttle compensation is justified

| | pressure closure | temperature rise | fitted Pa per degC | fitted Pa per throttle count |
|---|---|---|---|---|
| **branch** | +2 Pa ( about 17 cm equivalent ) | 30.6 -> 41.7 C ( **11.1 C** ) | **-0.48** ( r -0.18 ) | -0.033 ( r -0.12 ) |
| **master** | -1 Pa ( about 8 cm ) | 32.1 -> 42.1 C ( **10.1 C** ) | **+0.56** ( r +0.20 ) | +0.016 ( r +0.20 ) |

This is a clean null result, and the flights had the span to prove it: the die warmed **10-11 C in both
flights**, which at MagisV2's -2.17 to -2.42 Pa/degC would have moved the pressure by 22-27 Pa, about
1.8-2.2 m of apparent altitude. Nothing of the sort appears. The fitted slopes are weak **and opposite
in sign between the two flights**, i.e. noise, and the raw pressure closed to within 2 Pa over 75-78 s
in both. The throttle fit is the same story.

**Conclusion: the MS5611 on this board shows no measurable temperature or throttle dependence, so
porting `4657cbf`'s compensation would be fitting a correction to an effect that is not there.** The
risk runs the other way: MagisV2's coefficients applied here would inject roughly 2 m of error across a
10 C warm-up. This is exactly what measuring first was for. Recommendation: **drop task 15**.

( MagisV2's problem was real on its own hardware: a different sensor, the ICP-10111, measured over
seven flights. Magis also already converts pressure to altitude at a fixed ground temperature, which is
the part of `4657cbf` that matters most. )

#### 3. What the two logs confirm about the port

Comparable fields only ( `n t E V C P H` and `Z` bits 4 / 8 / 16 ):

- **Shaping works and master's does not**: the branch's `S` never left -30 .. +39 and `A` moved
  continuously; master's `S` reached -74 and +120 and its `A` jumps, as the old raw-rate controller does.
- **The I-term freeze engaged in flight again** ( branch `Z` 15 during the stick-down at about 60 s ),
  and master never shows bits 1 or 2, which do not exist there.
- **Tracking**: the branch held `A - E` within about -18 .. +24 cm across the flight.
- **Hover thrust** ( `H`, the throttle handed to the mixer ) sat at 1600-1690 on the branch and
  1607-1790 on master, i.e. the same ballpark; no sign that the port costs thrust.

#### 4. Carried forward

- Log-1's 1.9 m is attributed to calibration rather than the firmware, but that is an inference from two
  clean flights, not a proof. **Every future flight keeps the 10 s at-rest windows**: `V` must read
  about 0 at rest before arming, and must decay to about 0 within a few seconds of the motors stopping.
- Touchdown still does not come from the velocity test: at the branch's disarm `V` was -54 cm/s, so the
  `vel > -8` path cannot have fired, as in log-1.

## Task 14: barometer ground zero ( 2026-11-22 )

Build: PRIMUSX clean, exit 0, gate `no new warnings in src/`; text 78 984 / data 1 140 / bss 8 220
( +128 B flash, +24 B RAM over task 5; the `TEMP-BENCH` log line is extra and goes at task 9 ).
Cumulative against the untouched 2.0.1 baseline: **flash +1 384 B, RAM +96 B** ( 77.5 KB of 252 KB,
9.1 KB of 40 KB ).

The drift log swaps `C` ( temperature, no longer needed with task 15 dropped ) for **`O`**, the ground
offset in cm, so a log shows it tracking while disarmed and frozen while armed.

**What to look for in the task 8 logs:**

| Phase | `O` should | `E` should |
|---|---|---|
| 10 s at rest before arming | settle and stop moving | approach 0 |
| at the arm tick | **stop dead** and not move again until the disarm | start the flight near 0 |
| through the flight | be exactly constant | - |
| 3 s after the landing disarm | start moving again | decay toward 0 |

**Leave about 10 s between a landing and the next arm.** Within 3 s of a disarm the ground zero is
still held, and for a few seconds after that the previous flight's error is only partly worked off, so
an immediate re-arm starts with a residual ( see CHANGES.md ). The standard pattern's closing 10 s at
rest already covers this.

## Task 8: validation flights ( 2026-11-22 )

### The build and the log

`Build/PRIMUSX/PRIMUSX_2.0.1.hex` is the validation build ( also kept as `hex-task8-validation.hex`,
sha256 starts `a709915ec1298b20` ): the whole port ( tasks 2-5, 14 ) plus the `TEMP-BENCH` log below.
**The last board flash was master ( log-3 ), so flashing this resets every setting: recalibrate the
accelerometer and the compass, on a surface checked level, before the first flight.** The `START` line
must read `Palt128`.

| Tag | Value | Unit |
|---|---|---|
| `n`, `t` | tick counter, `millis ( )` | -, ms |
| `A`, `E` | altitude target, estimate ( **relative to the arm point**, task 14 ) | cm |
| `V` | estimated climb rate | cm/s |
| `S` | stick / landing / flip request | cm/s |
| `Z` | packed: **1** rate I-term frozen, **2** latch, **4** armed, **8** BARO, **16** stick-armed, **32** goal running; flip state = `Z / 64` ( 0 = none ) | bits |
| `R`, `Q` | roll / pitch rate I-term | - |
| `O` | barometer ground offset: tracks while disarmed, **frozen from arm to disarm** | cm |
| `Y` | pitch, **nose-down negative, range +/-900** | deci-degrees |

Typical line about 110 B, worst case 117 B ( reviewed ).

**Reading the log, four things that would otherwise mislead:**

- **`Y` saturates at +/-900 and folds back toward 0 past vertical** ( PRIMUSX builds the DCM path, so
  pitch is an `asin`, not a full-circle angle ). **Find flips with `F` ( `Z / 64` ), never with `Y`.**
- **Roll is not logged**, so the payload lean test only sees pitch: **mount the payload at the front or
  the back, not to one side.** Adding roll would put the line at about 127 B, too close to the limit.
- **`G` is only meaningful while `Z` has bit 8 ( BARO ) set**; with BARO off it holds its last value.
  `F` is sampled at 10 Hz, so a short-lived flip state can fall between ticks: that is the log, not the
  flip.
- **`E` sliding toward 0 in the seconds after a landing is the ground-zero tracker resuming** ( 3 s
  after the disarm, then a ~2 s constant ), not an estimator fault.

### One combined flight ( log-4 )

The user cannot log while flipping, so the flips move out of the logged work ( see *Flips* below ) and
the payload lean test joins this flight. **Mount the off-centre payload at the front or the back**
before starting - the log carries pitch but not roll - and note its weight and position.

Rules for the flight:

- **10 s on the floor, disarmed, untouched, before arming and after landing**, and at least 10 s
  between a landing and any later arm ( task 14 ).
- ALT_HOLD on throughout, over a clear floor, with about 3 m of headroom: step 6 descends 1.5-1.8 m.
- Log into `logs/log-4.txt`. Note anything the log cannot show.

| # | Do | Pass if |
|---|---|---|
| 1 | 10 s at rest | `O` settles and stops moving; `E` approaches 0 |
| 2 | **Arm with the throttle stick fully down**; wait 5 s | `Z` **28** ( armed + BARO + stick-armed ); motors at idle; `A` = `E`; `R` `Q` about 0; **`O` stops dead at the arm tick** |
| 3 | Keep the stick fully down 5 s more, **without having raised it** | `Z` stays 28, **never with the frozen flag 1 or the latch 2**: on the ground the stick-low reset still runs, so `R` `Q` stay about 0. This is the negative half of the task 5 test |
| 4 | Raise the stick above centre, climb to about **2.2 m** | latch sets and stick-armed clears ( `Z` 14 ); `S` at most 40; `A` climbs smoothly with no jumps |
| 5 | Centre the stick; **hover 20 s settled** | `S` 0; `A` stops where it is and `E` settles on it with no overshoot step. This hover is the **pitch baseline** for step 6 |
| 6 | **Throttle fully down for 5-6 s**, then centre and hover 10 s | **the payload test**: `Z` has the frozen flag ( 15 ), **`Q` exactly constant** through the whole stick-down, and **`Y` mean within 3 deci-degrees ( 0.3 deg ) of the step-5 hover mean**. `S` -30 and `A` descending at 30 cm/s. The old reset leaned the craft 2-7 deg toward the payload |
| 7 | Stick partly down to about 0.8 m, centre, hover 5 s | `S` between -1 and -30; smooth stop, no snap |
| 8 | Bring it to the floor with the stick, then **hold the stick fully down on the floor for 5 s** | **the leash**: `A - E` stops at about -50 and goes no further; `S` -30 |
| 9 | Disarm from the app; 10 s at rest | `Z` 8; `O` frozen for 3 s after the disarm, then tracking again; `E` sliding toward 0 is that tracker, not a fault |
| all | arm to disarm | **`O` exactly constant** |

What this one flight covers: the whole of the old flight 1 ( arming, ground idle, the freeze staying off
before a throttle raise, stick shaping, the leash, the ground freeze ) and the payload lean test that
`a7f398d` exists for. What it does not: the app take-off profile ( already measured in log-1: +120 cm at
60 cm/s in 2.5 s ), the app landing ( also in log-1, at -75 cm/s ), and the flips.

### Flips: not logged

`plutoLoop ( )` only runs while the RC link is live, so the flip tests cannot carry this log. They are
the hardware validation of `c569c0f`, so fly them separately and judge **by eye**, with 3 m of headroom:

- Five or more app flips with ALT_HOLD on, settled between them. Each must **complete its rotation**
  - the regression `c569c0f` fixes is the flip's climb never reaching 100 cm/s, so its climb state times
  out after 2.2 s and the flip is abandoned - and must come back to roughly the height it was sent from
  rather than climbing away or sinking.
- One flip sent while the previous one is still flying back down, to see the return go to the original
  height rather than an intermediate one.

Record what happened in `TESTING.md` afterwards: number of flips, whether any stalled or disarmed, and
the eyeballed height before and after.

### Not flown on purpose

- **The flip ground guard** ( a flip while armed on the floor must not set a return goal ): with props on
  it launches at full throttle for 2.2 s before giving up, which off the ground means a climb to the
  ceiling. Verified by review; a props-off bench check is the safe way to see it.
- **The RC-loss landing**: it kills the link that carries the log, so it would truncate log-4. Worth an
  eye-only run of its own if wanted - the craft should start descending within a few seconds and land.

### log-4 results ( 2026-11-22 ): the port is validated in flight

[logs/log-4.txt](logs/log-4.txt), `Palt128`, 1 225 ticks over 124.3 s, **no counter breaks and every
field present on every tick** - the 117 B line holds. The log contains **two flights**: armed 6.1-77.8 s,
a 3.2 s gap on the ground, then armed again 81.2-122.7 s with an app take-off.

| Check | Evidence | Verdict |
|---|---|---|
| **The freeze does not engage before a throttle raise** ( the negative test log-1 never reached ) | 86 ticks armed with the stick down and the latch never set: `Z` exactly **28** throughout, the frozen flag never set, `R` and `Q` within -1..0 | **pass** |
| Arm, ground idle, hand-over | `Z` 28 ( stick-armed, motors idle ) for 8.6 s, then 30 as the throttle rose and the latch set, then 14 as stick-armed cleared | **pass** |
| **The I-term freeze in flight** | 3 freeze windows of **18.6 s, 5.5 s and 6.7 s**; in all three **`Q` is exactly constant** ( -108, -128, -121 ) and `R` too ( 42, 28, 16 ), resuming as soon as the stick centres. MagisV2's own test held 6.5 s | **pass** |
| **Payload lean** | pitch mean during each freeze against the 5 s before it: **-0.31 deg, -0.41 deg, -0.19 deg**. Against the 2-7 deg the bug caused, and in the same band as MagisV2's 0.3 deg. Caveat below | **pass, with a caveat** |
| **Ground freeze ( task 14 )** | `O` **exactly constant through each armed run**: -31 cm for the whole first flight, -45 cm for the whole second. It moves only while disarmed | **pass** |
| Stick caps | maximum up request **39** cm/s ( cap 40 ), stick descents **-29** ( cap 30 ) | **pass** |
| **Landing keeps its own descent** | the landing request is **-75 cm/s**, two and a half times the stick cap | **pass** |
| **The leash** | the target advanced past the 50 cm limit on **1 tick of 1 225**, and by 3 cm ( `A` 176 -> 173 at `A - E` -51 ). That is the discretisation of a once-per-tick test, not a runaway: the target is checked against `EstAlt +/- 50` before it moves, and `EstAlt` can shift afterwards | **pass** |
| Max `A - E` | 113 cm, all of it the estimate moving under a stationary target during manoeuvres, not the target running away | - |

**Caveat on the lean numbers.** The plan asked for a settled 20 s hover before each stick-down to give a
clean pitch baseline. The craft was being flown instead: in the 5 s before the first freeze the pitch
spans 316 deci-degrees ( 31 deg ), and 85 and 58 before the other two. So the differences above are
measured against a moving baseline and are worth about +/-0.2 deg themselves. What is *not* in doubt is
the mechanism: the pitch I-term is **bit-for-bit constant** through 18.6 s, 5.5 s and 6.7 s of full
stick-down, which is the behaviour `a7f398d` adds, and the lean never approaches the degrees the bug
produced.

**The in-log re-arm, and the task 14 limitation confirmed in the wild.** Between the two flights the
craft sat **on the ground** ( pilot ) disarmed for 3.2 s. The ground-zero hold is 3 s, so tracking resumed for only the last
0.2 s and `O` moved -31 -> -45 cm before the second arm froze it there. That is exactly the accepted
limitation recorded at task 14: **a re-arm within about 10 s of a landing inherits part of the previous
flight's error** - here about 14 cm in the absolute frame. Everything the controller does is relative,
so the second flight flew normally; only the ceiling and absolute heights carry it.

**Not covered by this log** ( by decision, not omission ): the flips ( the RC link cannot survive them,
so they are flown by eye ), the flip ground guard ( unsafe with props on ), and the RC-loss landing
( it would truncate the log ). The app take-off profile and the app landing were already measured in
log-1 and appear again here.

### Task 16: the flips, by eye ( 2026-11-22 )

Flown with ALT_HOLD on and judged by eye, since the RC link that carries the log cannot survive a flip.
Pilot's report:

- **One or two flips, all completed their rotation and returned to height.**
- **The back-to-back case worked**: a flip sent while the previous one was still flying back down
  returned to the **original** height, not the intermediate one.

**What this establishes.** The flip bypass is working. Without it the failure is deterministic and
total: the shaped stick cap holds the climb at 40 cm/s, the flip waits for 100 cm/s that never arrives,
and its climb state times out after 2.2 s, **abandoning the flip**. A flip that completes its rotation therefore
proves `flipActive ( )` is bypassing the shaping, and a flip that comes back to its height proves the
exit hand-over works: the setpoint reset, the `errorVelocityI` restore and the return goal all ran, in
the order they had to. The back-to-back result additionally confirms the **goal snapshot added beyond
MagisV2** in task 4 ( `altPreFlipAltHold = altGoalActive ? lrintf ( altGoal ) : AltHold` ), which is
what stops repeated flips ratcheting the height upward; MagisV2 stores the moving target there.

**What it does not establish.** One or two flips is a smaller sample than the five the task asked for,
and than the seven MagisV2 flew. A deterministic path is proven; **repeatability across battery states,
heights and flip directions is not**. The exit transient in particular ( MagisV2 measured a 16-42 cm
dip in the estimate with the velocity biased about -13 cm/s for a few seconds after the rotation ) was
not looked for here. Recorded as a limitation at commit; more flips before release would firm it up
cheaply.
