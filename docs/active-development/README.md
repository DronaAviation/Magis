# Active Development ( Magis )

Working documentation for firmware changes to the old Magis flight controller that are **in
progress**: what is being changed, why, how, and the evidence that it works. One folder per topic.
Same layout and rules as the MagisV2 `docs/fw-development-reference/active-development/` folder,
minus the pipeline docs ( Magis has none; the MagisV2 subsystem docs serve as the reference ).

## Rules

1. Start a topic folder here when a piece of work needs more than a commit message to explain.
2. Keep it current while the work is live: record changes, measurements and dead ends as they happen.
3. At commit: update `CHANGELOG.md`, set the topic **Closed** here and in its README with the commit hash.
4. Closed topics stay; mark anything superseded.

## Topic folder layout

| File | Contents |
|---|---|
| `README.md` | Status, branch, target, summary, gotchas, open items, next actions. Start here. |
| `TASKS.md` | Resume block, numbered tasks, decisions log. Written by `/pluto-grill`, worked with `/pluto-task` ( give it this folder's path: the skill defaults to the MagisV2 tree ). |
| `SCOUT.md` | The `pluto-scout` brief from planning. |
| `DESIGN.md` | `feature` topics: the options, the one chosen and why. |
| `CHANGES.md` | Each code change with file and line links, and a file index. |
| `TESTING.md` | Test method, measurements, results, how to reproduce. |

Links to source from a topic folder use `../../../src/main/...`.

## Topics

| Topic | Status | Branch | Summary |
|---|---|---|---|
| [magis-althold-backport](magis-althold-backport/README.md) | **Closed - pending commit** ( 22 Nov 2026, FW 2.1.0, API 1.0.2 ) | `althold-backport` | `feature`: hand-port of MagisV2 `2a8d59a` ( altitude setpoint shaping ), `c569c0f` ( flip bypass and return to height ), `a7f398d` ( airborne I-term freeze ) and the `4657cbf` pieces they need, including the barometer ground zero frozen on arm. Three places go beyond MagisV2: a 3 s ground-zero settle hold after any disarm ( Magis can disarm and re-arm in the air ), a flip floor guard against the ground reference, and a goal snapshot that stops repeated flips ratcheting the height. MagisV2\'s barometer temperature / throttle compensation was measured on this hardware and dropped ( no effect to correct ). Flight-validated on PRIMUSX ( log-1..4, flips by eye ); +1 384 B flash, +96 B RAM. |
