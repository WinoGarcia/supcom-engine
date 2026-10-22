# Moho's path spline: the frame a ground unit drives on

Design, 2026-10-22. M203d and M203g brought Moho's spline rules into the engine's drive: the speed cap,
the coast, the reverse states, the stopping speed and PPS_3. The drive still steps one scalar speed
along the unit's heading each tick. Moho instead plans a spline of nodes ahead and moves the unit onto
them, its velocity a vector. What the drive grid probe still finds comes from that frame:

- acceleration while turning;
- the tick a unit starts and stops;
- the speed when turning near the target.

This note plans the port of the frame itself.

Path prefixes:

- **FR:** faf-re's `src/sdk/moho/` (Draiget/faf-re, the decompiled FAF build of Moho)
- **EN:** this repository's `src/`

FAF's binary patches leave the spline, `CUnitMotion::CalcMoveCommon` and the steering alone. They patch
the turn events (`hooks/OnMotionTurnEvent.cpp`) and the navigator's personal position distance
(`PathfindingTweaks`), so faf-re reads as retail here.

---

## 1. Moho's frame

1. **The spline** (FR `ai/CAiPathSpline.cpp` `CAiPathSpline::Generate`, 0x005B2FF0) **[read]**.
   Nodes are generated from the unit's position, heading and velocity toward a destination:
   - **Limit:** up to 20 nodes; 5 when its formation is forming; three times as many for a unit whose
     `TurnRadius` is above its `TurnRate`.
   - **Each node** is one tick on. The heading turns at most the turn limit toward the destination.
     The velocity's side part is cut to `MaxSteerForce`. Its part along the heading goes toward the
     speed cap at `MaxAcceleration` or `MaxBrake`. The node takes the position, the heading and the
     spline state.
   - **Continuation:** the last node's position, heading, velocity and state are kept. A spline
     generated without continuation starts from them in their state, not from the unit; one with
     continuation chooses its start state afresh.
2. **Following it** (FR `ai/CAiSteeringImpl.cpp` `UpdateMotionPathPointers`, `ProcessSplineMovement`,
   0x005D2C00) **[read]**:
   - **Each tick** the steering hands the motion its current node and the next, and moves its index on.
   - **At the spline's end**, at the last node: if the unit is at its destination the steering stops;
     else it generates again, with the continuation.
   - **A new target** (`DriveToNextWaypoint`) generates afresh.
   - **The destination** is the waypoint moved up to 8 along the path beyond it
     (`TrySnapPosToWaypoint`).
3. **The motion** (FR `unit/CUnitMotion.cpp` `CalcMoveCommon`, 0x006C1E20) **[read]**:
   - **With a node,** the unit is put at the node's position, facing its heading. Its velocity is the
     step it took.
   - **Without one** (or pushed), it coasts on its velocity vector, damped to 0.8 and braked.
   - **A step onto ground it won't fit** puts it back and pushes it (`AddImpulse`).

## 2. The plan

Each step ends on a check, and only a step whose check passes is built on. Moho pathing
(`--moho-pathing`) takes the frame first. Grid pathing, the default, keeps today's drive until the
frame matches retail.

| Step | What | Check |
|---|---|---|
| 1 | `drive_diff.py --trace`: the largest position difference from retail along each leg, tick by tick. A baseline on today's engine. | ruff and basedpyright clean; baseline numbers recorded here. |
| 2 | `sim/path_spline.{hpp,cpp}`: `Generate` as a pure function (unit physics, position, heading, velocity, destination, path type, continuation in; nodes out). Not wired. | Unit tests: a straight run to the goal, a stop, a turn about, a back-up, against the profiles the drive and retail give (a Striker's hop: 2.38, 2.47, 2.14). Checksum unchanged. |
| 3 | The navigator follows the spline with Moho pathing: it generates on a new waypoint and at the spline's end, puts the unit on each node, and keeps its velocity as a vector. It goes into the snapshot and the checksum. | `--drive-test` with Moho pathing; drivegrid `--trace` better than step 1's baseline and no measure worse; `ctest -L arch`; save_load_moho_pathing. |
| 4 | The coast and pushes on the velocity vector (`CalcMoveCommon` with no node). | `--drive-test` coast checks; drivegrid coast; `--crowd-test`. |
| 5 | The steering's destination moved along the path (`TrySnapPosToWaypoint`), and its prediction of meetings read from the spline's nodes, not `path_ahead`. | `--steer-test`; the steer, cone, corner and crowd-gap probes against retail. |
| 6 | Grid pathing on the spline too, once 3 to 5 hold. | The full gate; drivegrid without `--moho-pathing`. |
| 7 | Docs and PRs: one per step or two. | — |

A step that makes a measure worse than the step before is fixed or rolled back before the next.

## 3. Baseline (step 1)

`drive_diff.py --trace` on drivegrid (337 legs, Moho pathing), the largest distance from retail's unit
at the same tick along each leg:

| Engine | half the legs within | 90% within | within 0.01 | within 0.1 | within 0.5 |
|---|---|---|---|---|---|
| before M203g | 0.094 | 1.744 | 0 | 177 | 272 |
| M203g (fix/moho-reverse-gears) | 0.064 | 0.540 | 114 | 214 | 301 |

The worst legs are the special units (`url0402`, `xsl0402`, `url0001`, `ual0401`). Each later step
reports the same table; none may lower the counts within 0.01, 0.1 or 0.5.
