# PRD: Camera-Based Line Following & Static Obstacle Avoidance (TurtleBot3)

Source proposal: *Kulkarni_Akshay_StaticObstacleAvoidance.pdf* — "Camera-Based Line Following and Static Obstacle Avoidance on TurtleBot3 (ROS2 Humble)"
Team: Akshay (ROS2 setup, TurtleBot3 integration, Gazebo/RViz2 sim) · Ankith (line detection/OpenCV, LiDAR obstacle avoidance, control logic)

**Status: all 7 action items below are implemented; obstacle avoidance was
rewritten from scratch in Section 5 for proposal alignment.** This
document is kept as a record of the original gap analysis and what
changed; see `README.md` for how to build/run/test the current system.

## 1. Alignment Summary (original gap analysis, now resolved)

| Proposal item | Original status | Resolution |
|---|---|---|
| Camera-based line detection | Done | Unchanged — [line_detector.py](src/line_follower/line_follower/line_detector.py) |
| HSV filtering for a **coloured** line | Partial/mismatched (`use_hsv=false`, line was black) | `use_hsv` now defaults `true`; both world files use an orange line matching the existing HSV bounds |
| LiDAR obstacle detection / avoidance / return-to-line | Done | Rewritten from scratch for proposal alignment — see Section 5. [obstacle_avoid.py](src/tb3_safety/tb3_safety/obstacle_avoid.py) |
| Control logic for switching behaviours | Done | Unchanged — [supervisor.py](src/line_follower/line_follower/supervisor.py) |
| Gazebo simulation | Done | Unchanged, plus five structural bugs fixed (see below) |
| RViz2 visualization | Missing | Added [rviz/line_follower.rviz](src/line_following_world/rviz/line_follower.rviz) + `use_rviz` launch arg |
| Multiple test scenarios | Missing (`line_world.world` was an empty stub) | `line_world.world` now has a bent line + two obstacles; `world_file` launch arg selects between the two |
| Real hardware path | Missing | Added [robot_bringup.launch.py](src/line_follower/launch/robot_bringup.launch.py) (turtlebot3_bringup base/IMU/LiDAR + Pi camera + this stack) |
| Tuning & testing | Partial (lint-only tests) | Added functional unit tests for detector contour logic, controller behavior, and the obstacle-avoid state machine (23 tests total, all packages) |
| Project documentation | Missing | Added top-level `README.md` |

## 2. Structural bugs found and fixed during implementation

Not in the original gap list — found while validating the "colour + HSV"
change against a live Gazebo run, since a naive parameter swap would have
looked done without actually working:

1. **Duplicate camera topic.** The world previously defined its own static
   overhead camera publishing to `/camera/image_raw`/`/camera/camera_info`,
   colliding with the TurtleBot3's own onboard camera on the same topics.
   Removed the world's camera model.
2. **Missing `TURTLEBOT3_MODEL` env var.** `spawn_turtlebot3.launch.py`
   reads it directly from the process environment; it was never set, so
   `ros2 launch line_following_world line_following.launch.py` threw
   `KeyError` in any fresh shell. Fixed via a `SetEnvironmentVariable`
   action driven by a new `turtlebot3_model` launch arg (same fix applied
   to the new `robot_bringup.launch.py` for `TURTLEBOT3_MODEL`/`LDS_MODEL`).
3. **ROI/camera-geometry mismatch.** The onboard camera has zero pitch, so
   the ground's horizon sits at the image's vertical midpoint; the old
   `roi_start=0.60` cropped to a region entirely below where the line is
   ever visible, so the detector saw the line almost never. Changed default
   to `roi_start=0.48`.
4. **Inverted steer sign.** `controller.py`'s own parameter default
   (`steer_sign=-1.0`, correct) was being overridden by
   `line_follow.launch.py`'s default (`1.0`, backwards), turning the
   control loop into positive feedback — any small error grew until the
   line was lost. Fixed the launch default to match.
5. **`SEARCH_LINE` spun too fast to ever reacquire.** Found by watching a
   full live run: the robot detected the obstacle, dodged it, and entered
   `SEARCH_LINE` correctly, but then spun in place indefinitely without
   ever confirming the line was found. `SEARCH_LINE` reused the same
   `turn_speed` (0.30 rad/s) as the obstacle dodge; at that spin rate, the
   same camera-geometry sensitivity described below made the detected line
   position swing too fast frame-to-frame to hold steady for the required
   confirm streak. Added a separate, slower `search_turn_speed` parameter
   (default `0.12`) used only for the search spin. Reverified live:
   `SEARCH_LINE` now reliably reaches `LINE reacquired` / `STATE=IDLE` a
   few seconds after starting, and normal line-following resumes cleanly.

Fixing all five (plus retuning `k_p`/`max_ang_z` down) took the stack from
"never detects the line" to a fully closed loop — line-following, obstacle
detection, dodge, search, and reacquisition — verified end-to-end in live
Gazebo testing, not just unit tests. See `README.md`'s
"Camera geometry note" for why the onboard camera's zero-pitch mount makes
gain tuning more sensitive on long straight runs.

## 3. Orphaned asset removed

`models/black_line/` was unreferenced by any world/launch file and not
installed via `setup.py` — both worlds define their line geometry inline
instead. Deleted rather than wired in, to avoid maintaining two parallel
ways of expressing the same line.

## 4. Second live-testing pass: livelock found and fixed, additional cleanup

A later, independent build-and-run audit of this workspace re-verified
Section 1's claims by actually launching Gazebo rather than trusting this
document, and found one of them incomplete:

- **`BACK_AWAY <-> TURN` livelock on abeam obstacles.** Section 2's fix #5
  (the `BACK_AWAY` state) resolved the *frontal* `STOP<->TURN` livelock it
  was built for, but a fixed-duration straight reverse turned out to be
  reproducibly insufficient once the obstacle ended up to the robot's side
  rather than directly ahead — the common case, since `TURN` already runs
  before `FORWARD`'s emergency check trips. Live testing reproduced 25+
  consecutive `BACK_AWAY<->TURN` cycles over 30+ seconds with no
  resolution, in both world files, contradicting this document's earlier
  "always eventually recovers" claim (Section 2 fix #5 was validated
  against a frontal encounter, not an abeam one). Fixed by growing the
  back-away duration on each consecutive failed retry
  (`back_away_growth_sec`, capped at `back_away_max_time_sec`) instead of
  repeating the same too-short reverse — see `README.md`'s "Obstacle-dodge
  behavior" fix #6 for the full root-cause writeup, including a rejected
  alternative (curving the reverse) that made clearance shrink instead of
  grow when tested live. Re-verified: both obstacles in `line_world.world`
  now reach `LINE reacquired` / `STATE=IDLE`.
- **`supervisor.py`'s own `avoid_topic` default** (`/cmd_vel_avoid`) didn't
  match `obstacle_avoid`'s actual publish topic (`/cmd_vel_obstacle`) —
  harmless only because `line_follow.launch.py` always overrode it. Fixed
  the default itself so a standalone/future launch can't silently drop
  obstacle-avoidance commands.
- **`line_cam.world` / `cam.sdf`** (in `line_follower/`) were unreferenced
  by any launch file or `setup.py`, the same category of leftover as the
  `models/black_line/` cleanup above. Deleted.

All fixes were validated with the full unit test suite (32 tests, no
regressions) and live Gazebo runs against the exact scenario that
reproduced the livelock.

## 5. Obstacle avoidance rewritten from scratch (proposal-alignment pass)

By this point `obstacle_avoid.py` had grown to 8 states and ~25 parameters
through incremental live-testing patches (Sections 2 and 4), and — despite
each individual patch being independently verified — the user reported it
still felt inaccurate. Re-reading the proposal directly (not just this
document's summary) confirmed the concern: the proposal's own scope for
this component is "use LiDAR to detect obstacles; if an obstacle is
detected, avoid it and then return to following the line," with an
explicit fallback to "use simpler control logic if integration is
complex." An 8-state, 25-parameter implementation does not match that
scope, and its accumulated complexity was itself a plausible reason for
reduced confidence in it. `obstacle_avoid.py` was rewritten from scratch:
six states (`IDLE -> STOP -> TURN -> FORWARD -> TURN_BACK -> SEARCH_LINE
-> IDLE`), dropping the standalone `BACK_AWAY` and `RETURN_TO_LINE` states
in favor of folding a growing reverse into `STOP` itself and relying on
`TURN_BACK` (odometry heading restore) plus `SEARCH_LINE` (camera-based
line search) alone for recovery.

The rewrite kept every *physically-grounded* fact the earlier debugging
had established (the LiDAR-to-hull mounting offset, contiguous sector
coverage, continuous proximity checks during motion, odometry-verified
state exits) since these aren't complexity for its own sake — they're the
minimum needed for correctness given the real robot's geometry. What it
dropped was the *defensive* complexity that had accumulated around
increasingly tight distance margins (multi-state retry recovery, a
closed-loop cross-track position controller). Re-verifying live surfaced
three more geometry bugs that the previous tight-margin tuning had been
papering over with retries rather than actually fixing:

- **Closest-approach clearance during the pass was tuned right at the
  emergency threshold.** For the ~40 degree dodge turn, closest approach
  to the obstacle is `(avoid_distance + hull_half_length +
  obstacle_half_depth) * sin(40deg)`, minus half-widths. At
  `avoid_distance=0.60` this worked out to ~0.25m — exactly
  `emergency_distance` — so ordinary noise reproducibly tipped otherwise-
  clean dodges into emergency replanning. Fixed by increasing
  `avoid_distance`.
- **`IDLE` reuses `avoid_distance` to decide "still too close," but the
  dodge's resting distance from the obstacle is set by a different term**
  (`forward_distance_m * sin(40deg)`) that barely changes with
  `avoid_distance`. Raising `avoid_distance` alone (to 0.90m) to fix the
  point above pushed that same threshold past the (unrelated) resting
  distance, so `IDLE` immediately re-detected the same obstacle and
  re-triggered a second dodge from almost the same spot.
- **Along-track clearance was undershooting the obstacle's full depth.**
  `forward_distance_m * cos(40deg)` needs to carry the robot's hull past
  the obstacle's *far* edge, not just its half-depth; undershooting this
  left the robot's resting position beside the obstacle rather than past
  it, so returning to the line (which passes through the obstacle's
  position) walked it straight back into the same obstacle, and the
  camera lost the line entirely during the resulting churn.

Final tuning (`avoid_distance=0.80`, `forward_distance_m=2.20`) satisfies
all three simultaneously and was verified live in `line_world.world`:
36/36 unit tests pass, and the live run reached `LINE reacquired` for both
obstacles with **zero** `EMERGENCY` events overall and minimum clearance
never below ~0.39m (the harder, bend-positioned obstacle needed two dodge
passes rather than one, but both were clean). See `README.md`'s
"Obstacle-dodge behavior" for the full geometry derivation and tunable
parameter list.

## 6. Project-wide review pass

Following the rewrite in Section 5, `controller.py` (`line_controller`),
`line_detector.py`, and `supervisor.py` were reviewed in detail and
live-tested against both world files. No concrete bugs were found in any
of the three — the P-controller (deadband, EMA smoothing, speed
ramp-down), contour tracking (jump-rejection with confirm-frame
switching), and priority-based arbitration (`avoid` over `line`) all
behaved correctly in live runs. None were rewritten, since there was no
evidence any of them needed it.

This pass did surface one real documentation gap: `line_following.world`
was described in `README.md` as simply a "straight 8m line," but it has
its own `obstacle_box` at 2m from the track's center — untested against
this session's `obstacle_avoid` rewrite until now. Live-tested: every
individual dodge against it stayed safe (zero `EMERGENCY` events across
five encounters in one run), but the robot can end up re-approaching the
same obstacle repeatedly rather than clearing it once, because the short
track leaves little room to absorb the same position-blind-spot limitation
already documented for the bend case in `line_world.world`. `README.md`
now describes this world accurately. `line_world.world` (the more
spacious two-obstacle scenario) was reconfirmed fully working: clean
single-pass dodges, comfortable clearance, stable recovery.
