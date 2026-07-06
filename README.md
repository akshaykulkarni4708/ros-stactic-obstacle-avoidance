# Camera-Based Line Following and Static Obstacle Avoidance (TurtleBot3, ROS2 Humble)

A TurtleBot3 stack that follows a coloured line using an onboard camera and
swerves around static obstacles using LiDAR, then returns to the line. Built
against the project proposal in `PRD.md` (kept up to date as the source of
truth for proposal-vs-implementation status).

## Packages

- **`line_follower`** -- camera-based line detection (`line_detector`), a
  proportional steering controller (`line_controller`), and an arbitration
  node (`supervisor`) that prioritizes obstacle-avoidance commands over
  line-following commands.
- **`tb3_safety`** -- `obstacle_avoid`, a six-state LiDAR-driven state
  machine (`IDLE -> STOP -> TURN -> FORWARD -> TURN_BACK -> SEARCH_LINE ->
  IDLE`) that stops, turns away from the tighter side, drives past the
  obstacle, turns back to its pre-obstacle heading (measured via
  odometry), then spins to reacquire the line before handing control
  back. If `TURN` or `FORWARD` gets too close again it recomputes the
  turn direction and retries from `STOP` -- see "Obstacle-dodge behavior"
  below for the geometry behind the tuning and how retries are handled.
- **`line_following_world`** -- Gazebo worlds, the top-level simulation
  launch file, and the RViz2 config.

## Topic graph

```
/camera/image_raw --> line_detector --> /line_error --> line_controller --> /cmd_vel_line --\
                                    \--> /line_error -----------------------> obstacle_avoid  |--> supervisor --> /cmd_vel
/scan -------------------------------------------------------------------> obstacle_avoid --> /cmd_vel_obstacle --/
```

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select line_follower tb3_safety line_following_world
source install/setup.bash
```

## Run in simulation

```bash
ros2 launch line_following_world line_following.launch.py
```

Useful arguments:

| Argument | Default | Purpose |
|---|---|---|
| `world_file` | `line_following.world` | `line_following.world` (14 m straight + 4 m 90-degree bend, three obstacles each with 4 m+ clearance) or `line_world.world` (5 m straight + 3 m bend, two obstacles) |
| `turtlebot3_model` | `waffle_pi` | TurtleBot3 model to spawn |
| `use_rviz` | `false` | Also launch RViz2 (`rviz/line_follower.rviz`: RobotModel, TF, LaserScan, onboard camera image, `/line_mask`, Odometry) |
| `use_sim_time` | `true` | Standard sim-clock switch |

Example: `ros2 launch line_following_world line_following.launch.py world_file:=line_world.world use_rviz:=true`

## Run on real hardware

```bash
ros2 launch line_follower robot_bringup.launch.py turtlebot3_model:=waffle_pi lds_model:=LDS-01 usb_port:=/dev/ttyACM0
```

Runs on the robot's onboard computer. Includes `turtlebot3_bringup`'s
`robot.launch.py` (base/IMU/LiDAR) and `camera.launch.py` (Pi camera via
`camera_ros`), then starts the same `line_follower`/`tb3_safety` nodes with
`use_sim_time:=false`. `camera_ros` must be installed on the robot; it is not
part of this workspace's dependency set since it only runs on-robot.

## Key tunable parameters

All declared as ROS parameters on their respective nodes and exposed as
launch arguments in `line_follower/launch/line_follow.launch.py`.

- **Detection** (`line_detector`): `use_hsv` (default `true`) with
  `hsv_lower_h/s/v` / `hsv_upper_h/s/v` tuned for the world's orange line;
  `roi_start` (default `0.48`) crops the image to everything below the
  camera's horizon row -- see "Camera geometry note" below.
- **Steering** (`line_controller`): `k_p`, `max_ang_z`, `steer_sign`,
  `error_deadband`, `angular_alpha` (smoothing), `slowdown_error` /
  `turn_in_place_error` (speed ramp-down as error grows).
- **Obstacle avoidance** (`obstacle_avoid`): `avoid_distance` /
  `emergency_distance` (reaction and hard-stop hull-clearance thresholds),
  `lidar_to_hull_margin` (default `0.20`, converts raw LiDAR ranges to
  true hull clearance), `back_off_speed` / `back_off_growth_sec` /
  `back_off_max_time_sec` (the reverse nudge `STOP` always performs, grown
  on consecutive retries -- this, not bigger base distances, is what
  actually prevents contact during retry-heavy episodes; see
  "Obstacle-dodge behavior"), `turn_direction_hysteresis_m` (default
  `0.10`, requires a clear margin before switching which way to turn, so
  near-tied `left`/`right` readings don't flip-flop the direction retry
  to retry), `turn_time_sec` / `turn_speed` (the dodge turn, exit verified
  via odometry, not just the timer), `forward_distance_m` /
  `forward_speed` (the post-turn drive, exit verified via odometry
  distance travelled), `search_turn_speed` (default `0.12`, the slower
  in-place spin rate used only while re-scanning for the line -- see
  "Camera geometry note"), `line_search_confirm_count`. See
  "Obstacle-dodge behavior" below for how `avoid_distance` and
  `forward_distance_m` were derived from the dodge geometry.

## Camera geometry note

The TurtleBot3 Waffle Pi's onboard camera is mounted with zero pitch (looking
straight ahead, not tilted down), so the ground's horizon sits at the
image's vertical midpoint. `roi_start=0.48` crops to just below that horizon
so the detector only ever sees ground, never sky. Because of this, pixels
near the top of the crop correspond to line segments far down the track --
optically very sensitive to small heading changes -- so an aggressive
steering gain oscillates and eventually loses the line on a long, unbroken
straight run. The current defaults (`k_p=0.004`, `max_ang_z=0.4`) were tuned
against this: both `line_following.world`'s 8 m straight line and
`line_world.world`'s bent line now converge to sub-pixel steady-state error
and hold it for the full run in testing. If you increase `k_p`/`max_ang_z`
for a faster response, re-validate against the long straight line first --
that's the scenario most sensitive to this effect.

**Cruise speed increased, `linear_x` 0.04 -> 0.10 m/s** (`min_linear_x`
0.02 -> 0.04 m/s), per request, without touching `k_p`/`max_ang_z` -- the
steering gains above are what govern tracking stability, not the base
forward speed, and the controller's existing slowdown-when-turning ramp
(full speed only when error is small, scaling down toward `min_linear_x`
as error grows) already protects sharp turns regardless of the cruise
speed. Verified live end-to-end on `line_following.world` at the new
speed: the long straight run held stable tracking (no oscillation), the
90-degree bend produced a large but transient error spike (as expected
for a sharp corner) that recovered cleanly without ever losing the line,
and all three obstacles dodged with **zero** `EMERGENCY` events.

The same sensitivity applies to `obstacle_avoid`'s `SEARCH_LINE` state,
which spins the robot in place to re-find the line after dodging an
obstacle: spinning at the full obstacle-dodge `turn_speed` (0.30 rad/s)
made the detected line position swing too fast to ever hold steady long
enough to confirm reacquisition. `search_turn_speed` (default `0.12`) is a
separate, slower rate used only for that spin, and consistently converges
smoothly once it starts (verified live: monotonic pixel-error convergence,
no oscillation).

## Obstacle-dodge behavior

`obstacle_avoid.py` was rewritten from scratch (see `PRD.md` section 5)
after an earlier, heavily-patched version grew to 8 states and ~25
parameters while still occasionally making real contact with obstacles.
The proposal's own scope for this component is simple -- "use LiDAR to
detect obstacles; if an obstacle is detected, avoid it and then return to
following the line" -- with an explicit fallback to "simpler control logic
if integration is complex." The rewrite targets exactly that: six states
(`IDLE -> STOP -> TURN -> FORWARD -> TURN_BACK -> SEARCH_LINE -> IDLE`),
sized so a single dodge attempt is the normal case, with continuous safety
checks rather than elaborate multi-state recovery machinery.

**Physically-grounded checks kept from the earlier debugging (still
necessary, not optional complexity):**

- `lidar_to_hull_margin` (0.20m) converts raw LiDAR ranges to true hull
  clearance -- `base_scan` sits 0.064m behind the robot's center while the
  footprint extends ~0.14m ahead of it, so raw ranges alone understate how
  close the hull actually is.
- The side sectors start exactly at `front_half_angle_deg`, by
  construction (no separate "side start" parameter that could drift out
  of sync), so there's no angular gap an obstacle could sit in
  undetected, and extend out to `side_sector_max_deg=170` -- covering
  nearly the full circle, leaving only a 10-degree wedge directly behind
  on each side unmonitored. This was originally 100 degrees (an
  80-degree-per-side blind wedge toward the rear); live testing at a bend
  -- where the robot changes heading while navigating past an obstacle --
  found an obstacle could sweep from that blind wedge into view only once
  already very close, with the very first `OBSTACLE detected` reading for
  that encounter showing negative computed clearance (contact already
  made) instead of an early warning. Widened so a heading change can no
  longer hide an obstacle from the safety checks.
- `TURN` and `FORWARD` both check `min(front, left, right)` against
  `emergency_distance` on every tick, not just at entry -- a rotating or
  translating robot can bring a previously-clear direction within
  emergency range, and checking only `front` misses obstacles that end up
  to the side after the initial turn.
- `STOP` always includes a brief reverse nudge (`back_off_speed`), growing
  on consecutive retries (`back_off_growth_sec`, capped at
  `back_off_max_time_sec`, reset once a dodge completes cleanly) --
  without this, re-entering `TURN` while still inside `emergency_distance`
  can re-trip the proximity check before any rotation happens, live-locking
  `STOP <-> TURN` with zero commanded velocity while momentum/contact
  keeps sliding the robot closer.
- `TURN_BACK` restores heading via measured odometry yaw, not a timed
  turn -- commanded angular velocity x time doesn't account for real
  acceleration lag.

**Tuning derived from dodge geometry, verified via an independent
geometric clearance check (from `/odom` plus each obstacle's known world
pose, not the LiDAR code under test) against live Gazebo runs:**

For a fixed ~40 degree turn (steep enough to clear a compact obstacle's
half-width, shallow enough that most of the subsequent drive still counts
as progress past it), three distances all depend on the same two
parameters (`avoid_distance`, `forward_distance_m`) and pull in different
directions:

1. **Closest-approach clearance during the pass** ~=
   `(avoid_distance + hull_half_length + obstacle_half_depth) *
   sin(40deg)`, minus the obstacle's and robot's half-widths. Too small an
   `avoid_distance` (0.60m initially) made this come out to ~0.25m --
   right at `emergency_distance`, so ordinary sensor/physics noise
   reproducibly tipped it into an emergency replanning cycle instead of a
   single clean pass.
2. **Resting distance from the obstacle once the dodge finishes** ~=
   `forward_distance_m * sin(40deg)` -- and `IDLE` reuses `avoid_distance`
   as its own "is the robot still too close" threshold. Raising
   `avoid_distance` to fix (1) in isolation (to 0.90m) pushed that same
   threshold past this resting distance (which barely moves with
   `avoid_distance`), so `IDLE` immediately re-detected the same obstacle
   and re-triggered a second dodge from almost the same spot.
3. **Along-track clearance** ~= `forward_distance_m * cos(40deg)` needs to
   carry the robot's hull genuinely past the obstacle's *far* edge, not
   just its half-depth -- undershooting this (at `forward_distance_m
   =1.60`) left the robot's resting position only barely past the
   obstacle's *near* edge, so returning to the line (which passes through
   the obstacle's position) walked it straight back into the same
   obstacle, repeatedly, and the camera lost the line entirely during the
   resulting churn.

`avoid_distance=0.80` / `forward_distance_m=2.20` satisfies all three
simultaneously (~0.38m closest-approach clearance, ~1.03m resting
distance comfortably above `avoid_distance`, and along-track clearance
that lands the robot's resting x-position past the obstacle's far edge,
not beside it) and was the tuning that finally verified clean end-to-end:
in the live run that validated this, `line_world.world`'s first obstacle
was dodged in a single clean pass with **zero** `EMERGENCY` events and
0.389m minimum clearance; the second (positioned right at the bend, a
harder case since it sits in the middle of a heading change) needed two
passes, also both with zero `EMERGENCY` events, before `SEARCH_LINE`
reacquired the line with a confident, stable detection and normal
line-following resumed. Minimum clearance across the whole run never
dropped below ~0.39m -- comfortably clear of `emergency_distance` (0.25m)
in every recorded encounter.

**Update: `forward_distance_m` tightened further, 2.20 -> 1.90 -> 1.75**,
to make the dodge as small a maneuver as safely possible (per request --
the original swing was wider than needed and took longer to rejoin the
line). `avoid_distance` and the ~40 degree turn angle were left unchanged
since they govern the safety-critical closest-approach clearance (point 1
above), which doesn't depend on `forward_distance_m`. At 1.75m, the two
distances that do scale with it are getting closer to their documented
failure thresholds but stay clear of them: resting distance is ~1.13m
(vs. `avoid_distance=0.80`, so `IDLE` doesn't immediately re-detect the
same obstacle) and along-track clearance is ~1.34m (vs. the ~1.23m that
previously undershot and caused repeated contact) -- a real but no longer
large margin, so this is close to the practical floor for this geometry;
pushing meaningfully lower without also revisiting `avoid_distance` or the
turn angle risks reintroducing that contact failure. Verified live: three
dodges against `line_following.world` (including the harder one right by
the bend), all with **zero** `EMERGENCY` events, lateral offset down to
~0.40m (from ~1.2-1.4m originally), and `SEARCH_LINE` reacquiring in as
little as ~3s.

Retry counts (both the `STOP`/`TURN` emergency-recompute kind and the
"needed a second full dodge near the bend" kind) will still vary run to
run with the exact angle/position at first detection -- that's a
performance characteristic of a reactive, non-planning dodge, not a
safety issue, since every check above is about clearance, not about
completing in a fixed number of attempts. Fully eliminating multi-pass
recovery near a bend would need the dodge to reason about *position*
relative to the line, not just heading -- e.g. a stored waypoint or a
local planner -- which is beyond the "simpler control logic" scope the
proposal itself calls out as an acceptable fallback.

**Recovery strength matters more than base distances.** Despite the
geometry above giving comfortable clearance on paper, live testing later
found genuine negative computed clearance -- real contact -- during a
retry-heavy episode against the bend obstacle: real Gazebo
acceleration/deceleration lag and LiDAR update latency ate into the
margin faster than the flat geometry predicted, and two compounding
issues let it get worse instead of self-correcting: (1) `STOP`'s reverse
was too timid (`back_off_speed=0.08`, capped at `back_off_max_time_sec
=3.00`) to create real separation once the hull was already close, and
(2) `turn_dir` could flip-flop between left and right on near-tied
`left`/`right` readings, undoing whatever separation the previous retry
had gained. The first attempt at a fix raised `avoid_distance` /
`emergency_distance` / `forward_distance_m` (1.05/0.35/2.40) -- this did
eliminate the negative readings, but made every dodge's lateral drift
much larger, which left `SEARCH_LINE` spinning in place for minutes
afterward trying to reacquire the line (visually: the robot just sitting
there not moving). That traded one problem for another instead of fixing
the actual cause. The real fix was strengthening *recovery*, not the base
geometry: `back_off_speed` raised to 0.12 and `back_off_max_time_sec` to
5.00 (up to 0.6m of guaranteed reverse, enough to reliably break contact
rather than just reduce it), plus a `turn_direction_hysteresis_m=0.10`
band so `turn_dir` only changes on a clear margin, not sensor noise.
`avoid_distance`/`emergency_distance`/`forward_distance_m` were reverted
to the original, more compact 0.80/0.25/2.20 -- re-verified live against
the exact scenario that produced the negative clearance: zero
`EMERGENCY` events across six obstacle encounters in one run, minimum
clearance never below ~0.39m, and `SEARCH_LINE` recovering in
single-digit seconds after a clean pass (versus 100+ seconds with the
bigger, first-attempt geometry).

`line_following.world` was originally a single 8m straight line with one
`obstacle_box` only 2m from its center -- tight enough that the robot
could end up re-approaching the same obstacle from the opposite
direction after a dodge instead of clearing it and moving on. It's now
an 18m course (14m straight + a 4m 90-degree bend) with three obstacles,
each with 4m+ clearance on both sides, closer to `line_world.world`'s
more forgiving spacing. The underlying position-blind-spot limitation
described above (the dodge logic tracks heading, not position relative
to the line) is unchanged -- more room around each obstacle just makes
a same-obstacle re-encounter far less likely to happen in the first
place.

## Tests

```bash
colcon test --packages-select line_follower tb3_safety line_following_world
colcon test-result --all
```

Covers `ament_flake8`/`ament_pep257`/`ament_copyright` style checks plus
functional unit tests:

- `line_follower/test/test_line_detector_logic.py` -- contour selection
  (largest-area fallback, nearest-to-tracked preference, minimum-area
  rejection) and jump-confirmation logic.
- `line_follower/test/test_controller_logic.py` -- deadband, steer-sign
  direction, turn-in-place threshold, lost-line search behavior.
- `tb3_safety/test/test_obstacle_avoid_logic.py` -- the full
  `IDLE -> STOP -> TURN -> FORWARD -> TURN_BACK -> SEARCH_LINE -> IDLE`
  state cycle, odometry-verified turn-angle and forward-distance exits,
  contiguous sector coverage, growing back-off on retry, and the
  proximity checks in `TURN`/`FORWARD`.

## Local environment note

`rviz2` may fail to start with
`symbol lookup error: .../snap/core20/.../libpthread.so.0: undefined symbol:
__libc_pthread_init`. This isn't a workspace bug -- it happens when the
shell was itself launched from inside a snap-confined app (e.g. VS Code
installed via snap), which leaks `GTK_PATH`, `GTK_EXE_PREFIX`,
`GDK_PIXBUF_MODULE_FILE`, `GDK_PIXBUF_MODULEDIR`, `GIO_MODULE_DIR`,
`GSETTINGS_SCHEMA_DIR`, and `LOCPATH` into the environment, pointing at
the confined snap's own (incompatible) library set. Qt's GTK theme
integration picks those up and pulls in a mismatched `libpthread` from the
`core20` base snap. Confirmed fix -- unset those specific variables before
launching:

```bash
env -u GTK_PATH -u GTK_EXE_PREFIX -u GDK_PIXBUF_MODULE_FILE \
    -u GDK_PIXBUF_MODULEDIR -u GIO_MODULE_DIR -u GSETTINGS_SCHEMA_DIR \
    -u LOCPATH -u GIO_LAUNCHED_DESKTOP_FILE \
    ros2 launch line_following_world line_following.launch.py use_rviz:=true
```

Verified working this way: `rviz2` starts, initializes OpenGL, and
subscribes to every topic in `rviz/line_follower.rviz`
(`/camera/image_raw`, `/line_mask`, `/scan`, `/odom`, `/robot_description`).
