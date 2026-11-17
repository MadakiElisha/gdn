# TEST-034: three vehicles with the GDN payload (R2.3)

**Question.** R2.3: PX4's Gazebo multirotor (x500), standard VTOL and fixed-wing
models, each with a GDN sensor payload attached through an SDF include. Do all three
carry the same payload, and do they fly the Stage A route with GDN alongside?

**Answer, short.**
- **One payload, three vehicles.** `sim/models/gdn_payload` holds GDN's sensors (the radar
  altimeter for now; R2.4 adds the rest). `x500_gdn`, `standard_vtol_gdn` and
  `rc_cessna_gdn` are each PX4's own model with the payload included and fixed to the
  body. `tools/sim_vehicles.py` holds the table: model, PX4 airframe (4001, 4004, 4003),
  PX4 settings, how it flies. `GDN_VEHICLE=... tools/sim_run.sh` picks one.
- **`tools/check_vehicles.py`, written first,** checks the models (the payload included
  once and fixed to the body, no GDN sensors of their own, the matching PX4 airframe,
  `gz sdf` resolves them) and judges a flight (phases, whole route, cross-track, terrain
  clearance, the radar along its beam). On main it fails: the x500 carries its own radar
  and there is no VTOL or fixed-wing.
- **All three flew the full 7.9 km route on two worlds and pass**: Zermatt (the Stage A
  world, mountain) and Emmental (R2.2, rolling hills), in the dev image on the GPU.
  - **x500:** as before, within 1.3 m of the route (p95).
  - **standard VTOL:** climbs, changes to plane flight in 7 to 8 s, flies the route
    within 6 to 7 m (p95), changes back in 5 s, lands on the pad.
  - **rc_cessna:** catapult launch (a Gazebo push to 17 m/s in half a second, PX4's
    launch detection), the route within 7 to 8 m, then circles over the pad. It doesn't
    land: a real site has no runway.
  - **All of them** stayed at least 55 m above the ground away from the pad.
- **Found on the way:**
  - **PX4 1.16 can't do a VTOL transition in offboard.** Its transition flight task
    publishes the same setpoint topic offboard uses; the plane controller then has no
    valid setpoint and gives no forward thrust. The VTOL hung, sank 20 m and PX4 aborted
    (quad-chute). The transitions now happen in Hold.
  - **GDN's altimeter input is metres off on every vehicle (OI-016).** The radar is
    right along its beam, but the adapter converts the slant range to a height as if
    the ground were flat. Over slopes, tilted, that's 6 to 9 m RMS on the x500 (which
    cruises tilted) and 2 to 6 m on the planes, against GDN's 10 m sigma. Earlier
    reports (TEST-031 to 033) gave 0.3 m: they quoted the last 10 s, the landing over
    the level pad. Whether this feeds OI-012 isn't tested yet.
- **GDN is not judged here.** It ran in shadow mode on all six flights with the
  multirotor's settings (per-airframe profiles are R2.5), and drifted to kilometres on
  all of them by the end.

![TEST-034](test034_vehicles.png)

*TEST-034 (`tools/plot_test034.py`, from `test034_data/`).*
- *Top: the true tracks of the three vehicles over the two worlds, on the planned route
  (white). The VTOL's and the cessna's corners are flown wide; the cessna's last circles
  are around the pad.*
- *Bottom left: height above the ground on Zermatt. The x500 flies the route in 15
  minutes, the planes in 9.*
- *Bottom middle: cross-track from the route while `sim_fly.py` steers (not in the
  transitions or the cessna's last circles): median, 95th percentile and the check's
  limit.*
- *Bottom right: the altimeter by tilt on Zermatt. Squares: the radar against the
  terrain where its beam lands, the sensor's noise at every tilt. Dots: the same reading
  as a height (range times the cosine of the tilt) against the height straight below,
  which is what GDN takes it for (OI-016).*

## The vehicles

| `GDN_VEHICLE` | Model | PX4 airframe | Take-off | Route | End |
|---|---|---|---|---|---|
| `x500` | `x500_gdn` | 4001 `gz_x500` | offboard climb | a setpoint moving at 10 m/s ("carrot") | lands on the pad |
| `standard_vtol` | `standard_vtol_gdn` | 4004 `gz_standard_vtol` | offboard climb, front transition in Hold | as a plane, about 15 m/s | back transition in Hold 350 m out, lands on the pad |
| `rc_cessna` | `rc_cessna_gdn` | 4003 `gz_rc_cessna` | catapult: Takeoff mode, launch detection, climb to 50 m | as a plane, about 15 m/s | 30 s circling over the pad in Hold |

**The payload.** `gdn_payload` is a model of its own: one link with the radar altimeter
(a single-beam `gpu_lidar` on `gdn/agl_radar`, 0.2 to 1500 m, 20 Hz, 0.3 m noise).
Each vehicle includes it with its mount pose and fixes it to `base_link`:

```xml
<include>
  <uri>model://gdn_payload</uri>
  <pose relative_to="base_link">0 0 -0.06 0 0 0</pose>
</include>
<joint name="gdn_payload_joint" type="fixed">
  <parent>base_link</parent>
  <child>gdn_payload::gdn_payload_link</child>
</joint>
```

The radar sits 6 cm under the x500's plate and the cessna's fuselage, 3 cm under the
VTOL's hull. Its beam starts at 0.2 m, so the airframe around it can't block it. The
truth publisher stays in each vehicle model: it isn't a sensor the real vehicle carries.
PX4's models already have the barometer and magnetometer GDN uses, under the same names
on all three, so the bridge is unchanged.

**Spawning.** The worlds (R2.2) place the x500. `sim_vehicles.py world` writes a copy for
the run with the chosen vehicle in its place, so the worlds, `view_site.sh` and the world
checks stay as they were. The cessna faces its launch heading: the one where the terrain
stays furthest below a 12% climb from the pad, computed from the world's map (Zermatt
120°, Emmental 190°).

**PX4 settings** (`VEHICLE_PARAMS`): `IMU_INTEG_RATE 100`, `NAV_DLL_ACT 0` and
`EKF2_GPS_CTRL 7` for all, as before; for the cessna `RWTO_TKOFF 0` (no runway),
`FW_LAUN_DETCN_ON 1` and `MIS_TAKEOFF_ALT 50`. PX4 resets its parameters when the
airframe changes.

## How the planes fly the route

- **As a line.** `sim_fly.py` sends a point 60 m ahead of the plane's place on the route,
  with the route's direction as velocity. PX4's fixed-wing controller treats a position
  with a velocity as a path to follow (its "offboard position with velocity setpoints").
- **Height.** The route's height is limited to a 25% (VTOL) or 17% (cessna) gradient, about
  half of what PX4's climb limits allow at cruise speed (`FW_T_CLMB_MAX` 8 and 5 m/s at
  15 m/s). Climbs start earlier and descents wait; the x500's route is as before.
- **The catapult.** PX4 waits in Takeoff mode with the motor at idle until the forward
  acceleration passes 30 m/s² for 0.05 s. `sim_fly.py` then asks Gazebo's ApplyLinkWrench
  (in PX4's server config) to push the plane forward at 40 m/s² until Gazebo's truth says
  16 m/s, and clears the push. It took 0.46 to 0.48 s and released at 17.2 to 17.4 m/s.
- **No landing for the cessna.** It circles over the pad for 30 s in Hold and the run
  ends. A real site has no runway; fixed-wing recovery is Phase 8.

## The VTOL transition in offboard (fixed by doing it in Hold)

The first VTOL flights asked for the front transition in offboard, as `sim_fly.py` flies
everything else. PX4 started it and aborted after 6.6 s (`Quad-chute triggered`). The
VTOL then flew the route hovering (`vtol_transition_in_offboard_check.txt`: the check
fails it, "missing: front transition done").

PX4's own log of the second attempt (`ulog`, copied out of the container) showed why:
- **No forward motion at all:** ground speed 0.0 to 0.1 m/s through the transition, and
  the pusher motor off (`actuator_motors` index 4 NaN) with the plane controller's thrust
  at 0.
- **The setpoints weren't ours.** `trajectory_setpoint` held no position, only a climb
  rate: PX4's transition flight task publishes that topic itself during a transition,
  the same uORB topic offboard setpoints arrive on. The plane controller reads it in
  offboard, finds no valid position, and gives no thrust.
- **The VTOL sank** from 59.6 to 41.7 m above the pad, and PX4's transition
  altitude-loss check (`VT_QC_T_ALT_LOSS`, 20 m) aborted it.

So `sim_fly.py` now switches to Hold, PX4's own mode, for both transitions, sends no
setpoints meanwhile (they'd overwrite Hold's), and goes back to offboard once PX4 reports
plane or hover flight. Front transition 7 to 8 s, back transition 5 s on all later
flights.

## The check, written first

`tools/check_vehicles.py`:
- **models** (no simulator needed; the PX4 and `gz sdf` parts where installed): each
  vehicle merges a PX4 model, includes `gdn_payload` exactly once with a fixed joint to
  `base_link`, has no GDN sensor of its own, publishes truth; its PX4 airframe exists
  and simulates the same model; `gz sdf -p` resolves it with the payload, magnetometer
  and barometer.
- **`--run LOG_DIR`**, a flight:
  - its phases from `fly.log`, and no fail-safe;
  - the whole route: every route point (away from the pad) passed within 60 m;
  - cross-track p95 while `sim_fly.py` steers (PX4 in offboard, not in a transition):
    15 m hovering, 40 m as a plane;
  - at least 25 m above the world's terrain, away from the pad;
  - the radar along its beam: each range (2 Hz of its 20) against the range the truth
    predicts at the same Gazebo time, casting the beam at the true attitude onto the
    world's terrain; RMS under 1 m over the readings GDN uses (tilt up to the adapter's
    35°);
  - GDN's scorer reported.
- **`--selfcheck`**: the judging on made-up tracks (a good route, half a route, 35 m off,
  too low, a missing transition, a fail-safe).

**Before:** on main's x500 model it fails twice (no payload include, a radar of its own;
`models_check_on_main.txt`). The VTOL flown with its transitions in offboard fails its
phases. **After:** the models pass in the dev image (`models_check.txt`: 19 model checks,
then the 9 selfchecks), and so do all six flights.

**The radar test took two tries.** The first version compared GDN's AGL input
(`/gdn/agl_meas`) with the truth paired by bag arrival time. On a tilting x500 that gave
1.34 m RMS, and shifting the pairing by 50 ms moved it to 1.04 or 3.0 m: timing, not the
sensor. The radar scan, recorded in the bag now, carries Gazebo's time like the truth,
so they are compared at the same instant (0.41 m on a short x500 flight). Beyond 35° of
tilt the beam meets the slopes at a grazing angle, where a tenth of a degree of attitude
is metres of range; the adapter drops those readings, so the check does too.

## Results

All six pass (`*_check.txt`).

| World | Vehicle | Airborne | Route points within | Cross-track p95 (max) hovering | as a plane | Lowest above ground | Radar along its beam |
|---|---|---|---|---|---|---|---|
| Zermatt | x500 | 926 s, 8.0 km | 2 m | 1.3 m (3.3) | | 59.1 m | 0.54 m (1775 readings) |
| Zermatt | standard VTOL | 552 s, 8.0 km | 27 m | 1.3 m (1.3) | 6.7 m (34.8) | 55.2 m | 0.36 m (1055) |
| Zermatt | rc_cessna | 531 s, 8.9 km | 28 m | | 6.7 m (25.0) | 57.2 m | 0.39 m (1029) |
| Emmental | x500 | 1016 s, 7.9 km | 2 m | 1.2 m (2.4) | | 59.4 m | 0.65 m (1936) |
| Emmental | standard VTOL | 612 s, 7.8 km | 27 m | 0.7 m (0.8) | 6.1 m (35.1) | 58.1 m | 0.47 m (1182) |
| Emmental | rc_cessna | 551 s, 8.5 km | 28 m | | 7.5 m (25.1) | 56.4 m | 0.51 m (1083) |

- **The planes' cross-track median is about 1 m;** the p95 and the maxima are the 90°
  corners, flown wide (top of the figure).
- **"Route points within" 27 to 28 m** for the planes is the same: the corner points.
- **The cessna's longer track** (8.5 to 8.9 km) is its climb-out on the launch heading
  and the circles at the end.

## GDN's altimeter input (OI-016)

The radar is right where its beam lands. GDN, though, gets range times the cosine of the
tilt (`sim_adapter.py`) and takes it as the height straight below. On a slope, tilted,
the beam lands beside that point, on higher or lower ground. By tilt, Zermatt
(`radar_by_tilt.csv`; on Emmental the straight-below column is 1.4 to 2 times larger):

| Tilt | x500: along the beam / straight below | VTOL | cessna |
|---|---|---|---|
| 0 to 5° | 0.31 / 0.61 m | 0.34 / 0.74 m | 0.33 / 0.91 m |
| 5 to 15° | 0.39 / 2.87 m | 0.38 / 2.25 m | 0.40 / 2.24 m |
| 15 to 25° | 0.60 / 8.15 m | 0.35 / 4.68 m | 0.35 / 3.50 m |
| 25 to 35° | 0.92 / 11.75 m | 0.53 / 8.09 m | 0.56 / 9.76 m |

The x500 is hit hardest: cruising at 10 m/s it's tilted 15° or more about half the time,
while the planes fly the legs level and bank in the turns. Over whole flights, what GDN
got against the height straight below (the adapter's own comparison at each reading,
`flights.txt`):

| | Zermatt | Emmental |
|---|---|---|
| x500 | 6.1 m RMS (worst 10 s: 24 m) | 9.4 m (23 m) |
| standard VTOL | 2.2 m (8 m) | 4.5 m (16 m) |
| rc_cessna | 3.2 m (10 m) | 5.9 m (22 m) |

**This corrects TEST-031, TEST-032 and TEST-033,** which gave the altimeter as 0.29 to
0.34 m RMS "in flight". `sim_run.sh` printed the adapter's last three 10 s reports, and
the last is the landing over the level pad; that's what was quoted. Those reports now
carry a correction, and `sim_run.sh` prints the whole flight's figure instead.

**What it means for GDN** isn't measured yet. The error is under GDN's 10 m altimeter
sigma on average but correlated with the terrain and the flight. And OI-012's runaway
was found on the x500, where this error is largest, with the filter overconfident
whenever altimeter updates are on. Next step, in OI-016: fly the x500 several times with
`GDN_SIM_ADAPTER_ARGS="-p agl_source:=truth"` against as flown. The fix is to predict the
measurement along the beam in GDN's TRN update.

## GDN on the three vehicles (not judged)

| World | Vehicle | GDN horizontal error, 1 to 5 min: RMS (max) | whole run: RMS (max) |
|---|---|---|---|
| Zermatt (GDN's own map) | x500 | 72 m (138) | 4.0 km (12.7) |
| | standard VTOL | 34 m (65) | 0.5 km (1.2) |
| | rc_cessna | 43 m (121) | 1.0 km (2.7) |
| Emmental (GLO-30 map) | x500 | 590 m (1529) | 6.6 km (13.3) |
| | standard VTOL | 2.1 km (4.8) | 10.1 km (21.1) |
| | rc_cessna | 320 m (924) | 1.5 km (3.7) |

GDN ran with the multirotor's settings on all of them (per-airframe profiles are R2.5)
and runs away on every vehicle once its estimate leaves the map, as in OI-012. On
Zermatt the planes start better; one flight each says little, since the same x500 route
scored 72 m and 851 m in two runs. These flights are the baseline for R2.5 and Phase 3,
not a result.

## Also changed

- **`sim_run.sh` records `/gdn/sim/agl_scan`** in the bag (the check needs it), and prints
  the whole flight's altimeter figure.
- **`check_sim_radar.py` reads the radar from `gdn_payload`.** It passes as before on the
  Zermatt and Emmental worlds (26/26; the beam offset of OI-015 unchanged, 1.06 m).
- **`sim_adapter.py`** points Gazebo's follow camera at whichever vehicle flies
  (`gz_model`).
- **`sim_fly.py`** gives up after 3 minutes without reaching cruise height, so a launch
  that never happens ends the run.
- **CI** runs the model check and the selfcheck; `docker_check.sh` runs the model check
  with PX4 and Gazebo.

## Limits

- **The cessna doesn't land,** and its launch is a push, not a modelled catapult. Fine
  for navigation; recovery is Phase 8.
- **GDN is not tuned for the planes** (R2.5) and its altimeter input is off over slopes
  on every vehicle (OI-016).
- **One flight per vehicle and world.** Enough to show they fly; R2.6 to R2.8 bring the
  repeated, scored scenarios.
- **The VTOL's corners** take it up to 35 m off the route; the route was planned for a
  multirotor, with 250 m between legs.
- **Only the radar is in the payload** so far; R2.4 adds the Mid-360S, TFmini-S and OAK-D
  Lite to `gdn_payload`, and every vehicle gets them.

## Reproduce

```bash
tools/docker_dev.sh shell --gpu --gui           # or "run --gpu --" for one command
python3 tools/check_vehicles.py                 # the models
python3 tools/check_vehicles.py --selfcheck
GDN_VEHICLE=standard_vtol tools/sim_run.sh      # with windows; HEADLESS=1 for none
GDN_VEHICLE=rc_cessna GDN_SITE=ch_emmental HEADLESS=1 GDN_LOG_DIR=/tmp/fw tools/sim_run.sh
python3 tools/check_vehicles.py --run /tmp/fw
# the figure, from run directories, then from the committed data:
python3 tools/plot_test034.py --export docs/test_reports/test034_data <run dirs...>
python3 tools/plot_test034.py docs/test_reports/test034_data --out docs/test_reports/test034_vehicles.png
```

## Data (`test034_data/`)

- `<world>_<vehicle>_track.csv`: the true track at 2 Hz (height above the world's
  terrain, plane or not, following or not); `_route.csv`: the planned route;
  `_check.txt`: `check_vehicles.py --run`.
- `<world>_terrain.csv`: the world's terrain at 40 m, for the figure.
- `radar_by_tilt.csv`: the altimeter against the terrain along the beam and straight
  below, by tilt.
- `flights.txt`: each flight's phases, GDN's scores and the whole-flight altimeter input.
- `models_check.txt`, `models_check_on_main.txt`, `vtol_transition_in_offboard_check.txt`:
  the check, after and before.
