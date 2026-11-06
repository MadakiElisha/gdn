# TEST-020: separate truth world and onboard maps (R1.3)

**Question.** Until now the Monte Carlo and the bench simulated every height
measurement from the same map the filter navigates with, so the onboard map was
perfect. A real onboard map comes from a satellite DEM: noisy, possibly offset, and
coarser than the ground (REQ-DB-002). How much does that cost, and does the filter stay
honest about it?

**Pass rule.** The MC and bench can fly over one map and navigate with another, in a
shared frame; degraded maps are reproducible from `make_data.sh`; and results are
reported with the consistency check (REQ-INT-001) as well as accuracy.

## What changed

- `MapDb::SetOrigin(lat, lon)`. Each map's local frame used to be its own grid centre,
  so two maps cut or sampled differently gave the same local coordinate two different
  places on Earth (165 m apart in the unit test). The onboard map is now put in the
  truth map's frame. Node: `origin_lat` / `origin_lon` parameters (default: the map's
  own centre).
- `tools/make_db_map.py`: builds an onboard map from a truth map with any mix of
  correlated height noise (`--noise`, `--corr`: DEM errors come in patches), a constant
  `--bias`, a coarser grid (`--coarsen`, box average, same geographic footprint) and
  `--smooth`. Prints the height error against the truth. With no options it writes the
  input back byte for byte.
- `tools/make_data.sh --db-maps`: builds `data/maps/{zermatt,sigma4}_db_{noise5,
  noise10,bias10,coarse3,srtm}.bin` with fixed seeds. "srtm" is 3x coarser cells
  (about 3 arc-seconds against a 1 arc-second truth), 6 m noise in 200 m patches and a
  3 m bias.
- `mc_campaign --db-map=PATH` (the filter's map; the positional map stays the truth)
  and `--seed0=N`. `bench.sh`: `GDN_DB_MAP` for the node, put in `GDN_MAP`'s frame.
- `tools/mc_map_ensemble.py`: builds K maps from one recipe with different noise seeds,
  flies distinct flights over each, and pools every sample (see the first finding for
  why). Output in mc_campaign's format, so `ci_check_mc.py` reads it.
- `bench.sh` and `bench_viz.sh` take a lock (`log/bench.lock`): a second bench on the
  same machine now refuses to start instead of killing the first one's processes,
  which is what happened when two ran at once during this test.
- CI: Zermatt, SRTM-like onboard maps, 5 patterns x 20 flights, node start-up, AGL;
  guard CEP50 at most 35 m and at least 80 of 100 passing (today 25.6 m, 91/100).

The map handling is vehicle-independent: the same maps and tools serve the
fixed-wing recordings now and the x500 and quadplane SITL runs later.

## Unit test (selftest, -O2 and -O3)

Same terrain sampled into two maps whose centres are 0.00437 deg N and 0.00331 deg E
apart (not a whole number of cells): with a shared origin they agree to 0.71 m (the
bilinear sampling difference) and their gradients match; with each map's own centre
they disagree by 165 m. The control fails as expected; the shared frame passes.

## Results: Zermatt, AGL model, node start-up

Each row pools 10 onboard maps (different error patterns) x 20 flights = 200 flights,
scored 60 to 300 s. Map error is the RMS against the truth at the truth grid points.

| Onboard map | Map error RMS | CEP50 | CEP95 | Passing | Error > 3 sigma (REQ-INT-001 <= 1%) | NEES median |
|---|---|---|---|---|---|---|
| Truth (no error) | 0 | 11.3 m | 313 m | 171 | 14.8% | 1.09 |
| Noise 2.5 m | 2.5 m | 13.8 m | 358 m | 165 | 19.5% | 1.74 |
| Noise 5 m | 5 m | 21.9 m | 628 m | 142 | 30.9% | 4.51 |
| Noise 10 m | 10 m | 164 m | 901 m | 80 | 62.7% | 74.6 |
| 3x coarser | 4.9 m (max 61 m) | 12.7 m | **44.6 m** | **200** | **0.10%** | 0.94 |
| SRTM-like (coarser + 6 m noise + 3 m bias) | 8 to 10 m | 27.1 m | 199 m | 181 | 13.1% | 4.67 |

Flights passing per map, out of 20, for the noise-10 m recipe: 19, 2, 0, 6, 12, 1, 16,
0, 4, 20.

A constant 10 m bias (single map, 100 flights) leaves the AGL model's horizontal result
unchanged (10.8 m, 87/100, identical to the truth map): the altitude datum is set from
map + AGL at start-up and absorbs it. Only absolute altitude moves (p95 4.1 m to
10.6 m). The old oracle model (`--model=ideal`, terrain height measured directly) with
the same bias: 1163 m CEP50, 0 of 100.

## Results: flight001, AGL model, 5 m baro drift

World map: `terrain_db_sigma4.bin` (already smoothed to sigma 4 cells, gentler
terrain). Truth start-up, as in the CI configuration. 10 maps x 20 flights per row.

| Onboard map | Map error RMS | CEP50 | CEP95 | Passing | Error > 3 sigma (REQ-INT-001 <= 1%) | NEES median |
|---|---|---|---|---|---|---|
| Truth (no error) | 0 | 25.2 m | 63.5 m | 200 | **0.02%** | 0.87 |
| Noise 2.5 m | 2.5 m | 26.4 m | 71.4 m | 198 | **0.66%** | 0.92 |
| Noise 5 m | 5 m | 30.0 m | 85.3 m | 196 | 1.35% | 1.13 |
| Noise 10 m | 10 m | 47.4 m | 591 m | 172 | 9.9% | 2.31 |
| 3x coarser | 2.3 m (max 16.5 m) | 49.4 m | 759 m | 176 | 10.2% | 1.83 |
| SRTM-like | 7 m | 48.7 m | 744 m | 171 | 10.8% | 1.99 |

The constant 10 m bias again leaves the horizontal result unchanged (25.6 m, 100/100,
single map); altitude p95 goes from 4.1 m to 11.4 m.

## Results: ROS bench

Zermatt bag, `GDN_MAP=zermatt.bin` (truth, for the simulators) and
`GDN_DB_MAP=zermatt_db_srtm.bin` (the node), one run. The node loaded the 43 x 43
onboard map with the truth map's origin (45.945278, 7.661944), set its altitude datum
from it (3043.3 m AMSL, 4 m below the truth-map datum because of the onboard map's
error under the start point) and latched airborne at t = 30.5 s.

| Onboard map | RMS 60 to 300 s | MAX 60 to 300 s |
|---|---|---|
| Truth map (TEST-018, two runs) | 11.3 m, 20.8 m | 23.8 m, 54.6 m |
| `zermatt_db_srtm.bin` | 56.0 m | 107.2 m |

In line with the MC on this particular map (CEP50 77.7 m, 44 of 100 passing); the
ensemble shows it is one of the unlucky patterns.

## Findings

1. **One onboard map is one sample of map error.** Every flight in a run shares the
   same error pattern, so a single map can be lucky or unlucky: on single maps, 5 m of
   noise scored worse than 10 m on both flights, and per-map pass rates at 10 m range
   from 0 to 20 of 20. Map error has to be evaluated over an ensemble of patterns
   (`mc_map_ensemble.py`), which is what the tables use.
2. **Map noise costs accuracy steadily, and honesty faster.** On Zermatt, passing
   flights go 171, 165, 142, 80 of 200 as noise goes 0, 2.5, 5, 10 m, while the NEES
   median goes from 1.1 to 75: the filter doesn't know its map is wrong. `r_agl`
   (10 m sigma) already budgets for map height error, but correlated noise also bends
   the map gradient, and the gradient is what moves the position. That error isn't in
   the model at all.
3. **The best onboard resolution depends on the terrain.** On Zermatt (raw 1
   arc-second map, rough mountains) the 3x coarser map passes all 200 flights and meets
   REQ-INT-001 (0.10%), and even the SRTM-like map beats the perfect full-resolution
   one (181 against 171): coarse cells average out the small ridges that look alike
   and cause false locks. On flight001 (map already smoothed to sigma 4, gentle
   terrain) the same coarsening throws real information away: 200 to 176 passing, and
   REQ-INT-001 goes from 0.02% to 10.2%. So "coarser" is not a fix. The onboard map's
   resolution and smoothing have to be matched to terrain roughness, which REQ-DB-001
   (sigma 4 smoothing) does for flight001 and `zermatt.bin` does not. That choice
   belongs with R1.4 (gradient consistent with interpolation) and the TRN work in
   Phase 4; this item doesn't change the default maps.
5. **On gentle terrain the filter tolerates small map error honestly.** flight001
   meets REQ-INT-001 with up to 2.5 m of map noise (0.66%) and just misses at 5 m
   (1.35%). An SRTM-like onboard map (about 7 m RMS against truth) costs it
   29 of 200 flights and REQ-INT-001 (10.8%).
4. **A constant map bias is harmless to the AGL model** and fatal to the old oracle,
   which confirms the R1.1 measurement model.

## Limits

- Both maps derive from the same SRTM tile. A truly independent source (1 to 5 m DEM
  world, SRTM or GLO-30 onboard) arrives with the Gazebo worlds (R2.2).
- The noise model is Gaussian, stationary and isotropic. Real DEM error grows on steep
  slopes and has voids and radar shadows.
- The pooled ensemble output has no NIS line (the trace doesn't carry innovations), and
  `ci_check_mc.py` refuses an NIS threshold on it rather than passing it silently.

## Reproduce

```bash
tools/run_selftest.sh
tools/make_data.sh --db-maps
tools/mc_run.sh 100 data/flight_zermatt_csv data/maps/zermatt.bin --model=agl --startup=node \
    --db-map=data/maps/zermatt_db_srtm.bin
python3 tools/mc_map_ensemble.py --world data/maps/zermatt.bin --csv data/flight_zermatt_csv \
    --recipe "--noise 5 --corr 150" --maps 10 --per-map 20 -- --model=agl --startup=node
GDN_BAG=data/flight_zermatt GDN_MAP=data/maps/zermatt.bin \
    GDN_DB_MAP=data/maps/zermatt_db_srtm.bin tools/bench.sh
```
