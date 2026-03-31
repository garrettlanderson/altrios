# Train Placement at Origin

## Overview

When a train departs from an origin, the simulation must position the train on
the network at the specified origin `(link_idx, offset)` *before* the
simulation time-step loop begins.  The `Location` struct in the origin list
carries a field `is_front_end` that determines **which end of the train** is
pinned to the `(link_idx, offset)` reference point.

## Placement Modes

### TAIL-end placement (`is_front_end = false`, default)

The reference point `(link_idx, offset)` is the **tail** of the train.

* The head of the train extends `train_length` forward from the tail.
* The origin link's full length is used as the available *backward* context
  for braking-physics initialisation.  If the full link length is still
  shorter than `train_length`, real previous links (via `idx_prev`) are
  collected; if those are exhausted, a **virtual link** is created (see
  below).
* The simulation's initial offset is set to
  `orig.offset + train_length` inside the extended path.
* Dispatching seed events use `dist_to_next = orig.offset` for the *Arrive*
  event and `dist_to_next = orig.offset + train_length` for the *Clear* event.

This is the **original algorithm** and remains the default.  It is equivalent
to "head at end of link" when `orig.offset = 0` and
`train_length ≤ link.length`.

### HEAD-end placement (`is_front_end = true`)

The reference point `(link_idx, offset)` is the **head** (front locomotive)
of the train.

* The head is pinned at `offset` metres from the start of `orig.link_idx`;
  the tail extends `train_length` backward.
* `offset` itself is the available backward space on the origin link.  When
  `offset < train_length`, additional previous links — or a virtual link —
  are collected.
* After extending the path, the simulation's initial offset is explicitly set
  to `path_end - link.length + orig.offset` so that the head is exactly at
  the specified position.  `recalc_braking_points` is called afterward.
* Dispatching seed events: `dist_to_next = orig.offset` (Arrive),
  `dist_to_next = orig.offset + train_length` (Clear).
* **The head position is consistent regardless of train length** — only the
  required backward path length changes.

## Virtual Link

When there are no (or insufficient) previous links to accommodate the train
body behind the reference point, a **virtual link** is created at runtime:

| Property | Value |
|---|---|
| **Length** | 5 miles (~8,047 m) |
| **Speed restriction** | 70 mph for the entire link |
| **Heading** | Same as the first heading of the end-of-track link |
| **Elevation** | Flat (same as first elevation of the end-of-track link) |
| **`idx_curr`** | `network.len()` (appended to end of network) |
| **`idx_next`** | Points to the link it's extending (the one with no real previous links) |
| **`idx_prev`** | Fake (default) — it is the new start of the track |

The virtual link only exists in a temporary extended network; the original
network is never modified.

```
Before (error case):
  [origin_link]  ← no idx_prev, train too long → error!

After (virtual link):
  [virtual_link (5 mi, 70 mph)] → [origin_link] → [rest of path…]
```

## Algorithm Summary

```
head_dist_in_link =
  is_front_end ? orig.offset
              : network[orig.link_idx].length    // full link for TAIL mode

link_path = get_links_for_train_placement(
    orig.link_idx, head_dist_in_link, train_length, network)
# → [virtual_link?, prev_links…, origin_link]

extend_path_tpc(link_path)

if is_front_end:
    head_pos = path_end - link.length + orig.offset
    train_sim.state.offset = head_pos
    recalc_braking_points()
# else: default initial offset (= train_length) applies
```

## `is_front_end` in `locations.csv`

The `Is Front End` column in a `locations.csv` file controls this behaviour
for every train that uses a given location as its origin:

| Value | Placement | Reference point |
|---|---|---|
| `false` (default) | TAIL-end | Tail at `(link_idx, offset)` |
| `true` | HEAD-end | Head at `(link_idx, offset)` |

## Files Changed

### `altrios-core/src/meet_pass/est_times/mod.rs`

* **`get_links_for_train_placement`** — signature extended with a
  `head_dist_in_link: si::Length` parameter (the available backward space on
  the origin link).  The first check now compares `head_dist_in_link` against
  `train_length` instead of the hard-coded link length.

* **`make_est_times` origin loop** — the two `ensure!` guards that required
  `orig.offset == 0` and `!orig.is_front_end` have been removed.
  `head_dist_in_link` is computed per origin, and for HEAD-end placement the
  train's initial offset is corrected after `extend_path_tpc`.

### `altrios-core/src/train/speed_limit_train_sim.rs`

* **`recalc_braking_points`** — visibility changed from private to `pub` so
  that `make_est_times` can trigger a recalculation after repositioning the
  train for HEAD-end placement.

## Tests

All unit tests live in `mod test_train_placement` inside `est_times/mod.rs`
and `mod test_train_disp` inside `train_disp/mod.rs`:

| Test | Description |
|---|---|
| `test_get_links_for_train_placement_fits_on_single_link` | TAIL-end: train (1800 m) fits on origin link (~2682 m) — no virtual link |
| `test_get_links_for_train_placement_too_long_no_previous` | TAIL-end: train (3000 m) exceeds origin link, no prev links — virtual link created |
| `test_get_links_for_train_placement_500_cars` | TAIL-end: 500-car train (9000 m) — virtual link + origin provide enough length |
| `test_get_links_for_train_placement_uses_previous_links` | TAIL-end: train slightly longer than origin — collects real previous links |
| `test_get_links_for_train_placement_head_end_zero_offset` | HEAD-end at offset 0 — always requires virtual / previous links |
| `test_get_links_for_train_placement_head_end_partial_offset` | HEAD-end at offset 2000 m — fits on origin link without virtual link |
| `test_make_train_fwd` | Full pipeline: TAIL-end, `is_front_end=false`, offset 0 |
| `test_make_train_rev` | Full pipeline: reverse direction, TAIL-end |
| `test_make_train_fwd_head_end` | Full pipeline: HEAD-end, `is_front_end=true`, offset 0 |
| `test_make_train_fwd_head_end_nonzero_offset` | Full pipeline: HEAD-end, `is_front_end=true`, offset 1000 m |

Run all placement tests with:
```bash
cargo test test_train_placement
cargo test test_make_train
```

