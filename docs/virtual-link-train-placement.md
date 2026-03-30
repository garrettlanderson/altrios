# Virtual Link for Train Placement

## Problem

When placing a train at an origin link in the network, the back of the train may extend beyond the beginning of that link. Previously, the code would walk backwards through previous links (`idx_prev`) to collect enough track length. However, if there were **not enough previous links** to accommodate the train length, the code would panic/bail with an error like:

```
Train too long for initial route: train length (9000.00 m) exceeds available
track length (2682.00 m). The origin link and its previous links do not
provide enough length to place the train.
```

This was the **third case** that could occur:

1. **Train fits on origin link** — no extra links needed.
2. **Train is longer than origin link but previous links exist** — walk backwards and collect them.
3. **Train is longer than origin link and there are no (or not enough) previous links** — previously a panic, now handled with a virtual link.

## Solution

A **virtual link** is created at runtime to extend the track when there aren't enough previous links. The virtual link properties:

| Property | Value |
|---|---|
| **Length** | 5 miles (~8,047 m) |
| **Speed restriction** | 70 mph for the entire link |
| **Heading** | Same as the first heading of the end-of-track link |
| **Elevation** | Same as the first elevation of the end-of-track link (flat) |
| **`idx_curr`** | `network.len()` (appended to end of network) |
| **`idx_next`** | Points to the link it's extending (the one with no previous links) |
| **`idx_prev`** | Fake (default) — it's the new start of the track |

## Files Changed

### `altrios-core/src/meet_pass/est_times/mod.rs`

- **`create_virtual_link` function** (new, `pub(crate)`) — Builds a `Link` with the properties above. Takes the end-of-track link as reference for heading/elevation.

- **`get_links_for_train_placement` function** (modified) — Return type changed from `Result<Vec<LinkIdx>>` to `Result<(Vec<LinkIdx>, Option<Link>)>`. Instead of bailing when no previous links exist, it calls `create_virtual_link` and returns the virtual link alongside the path.

- **`make_est_times` call site** (modified) — When a virtual link is returned, creates a temporary extended network copy with the virtual link appended, updates the connecting link's `idx_prev`, and passes it to `extend_path_tpc`.

- **Tests** — Tests 2 and 3 updated to expect success with a virtual link instead of an error. Virtual link properties (length, speed, connectivity) are verified.

### `altrios-core/src/train/speed_limit_train_sim.rs`

- **`walk_timed_path` function** (modified) — Same pattern as `make_est_times`: creates a virtual link instead of bailing, then uses a temporary extended network for the initial path extension.

- **Added import** — `use crate::meet_pass::est_times::create_virtual_link;`

## How It Works

```
Before (panic case):
  [origin_link] ← no idx_prev → PANIC!

After (virtual link):
  [virtual_link (5 mi, 70 mph)] → [origin_link] → [rest of path...]
```

When the caller receives `Some(virtual_link)`:
1. Clone the network into a temporary `Vec<Link>`
2. Set `extended_network[connecting_link].idx_prev = virtual_link.idx_curr`
3. Push the virtual link onto the extended network
4. Use the extended network for `extend_path_tpc`

The virtual link only exists in the temporary extended network — the original network is not modified.

## Tests

All tests are in `mod test_train_placement` at the bottom of `est_times/mod.rs`:

| Test | Description |
|---|---|
| `test_get_links_for_train_placement_fits_on_single_link` | Train (1800m) fits on origin link (~2682m) — no virtual link needed |
| `test_get_links_for_train_placement_too_long_no_previous` | Train (3000m) exceeds origin link, no prev links — virtual link created |
| `test_get_links_for_train_placement_500_cars` | Train (9000m) — virtual link (5 mi) + origin link provides enough length |
| `test_get_links_for_train_placement_uses_previous_links` | Train slightly longer than origin — collects real previous links |

Run with:
```bash
cargo test test_train_placement
```

