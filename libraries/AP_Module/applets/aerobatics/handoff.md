# Aerobatics handoff snapshot, version 1

Rust, Ada and Fortran use shared block ID `0x4145524f`, size 2048. All fields
are little-endian 32-bit words. Floating values contain IEEE float32 bits.
Unused words are zero. Changes to this layout or path reconstruction semantics
require incrementing word 1 in all three implementations.

| Words | Contents |
| --- | --- |
| 0 | Magic `0x4145524f` |
| 1 | Schema version, 1 |
| 2 | Owning shared-state handle token, or 0 after an explicit yield |
| 3 | 1 for a captured runner; 0 for the initial empty runner |
| 4 | Active controller flag, 0 or 1 |
| 5–6 | Vehicle owner: 0 none, 1 RC, 2 mission; RC mode or mission command ID |
| 7–8 | Last delivered command time in microseconds, low then high word |
| 9 | Global controller iteration count |
| 10 | Last RC selection, one-based; 0 for none |
| 11–13 | Last RC action, mission command ID and preloaded mission index; `0xffffffff` for none |
| 14 | Loaded trick ID |
| 15–18 | Four trick arguments |
| 19–21 | Original path load speed, entry rate and box width |
| 22 | FNV-1a-32 of exact schedule file bytes; 0 for a built-in maneuver |
| 23 | Composite cache record count, at most 112 |
| 24–59 | Controller state, below |
| 60–63 | Reserved |
| 64 onward | Four words per composite cache record, in tree preorder |

The controller's 36 words, relative to word 24, are:

| Offset | Contents |
| --- | --- |
| 0 | Iteration count |
| 1–3 | Target speed, path length, total path time |
| 4–7 | Initial orientation quaternion, q1 through q4 |
| 8–10 | Initial position, x/y/z |
| 11–14 | Accumulated orientation quaternion |
| 15 | Previous observation time in seconds |
| 16–18 | Previous commanded rates, x/y/z |
| 19 | Normalized schedule position `path_t` |
| 20–22 | Previous target position |
| 23 | Filtered roll |
| 24–26 | XY shift vector |
| 27 | Previous XY shift attribute, 0 or 1 |
| 28–30 | Previous path tangent |
| 31–35 | Speed PI: P, I, integral, total and previous update time |

Each composite record contains child count (plus bit 16 for speed-dependent
barrel banking), highest announced child (one-based, 0 for none), last sampled
child (zero-based), and the current barrel bank angle (0 for other composites).
Arena node numbers never appear on the wire. Leaf geometry, segment transforms,
attributes and alignment distances are reconstructed from the original recipe;
the file fingerprint and composite shape checks reject incompatible paths.

Only a worker holding word 2 may run the vehicle controller. On deselection it
commits the complete snapshot with word 2 cleared, using the revision returned
by its claim. It then drops local vehicle ownership without acknowledging a
mission item or disabling RC control. The selected incoming worker reads the
entire block, claims word 2 with a revision-conditional write, restores state,
and rechecks selection, the command deadline and vehicle ownership before
stepping. Other workers remain asleep. This also handles rapid selector changes:
a worker selected during reconstruction yields the restored state on its next
tick if another language has since been selected.

An all-zero block represents startup. It never captures a new worker's held
execute switch as a start request. A complete `MOD_RESTART` closes every reference
and resets the block on the next open. Existing one-second watchdog, mode-change,
mission-expiry and RC-abort behavior continues to apply. Ownership is never stolen
on a timer: a failed owner requires a module restart, preventing a late command
from an old worker from racing with its replacement.
