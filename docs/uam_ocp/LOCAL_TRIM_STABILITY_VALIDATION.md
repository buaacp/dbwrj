# Local trim stability validation

Overall: **PASS**

Selected configurations: `['neutral', 'fully_extended', 'p2_terminal', 'left_offset']`

## Automatic margin selection

- Lowest rotor margin: `3.990211e+00 N` at `left_offset`, rotor `2` (`front_left`), `lower` side.
- Lowest joint torque margin: `1.998464e-01 N m` at `fully_extended`, joint 6 `left_knuckle_joint`.

## Spectral and nonlinear recovery results

| Configuration | rho open | rho closed | Open unstable | Closed unstable | Controllability rank | Stabilizable | Closed cases recovered |
|---|---:|---:|---:|---:|---:|---|---:|
| neutral | 1.040679 | 0.990257 | 4 | 0 | 24/24 | True | 4/4 |
| fully_extended | 1.267737 | 0.990255 | 4 | 0 | 24/24 | True | 4/4 |
| p2_terminal | 1.100689 | 0.990258 | 4 | 0 | 24/24 | True | 4/4 |
| left_offset | 1.129815 | 0.990262 | 4 | 0 | 24/24 | True | 4/4 |

## Aggregate

- Maximum closed-loop spectral radius: `9.902622e-01`
- Maximum sustained recovery time: `2.210000e+00 s`
- Maximum rotor thrust occupancy: `4.486343e-01`
- Maximum joint torque occupancy: `2.424349e-01`
- Maximum saturated-step ratio: `0.000000e+00`

Open-loop recovery is expected to fail for position offsets and unstable arm modes because a fixed equilibrium input contains no restoring feedback. Closed-loop PASS means recovery only for the tested nominal-model local perturbations.
