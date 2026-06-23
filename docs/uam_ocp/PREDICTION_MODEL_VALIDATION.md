# Prediction model validation

Overall: **PASS**

## P2 consistency

- Test A single-step maximum manifold error: `0.000000e+00` (threshold `1.0e-10`)
- Test B 10-step rollout maximum manifold error: `0.000000e+00` (threshold `1.0e-09`)
- Quaternion-bearing states were compared with `StateMultibody.diff()`, not direct subtraction.

## Linearization

- Source: `Crocoddyl calcDiff`
- Epsilons: `[0.001, 0.0001, 1e-05]`
- Remainder errors: `[8.480711039024282e-05, 8.478431582576177e-07, 8.478199408661951e-09]`
- Successive error ratios: `[100.02688535522053, 100.0027384814043]`
- A tenfold perturbation reduction produces approximately a hundredfold remainder reduction, consistent with a first-order local model.

## Two-second hover prediction

The required nominal hover input uses equal `m_total*g/4` rotor thrust and zero arm torque.
It produced position displacement `1.473195e+01 m`, base angular speed `1.204641e+01 rad/s`, and arm joint-speed norm `1.778461e+02 rad/s`.
For comparison, full-model static trim produced `4.842608e-11 m`, `4.785852e-09 rad/s`, and `1.777737e-06 rad/s` respectively.

The nominal input is not a physical equilibrium because the arm COM is offset and zero arm torque does not compensate arm gravity. The model also excludes servo damping and hard joint-stop reactions. The large nominal drift is therefore reported, not hidden or retuned away.

## Result

- Prediction model: PASS
- P2 consistency: PASS
- Linearization: PASS
- Hover simulation finite: PASS
