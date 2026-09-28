# CKKS dynamic control: TAC and periodic u/y reconstruction

This repository is organized as a step-by-step tutorial for a 4-state SISO dynamic controller. The same plant and selectable controller profiles are reused throughout, so a new reader can move from exact plaintext arithmetic to OpenFHE CKKS without changing the control problem at every step.

## Tutorial path

| step | implementation | coefficients | language |
|---|---|---|---|
| `01_plain_tac` | original controller vs TAC reformulation | plaintext | Python |
| `02_plain_uy_reconstruction` | periodic state reconstruction from \(u,y\) | plaintext | Python |
| `03_openfhe_ptct_uy_reconstruction` | same \(u,y\) reconstruction | plaintext coefficients, ciphertext signals | C++ / OpenFHE |
| `04_openfhe_ctct_uy_reconstruction` | same \(u,y\) reconstruction | ciphertext coefficients and signals | C++ / OpenFHE |

Every stage produces the same four primary figures:

- `u.png`
- `plant_state.png`
- `u_error.png`
- `plant_state_error.png`

The primary success metrics are the control input and plant trajectory. Controller-state error is diagnostic only.

## Controller profile selection

All stages support three presets:

```text
stable
unstable_low
unstable_high
```

The short summary is:

| profile | rho(F) | rho(Acl) | internal transient |
|---|---:|---:|---|
| `stable` | 0.6000 | 0.6893 | low |
| `unstable_low` | 1.0460 | 0.7852 | low |
| `unstable_high` | 1.0460 | 0.7852 | high |

`unstable_high` is a similarity-transformed realization of `unstable_low`. Their exact ideal control/plant trajectories are the same, but the high-transient realization has much larger internal amplification and worse conditioning. See [`docs/CONTROLLER_PROFILES.md`](docs/CONTROLLER_PROFILES.md).

## 0. Python setup

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

The C++ examples assume OpenFHE v1.5.x is available at

```bash
$HOME/openfhe-development
```

or set

```bash
export OPENFHE_ROOT=/path/to/openfhe-development
```

## 1. Plain TAC

```bash
python3 01_plain_tac/tac_plain.py --profile stable
python3 01_plain_tac/tac_plain.py --profile unstable_low
python3 01_plain_tac/tac_plain.py --profile unstable_high
```

The script compares

\[
x_c^+=Fx_c+Gy
\]

with the TAC identity

\[
x_c^+=(F-RH)x_c+Gy+Ru,
\qquad u=Hx_c,
\]

where \(R\) is selected so that \(F-RH\) has user-specified Schur poles.

## 2. Plain u/y reconstruction

Default: reconstruction period \(K=4\), reconstruction horizon \(\nu=4\).

```bash
python3 02_plain_uy_reconstruction/uy_reconstruction_plain.py \
  --profile unstable_low --blocks 50 --period 4 --nu 4
```

You can separate the two parameters in plaintext, for example

```bash
python3 02_plain_uy_reconstruction/uy_reconstruction_plain.py \
  --profile unstable_low --blocks 20 --period 12 --nu 4
```

The state is reconstructed from the last \(\nu\) controls and measurements of each period.

## 3. OpenFHE pt-ct

This stage keeps controller/reconstruction coefficients in plaintext and signals in ciphertext.

```bash
./03_openfhe_ptct_uy_reconstruction/scripts/build.sh
./03_openfhe_ptct_uy_reconstruction/scripts/run.sh stable 10
./03_openfhe_ptct_uy_reconstruction/scripts/run.sh unstable_low 10
./03_openfhe_ptct_uy_reconstruction/scripts/run.sh unstable_high 10
```

The current OpenFHE tutorial path uses \(K=\nu=4\) so it matches the validated baseline circuit.

## 4. OpenFHE ct-ct

Controller and reconstruction coefficients are encrypted once during setup and reused online.

```bash
./04_openfhe_ctct_uy_reconstruction/scripts/build.sh
./04_openfhe_ctct_uy_reconstruction/scripts/run.sh stable 10
./04_openfhe_ctct_uy_reconstruction/scripts/run.sh unstable_low 10
./04_openfhe_ctct_uy_reconstruction/scripts/run.sh unstable_high 10
```

For long runs, first verify the 3- or 10-block smoke test. The high-transient realization is deliberately a numerical stress test.

## Regenerating the shared C++ profile header

The Python profile definitions are the readable source of the presets. Regenerate the C++ header with

```bash
python3 tools/generate_controller_profiles.py
```

This writes `include/controller_profiles.h` and `config/controller_profiles.json` and prints a profile summary.

## Previous numerical-error experiments

The `experiments/` directory contains the earlier long-horizon and diagnostic results that motivated this tutorial structure. Those results are intentionally separated from the four beginner-facing implementations.
