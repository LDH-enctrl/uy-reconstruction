# CKKS 동적 제어: TAC 및 주기적 u/y reconstruction

처음 보는 사람이 동일한 plant/controller 문제를 단계적으로 따라갈 수 있도록 구성한 저장소입니다.

## 구성

1. `01_plain_tac` — Python으로 원래 controller와 TAC reformulation 비교
2. `02_plain_uy_reconstruction` — Python으로 주기적 `u,y` 기반 controller-state reconstruction
3. `03_openfhe_ptct_uy_reconstruction` — OpenFHE, 신호는 ciphertext / 계수는 plaintext
4. `04_openfhe_ctct_uy_reconstruction` — OpenFHE, 신호와 controller 계수 모두 ciphertext

모든 단계의 기본 결과는 다음 네 그림으로 통일합니다.

- `u.png`
- `plant_state.png`
- `u_error.png`
- `plant_state_error.png`

성공 여부는 controller-state error 하나가 아니라 실제 control input과 plant trajectory를 우선해서 판단합니다.

## Controller profile

세 가지 preset을 공통으로 사용합니다.

| profile | rho(F) | rho(Acl) | 특징 |
|---|---:|---:|---|
| `stable` | 0.6000 | 0.6893 | Schur-stable F baseline |
| `unstable_low` | 1.0460 | 0.7852 | unstable F, 낮은 internal transient |
| `unstable_high` | 1.0460 | 0.7852 | unstable F, 큰 internal transient |

특히 `unstable_high`는 `unstable_low`의 similarity-transformed realization입니다.

\[
F_h=T F_lT^{-1},\qquad G_h=TG_l,\qquad H_h=H_lT^{-1}.
\]

초기 controller state도 \(x_{c,h}(0)=Tx_{c,l}(0)\)로 변환하므로 **exact arithmetic에서는 두 profile의 u와 plant trajectory가 동일**합니다. 다만 내부 realization의 수치적 증폭과 conditioning은 크게 다릅니다. 따라서 CKKS에서 controller realization 자체가 numerical robustness에 미치는 영향을 비교하기 좋습니다.

자세한 수치는 `docs/CONTROLLER_PROFILES.md`를 참고하면 됩니다.

## Python 실행

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### TAC

```bash
python3 01_plain_tac/tac_plain.py --profile stable
python3 01_plain_tac/tac_plain.py --profile unstable_low
python3 01_plain_tac/tac_plain.py --profile unstable_high
```

### u/y reconstruction

```bash
python3 02_plain_uy_reconstruction/uy_reconstruction_plain.py \
  --profile unstable_low --blocks 50 --period 4 --nu 4
```

Plain 단계에서는 \(K\)와 \(\nu\)를 분리해서 볼 수 있습니다.

```bash
python3 02_plain_uy_reconstruction/uy_reconstruction_plain.py \
  --profile unstable_low --blocks 20 --period 12 --nu 4
```

## OpenFHE pt-ct

```bash
export OPENFHE_ROOT="$HOME/openfhe-development"
./03_openfhe_ptct_uy_reconstruction/scripts/build.sh
./03_openfhe_ptct_uy_reconstruction/scripts/run.sh stable 10
./03_openfhe_ptct_uy_reconstruction/scripts/run.sh unstable_low 10
./03_openfhe_ptct_uy_reconstruction/scripts/run.sh unstable_high 10
```

## OpenFHE ct-ct

```bash
./04_openfhe_ctct_uy_reconstruction/scripts/build.sh
./04_openfhe_ctct_uy_reconstruction/scripts/run.sh stable 10
./04_openfhe_ctct_uy_reconstruction/scripts/run.sh unstable_low 10
./04_openfhe_ctct_uy_reconstruction/scripts/run.sh unstable_high 10
```

OpenFHE 단계는 현재 검증한 회로와 맞추기 위해 `K=nu=4`로 고정합니다. 먼저 3~10 block smoke test를 돌리고 이후 50/100 block으로 늘리는 것을 권장합니다.

## Profile 검증

```bash
python3 tools/validate_profiles.py
```

이 스크립트는 모든 profile에서 observability rank와 closed-loop stability를 확인하고, `unstable_low`와 `unstable_high`의 이상적인 control/plant trajectory가 동일한지도 검사합니다.

## 기존 장기 실험

`experiments/archive_2026-09`에는 지금까지 진행한 long-horizon, same-measurement error-mode, tower-budget, slot-representation 진단 결과를 보관했습니다. 처음 보는 경우에는 먼저 01~04를 순서대로 재현한 뒤 보는 것을 권장합니다.
