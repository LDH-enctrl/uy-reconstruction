# CKKS 기반 동적 제어기와 $u,y$-Reconstruction 실험

이 저장소는 **동적 제어기의 내부 행렬 $F$가 stable 또는 unstable한 경우**,  
$u,y$ history를 이용한 controller-state reconstruction과 CKKS 기반 암호화 구현에서 발생하는 수치 오차를 비교하기 위한 실험 코드입니다.

처음 보는 사람도 결과를 재현하고 구조를 따라갈 수 있도록 다음 네 단계로 구성했습니다.

1. **Plain TAC formulation**
2. **Plain $u,y$-based state reconstruction**
3. **OpenFHE CKKS $u,y$-reconstruction: pt-ct**
4. **OpenFHE CKKS $u,y$-reconstruction: ct-ct**

기본적인 권장 순서는 `01 → 02 → 03 → 04`입니다.

---

## 1. 문제 설정

Plant는 다음과 같은 discrete-time LTI system을 사용합니다.

$$
x_p(k+1)=Ax_p(k)+Bu(k),
$$

$$
y(k)=Cx_p(k).
$$

동적 제어기는

$$
x_c(k+1)=Fx_c(k)+Gy(k),
$$

$$
u(k)=Hx_c(k)
$$

형태입니다.

전체 closed-loop dynamics는

$$
\begin{bmatrix}
x_p(k+1)\\
x_c(k+1)
\end{bmatrix}
=
\underbrace{
\begin{bmatrix}
A & BH\\
GC & F
\end{bmatrix}}_{A_{\mathrm{cl}}}
\begin{bmatrix}
x_p(k)\\
x_c(k)
\end{bmatrix}.
$$

이 저장소에서는 특히

$$
\rho(F)>1,\qquad \rho(A_{\mathrm{cl}})<1
$$

인 controller realization에서도 암호화 구현이 어떻게 동작하는지 살펴봅니다.

즉, **closed loop는 안정하지만 controller realization 내부에는 unstable mode가 존재할 수 있는 경우**를 주요 관심 대상으로 둡니다.

---

## 2. Controller profile

모든 실험에서는 동일한 인터페이스로 다음 세 가지 controller profile을 선택할 수 있습니다.

### `stable`

내부 controller matrix $F$가 Schur stable인 기준 case입니다.

$$
\rho(F)<1.
$$

암호화 구현에서 가장 안정적인 baseline으로 사용합니다.

---

### `unstable_low`

$$
\rho(F)>1
$$

이지만 finite-horizon 내부 증폭이 비교적 작은 controller realization입니다.

현재 설정에서는

$$
\rho(F)\approx 1.0460,
$$

$$
\rho(A_{\mathrm{cl}})\approx 0.7852.
$$

즉 controller 내부 $F$는 unstable이지만 전체 physical closed loop는 stable입니다.

---

### `unstable_high`

`unstable_low`와 동일한 closed-loop input-output behavior를 가지면서, controller realization 내부의 transient amplification이 더 크게 나타나도록 구성한 case입니다.

두 realization은 exact arithmetic에서 동일한 $u(k)$와 plant trajectory $x_p(k)$를 생성하도록 구성되어 있습니다.

따라서 `unstable_low`와 `unstable_high`를 비교하면 **closed-loop control problem 자체의 차이보다 controller realization의 numerical sensitivity 차이**를 보기 쉽습니다.

---

## 3. Repository 구성

```text
.
├── 01_plain_tac/
├── 02_plain_uy_reconstruction/
├── 03_openfhe_ptct_uy_reconstruction/
├── 04_openfhe_ctct_uy_reconstruction/
├── config/
├── include/
├── tools/
├── docs/
└── experiments/
```

각 파트는 가능한 한 동일한 plant/controller parameter와 동일한 출력 형식을 사용합니다.

---

# 4. Part 1 — Plain TAC formulation

폴더:

```text
01_plain_tac/
```

이 단계에서는 암호화를 사용하지 않고, 기본 동적 제어기와 TAC 형태의 reformulation이 동일한 closed-loop behavior를 만드는지 확인합니다.

기본 controller:

$$
x_c(k+1)=Fx_c(k)+Gy(k),
\qquad
u(k)=Hx_c(k).
$$

TAC 형태:

$$
x_c(k+1)
=
(F-RH)x_c(k)+Gy(k)+Ru(k).
$$

이 단계의 목적은 **controller reformulation 자체가 원래 controller와 동등함을 확인하는 것**입니다.

### 실행 예시

```bash
python3 01_plain_tac/tac_plain.py --profile stable
```

```bash
python3 01_plain_tac/tac_plain.py --profile unstable_low
```

```bash
python3 01_plain_tac/tac_plain.py --profile unstable_high
```

결과는 해당 실험의 `results/` 폴더에 저장됩니다.

주요 출력:

```text
u.png
plant_state.png
u_error.png
plant_state_error.png
sample_trace.csv
```

여기서

$$
e_u(k)=u_{\mathrm{test}}(k)-u_{\mathrm{ref}}(k)
$$

및

$$
e_p(k)=x_{p,\mathrm{test}}(k)-x_{p,\mathrm{ref}}(k)
$$

를 확인할 수 있습니다.

Plain arithmetic에서는 정상 구현 시 두 오차가 floating-point roundoff 수준이어야 합니다.

---

# 5. Part 2 — Plain $u,y$-based state reconstruction

폴더:

```text
02_plain_uy_reconstruction/
```

이 단계에서는 최근의 control/output history를 이용해 controller state를 다시 구성합니다.

Reconstruction horizon을 $\nu$라고 하면

$$
U_t=
\begin{bmatrix}
u(t)\\
u(t+1)\\
\vdots\\
u(t+\nu-1)
\end{bmatrix},
$$

$$
Y_t=
\begin{bmatrix}
y(t)\\
y(t+1)\\
\vdots\\
y(t+\nu-1)
\end{bmatrix}.
$$

Observability matrix는

$$
\mathcal O_\nu=
\begin{bmatrix}
H\\
HF\\
\vdots\\
HF^{\nu-1}
\end{bmatrix}.
$$

$\mathcal O_\nu$가 full column rank이면

$$
U_t
=
\mathcal O_\nu x_c(t)+T_\nu Y_t
$$

로부터 controller state를 복원할 수 있습니다.

결과적으로

$$
x_c(t+\nu)
=
M_uU_t+M_yY_t
$$

형태의 reconstruction을 얻습니다.

---

## Refresh period $K$와 reconstruction horizon $\nu$

두 파라미터는 서로 다른 의미를 가집니다.

- $K$: state reconstruction을 수행하는 주기
- $\nu$: reconstruction에 사용하는 $u,y$ history 길이

따라서 반드시

$$
K=\nu
$$

일 필요는 없습니다.

예를 들어

$$
K=12,\qquad \nu=4
$$

처럼 설정할 수도 있습니다.

### 실행 예시

```bash
python3 02_plain_uy_reconstruction/uy_reconstruction_plain.py \
    --profile unstable_low \
    --blocks 20 \
    --period 4 \
    --nu 4
```

또는

```bash
python3 02_plain_uy_reconstruction/uy_reconstruction_plain.py \
    --profile unstable_low \
    --blocks 20 \
    --period 12 \
    --nu 4
```

실행 시 다음 항목들을 함께 확인하는 것을 권장합니다.

```text
rho(F)
rho(Acl)
rank(O_nu)
cond(O_nu)
||Mu||_inf
||My||_inf
max |u error|
max ||plant-state error||_inf
```

---

# 6. Part 3 — OpenFHE CKKS pt-ct reconstruction

폴더:

```text
03_openfhe_ptct_uy_reconstruction/
```

Part 2의 $u,y$-reconstruction 구조를 CKKS ciphertext에 적용합니다.

이 구현에서는 controller state와 input/output data는 ciphertext로 처리하지만, controller coefficient는 plaintext로 둡니다.

예를 들어

$$
P_i x_c
$$

연산은

$$
\operatorname{pt}(P_i)\odot c_x
$$

형태로 수행합니다.

Reconstruction도

$$
c_{x,\mathrm{next}}
=
\sum_i
\operatorname{pt}(M_u[:,i])
\odot
Boot(c_{u_i})
+
\sum_i
\operatorname{pt}(M_y[:,i])
\odot
c_{y_i}
$$

형태입니다.

따라서 이 단계는 **CKKS encrypted-state implementation의 baseline**으로 사용할 수 있습니다.

---

## OpenFHE 준비

이 파트부터 OpenFHE가 필요합니다.

예시 환경:

```text
Ubuntu / WSL2
C++
OpenFHE
CMake
Python 3 (plot/result processing)
```

OpenFHE 설치 위치를 환경변수로 지정합니다.

```bash
export OPENFHE_ROOT="$HOME/openfhe-development"
```

### 실행 예시

```bash
./03_openfhe_ptct_uy_reconstruction/scripts/run.sh unstable_low 10
```

다른 profile도 동일하게 사용할 수 있습니다.

```bash
./03_openfhe_ptct_uy_reconstruction/scripts/run.sh stable 10
```

```bash
./03_openfhe_ptct_uy_reconstruction/scripts/run.sh unstable_high 10
```

---

# 7. Part 4 — OpenFHE CKKS ct-ct reconstruction

폴더:

```text
04_openfhe_ctct_uy_reconstruction/
```

이 단계에서는 controller coefficient까지 ciphertext로 암호화합니다.

즉

$$
Enc(P_i)\odot Enc(x_c)
$$

및

$$
Enc(M_u[:,i])
\odot
Boot(c_{u_i})
$$

와 같은 ciphertext-ciphertext multiplication을 사용합니다.

이 구현이 현재 장기 numerical-error 분석의 주요 대상입니다.

### 실행 예시

```bash
./04_openfhe_ctct_uy_reconstruction/scripts/run.sh stable 10
```

```bash
./04_openfhe_ctct_uy_reconstruction/scripts/run.sh unstable_low 10
```

```bash
./04_openfhe_ctct_uy_reconstruction/scripts/run.sh unstable_high 10
```

---

# 8. 결과 파일

가능한 한 모든 파트에서 동일한 결과 형식을 사용합니다.

```text
results/
├── u.png
├── plant_state.png
├── u_error.png
├── plant_state_error.png
└── sample_trace.csv
```

### `u.png`

Reference control input과 test/encrypted control input을 비교합니다.

### `plant_state.png`

Reference plant trajectory와 test/encrypted plant trajectory를 비교합니다.

### `u_error.png`

$$
|u_{\mathrm{test}}(k)-u_{\mathrm{ref}}(k)|
$$

를 표시합니다.

### `plant_state_error.png`

$$
\|x_{p,\mathrm{test}}(k)-x_{p,\mathrm{ref}}(k)\|_\infty
$$

를 표시합니다.

이 저장소에서는 **controller-state error 자체를 최종 성공/실패 지표로 사용하지 않습니다.**

주요 성능 평가는

$$
u(k)
$$

와

$$
x_p(k)
$$

의 차이를 기준으로 합니다.

Controller-state numerical error는 내부 오차 메커니즘을 분석하기 위한 diagnostic으로 사용합니다.

---

# 9. 현재까지의 주요 관찰

현재 `unstable_low` ct-ct 실험에서는 block 단위 controller-internal numerical error가

$$
F^K
$$

의 unstable mode와 매우 유사한 증가율을 보였습니다.

특히 $K=4$인 경우

$$
\rho(F)^4
\approx 1.19728
$$

이며, 실제 same-measurement reference와의 controller-state numerical error에서도 이에 매우 가까운 block-wise growth가 반복적으로 관찰되었습니다.

이 현상을 수식적으로 보면, 새로운 HE error가 없다고 가정한 경우 inherited controller-state error는

$$
\delta x_{j+1}
=
F^K\delta x_j
$$

형태로 전달될 수 있습니다.

다만 이것이 곧바로

$$
\rho(F)>1
\Rightarrow
\mathrm{physical\ closed\!-\!loop\ failure}
$$

를 의미하는 것은 아닙니다.

Physical closed-loop stability는

$$
A_{\mathrm{cl}}
$$

에 의해 결정되며, 실제 암호화 구현에서는

- controller-internal numerical error
- bootstrap approximation error
- ciphertext arithmetic error
- replicated scalar representation의 불일치
- actuator-visible control error

를 구분해서 볼 필요가 있습니다.

---

# 10. 추가 실험

고급 진단 실험은

```text
experiments/
```

아래에 분리해 두었습니다.

주요 실험은 다음과 같습니다.

### Long-horizon experiment

장시간 실행에서 control/plant error 및 CKKS approximation failure를 확인합니다.

### Same-measurement controller-error experiment

Reference controller와 encrypted controller에 동일한 measurement $y$를 입력해 controller-side numerical error만 분리합니다.

### Post-bootstrap budget experiment

Bootstrap 이후 남는 RNS tower/modulus budget을 변경해 error growth의 원인을 확인합니다.

### Control-representation consistency experiment

하나의 scalar control $u_i$를 표현하는 ciphertext가 이상적으로

$$
[u_i,u_i,u_i,u_i]
$$

형태를 유지하는지 확인합니다.

실제 CKKS 연산에서는 slot별 오차가 다르게 발생할 수 있으므로, 이를 actuator-visible scalar control error와 controller-internal representation error로 나누어 분석합니다.

---

# 11. 권장 실행 순서

처음 저장소를 보는 경우 아래 순서를 권장합니다.

### Step 1

```bash
python3 01_plain_tac/tac_plain.py --profile stable
```

### Step 2

```bash
python3 01_plain_tac/tac_plain.py --profile unstable_low
```

### Step 3

```bash
python3 02_plain_uy_reconstruction/uy_reconstruction_plain.py \
    --profile unstable_low \
    --blocks 20 \
    --period 4 \
    --nu 4
```

### Step 4

OpenFHE 환경을 준비한 뒤

```bash
./03_openfhe_ptct_uy_reconstruction/scripts/run.sh unstable_low 10
```

### Step 5

마지막으로 ct-ct 구현을 실행합니다.

```bash
./04_openfhe_ctct_uy_reconstruction/scripts/run.sh unstable_low 10
```

그 이후 `stable`, `unstable_low`, `unstable_high`를 서로 비교하는 것을 권장합니다.

---

# 12. Controller profile 비교 시 주의할 점

`stable`, `unstable_low`, `unstable_high`를 비교할 때 단순히

$$
\rho(F)
$$

만 보는 것은 충분하지 않습니다.

특히 unstable realization에서는 다음 값들도 함께 확인하는 것이 좋습니다.

$$
\max_{1\le i\le K}\|F^i\|,
$$

$$
\|M_u\|,
$$

$$
\operatorname{cond}(\mathcal O_\nu),
$$

그리고 실제 encrypted implementation에서의

$$
|e_u(k)|,
\qquad
\|e_p(k)\|_\infty.
$$

`unstable_low`와 `unstable_high`는 가능한 한 동일한 ideal closed-loop behavior를 유지하면서 controller realization의 internal numerical amplification 차이를 비교하기 위한 profile입니다.

---

# 13. 현재 연구 질문

현재 관심 있는 핵심 질문은 다음과 같습니다.

> Stable physical closed loop 내부에서 CKKS 구현으로 추가되는 numerical-error dynamics는 어떤 형태를 가지며, 이 dynamics는 controller realization $F$, refresh period $K$, reconstruction horizon $\nu$와 어떻게 결합되는가?

특히 일반적인 $K,\nu$에서

$$
\delta x_{j+1}
=
F^K\delta x_j+d_j
$$

형태의 controller-internal error model을 분석하고,

이를 실제 control error

$$
e_u
$$

및 plant-state error

$$
e_p
$$

로 연결하는 것이 현재 분석의 주요 방향입니다.

---

## Repository

GitHub:

`LDH-enctrl/uy-reconstruction`

이 저장소는 현재 연구 진행 과정에서 계속 업데이트될 예정입니다.
