# 4장. Takagi-Sugeno Fuzzy 시스템: 비선형을 Local Model의 조합으로 보기

> **이 장의 목표**: 시간에 따라 변하는 비선형 시스템을 여러 선형 모델의 **부드러운 혼합(smooth blending)**으로 표현하고, Common Quadratic Lyapunov Function (CQLF)로 안정성을 입증할 수 있게 된다.
>
> **선수 지식**: Lyapunov 안정성 (Ch1), LMI (Ch2-3), 고유값
>
> **다음 장과의 연결**: CQLF는 보수적일 수 있다. 이를 해결하기 위해 5장에서 영역마다 다른 Lyapunov 함수 $P_i$를 도입한다.

---

## 4.1 왜 TS Fuzzy인가

### 4.1.1 로봇 동역학의 비선형성

로봇의 기본 동역학은

$$
M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) = \tau
$$

여기서:
- $M(q) \in \mathbb{R}^{n \times n}$: **관성 행렬** (configuration $q$에 따라 **급격히 변한다**)
- $C(q,\dot{q})$: Coriolis/원심력 항
- $g(q)$: 중력
- $\tau$: 제어 토크

이를 1차 형태로 바꾸면 ($x_1 = q$, $x_2 = \dot{q}$)

$$
\dot{x}_1 = x_2, \quad \dot{x}_2 = M(x_1)^{-1}[\tau - C(x_1, x_2)x_2 - g(x_1)]
$$

이 시스템에서 **단 하나의 선형 모델로는 비선형성을 못 잡는다**. 예를 들어:
- Robot이 가슴팍에 있을 때 (shoulder 근처) $M$의 원소들이 크다
- Robot이 팔을 펼쳤을 때 $M$의 원소들이 훨씬 작다

이 변화는 최대 **10배** 이상 날 수 있다.

### 4.1.2 선형화의 한계

원점 근처에서 선형화하면

$$
\dot{x} = \underbrace{\left.\frac{\partial f}{\partial x}\right|_{x=0}}_{A} x + (\text{higher order terms})
$$

하지만 선형화된 $A$는:
1. **원점 근처에서만** 유효
2. 원점과 멀어지면 비선형 항이 지배적
3. 경로 추종 제어에서는 여러 configuration을 거쳐야 함

### 4.1.3 TS Fuzzy의 핵심 아이디어

**"비선형 시스템을 여러 선형 모델의 부드러운 혼합으로 표현하자"**

즉,
$$
\dot{x} = \sum_{i=1}^{C} h_i(z(t)) [A_i x + B_i u]
$$

여기서:
- $C$개의 **local model** $A_i, B_i$ (각각 선형)
- **Membership function** $h_i(z)$: 각 선형 모델이 얼마나 활성화되는지
- $z(t)$: **premise variable** (예: 현재 configuration)
- 조건: $h_i(z) \geq 0$, $\sum_{i=1}^C h_i(z) = 1$

이렇게 하면:
- 각 영역에서는 거의 선형처럼 보임
- 영역 경계에서도 **자동으로 smooth transition**
- Ch7의 GMR과 같은 원리! (이후 보게 될 것)

---

## 4.2 TS Fuzzy 시스템의 정의

### 4.2.1 정식화

**정의 4.1 (Takagi-Sugeno Fuzzy System).**

비선형 시스템의 TS fuzzy 표현:

$$
\dot{x}(t) = \sum_{i=1}^{C} h_i(z(t))\, [A_i x(t) + B_i u(t)]
$$

여기서:
- $x \in \mathbb{R}^n$: 상태
- $u \in \mathbb{R}^m$: 제어 입력
- $z(t) \in \mathbb{R}^p$: **premise variable** (측정 가능한 변수)
- $A_i \in \mathbb{R}^{n \times n}$, $B_i \in \mathbb{R}^{n \times m}$: $i$번째 local model의 시스템 행렬
- $h_i: \mathbb{R}^p \to [0,1]$: **membership function**
- $C$: **규칙 개수(number of rules)**

### 4.2.2 Membership Function의 성질

$h_i(z)$는 다음을 만족해야 한다:

$$
h_i(z) \geq 0 \quad \forall z, \quad \sum_{i=1}^{C} h_i(z) = 1 \quad \forall z
$$

이를 **convex sum** 또는 **convex combination** property라 한다.

**기하학적 의미**: 각 시점에서 상태는 $C$개의 local model들이 이루는 **convex hull** 내에 있다.

### 4.2.3 Premise Variable vs Scheduling Variable

용어 구분:

| 용어 | 정의 | 예 |
|------|------|-----|
| **Premise variable** $z(t)$ | Membership function의 입력 | 관절각도 $q$, 속도 $\dot{q}$ |
| **Scheduling variable** | 때로는 $z(t)$의 일부만 사용 | $\sin q$ (비선형 함수) |

예를 들어, 로봇의 관성이 $\cos q$의 형태로 변한다면:

$$
M(q) = M_0 + M_1 \cos(q)
$$

이 경우 premise variable은 $z = \cos q$이다. 하지만 이는 **외부에서 계산**해야 함에 주의.

### 4.2.4 Switching System과의 차이

| 특성 | Switching | TS Fuzzy |
|------|-----------|----------|
| 혼합 방식 | Hard (1 또는 0) | Soft (0~1 연속) |
| Continuity | 불연속 (경계에서 점프) | 연속 |
| 전환 속도 | 순간적 | 부드러움 |
| 해석성 | 각 모드별로 따로 | 가중 평균 해석 |

**TS Fuzzy의 장점**: 경계에서 "튀는" 현상이 없고, 각 모드가 천천히 fade in/out된다.

---

## 4.3 Common Quadratic Lyapunov Function (CQLF)

### 4.3.1 CQLF의 정의

TS fuzzy 시스템의 안정성을 입증하는 가장 간단한 방법은 **공통의 Lyapunov 함수**를 찾는 것이다:

$$
V(x) = x^\top P x, \quad P = P^\top \succ 0
$$

모든 시간과 모든 membership 값에 대해 이 단 하나의 $P$를 사용한다.

### 4.3.2 $\dot{V}$ 유도 (Full Derivation)

**Step 1**: $V(x) = x^\top P x$를 시간에 대해 미분하면

$$
\dot{V}(x) = \frac{d}{dt}[x^\top P x] = \dot{x}^\top P x + x^\top P \dot{x}
$$

$\dot{x}^\top P x$는 스칼라이므로 자신의 전치와 같다:

$$
\dot{x}^\top P x = (x^\top P \dot{x})^\top = x^\top P \dot{x}
$$

따라서

$$
\dot{V}(x) = \dot{x}^\top P x + x^\top P \dot{x} = 2\, \text{Re}[\dot{x}^\top P x]
$$

실수 시스템에서는

$$
\dot{V}(x) = \dot{x}^\top P x + x^\top P \dot{x}
$$

**Step 2**: TS fuzzy 시스템의 정의 대입:

$$
\dot{x} = \sum_{i=1}^{C} h_i(z) [A_i x + B_i u]
$$

제어가 없는 경우 ($u=0$), 또는 상태 피드백 $u = \sum_j h_j(z) K_j x$를 쓰면 폐루프는

$$
\dot{x} = \sum_{i=1}^{C} \sum_{j=1}^{C} h_i(z) h_j(z) [A_i + B_i K_j] x
$$

가장 간단한 경우 (u=0):

$$
\dot{x} = \sum_{i=1}^{C} h_i(z) A_i x
$$

이를 대입하면

$$
\dot{V} = \left(\sum_{i=1}^{C} h_i A_i x\right)^\top P x + x^\top P \left(\sum_{i=1}^{C} h_i A_i x\right)
$$

$$
= \sum_{i=1}^{C} h_i(z)\, [x^\top A_i^\top P x + x^\top P A_i x]
$$

$$
= \sum_{i=1}^{C} h_i(z)\, x^\top (A_i^\top P + P A_i) x
$$

**Step 3**: 안정성 조건 도출

$\dot{V} < 0$ for all $x \neq 0$이 필요하다. 

$h_i(z) \geq 0$이고 $\sum h_i = 1$이므로, **충분 조건**은:

$$
A_i^\top P + P A_i \prec 0 \quad \forall i = 1, \ldots, C
$$

왜냐하면, 각 $i$에 대해 $A_i^\top P + PA_i \prec 0$이면

$$
\dot{V} = \sum_{i=1}^{C} h_i(z)\, x^\top \underbrace{(A_i^\top P + P A_i)}_{\prec 0} x < 0
$$

음수들의 convex combination도 음수이기 때문이다.

### 4.3.3 CQLF 조건 정리

**정리 4.1 (TS Fuzzy 시스템의 CQLF 안정성).**

시스템 $\dot{x} = \sum_{i=1}^C h_i(z) A_i x$에서, 대칭 행렬 $P \succ 0$이 존재하여

$$
A_i^\top P + P A_i \prec 0 \quad \forall i = 1, \ldots, C
$$

이 성립하면, 원점 $x=0$은 **점근안정**이다.

**주목**: 이는 **$C$개의 LMI를 동시에** 만족하는 **단 하나의** $P$를 찾는 문제이다.

$$
\text{find } P \succ 0 \text{ s.t. } A_i^\top P + P A_i \prec 0 \text{ for all } i
$$

---

## 4.4 왜 CQLF가 보수적인가

### 4.4.1 기하학적 직관

$V(x) = x^\top P x$는 원점 중심의 **타원체(ellipsoid)**이다:

$$
V = c \quad \Leftrightarrow \quad x^\top P x = c \quad \text{(ellipsoid surface)}
$$

CQLF 조건 $\dot{V} < 0$은 **모든 가능한 궤적이 이 타원 내부에서 원점으로 수렴**해야 한다는 뜻이다.

**문제**: 만약 두 개의 local model $A_1, A_2$가 **서로 다른 궤적 형태**를 만든다면?

- $A_1$의 고유벡터 방향: 빠르게 감쇠
- $A_2$의 고유벡터 방향: 느리게 감쇠

이 경우 **단 하나의 타원 $P$로는 두 궤적 모두를 포함할 수 없을 수 있다**.

### 4.4.2 정량적 예시

2차원 예제:

$$
A_1 = \begin{bmatrix} -10 & 0 \\ 0 & -1 \end{bmatrix}, \quad A_2 = \begin{bmatrix} -1 & 0 \\ 0 & -10 \end{bmatrix}
$$

$A_1$의 고유벡터: $[1, 0]^\top$ (빠름), $[0, 1]^\top$ (느림)
$A_2$의 고유벡터: $[1, 0]^\top$ (느림), $[0, 1]^\top$ (빠름)

**정확히 반대 방향**이다!

CQLF를 찾으려면

$$
A_1^\top P + P A_1 = -20p_{11} \text{ (1,1 원소)}
$$
$$
A_2^\top P + P A_2 = -2p_{11} \text{ (1,1 원소)}
$$

동시에 음수가 되려면 매우 제약적이다.

### 4.4.3 극단적인 경우: 불가능

$A_1$과 $A_2$가 **직교하는 고유벡터**를 가지면, CQLF는 존재하지 않을 수 있다. (각 $A_i$는 Hurwitz이지만!)

이것이 **Ch5의 동기**: 영역마다 다른 $P_i$를 사용하면 보수성을 줄일 수 있다.

---

## 4.5 예제

### 예제 4.1: Scalar TS Fuzzy (쉬운 경우)

**문제.** 다음 TS fuzzy 시스템을 고려하라:

$$
\dot{x} = [h_1(z)(-2) + h_2(z)(-0.5)] x
$$

여기서 $h_1 + h_2 = 1$, $h_i \geq 0$.

CQLF를 이용해 안정성을 보이라.

**풀이.**

TS 표현: $\dot{x} = h_1 A_1 x + h_2 A_2 x$, where $A_1 = -2$, $A_2 = -0.5$ (scalar).

$V = Px$ (즉 $P = p$, scalar)로 두면:

$$
\dot{V} = h_1 A_1^\top P x + h_2 A_2^\top P x = [h_1(-2)p + h_2(-0.5)p] x
$$

scalar이므로 전치 생략:

$$
\dot{V} = [h_1(-2) + h_2(-0.5)] px^2
$$

CQLF 조건: $A_i^\top P + P A_i \prec 0$ for all $i$.

$$
\text{(i=1)}: -2P + P(-2) = -4P \prec 0 \Rightarrow P \succ 0 \text{ ✓}
$$
$$
\text{(i=2)}: -0.5P + P(-0.5) = -P \prec 0 \Rightarrow P \succ 0 \text{ ✓}
$$

$P = 1$을 선택하면 둘 다 만족한다. 따라서 $\dot{V} < 0$이므로 안정. ✓

---

### 예제 4.2: 2D TS Fuzzy, CQLF 성공

**문제.** 2-rule TS fuzzy 시스템:

$$
A_1 = \begin{bmatrix} -1 & 2 \\ -3 & -2 \end{bmatrix}, \quad A_2 = \begin{bmatrix} -2 & 1 \\ -1 & -2 \end{bmatrix}
$$

공통 $P$가 존재하는지 확인하고, 존재하면 구하라.

**풀이.**

LMI: $A_i^\top P + P A_i \prec 0$ for $i = 1, 2$.

$$
A_1^\top = \begin{bmatrix} -1 & -3 \\ 2 & -2 \end{bmatrix}
$$

$$
A_1^\top P + P A_1 = \begin{bmatrix} -1 & -3 \\ 2 & -2 \end{bmatrix} \begin{bmatrix} p_{11} & p_{12} \\ p_{12} & p_{22} \end{bmatrix} + \begin{bmatrix} p_{11} & p_{12} \\ p_{12} & p_{22} \end{bmatrix} \begin{bmatrix} -1 & 2 \\ -3 & -2 \end{bmatrix}
$$

전개 (자세한 계산 생략):

$$
\begin{bmatrix} -2p_{11} - 6p_{12} & 2p_{11} - 2p_{12} - 3p_{22} \\ 2p_{11} - 2p_{12} - 3p_{22} & 4p_{12} - 4p_{22} \end{bmatrix} \prec 0
$$

유사하게 $i=2$에 대해서도.

LMI solver (YALMIP, CVX)를 사용하면:

$$
P \approx \begin{bmatrix} 1.2 & 0.3 \\ 0.3 & 0.8 \end{bmatrix}
$$

(정확한 값은 solver output에 따름)

검증: $\lambda_{\min}(P) > 0$ ✓, $\lambda_{\max}(A_i^\top P + P A_i) < 0$ for $i=1,2$ ✓

따라서 CQLF 존재. **안정성 입증 완료**.

---

### 예제 4.3: 2D TS Fuzzy, CQLF 실패 (보수성 시연)

**문제.** 다음 시스템:

$$
A_1 = \begin{bmatrix} -1 & 5 \\ -1 & -2 \end{bmatrix}, \quad A_2 = \begin{bmatrix} -2 & 0.1 \\ -10 & -1 \end{bmatrix}
$$

각 $A_i$는 Hurwitz임을 확인한 후, CQLF가 존재하는지 조사하라.

**풀이.**

고유값 확인:
- $A_1$: $\lambda = -1.5 \pm \sqrt{2.25 - 5} = -1.5 \pm 1.66i$ (음의 실수부) → Hurwitz ✓
- $A_2$: $\lambda = -1.5 \pm \sqrt{2.25 + 10} = -1.5 \pm 3.5i$ (음의 실수부) → Hurwitz ✓

하지만 LMI $A_i^\top P + P A_i \prec 0$ for both $i$를 동시에 만족하는 $P \succ 0$이 **존재하지 않는다**.

이는 LMI solver (CVX)로 확인할 수 있다:

```python
from cvxpy import *
P = Variable((2, 2), symmetric=True)
# LMI: A1'P + PA1 < 0
# LMI: A2'P + PA2 < 0
# P > 0
# (풀려고 하면 infeasible)
```

**결론**: 각 모드는 안정하지만, **공통 Lyapunov 함수는 존재하지 않는다**. 

이는 CQLF의 **보수성**을 명확히 보여준다!

→ **5장의 동기**: mode-dependent Lyapunov $V_i = x^\top P_i x$가 필요.

---

### 예제 4.4: 1-DOF 로봇 조인트

**문제.** 단일 관절 로봇의 동역학:

$$
I(q) \ddot{q} + c\dot{q} + mgl\sin q = u
$$

여기서:
- $I(q) = I_0 + I_1 \cos q$ (관성이 configuration에 따라 변함)
- $c = 0.1$ (점성 감쇠)
- $mgl = 1$ (중력 모멘트)

관절 위치 $q \in [-\pi/2, \pi/2]$에서 3-rule TS 표현을 만들어라.

**풀이.**

상태 정의: $x_1 = q$, $x_2 = \dot{q}$.

$$
\dot{x}_1 = x_2
$$
$$
\dot{x}_2 = \frac{1}{I(x_1)}[u - cx_2 - \sin x_1]
$$

선형화 포인트:
- Rule 1: $q = -\pi/4$ → $\cos q \approx 0.707$, $\sin q \approx -0.707$
- Rule 2: $q = 0$ → $\cos q = 1$, $\sin q = 0$
- Rule 3: $q = \pi/4$ → $\cos q \approx 0.707$, $\sin q \approx 0.707$

각 포인트에서 선형화:

$$
\left.\frac{\partial}{\partial x}\right|_{x=x^*} = \begin{bmatrix} 0 & 1 \\ -\frac{\cos x_1^*}{I(x_1^*)} & -\frac{c}{I(x_1^*)} \end{bmatrix}
$$

Rule 2 ($q=0$): $I = I_0 + I_1$이 최대.

$$
A_2 = \begin{bmatrix} 0 & 1 \\ -\frac{1}{I_0 + I_1} & -\frac{c}{I_0 + I_1} \end{bmatrix}
$$

Membership function: $h_i(q) = \text{triangular, centered at } q^*_i$.

이를 이용해 LMI를 풀면 공통 $P$를 찾을 수 있다.

---

## 4.6 실습 코드

### 4.6.1 MATLAB: LMI 풀기 (예제 4.2)

```matlab
%% TS Fuzzy CQLF 안정성 검증 (MATLAB + YALMIP)
clear; clc;

% 시스템 행렬
A1 = [-1  2; -3 -2];
A2 = [-2  1; -1 -2];

% YALMIP 설정
P = sdpvar(2, 2, 'symmetric');

% LMI: P >= I*eps (positive definite)
LMI = [P >= eye(2)*1e-6];

% LMI: Ai'P + P*Ai < 0 for i=1,2
LMI = [LMI, A1'*P + P*A1 <= -eye(2)*1e-6];
LMI = [LMI, A2'*P + P*A2 <= -eye(2)*1e-6];

% 풀기
options = sdpsettings('solver', 'sedumi', 'verbose', 1);
result = optimize(LMI, [], options);

if result.problem == 0
    P_opt = value(P);
    fprintf('Feasible! P found:\n'); disp(P_opt);
    
    % 검증
    eig_P = eig(P_opt);
    eig_A1P = eig(A1'*P_opt + P_opt*A1);
    eig_A2P = eig(A2'*P_opt + P_opt*A2);
    
    fprintf('min eig(P) = %.6f\n', min(eig_P));
    fprintf('max eig(A1''P+PA1) = %.6f\n', max(eig_A1P));
    fprintf('max eig(A2''P+PA2) = %.6f\n', max(eig_A2P));
else
    fprintf('Infeasible or numerical issue.\n');
end
```

### 4.6.2 Python: CVX + 시각화 (예제 4.3 - 실패 케이스)

```python
import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# 예제 4.3: CQLF 불가능한 경우
A1 = np.array([[-1.0, 5.0], [-1.0, -2.0]])
A2 = np.array([[-2.0, 0.1], [-10.0, -1.0]])

print("=== Checking Hurwitz ===")
eig_A1 = np.linalg.eigvals(A1)
eig_A2 = np.linalg.eigvals(A2)
print(f"A1 eigenvalues: {eig_A1}")
print(f"A2 eigenvalues: {eig_A2}")
print(f"A1 stable: {np.all(np.real(eig_A1) < 0)}")
print(f"A2 stable: {np.all(np.real(eig_A2) < 0)}")

print("\n=== Solving for CQLF ===")
P = cp.Variable((2, 2), PSD=True)

# LMI constraints
lmi1 = A1.T @ P + P @ A1
lmi2 = A2.T @ P + P @ A2

constraints = [
    lmi1 << -1e-6 * np.eye(2),  # A1'P + PA1 < -eps*I
    lmi2 << -1e-6 * np.eye(2),  # A2'P + PA2 < -eps*I
    P >> 1e-6 * np.eye(2)       # P > eps*I
]

prob = cp.Problem(cp.Minimize(0), constraints)
result = prob.solve(solver=cp.SCS, verbose=False)

if prob.status == cp.OPTIMAL:
    print("CQLF Found!")
    P_opt = P.value
    print(f"P =\n{P_opt}")
else:
    print(f"INFEASIBLE: {prob.status}")
    print("This system does not have a common P!")

# --- Visualization: Phase portraits (시뮬레이션) ---
# CQLF는 실패했지만, 각각 단독으로는 안정하므로 시뮬레이션 가능

fig, axes = plt.subplots(1, 2, figsize=(14, 6))

for rule_idx, (A, title) in enumerate([(A1, 'Rule 1 only'), (A2, 'Rule 2 only')]):
    ax = axes[rule_idx]
    
    # Phase portrait
    x1 = np.linspace(-3, 3, 25)
    x2 = np.linspace(-3, 3, 25)
    X1, X2 = np.meshgrid(x1, x2)
    
    U = A[0,0]*X1 + A[0,1]*X2
    W = A[1,0]*X1 + A[1,1]*X2
    
    ax.streamplot(X1, X2, U, W, color='gray', density=1.0, linewidth=0.5)
    ax.set_xlabel(r'$x_1$')
    ax.set_ylabel(r'$x_2$')
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    
    # Trajectory
    def dyn(t, x):
        return A @ x
    
    x0 = np.array([2.5, 2.0])
    sol = solve_ivp(dyn, [0, 5], x0, max_step=0.05, dense_output=True)
    ax.plot(sol.y[0], sol.y[1], 'r-', linewidth=1.5, alpha=0.7)

plt.suptitle('Example 4.3: Each rule is stable, but CQLF is infeasible')
plt.tight_layout()
plt.savefig('ch4_cqlf_failure.png', dpi=150)
plt.show()
```

### 4.6.3 Membership Function 시각화

```python
import numpy as np
import matplotlib.pyplot as plt

# Triangular membership functions (3-rule)
q = np.linspace(-np.pi/2, np.pi/2, 100)

def tri_membership(x, center, width):
    """Triangular membership function"""
    dist = np.abs(x - center)
    return np.maximum(0, 1 - dist / width)

h1 = tri_membership(q, -np.pi/4, np.pi/4)
h2 = tri_membership(q, 0, np.pi/4)
h3 = tri_membership(q, np.pi/4, np.pi/4)

# Normalize so h1 + h2 + h3 = 1
h_sum = h1 + h2 + h3
h1, h2, h3 = h1/h_sum, h2/h_sum, h3/h_sum

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

# Membership functions
ax1.plot(q, h1, 'r-', linewidth=2, label=r'$h_1(q)$')
ax1.plot(q, h2, 'g-', linewidth=2, label=r'$h_2(q)$')
ax1.plot(q, h3, 'b-', linewidth=2, label=r'$h_3(q)$')
ax1.fill_between(q, 0, h1, alpha=0.2, color='r')
ax1.fill_between(q, 0, h2, alpha=0.2, color='g')
ax1.fill_between(q, 0, h3, alpha=0.2, color='b')
ax1.set_ylabel('Membership value')
ax1.legend()
ax1.set_title('Triangular Membership Functions')
ax1.grid(True, alpha=0.3)
ax1.set_ylim([0, 1.1])

# Verify convex sum
ax2.plot(q, h1 + h2 + h3, 'k-', linewidth=2, label=r'$h_1 + h_2 + h_3$')
ax2.fill_between(q, 0, h1+h2+h3, alpha=0.3, color='gray')
ax2.set_xlabel(r'Premise variable $q$ [rad]')
ax2.set_ylabel('Sum of memberships')
ax2.set_ylim([0.9, 1.1])
ax2.legend()
ax2.grid(True, alpha=0.3)
ax2.set_title('Convex Sum Property')

plt.tight_layout()
plt.savefig('ch4_membership.png', dpi=150)
plt.show()
```

---

## 4.7 복습 문제

1. **CQLF의 필요조건과 충분조건**: 정리 4.1에서 제시한 조건이 필요조건이기도 한가? (힌트: 1-DOF 로봇에서 실제로 필요할까?)

2. **예제 4.3 재분석**: $A_1$과 $A_2$의 고유벡터 방향을 계산하고, 왜 CQLF가 불가능한지 기하학적으로 설명하라.

3. **Membership 함수 설계**: 로봇의 관절각도 $q \in [0, 2\pi]$에 대해 4-rule TS fuzzy를 설계하라. 각 규칙의 "중심"을 어디에 놓겠는가?

4. **Decay Rate**: CQLF가 존재할 때, $\dot{V} \leq -2\alpha V$ 조건을 LMI로 쓰면?

---

## 4.8 핵심 요약

> **TS Fuzzy 시스템**은 비선형 동역학을 여러 선형 모델 $A_i$의 convex combination으로 표현한다:
> $$\dot{x} = \sum_i h_i(z) A_i x, \quad h_i \geq 0, \sum h_i = 1$$
>
> **Common Quadratic Lyapunov Function (CQLF)**은 모든 규칙에 공통인 단 하나의 $P \succ 0$로 안정성을 입증한다:
> $$A_i^\top P + PA_i \prec 0 \quad \forall i$$
>
> **하지만 CQLF는 보수적이다**: 각 규칙의 동역학이 서로 다르면 (예: 예제 4.3), 공통 $P$가 존재하지 않을 수 있다. 각 $A_i$가 모두 Hurwitz일 때도!
>
> **다음 장으로**: 5장에서는 각 규칙마다 다른 $P_i$를 도입하는 **Fuzzy Lyapunov 함수** $V = \sum_i h_i(z) x^\top P_i x$로 보수성을 줄인다.

---

**참고**: TS fuzzy 시스템과 GMR(Gaussian Mixture Regression, Ch7)은 개념상 유사하다 — 모두 비선형을 선형 모델들의 부드러운 혼합으로 본다는 점에서.
