# 5장. Fuzzy Lyapunov Function: 영역마다 다른 에너지 함수

> **이 장의 목표**: TS fuzzy 시스템에서 고정된 Lyapunov 행렬 $P$의 보수성을 극복하기 위해, 각 부분 시스템마다 다른 에너지 함수를 갖는 Fuzzy Lyapunov Function (FLF)을 설계한다.
>
> **선수 지식**: 4장의 Common Quadratic Lyapunov Function (CQLF), TS fuzzy 시스템의 기본 개념
>
> **다음 장과의 연결**: 6장에서 외란이 있을 때 Uniformly Ultimately Bounded (UUB) 안정성으로 확장된다.

---

## 5.1 왜 Fuzzy Lyapunov인가

### 5.1.1 4장의 CQLF 한계 재검토

4장에서 다룬 Common Quadratic Lyapunov Function (CQLF) 접근은 다음 조건이었다:

$$
\dot{x} = \sum_{i=1}^{C} h_i(z) A_i x
$$

**고정된 행렬 $P \succ 0$에 대해**:

$$
V(x) = x^\top P x
$$

$$
A_i^\top P + P A_i \prec 0 \quad \text{for all } i = 1, \ldots, C
$$

이 조건은 충분조건이지만 **매우 보수적**이다. 왜냐하면:

1. **지역 특성 무시**: 고정된 $P$는 모든 부분 시스템 $\dot{x} = A_i x$에 동일하게 적용된다.
2. **다양한 동역학 제약**: 예를 들어 $A_1$은 매우 안정적이고 $A_2$는 약간만 안정적이면, 공통 $P$는 $A_2$의 엄격한 조건에 맞춰져야 한다.
3. **실용적 실패**: CQLF 조건이 LMI로 실행 불가능(infeasible)일 때가 많다.

### 5.1.2 Fuzzy Lyapunov Function의 핵심 아이디어

**Fuzzy Lyapunov Function (FLF)**은 다음 아이디어에서 출발한다:

> "고정된 그릇 하나" 대신, "membership에 따라 모양이 달라지는 그릇"을 사용하자.

$$
V(x) = x^\top P(h) x, \quad P(h) = \sum_{i=1}^{C} h_i(z) P_i
$$

여기서:
- $P_i \succ 0$: $i$번째 부분 시스템 근처에서의 Lyapunov 행렬
- $h_i(z) \in [0,1]$, $\sum h_i = 1$: membership 함수
- $P(h)$: 시간과 위치에 따라 변하는 행렬

**직관**: membership $h_i$가 크면 $P_i$의 영향이 크고, 이는 $i$번째 부분 시스템의 특성에 맞춘다.

**기하학적 해석**: $V(x) = 1$이라는 일정한 에너지 등고면이 상태가 다양한 부분 시스템을 거치면서 **타원 모양이 변한다**.

---

## 5.2 FLF 정의와 $\dot{V}$ 완전 전개

### 5.2.1 Lyapunov 함수 정의

$$
V(x) = x^\top P(h) x, \quad P(h) = \sum_{i=1}^{C} h_i(z) P_i
$$

시간 미분:

$$
\dot{V} = \dot{x}^\top P(h) x + x^\top P(h) \dot{x} + x^\top \dot{P}(h) x
$$

첫 두 항: $\dot{x} = A(h) x = \sum_{i=1}^{C} h_i A_i x$를 대입하면

$$
\dot{x}^\top P(h) x + x^\top P(h) \dot{x} = x^\top [A(h)^\top P(h) + P(h) A(h)] x
$$

세 번째 항: $\dot{P}(h) = \sum_{i=1}^{C} \dot{h}_i P_i$이므로

$$
x^\top \dot{P}(h) x = x^\top \left(\sum_{i=1}^{C} \dot{h}_i P_i\right) x
$$

### 5.2.2 $A(h)^\top P(h) + P(h) A(h)$ 전개

$$
A(h)^\top P(h) + P(h) A(h) = \left(\sum_{i=1}^{C} h_i A_i\right)^\top \left(\sum_{j=1}^{C} h_j P_j\right) + \left(\sum_{j=1}^{C} h_j P_j\right) \left(\sum_{i=1}^{C} h_i A_i\right)
$$

전개하면:

$$
= \sum_{i=1}^{C} \sum_{j=1}^{C} h_i h_j (A_i^\top P_j + P_j A_i)
$$

따라서:

$$
x^\top [A(h)^\top P(h) + P(h) A(h)] x = \sum_{i=1}^{C} \sum_{j=1}^{C} h_i h_j \, x^\top (A_i^\top P_j + P_j A_i) x
$$

### 5.2.3 $\dot{V}$ 최종 형태

$$
\dot{V}(x) = \underbrace{\sum_{i=1}^{C} \sum_{j=1}^{C} h_i h_j \, x^\top (A_i^\top P_j + P_j A_i) x}_{L_1 \text{ (A-P 항)}} + \underbrace{x^\top \left(\sum_{i=1}^{C} \dot{h}_i P_i\right) x}_{L_3 \text{ (membership 변화)}}
$$

**해석**:
- $L_1$: 동역학과 Lyapunov 행렬의 상호작용
- $L_3$: membership 함수의 변화율이 야기하는 양의 항

---

## 5.3 $L_1$의 구조 분석: 대각항 vs 비대각항

### 5.3.1 대각항 ($i = j$)

$$
h_i^2 \, x^\top (A_i^\top P_i + P_i A_i) x \quad \text{for } i = 1, \ldots, C
$$

**성질**: 이 항들은 **로컬 안정성**을 나타낸다. 예를 들어 $i$번째 부분 시스템이 주도적이면 ($h_i \approx 1$), 이 항이 지배적이고, $A_i^\top P_i + P_i A_i \prec 0$이면 로컬에서 감소한다.

### 5.3.2 비대각항 ($i \neq j$)

$$
h_i h_j \, x^\top (A_i^\top P_j + P_j A_i) x \quad \text{for } i \neq j
$$

**문제**: 비대각항이 양수일 수 있다 → $\dot{V}$가 증가할 수 있다.

**해결**: Tanaka-Wang relaxation (섹션 5.4)으로 이를 제어한다.

---

## 5.4 Tanaka-Wang Relaxation

### 5.4.1 Naïve 접근과 보수성

**Naïve 조건**: 모든 쌍 $(i, j)$에 대해 $A_i^\top P_j + P_j A_i \prec 0$을 요구.

이는 매우 **제한적**이다. 예: $C = 5$면 $5 \times 5 = 25$개 조건이 필요하고, 대부분은 불필요하게 강하다.

### 5.4.2 Tanaka-Wang 트릭

**관찰**: 쌍 $(i, j)$에서 $h_i h_j \geq 0$이므로, 다음 합은 항상 비음수이다:

$$
h_i h_j X_{ij} + h_j h_i X_{ji} = h_i h_j (X_{ij} + X_{ji})
$$

따라서 조건

$$
X_{ij} + X_{ji} \preceq 0 \quad \text{for all } i \neq j
$$

이 성립하면, 비대각항의 기여는

$$
h_i h_j (A_i^\top P_j + P_j A_i) + h_j h_i (A_j^\top P_i + P_i A_j) \leq 0
$$

**구체적으로**, Tanaka-Wang 조건은:

$$
A_i^\top P_j + P_j A_i + A_j^\top P_i + P_i A_j \preceq 0 \quad \text{for } i < j
$$

### 5.4.3 조건의 개수

**필요한 LMI 개수**:
- **대각항**: $C$개 ($i = 1, \ldots, C$)
- **교차항**: $\binom{C}{2} = \frac{C(C-1)}{2}$개 ($i < j$)
- **합계**: $C + \frac{C(C-1)}{2} = \frac{C(C+1)}{2}$개

예:
- $C = 2$: $1 + 1 = 2$
- $C = 3$: $3 + 3 = 6$
- $C = 5$: $5 + 10 = 15$

### 5.4.4 조건의 정확한 명시

**Tanaka-Wang FLF 조건**:

$$
\begin{cases}
A_i^\top P_i + P_i A_i \prec 0 & \text{(대각, } i=1,\ldots,C\text{)} \\
A_i^\top P_j + P_j A_i + A_j^\top P_i + P_i A_j \preceq 0 & \text{(교차, } i < j\text{)}
\end{cases}
$$

이 조건들이 성립하면:

$$
\sum_{i=1}^{C} \sum_{j=1}^{C} h_i h_j (A_i^\top P_j + P_j A_i) \preceq -\sum_{i=1}^{C} h_i^2 (A_i^\top P_i + P_i A_i) + \underbrace{O(h_i h_j \text{ cross-terms})}_{\leq 0 \text{ by TW}}
$$

---

## 5.5 $L_1$의 하한: Cauchy-Schwarz와 Decay Rate

### 5.5.1 Cauchy-Schwarz 부등식의 적용

**핵심 보조정리**.

$$
1 = \left(\sum_{i=1}^{C} h_i\right)^2 = \sum_{i=1}^{C} h_i^2 + 2\sum_{i<j} h_i h_j \leq C \sum_{i=1}^{C} h_i^2
$$

따라서:

$$
\sum_{i=1}^{C} h_i^2 \geq \frac{1}{C}
$$

**증명**: 코시-슈바르츠 부등식 $\left(\sum_i a_i b_i\right)^2 \leq (\sum_i a_i^2)(\sum_i b_i^2)$에서 $a_i = h_i$, $b_i = 1$을 놓으면:

$$
\left(\sum_{i=1}^{C} h_i\right)^2 \leq \left(\sum_{i=1}^{C} h_i^2\right) \cdot C
$$

즉, $1^2 \leq C \sum_i h_i^2$. $\square$

### 5.5.2 $L_1$의 상한 유도

Tanaka-Wang 조건으로부터:

$$
\sum_{i=1}^{C} \sum_{j=1}^{C} h_i h_j (A_i^\top P_j + P_j A_i) \leq \sum_{i=1}^{C} h_i^2 (A_i^\top P_i + P_i A_i)
$$

**정의**: $Q_i \succ 0$를 다음과 같이 정의하자:

$$
A_i^\top P_i + P_i A_i = -Q_i
$$

그러면:

$$
L_1 \leq -\sum_{i=1}^{C} h_i^2 \|Q_i\| \|x\|^2
$$

더 강한 형태로:

$$
L_1 \leq -\min_{i} \lambda_{\min}(Q_i) \cdot \sum_{i=1}^{C} h_i^2 \cdot \|x\|^2
$$

Cauchy-Schwarz를 적용하면:

$$
L_1 \leq -\min_{i} \lambda_{\min}(Q_i) \cdot \frac{1}{C} \cdot \|x\|^2
$$

**따라서**:

$$
\alpha_1 = \frac{1}{C} \min_{i=1,\ldots,C} \lambda_{\min}(Q_i)
$$

### 5.5.3 Cauchy-Schwarz 경고

**일반적인 실수**: $\frac{1}{C}$ 인수를 빠뜨리는 경우.

만약 $\alpha_1 = \min_i \lambda_{\min}(Q_i)$로 정의하면, 이는 $C \to \infty$일 때 과도하게 보수적이다.

**정확한 형태**: membership 개수 $C$에 반비례하는 $\frac{1}{C}$ 인수는 **필수**이다.

---

## 5.6 $\dot{h}_i$ 항이 왜 문제인가

### 5.6.1 Membership 변화율의 영향

$L_3$ 항:

$$
L_3 = x^\top \left(\sum_{i=1}^{C} \dot{h}_i P_i\right) x
$$

만약 membership이 급격히 변하면 ($|\dot{h}_i|$가 크면), $L_3 > 0$이 되어 $\dot{V}$에 양의 기여를 한다.

예: 비선형 membership 함수 $h_1(z) = \sin^2(z)$가 $z$가 빠르게 변할 때, $\dot{h}_1$은 크다.

### 5.6.2 Membership 변화율 경계

**가정 (일반 TS)**: membership 변화율에 경계가 있다:

$$
|\dot{h}_i(z)| \leq \bar{h} \quad \text{for all } i, z
$$

여기서 $\bar{h} > 0$는 설계자가 주어지는 상수 (보수적인 상한).

### 5.6.3 GMM 기반 TS의 장점

**6장 미리보기**: GMM (Gaussian Mixture Model)으로 정의된 TS fuzzy 시스템에서는, membership 변화율이 **계산 가능**하다:

$$
\dot{h}_i \text{는 공분산 행렬로부터 유도 가능}
$$

따라서 보수적인 가정 대신 **정확한 경계**를 구할 수 있다.

### 5.6.4 $L_3$ 경계

**상한**:

$$
|L_3| = \left| x^\top \left(\sum_{i=1}^{C} \dot{h}_i P_i\right) x \right| \leq \sum_{i=1}^{C} |\dot{h}_i| \|P_i\| \|x\|^2
$$

$$
\leq C \bar{h} \max_{i} \|P_i\| \|x\|^2 = C \bar{h} \bar{\lambda}_{11} \|x\|^2
$$

여기서 $\bar{\lambda}_{11} = \max_i \lambda_{\max}(P_i)$.

**따라서**:

$$
\alpha_3 = C \bar{h} \bar{\lambda}_{11}
$$

---

## 5.7 예제

### 예제 5.1: 2-Rule Scalar FLF

**시스템**:

$$
\dot{x} = (h_1 A_1 + h_2 A_2) x
$$

$$
A_1 = -1, \quad A_2 = -0.5
$$

**Fuzzy Lyapunov Function**:

$$
V_f = (h_1 p_1 + h_2 p_2) x^2
$$

**시간 미분**:

$$
\dot{V}_f = 2(h_1 p_1 + h_2 p_2) x \dot{x} + 2x^2(\dot{h}_1 p_1 + \dot{h}_2 p_2)
$$

$$
= 2(h_1 p_1 + h_2 p_2)(h_1 A_1 + h_2 A_2) x^2 + 2x^2(\dot{h}_1 p_1 + \dot{h}_2 p_2)
$$

**전개**:

$$
L_1 = 2[h_1^2 A_1 p_1 + h_1 h_2 (A_1 p_2 + A_2 p_1) + h_2^2 A_2 p_2] x^2
$$

$$
L_3 = 2(\dot{h}_1 p_1 + \dot{h}_2 p_2) x^2
$$

**안정 조건**:
1. $A_1 p_1 = -p_1 < 0$ → $p_1 > 0$
2. $A_2 p_2 = -0.5 p_2 < 0$ → $p_2 > 0$
3. Tanaka-Wang: $A_1 p_2 + A_2 p_1 + \text{symmetric} \leq 0$
4. $|\dot{h}_i| \leq \bar{h}$ 가정 하에 $2 \bar{h}(p_1 + p_2) \cdot \min(p_1, p_2)^{-1} \cdot C \leq \alpha_1$

**선택**: $p_1 = 1.0$, $p_2 = 0.8$ (각 부분 시스템에 맞춤).

### 예제 5.2: 4장의 실패한 예제를 FLF로 재시도

**시스템** (4장에서 CQLF가 실패):

$$
A_1 = \begin{bmatrix} -2 & 1 \\ 0 & -1 \end{bmatrix}, \quad A_2 = \begin{bmatrix} -0.8 & 0.2 \\ 0 & -0.5 \end{bmatrix}
$$

**CQLF의 문제**: 단일 $P$로 $A_1^\top P + PA_1 \prec 0$과 $A_2^\top P + PA_2 \prec 0$을 동시에 만족하는 $P \succ 0$가 없었다.

**FLF로 재시도**:

각 $P_i$를 따로 설계:
- $P_1$: $A_1$에 맞춤
- $P_2$: $A_2$에 맞춤

Tanaka-Wang 조건:

$$
A_1^\top P_2 + P_2 A_1 + A_2^\top P_1 + P_1 A_2 \preceq 0
$$

이 교차 조건은 **완화되었기 때문에 종종 실행 가능**하다.

**결과**: 예를 들어
$$
P_1 = \begin{bmatrix} 0.6 & 0.1 \\ 0.1 & 0.5 \end{bmatrix}, \quad P_2 = \begin{bmatrix} 0.4 & 0.05 \\ 0.05 & 0.3 \end{bmatrix}
$$

가 feasible solution이 될 수 있다 (YALMIP으로 확인).

### 예제 5.3: $C$ 의존성 비교

**같은 시스템**, 부분 시스템 개수만 다르게:

1. **$C = 2$**: $\alpha_1 \approx 0.5 \cdot \lambda_{\min}(Q_i)$
2. **$C = 3$**: $\alpha_1 \approx 0.333 \cdot \lambda_{\min}(Q_i)$
3. **$C = 5$**: $\alpha_1 \approx 0.2 \cdot \lambda_{\min}(Q_i)$

**해석**: $C$가 크면, Cauchy-Schwarz 부등식의 $\frac{1}{C}$ 인수로 인해 **decay rate가 보수적**이다. 이는 membership의 불확실성이 클수록 더 엄격한 조건이 필요함을 의미한다.

---

## 5.8 실습 코드

### 5.8.1 MATLAB: Tanaka-Wang LMI 설정

```matlab
%% 예제 5.2: FLF와 Tanaka-Wang 조건
clear; clc;

% 시스템 정의
A1 = [-2 1; 0 -1];
A2 = [-0.8 0.2; 0 -0.5];
C = 2;  % 부분 시스템 개수

% YALMIP 변수
P1 = sdpvar(2, 2, 'symmetric');
P2 = sdpvar(2, 2, 'symmetric');

% 제약 조건
constraints = [
    P1 >= eye(2)*1e-4  % P1 > 0
    P2 >= eye(2)*1e-4  % P2 > 0
];

% 대각 조건
constraints = [constraints
    A1'*P1 + P1*A1 <= -eye(2)*1e-3
    A2'*P2 + P2*A2 <= -eye(2)*1e-3
];

% Tanaka-Wang 교차 조건
TW_cross = A1'*P2 + P2*A1 + A2'*P1 + P1*A2;
constraints = [constraints, TW_cross <= -eye(2)*1e-3];

% 최적화: min trace(P1 + P2)
options = sdpsettings('solver', 'sedumi', 'verbose', 1);
optimize(constraints, trace(P1 + P2), options);

% 결과 출력
fprintf('===== FLF 결과 =====\n');
P1_sol = value(P1);
P2_sol = value(P2);
fprintf('P1 = \n'); disp(P1_sol);
fprintf('P2 = \n'); disp(P2_sol);

% 검증
eig_A1P1_P1A1 = eig(A1'*P1_sol + P1_sol*A1);
eig_A2P2_P2A2 = eig(A2'*P2_sol + P2_sol*A2);
fprintf('eig(A1''P1+P1A1) = '); disp(eig_A1P1_P1A1');
fprintf('eig(A2''P2+P2A2) = '); disp(eig_A2P2_P2A2');

% Q 행렬 및 alpha_1 계산
Q1 = -(A1'*P1_sol + P1_sol*A1);
Q2 = -(A2'*P2_sol + P2_sol*A2);
lambda_min_Q1 = min(eig(Q1));
lambda_min_Q2 = min(eig(Q2));
alpha1 = (1/C) * min(lambda_min_Q1, lambda_min_Q2);
fprintf('alpha_1 = %.6f\n', alpha1);
```

### 5.8.2 Python: CVXPy로 FLF 설계

```python
import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# 시스템
A1 = np.array([[-2.0, 1.0], [0.0, -1.0]])
A2 = np.array([[-0.8, 0.2], [0.0, -0.5]])
C = 2

# CVXPy 변수
P1 = cp.Variable((2, 2), symmetric=True)
P2 = cp.Variable((2, 2), symmetric=True)

# 제약 조건
constraints = [
    P1 >> 1e-4 * np.eye(2),
    P2 >> 1e-4 * np.eye(2),
    # 대각 조건
    A1.T @ P1 + P1 @ A1 << -1e-3 * np.eye(2),
    A2.T @ P2 + P2 @ A2 << -1e-3 * np.eye(2),
    # Tanaka-Wang 교차 조건
    A1.T @ P2 + P2 @ A1 + A2.T @ P1 + P1 @ A2 << -1e-3 * np.eye(2)
]

# 목적함수
objective = cp.Minimize(cp.trace(P1) + cp.trace(P2))

# 풀이
problem = cp.Problem(objective, constraints)
result = problem.solve(solver=cp.SCS, verbose=True)

print("===== FLF 결과 (CVXPy) =====")
P1_sol = P1.value
P2_sol = P2.value
print(f"P1 =\n{P1_sol}")
print(f"P2 =\n{P2_sol}")

# 검증
eig_A1P1 = np.linalg.eigvalsh(A1.T @ P1_sol + P1_sol @ A1)
eig_A2P2 = np.linalg.eigvalsh(A2.T @ P2_sol + P2_sol @ A2)
print(f"max eig(A1'P1+P1A1) = {np.max(eig_A1P1):.6f} (should be < 0)")
print(f"max eig(A2'P2+P2A2) = {np.max(eig_A2P2):.6f} (should be < 0)")

# Decay rate
Q1 = -(A1.T @ P1_sol + P1_sol @ A1)
Q2 = -(A2.T @ P2_sol + P2_sol @ A2)
alpha1 = (1/C) * min(np.min(np.linalg.eigvalsh(Q1)),
                     np.min(np.linalg.eigvalsh(Q2)))
print(f"alpha_1 = {alpha1:.6f}")

# --- 시뮬레이션 ---
def dynamics_h1(t, x):
    """h1 = 1, h2 = 0"""
    return A1 @ x

def dynamics_h2(t, x):
    """h1 = 0, h2 = 1"""
    return A2 @ x

def dynamics_mixed(t, x, h1_func):
    """일반 TS 시스템"""
    h1 = h1_func(x)
    h2 = 1 - h1
    A_mixed = h1 * A1 + h2 * A2
    return A_mixed @ x

# membership 함수 (예: sigmoid)
def membership_h1(x):
    return 1 / (1 + np.exp(-3 * (x[0] + x[1])))

# 초기조건
x0 = np.array([1.0, 1.0])

# 시뮬레이션
t_span = (0, 5)
t_eval = np.linspace(0, 5, 500)

sol_mixed = solve_ivp(
    lambda t, x: dynamics_mixed(t, x, membership_h1),
    t_span, x0, t_eval=t_eval, method='RK45'
)

# V(t) 계산
V_mixed = []
for i, t in enumerate(sol_mixed.t):
    h1 = membership_h1(sol_mixed.y[:, i])
    P_h = h1 * P1_sol + (1-h1) * P2_sol
    V = sol_mixed.y[:, i] @ P_h @ sol_mixed.y[:, i]
    V_mixed.append(V)

# 플롯
fig, axes = plt.subplots(1, 2, figsize=(12, 4))

# Phase portrait
ax = axes[0]
x1_range = np.linspace(-1.5, 1.5, 20)
x2_range = np.linspace(-1.5, 1.5, 20)
X1, X2 = np.meshgrid(x1_range, x2_range)
U = np.zeros_like(X1)
W = np.zeros_like(X2)

for i in range(len(x1_range)):
    for j in range(len(x2_range)):
        x = np.array([X1[j, i], X2[j, i]])
        h1 = membership_h1(x)
        dx = h1 * A1 @ x + (1-h1) * A2 @ x
        U[j, i] = dx[0]
        W[j, i] = dx[1]

ax.streamplot(X1, X2, U, W, color='gray', density=1.5)
ax.plot(sol_mixed.y[0], sol_mixed.y[1], 'b-', linewidth=2, label='trajectory')
ax.plot(x0[0], x0[1], 'ro', markersize=8)
ax.set_xlabel(r'$x_1$')
ax.set_ylabel(r'$x_2$')
ax.set_title('Phase Portrait (FLF)')
ax.legend()
ax.grid(True, alpha=0.3)
ax.set_aspect('equal')

# V(t) 감소
ax = axes[1]
ax.semilogy(sol_mixed.t, V_mixed, 'b-', linewidth=2, label=r'$V(x(t))$')
V0 = V_mixed[0]
ax.semilogy(sol_mixed.t, V0 * np.exp(-2*alpha1*sol_mixed.t), 'r--',
            linewidth=1.5, label=rf'$V(0)e^{{-2\alpha_1 t}}$')
ax.set_xlabel('Time [s]')
ax.set_ylabel(r'$V(x)$')
ax.set_title('Lyapunov Function Decay')
ax.legend()
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('ch5_flf.png', dpi=150)
plt.show()
```

### 5.8.3 CQLF vs FLF 비교

```matlab
%% 비교: CQLF vs FLF 실행 가능성
clear; clc;

% 같은 시스템
A1 = [-2 1; 0 -1];
A2 = [-0.8 0.2; 0 -0.5];

%% CQLF 시도
fprintf('===== CQLF 시도 =====\n');
P = sdpvar(2, 2, 'symmetric');
constraints_cqlf = [
    P >= eye(2)*1e-4
    A1'*P + P*A1 <= -eye(2)*1e-3
    A2'*P + P*A2 <= -eye(2)*1e-3
];
options = sdpsettings('solver', 'sedumi', 'verbose', 0);
result_cqlf = optimize(constraints_cqlf, [], options);

if result_cqlf.problem == 0
    fprintf('CQLF: 실행 가능\n');
else
    fprintf('CQLF: 실행 불가능 (dual infeasible)\n');
end

%% FLF 시도
fprintf('\n===== FLF 시도 =====\n');
P1 = sdpvar(2, 2, 'symmetric');
P2 = sdpvar(2, 2, 'symmetric');
constraints_flf = [
    P1 >= eye(2)*1e-4
    P2 >= eye(2)*1e-4
    A1'*P1 + P1*A1 <= -eye(2)*1e-3
    A2'*P2 + P2*A2 <= -eye(2)*1e-3
    A1'*P2 + P2*A1 + A2'*P1 + P1*A2 <= -eye(2)*1e-3
];
result_flf = optimize(constraints_flf, trace(P1+P2), options);

if result_flf.problem == 0
    fprintf('FLF: 실행 가능 ✓\n');
    P1_sol = value(P1);
    P2_sol = value(P2);
    fprintf('Objective = %.6f\n', trace(P1_sol) + trace(P2_sol));
else
    fprintf('FLF: 실행 불가능\n');
end
```

---

## 5.9 복습 문제

1. **Cauchy-Schwarz 부등식 적용**: $\left(\sum h_i\right)^2 \leq C \sum h_i^2$를 증명하고, 이것이 왜 Decay rate $\alpha_1$에 $\frac{1}{C}$ 인수를 가져오는지 설명하라.

2. **Tanaka-Wang 트릭**: 왜 $X_{ij} + X_{ji} \preceq 0$ 조건만으로 $h_i h_j X_{ij} + h_j h_i X_{ji} \leq 0$를 보장할 수 있는가?

3. **$L_3$ 항의 의미**: Membership이 천천히 변한다는 가정 ($|\dot{h}_i| \leq \bar{h}$)이 없으면 왜 FLF도 안정성을 보장하지 못하는가?

4. **$C$의 역할**: $C = 2$일 때와 $C = 5$일 때 Decay rate $\alpha_1$의 비율은? 이것이 실제 시스템에 미치는 영향은?

---

## 5.10 핵심 요약

> **Fuzzy Lyapunov Function (FLF)**은 시간과 위치에 따라 에너지 함수의 모양을 바꾸는 기법이다.
>
> $$V(x) = x^\top P(h) x, \quad P(h) = \sum_{i=1}^{C} h_i P_i$$
>
> **CQLF의 보수성 극복**: 고정된 $P$ 대신 각 부분 시스템 $i$마다 다른 $P_i$를 사용하므로, 다양한 로컬 동역학을 더 잘 표현한다.
>
> **Tanaka-Wang 완화**: $\frac{C(C+1)}{2}$개의 LMI 조건으로 문제를 체계화한다. 특히 교차항 조건이 완화되어 실용적 실행 가능성이 높아진다.
>
> **Cauchy-Schwarz 한계**: Decay rate에서 $\frac{1}{C}$ 인수는 피할 수 없다. 부분 시스템이 많으면 보수적이 되는 근본 한계이다.
>
> **Membership 변화율**: $|\dot{h}_i| \leq \bar{h}$ 가정은 **필수**이다. 만약 이것이 보장되지 않으면 $L_3 > 0$로 인해 안정성이 깨질 수 있다.
>
> **다음 단계 (6장)**: FLF로도 외란이 있으면 $\dot{V} < 0$을 보장할 수 없다 → **Uniformly Ultimately Bounded (UUB)** 안정성으로 완화하고, **DOB (Disturbance Observer)**와 **GMM-기반 membership**을 도입한다.

---

**마지막 메시지**: FLF는 TS fuzzy 시스템의 **핵심 기법**이다. 이 장의 Tanaka-Wang 조건과 Cauchy-Schwarz 유도를 완전히 이해하면, 6장 이후의 고급 주제들이 자연스럽게 연결될 것이다.
