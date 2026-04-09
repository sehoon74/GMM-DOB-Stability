# 1장. Lyapunov 안정성: 궤적을 구하지 않고 안정성을 말하는 법

> **이 장의 목표**: "에너지가 줄어드는 함수를 찾으면 안정"이라는 아이디어를 수식으로 다룰 수 있게 된다.
>
> **선수 지식**: 선형대수 (고유값, 양의 정부호), 미분방정식 기초
>
> **다음 장과의 연결**: 여기서 얻은 $P \succ 0$, $A^\top P + PA \prec 0$ 조건이 2장에서 LMI라는 최적화 문제로 바뀐다.

---

## 1.1 왜 Lyapunov인가

### 1.1.1 고유값 분석의 한계

연속시간 LTI 시스템

$$
\dot{x} = Ax
$$

의 안정성은 $A$의 고유값 실수부가 모두 음수인지로 판정할 수 있다. 해가

$$
x(t) = e^{At} x(0)
$$

이므로, 고유값이 좌반면에 있으면 $x(t) \to 0$이다.

그런데 실제 로봇 시스템은

$$
M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) = \tau
$$

처럼 **비선형**이다. 고유값은 선형화 근방에서만 유효하고, 비선형 항이 큰 영역에서는 아무것도 보장하지 못한다. 또한 이 교재의 최종 목표인 TS fuzzy 시스템

$$
\dot{x} = \sum_{i=1}^{C} h_i(z) A_i x
$$

에서는 $A_i$가 여러 개이고 membership $h_i(z)$가 시간에 따라 변하므로, "어떤 $A_i$의 고유값을 봐야 하는가?"라는 질문 자체가 성립하지 않는다.

### 1.1.2 Lyapunov의 아이디어

Lyapunov의 접근은 궤적 $x(t)$를 직접 구하지 않고, 대신 **스칼라 함수** $V(x)$를 찾아서:

1. $V(x) > 0$ for all $x \neq 0$ (에너지는 항상 양수)
2. $\dot{V}(x) < 0$ along trajectories (에너지는 항상 감소)

이 두 조건만 확인하면 궤적이 원점으로 수렴한다는 것을 보장할 수 있다.

물리적 비유로 설명하면: 그릇 안의 공이 마찰에 의해 에너지를 잃으면 결국 바닥(원점)에 멈춘다. 이때 공의 정확한 궤적을 알 필요 없이, "에너지가 계속 줄어든다"는 사실만으로 수렴을 보장할 수 있다.

---

## 1.2 기본 정의와 표기

### 1.2.1 표기 규약

이 교재 전체에서 사용하는 표기:

- $x \in \mathbb{R}^n$: 상태벡터
- $A \in \mathbb{R}^{n \times n}$: 시스템 행렬
- $P = P^\top$: 대칭 행렬
- $P \succ 0$ ($P \succeq 0$): 양의 정부호 (양의 반정부호)
- $P \prec 0$ ($P \preceq 0$): 음의 정부호 (음의 반정부호)
- $\lambda_{\min}(P)$, $\lambda_{\max}(P)$: $P$의 최소/최대 고유값
- $\|x\| = \sqrt{x^\top x}$: 유클리드 노름
- $\|A\| = \sigma_{\max}(A)$: 행렬의 스펙트럼 노름 (최대 특이값)

### 1.2.2 양의 정부호의 의미

$P \succ 0$이란 **모든** $x \neq 0$에 대해

$$
x^\top P x > 0
$$

을 뜻한다. 기하학적으로, $x^\top P x = c$ (상수)의 등고면은 **타원체(ellipsoid)** 이다. $P$의 고유값이 클수록 해당 방향으로 타원이 납작해진다.

핵심 부등식은 다음이다:

$$
\lambda_{\min}(P) \|x\|^2 \leq x^\top P x \leq \lambda_{\max}(P) \|x\|^2
$$

**증명**: $P$는 대칭이므로 직교 대각화 $P = U \Lambda U^\top$이 가능하다. $y = U^\top x$로 놓으면 $x^\top P x = y^\top \Lambda y = \sum_i \lambda_i y_i^2$. 그러면 $\lambda_{\min} \sum_i y_i^2 \leq \sum_i \lambda_i y_i^2 \leq \lambda_{\max} \sum_i y_i^2$이고, $\|y\| = \|x\|$이므로 성립한다. $\square$

이 부등식의 의미: $V(x) = x^\top P x$는 $\|x\|^2$과 "동치인 에너지"이다. $V$가 줄어들면 $\|x\|$도 줄어든다.

---

## 1.3 Lyapunov 안정성 정리

### 1.3.1 LTI 시스템의 경우

**정리 1.1 (Lyapunov stability for LTI).**
시스템 $\dot{x} = Ax$에서, 만약 대칭 행렬 $P$가 존재하여

$$
P \succ 0, \qquad A^\top P + PA \prec 0
$$

이면, 원점 $x = 0$은 **점근 안정(asymptotically stable)** 이다.

**증명.**

Lyapunov 후보 함수를 $V(x) = x^\top P x$로 두자.

**Step 1: $V > 0$.** $P \succ 0$이므로 $x \neq 0$에 대해 $V(x) > 0$이고, $V(0) = 0$이다.

**Step 2: $\dot{V}$ 계산.** 시간 미분하면

$$
\dot{V} = \dot{x}^\top P x + x^\top P \dot{x}
$$

여기에 $\dot{x} = Ax$를 대입하면

$$
\dot{V} = (Ax)^\top P x + x^\top P(Ax) = x^\top A^\top P x + x^\top PA x
$$

$$
= x^\top (A^\top P + PA) x
$$

**Step 3: $\dot{V} < 0$.** $A^\top P + PA \prec 0$이므로, $x \neq 0$에 대해 $\dot{V}(x) < 0$이다.

따라서 $V(x)$는 양의 정부호이고 시간에 따라 순감소하므로, $x(t) \to 0$ as $t \to \infty$. $\square$

### 1.3.2 왜 이게 강력한가

이 정리의 힘은 **$x(t)$를 한 번도 구하지 않았다**는 점에 있다. $e^{At}$를 계산할 필요 없이, 행렬 부등식 $A^\top P + PA \prec 0$의 해 존재성만 확인하면 된다.

또한 이 조건은 **충분조건이 아니라 필요충분조건**이다 (LTI의 경우):

$$
A \text{ is Hurwitz} \quad \Longleftrightarrow \quad \exists P \succ 0 \text{ s.t. } A^\top P + PA \prec 0
$$

$(\Leftarrow)$는 위에서 증명했다. $(\Rightarrow)$는 Lyapunov 방정식 $A^\top P + PA = -Q$를 $Q \succ 0$에 대해 풀면 $P \succ 0$인 해가 존재한다는 것으로부터 나온다 (Khalil, Theorem 4.6).

### 1.3.3 Lyapunov 방정식

실제로 $P$를 찾는 한 가지 방법은 **Lyapunov 방정식**을 푸는 것이다:

$$
A^\top P + PA = -Q
$$

$Q \succ 0$를 정해주면 (예: $Q = I$), $A$가 Hurwitz일 때 유일한 해 $P \succ 0$가 존재한다.

**증명 (적분 표현):**

$$
P = \int_0^\infty e^{A^\top t} Q\, e^{At}\, dt
$$

$A$가 Hurwitz이면 이 적분은 수렴하고, 직접 미분으로 $A^\top P + PA = -Q$를 확인할 수 있다. $P \succ 0$는 $Q \succ 0$로부터 자명하다. $\square$

---

## 1.4 Decay Rate: 얼마나 빠르게 수렴하는가

안정한 것만으로는 부족할 때가 있다. "얼마나 빠르게 줄어드는가?"도 알고 싶다면, 다음 조건을 쓴다:

$$
\dot{V} \leq -2\alpha V
$$

여기서 $\alpha > 0$는 **decay rate**이다.

$V(x) = x^\top Px$에 대해 이 조건을 전개하면:

$$
x^\top (A^\top P + PA) x \leq -2\alpha\, x^\top P x
$$

모든 $x$에 대해 성립하려면

$$
A^\top P + PA + 2\alpha P \preceq 0
$$

이면 된다.

이 조건이 성립하면, Grönwall 부등식에 의해

$$
V(x(t)) \leq V(x(0))\, e^{-2\alpha t}
$$

이고, $\lambda_{\min}(P)\|x\|^2 \leq V \leq \lambda_{\max}(P)\|x\|^2$를 쓰면

$$
\|x(t)\| \leq \sqrt{\frac{\lambda_{\max}(P)}{\lambda_{\min}(P)}}\, \|x(0)\|\, e^{-\alpha t}
$$

즉, 상태가 지수적으로 감쇠하며, 감쇠율은 $\alpha$이다.

---

## 1.5 예제

### 예제 1.1: Scalar 시스템

**문제.** $\dot{x} = -2x$의 안정성을 Lyapunov로 증명하라.

**풀이.** $V = x^2$ (즉 $P = 1$)으로 두면:

$$
\dot{V} = 2x \dot{x} = 2x(-2x) = -4x^2 < 0 \quad \text{for } x \neq 0
$$

$P = 1 > 0$이고 $A^\top P + PA = (-2)(1) + (1)(-2) = -4 < 0$이므로 점근안정. ✓

Decay rate: $-4 \leq -2\alpha \cdot 1$ → $\alpha \leq 2$. 최대 decay rate는 2이다.

### 예제 1.2: 2D 안정 시스템

**문제.** 다음 시스템의 안정성을 판별하라.

$$
A = \begin{bmatrix} -1 & 0.5 \\ 0 & -2 \end{bmatrix}
$$

**풀이.** 먼저 고유값: $\lambda_1 = -1$, $\lambda_2 = -2$ (둘 다 좌반면). Hurwitz이므로 안정하다.

Lyapunov 방정식 $A^\top P + PA = -I$를 풀자:

$$
\begin{bmatrix} -1 & 0 \\ 0.5 & -2 \end{bmatrix}
\begin{bmatrix} p_{11} & p_{12} \\ p_{12} & p_{22} \end{bmatrix}
+
\begin{bmatrix} p_{11} & p_{12} \\ p_{12} & p_{22} \end{bmatrix}
\begin{bmatrix} -1 & 0.5 \\ 0 & -2 \end{bmatrix}
= -I
$$

전개하면:
- $(1,1)$: $-2p_{11} = -1$ → $p_{11} = 0.5$
- $(2,2)$: $-4p_{22} + p_{12} = -1$
- $(1,2)$: $-3p_{12} + 0.5p_{11} = 0$ → $p_{12} = 1/12$

$(2,2)$에서: $p_{22} = (1 + 1/12)/4 = 13/48$

$$
P = \begin{bmatrix} 0.5 & 1/12 \\ 1/12 & 13/48 \end{bmatrix}
$$

검증: $\det(P) = 0.5 \times 13/48 - (1/12)^2 = 13/96 - 1/144 = (13 \times 3 - 2)/(288) = 37/288 > 0$이고 $p_{11} > 0$이므로 $P \succ 0$. ✓

### 예제 1.3: 2D 불안정 시스템

**문제.** 다음 시스템은 왜 불안정한가?

$$
A = \begin{bmatrix} 0 & 1 \\ 2 & 0.5 \end{bmatrix}
$$

**풀이.** 고유값: $\lambda = 0.25 \pm \sqrt{0.0625 + 2} = 0.25 \pm 1.436$. $\lambda_1 \approx 1.686 > 0$이므로 불안정.

Lyapunov 방정식 $A^\top P + PA = -I$를 풀면, $P$의 고유값 중 하나가 음수가 되어 $P \succ 0$을 만족하는 해가 존재하지 않는다. 이것이 "안정하지 않으면 Lyapunov 함수를 찾을 수 없다"의 의미이다.

### 예제 1.4: 비선형 시스템 — 단진자

**문제.** $\ddot{\theta} + 0.5\dot{\theta} + \sin\theta = 0$의 원점 안정성을 보여라.

**풀이.** 상태변수 $x_1 = \theta$, $x_2 = \dot{\theta}$로 두면:

$$
\dot{x}_1 = x_2, \qquad \dot{x}_2 = -\sin x_1 - 0.5 x_2
$$

에너지 함수:

$$
V = \frac{1}{2}x_2^2 + (1 - \cos x_1)
$$

$V \geq 0$이고 $V = 0 \iff x_1 = 0, x_2 = 0$ ($|x_1| < \pi$에서).

$$
\dot{V} = x_2 \dot{x}_2 + \sin x_1 \cdot \dot{x}_1 = x_2(-\sin x_1 - 0.5x_2) + \sin x_1 \cdot x_2 = -0.5 x_2^2 \leq 0
$$

$\dot{V} \leq 0$ (not strictly $< 0$)이므로, LaSalle의 invariance principle로 원점의 점근안정을 결론짓는다.

이 예제는 LTI Lyapunov의 한계를 보여준다: 비선형 시스템에서는 $V = x^\top Px$ 형태가 아닌 **물리적 에너지**를 직접 써야 할 때가 있다. 하지만 이 교재가 다루는 TS fuzzy 시스템에서는 다시 2차 형식 $V = x^\top P(h) x$로 돌아온다 (5장).

---

## 1.6 실습 코드

### 1.6.1 MATLAB: Lyapunov 방정식 풀기

```matlab
%% 예제 1.2: 2D 안정 시스템
A = [-1 0.5; 0 -2];
Q = eye(2);

% Lyapunov 방정식 A'P + PA = -Q
P = lyap(A', Q);

fprintf('P = \n'); disp(P)
fprintf('min eig(P) = %.4f\n', min(eig(P)))
fprintf('max eig(A''P + PA) = %.6f (should be < 0)\n', max(eig(A'*P + P*A)))

% Decay rate: A'P + PA + 2*alpha*P <= 0
% 즉 max eig(A'P + PA + 2*alpha*P) <= 0
alpha_max = -max(eig(A'*P + P*A)) / (2*max(eig(P)));
fprintf('Maximum decay rate alpha = %.4f\n', alpha_max)
```

### 1.6.2 Python: Lyapunov 방정식 + 시각화

```python
import numpy as np
from scipy.linalg import solve_continuous_lyapunov
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# 예제 1.2: 2D 안정 시스템
A = np.array([[-1, 0.5], [0, -2]])
Q = np.eye(2)

# A'P + PA = -Q  →  scipy convention: A X + X A^H = Q  →  pass A', -Q
P = solve_continuous_lyapunov(A.T, -Q)

print(f"P = \n{P}")
print(f"min eig(P) = {np.min(np.linalg.eigvalsh(P)):.4f}")
print(f"max eig(A'P+PA) = {np.max(np.linalg.eigvalsh(A.T @ P + P @ A)):.6f}")

# --- Phase portrait + V(x) 등고선 ---
fig, axes = plt.subplots(1, 2, figsize=(14, 6))

# (a) Phase portrait with V contours
x1 = np.linspace(-3, 3, 30)
x2 = np.linspace(-3, 3, 30)
X1, X2 = np.meshgrid(x1, x2)

# V(x) = x' P x
V = P[0,0]*X1**2 + 2*P[0,1]*X1*X2 + P[1,1]*X2**2

# Vector field
U = A[0,0]*X1 + A[0,1]*X2
W = A[1,0]*X1 + A[1,1]*X2

ax = axes[0]
ax.contour(X1, X2, V, levels=10, cmap='Blues', alpha=0.6)
ax.streamplot(X1, X2, U, W, color='gray', density=1.2, linewidth=0.5)
ax.set_xlabel(r'$x_1$')
ax.set_ylabel(r'$x_2$')
ax.set_title('Phase Portrait + V(x) Contours')
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)

# (b) V(t) along trajectory
def dynamics(t, x):
    return A @ x

x0 = np.array([2.0, 1.5])
sol = solve_ivp(dynamics, [0, 5], x0, max_step=0.01)

V_t = np.array([s @ P @ s for s in sol.y.T])
V0 = x0 @ P @ x0

# Theoretical bound
alpha = -np.max(np.linalg.eigvalsh(A.T @ P + P @ A)) / (2 * np.max(np.linalg.eigvalsh(P)))
V_bound = V0 * np.exp(-2 * alpha * sol.t)

ax = axes[1]
ax.semilogy(sol.t, V_t, 'b-', linewidth=2, label=r'$V(x(t))$ (simulation)')
ax.semilogy(sol.t, V_bound, 'r--', linewidth=1.5, label=rf'$V(0) e^{{-2\alpha t}}$, $\alpha={alpha:.2f}$')
ax.set_xlabel('Time [s]')
ax.set_ylabel(r'$V(x)$')
ax.set_title(r'Lyapunov Function Decay')
ax.legend()
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('ch1_lyapunov.png', dpi=150)
plt.show()
```

### 1.6.3 실행 결과 해석

위 코드를 실행하면 두 가지 그래프를 얻는다:

1. **왼쪽 (Phase Portrait)**: 모든 궤적이 원점으로 수렴하며, $V(x)$의 등고선(타원)을 안에서 바깥으로 횡단하지 않는다. 즉, 궤적은 항상 $V$가 감소하는 방향으로 움직인다.

2. **오른쪽 ($V(t)$ 그래프)**: 파란 실선(시뮬레이션)이 빨간 점선(이론적 상한 $V(0)e^{-2\alpha t}$) 아래에 있다. 이론이 보수적이지만 violation은 없다.

---

## 1.7 복습 문제

1. $P \succ 0$이면 왜 $\lambda_{\min}(P) > 0$인가?
2. $\dot{V} = x^\top(A^\top P + PA)x$ 유도에서 어떤 성질이 사용되었나? (힌트: $(AB)^\top = B^\top A^\top$)
3. 예제 1.4에서 $\dot{V} = -0.5 x_2^2 \leq 0$인데 $\dot{V} \not< 0$이다. 왜 $x_2 = 0$, $x_1 \neq 0$인 경우에도 원점으로 수렴하는가? (힌트: LaSalle)
4. 비선형 시스템 $\dot{x} = -x^3$에 대해 $V = x^2$으로 안정성을 보여라. 이 경우 decay rate $\alpha$를 정의할 수 있는가?

---

## 1.8 이 장의 핵심 요약

> **Lyapunov 방법**은 궤적을 구하지 않고 스칼라 에너지 함수 $V(x)$의 감소를 확인하여 안정성을 보증한다.
>
> LTI 시스템에서는 $V(x) = x^\top Px$로 두면, 안정성 조건이 **행렬 부등식** $P \succ 0$, $A^\top P + PA \prec 0$이 된다.
>
> **하지만 $P$를 어떻게 체계적으로 찾을 것인가?** 이 질문이 2장의 출발점이다.
