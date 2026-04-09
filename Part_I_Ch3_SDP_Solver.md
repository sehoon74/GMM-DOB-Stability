# 3장. SDP Solver의 실전 감각

> **이 장의 목표**: Solver를 "증명의 도구"로 다루는 법을 배운다. "optimal" 신호도 수치적으로 거짓일 수 있으며, 이를 검증하는 엔지니어링 감각을 기른다.
>
> **선수 지식**: 2장의 LMI 기본 형식, 양의 정부호 조건
>
> **다음 장과의 연결**: 단일 시스템 $A$에 대한 검증을 마스터한 후, Part II에서 여러 개의 $A_i$를 섞은 TS fuzzy 시스템으로 확장한다.

---

## 3.1 왜 Solver 감각이 필요한가

### 3.1.1 "Optimal"은 거짓말일 수 있다

Solver가 "optimal solution found"라고 알려주는 것은 **수치 계산 관점**에서의 최적성이다. 하지만:

1. **부동소수점 오차 축적**: 대규모 문제에서 반올림 오차가 누적되면, 반환된 $P$가 $P \succ 0$을 정확히 만족하지 않을 수 있다.

2. **"barely feasible" 함정**: $P$의 최소 고유값이 $10^{-8}$이라면? 이론상 $\succ 0$이지만 하나의 작은 섭동에도 음의 정부호로 변할 수 있다.

3. **조건수 문제**: $\lambda_{\max}(P) / \lambda_{\min}(P)$가 크면 ($\gg 10^4$), 원래 문제가 수치적으로 불안정하다는 신호다.

**핵심**: Solver는 **증명(certificate)을 생성하는 도구**이지, 최종 답이 아니다.

---

## 3.2 SDP 표준형과 LMI의 관계

### 3.2.1 SDP 표준형

반정부호 프로그래밍의 표준형:

$$
\begin{aligned}
\text{minimize} \quad & c^\top x \\
\text{subject to} \quad & F_0 + \sum_{j=1}^{n} x_j F_j \succeq 0 \\
& (A_{\text{ineq}})
\end{aligned}
$$

여기서 $x \in \mathbb{R}^n$는 최적화 변수, $c \in \mathbb{R}^n$는 비용 벡터, $F_i$는 대칭 행렬들이다.

### 3.2.2 LMI를 SDP 표준형으로 변환

2장의 상태궤환 제어 문제:

$$
\begin{aligned}
\text{find} \quad & P, K \\
\text{subject to} \quad & P \succ 0 \\
& (A + BK)^\top P + P(A + BK) \prec 0
\end{aligned}
$$

$Y = KP$로 치환하면:

$$
\begin{aligned}
\text{minimize} \quad & 0 \\
\text{subject to} \quad & P \succ 0 \\
& A^\top P + PA + Y^\top B^\top + BY \prec 0
\end{aligned}
$$

이를 SDP 표준형으로 쓰면:

$$
\begin{bmatrix}
P & 0 \\
0 & -(A^\top P + PA + Y^\top B^\top + BY)
\end{bmatrix} \succeq 0
$$

### 3.2.3 Duality: Primal과 Dual

SDP의 쌍대성(duality):

**Primal:** $\min\, c^\top x$ s.t. $F(x) \succeq 0$

**Dual:** $\max\, -F_0 \bullet Z$ s.t. $(F_i)_{jk} \bullet Z = c_j$ for all $j,k$, $Z \succeq 0$

여기서 $A \bullet B = \text{tr}(A^\top B)$는 Frobenius 내적.

Solver는 **primal-dual 방법**을 써서 두 문제를 동시에 풀고, 그 차이(duality gap)를 보고한다:

$$
\text{Duality gap} = c^\top x + F_0 \bullet Z
$$

Gap이 작아야 수치적으로 믿을 수 있다.

---

## 3.3 Solver 선택과 특성

| Solver | 방법 | 속도 | 정확도 | 용도 |
|--------|------|------|--------|------|
| **SeDuMi** | Interior point | 중간 | 높음 | 교육, 연구 (중소 규모) |
| **MOSEK** | Interior point (상용) | 빠름 | 높음 | 산업, 대규모 문제 |
| **SCS** | First-order (operator splitting) | 매우 빠름 | 중간~낮음 | 대규모, 근사 허용 |
| **CVXOPT** | Interior point (Python) | 느림 | 높음 | 프로토타이핑 |

### 3.3.1 Interior Point Method (SeDuMi, MOSEK)

Interior point 방법은 feasible region의 **내부**에서 시작해서 경계 근처로 수렴한다:

$$
\min\, c^\top x + \mu \sum_i \log(1/\lambda_i(\cdot))
$$

여기서 $\lambda_i(\cdot)$는 제약의 고유값. 로그 배리어 항이 경계를 피하고 내부 경로를 추적한다.

**장점**: 보수적, 정확도 높음, 수치적으로 안정적  
**단점**: 반복 후반부 느림, 행렬 분해 필요

### 3.3.2 First-Order Method (SCS)

Operator splitting cone 방법:

$$
x_{k+1} = x_k - \eta \nabla f(x_k)
$$

배리어 없이 직접 일차 미분만 쓴다.

**장점**: 매우 빠름, 메모리 효율적, 대규모 문제 적합  
**단점**: 정확도 낮음 (1e-4 수준), 거의-부등호 검증 필수

### 3.3.3 언제 어떤 Solver를 쓸 것인가

- **프로토타입, 연구**: SeDuMi 또는 MOSEK (정확도 중시)
- **대규모 문제**: MOSEK (상용 라이선스 있으면) 또는 SCS
- **실시간 제어**: SCS (빠른 feedback)
- **일회성 검증**: 아무거나 → 다중 solver로 cross-check

---

## 3.4 수치 검증 체크리스트

Solver가 "optimal"을 반환해도, **항상** 다음을 검증하라:

### 체크 1: $P$의 최소 고유값

$$
\lambda_{\min}(P) > \varepsilon \quad \text{(e.g., } \varepsilon = 10^{-6}\text{)}
$$

**이유**: 수치적 마진을 확보. $\lambda_{\min}(P) = 10^{-12}$이면 $P$는 실제로 반정부호에 가깝다.

**코드** (MATLAB):
```matlab
lambda_min_P = min(eig(P));
if lambda_min_P < 1e-6
    warning('P is barely positive definite: lambda_min = %.2e', lambda_min_P)
end
```

### 체크 2: Lyapunov 부등식의 검증

$$
\lambda_{\max}(A^\top P + PA) < -\varepsilon \quad \text{(e.g., } \varepsilon = 10^{-6}\text{)}
$$

**이유**: 이 조건이 실제로 성립하는지 직접 확인. Solver의 내부 공식과 우리의 재계산이 일치하는지 보는 것.

**코드** (MATLAB):
```matlab
W = A'*P + P*A;
lambda_max_W = max(eig(W));
if lambda_max_W > -1e-6
    warning('A''P+PA not strictly negative: lambda_max = %.2e', lambda_max_W)
end
```

### 체크 3: Duality Gap과 Residual

Solver output에서:
- **Duality gap**: $< 10^{-8}$ (좋음), $10^{-5}$ ~ $10^{-8}$ (주의), $> 10^{-5}$ (신뢰 안 함)
- **Primal feasibility residual**: $\|F(x)\|$ (작을수록 좋음)
- **Dual feasibility residual**: 제약 위반의 크기

**코드** (MATLAB with CVX):
```matlab
disp(cvx_status)  % 'Solved'인지 확인
disp(cvx_optval)  % 최적값
% SeDuMi 내부 정보 (고급 사용자)
```

### 체크 4: 조건수

$$
\kappa(P) = \frac{\lambda_{\max}(P)}{\lambda_{\min}(P)}
$$

**해석**:
- $\kappa < 10^2$: 매우 좋음 (well-conditioned)
- $10^2 < \kappa < 10^4$: 괜찮음
- $10^4 < \kappa < 10^6$: 주의, 스케일링 고려
- $\kappa > 10^6$: 위험, 반드시 정규화 필요

**코드** (MATLAB):
```matlab
kappa_P = cond(P);
fprintf('Condition number of P: %.2e\n', kappa_P)
```

---

## 3.5 흔한 수치 문제와 해결

### 문제 1: Ill-conditioning

**증상**: 고유값이 매우 다르거나 시스템 행렬의 스케일이 큼.

**예**: $A = \begin{bmatrix} -1 & 0 \\ 0 & -10^6 \end{bmatrix}$

이 경우 $\lambda_1 = -1$과 $\lambda_2 = -10^6$의 차이가 $10^6$배 → 수치 오차 심화.

**해결책 1: 상태 정규화**

상태를 재스케일: $\tilde{x} = D x$, 여기서 $D = \text{diag}(\sqrt{\lambda_{\max}/\lambda_i})$.

그럼 새 시스템은 고유값이 모두 비슷해진다.

**해결책 2: 제약 정규화**

$P$에 trace 조건을 추가:

$$
\text{trace}(P) = 1
$$

또는

$$
P \succeq \varepsilon I
$$

이렇게 하면 Solver가 $P$를 "과도하게 크게" 만들지 못하도록 제어한다.

**MATLAB 예** (CVX):
```matlab
cvx_begin sdp
    variable P(n, n) symmetric
    minimize( ... )  % 또는 feasibility 문제
    subject to
        P >= 1e-6 * eye(n);       % 하한
        trace(P) <= 1;             % 상한
        A'*P + P*A <= -1e-6*eye(n);
cvx_end
```

### 문제 2: 부등호 방향 실수

**흔한 실수**:
```matlab
% 잘못된 예
A'*P + P*A >= 0  % 안정성이 아니라 불안정성!
```

**정확한 형식** (안정성):
$$
A^\top P + PA \prec 0
$$

코드로:
```matlab
A'*P + P*A <= -1e-6 * eye(n)
```

### 문제 3: 마진(margin) $\varepsilon$ 선택

Decay rate를 원하면:

$$
A^\top P + PA + 2\alpha P \prec 0
$$

이 경우 $\varepsilon$를 고르지 말고 $\alpha$를 결정해야 한다. 

**좋은 관행**: $\alpha = 0.1 \times \min(|\text{Re}(\lambda_i(A))|)$로 보수적으로 설정.

---

## 3.6 예제

### 예제 3.1: Well-conditioned 시스템

예제 2.2의 상태궤환 제어:

$$
A = \begin{bmatrix} 0 & 1 \\ -2 & -3 \end{bmatrix}, \quad B = \begin{bmatrix} 0 \\ 1 \end{bmatrix}
$$

이 시스템을 3개의 Solver로 풀어보자.

**MATLAB 코드**:
```matlab
A = [0 1; -2 -3];
B = [0; 1];
n = 2;

% SeDuMi
options_sedumi = sdpsettings('solver', 'sedumi', 'verbose', 1);
cvx_begin sdp
    variable P(n, n) symmetric
    subject to
        P >= 0.01*eye(n);
        A'*P + P*A + P*B*B'*P/4 <= -0.1*eye(n);
cvx_end
P_sedumi = P;
fprintf('SeDuMi: status = %s, lambda_min(P) = %.4e\n', cvx_status, min(eig(P_sedumi)))

% MOSEK
options_mosek = sdpsettings('solver', 'mosek', 'verbose', 1);
cvx_begin sdp
    variable P(n, n) symmetric
    subject to
        P >= 0.01*eye(n);
        A'*P + P*A + P*B*B'*P/4 <= -0.1*eye(n);
cvx_end
P_mosek = P;
fprintf('MOSEK: status = %s, lambda_min(P) = %.4e\n', cvx_status, min(eig(P_mosek)))

% SCS
options_scs = sdpsettings('solver', 'scs', 'verbose', 1);
cvx_begin sdp
    variable P(n, n) symmetric
    subject to
        P >= 0.01*eye(n);
        A'*P + P*A + P*B*B'*P/4 <= -0.1*eye(n);
cvx_end
P_scs = P;
fprintf('SCS: status = %s, lambda_min(P) = %.4e\n', cvx_status, min(eig(P_scs)))

% 비교
fprintf('\n=== 결과 비교 ===\n')
fprintf('||P_sedumi - P_mosek|| = %.4e\n', norm(P_sedumi - P_mosek, 'fro'))
fprintf('||P_mosek - P_scs|| = %.4e\n', norm(P_mosek - P_scs, 'fro'))
```

**예상 결과**:
- SeDuMi, MOSEK: $P$가 매우 유사, residual $< 10^{-6}$
- SCS: $P$가 약간 다름 (정확도), residual $\sim 10^{-4}$

---

### 예제 3.2: Ill-conditioned 시스템과 정규화

$$
A = \begin{bmatrix} -1 & 0 \\ 0 & -10^5 \end{bmatrix}
$$

**정규화 없이** (bad):
```matlab
A = diag([-1, -1e5]);
cvx_begin sdp
    variable P(2, 2) symmetric
    subject to
        P >= 0;
        A'*P + P*A <= -1e-6*eye(2);
cvx_end
disp(['κ(P) = ', num2str(cond(P))])  % 매우 큼, ~1e10
```

**상태 정규화로** (good):
```matlab
% 고유값 정규화: D = diag(sqrt(1e5/|lambda|))
D = diag([sqrt(1e5/1), sqrt(1e5/1e5)]);  % [1e2.5, 1]
D_inv = inv(D);

% 새 시스템
A_scaled = D * A * D_inv;  % 고유값이 비슷해짐

cvx_begin sdp
    variable P_tilde(2, 2) symmetric
    subject to
        P_tilde >= 1e-6*eye(2);
        A_scaled'*P_tilde + P_tilde*A_scaled <= -1e-6*eye(2);
cvx_end

% 원래 공간으로 변환
P = D_inv' * P_tilde * D_inv;

disp(['κ(P) = ', num2str(cond(P))])  % 훨씬 작음, ~1e3
```

---

### 예제 3.3: Feasibility vs. Optimization

2장의 문제를 두 가지로 풀자:

**(a) Pure feasibility** — 가능한 $P, K$만 찾으면 됨:
```matlab
cvx_begin sdp
    variable P(n, n) symmetric
    variable Y(m, n)
    subject to
        P >= 1e-6*eye(n);
        A'*P + P*A + Y'*B' + B*Y <= -1e-6*eye(n);
cvx_end
K = Y / P;  % 기약분수로 변환
```

**(b) Optimization** — 가능하면서 decay rate $\alpha$ 최대화:
```matlab
cvx_begin sdp
    variable P(n, n) symmetric
    variable Y(m, n)
    variable alpha
    subject to
        P >= 1e-6*eye(n);
        A'*P + P*A + Y'*B' + B*Y + 2*alpha*P <= -1e-6*eye(n);
        alpha >= 0;
    maximize( alpha )
cvx_end
K = Y / P;
fprintf('Maximum decay rate: α = %.4f\n', alpha)
```

**(a)**의 해는 더 빠르지만 임의적. **(b)**의 해는 최적이지만 계산량 약간 증가.

---

## 3.7 실습 코드

### 3.7.1 MATLAB: 검증 함수

```matlab
function [status, metrics] = verify_sdp_solution(P, A, epsilon)
% verify_sdp_solution - Lyapunov 해의 수치 검증
%
% 입력:
%   P: 반환된 Lyapunov 행렬 (n x n)
%   A: 시스템 행렬 (n x n)
%   epsilon: 마진 (기본값 1e-6)
%
% 출력:
%   status: 'PASS' 또는 'FAIL'
%   metrics: 구조체, 모든 검증값 포함

if nargin < 3
    epsilon = 1e-6;
end

n = size(A, 1);
status = 'PASS';
metrics = struct();

% Check 1: P >= 0
eigs_P = eig(P);
metrics.lambda_min_P = min(eigs_P);
metrics.lambda_max_P = max(eigs_P);

if metrics.lambda_min_P < epsilon
    fprintf('❌ P not sufficiently positive definite: λ_min = %.2e\n', metrics.lambda_min_P)
    status = 'FAIL';
else
    fprintf('✓ P positive definite: λ_min = %.2e\n', metrics.lambda_min_P)
end

% Check 2: A'P + PA < 0
W = A'*P + P*A;
eigs_W = eig(W);
metrics.lambda_max_W = max(eigs_W);

if metrics.lambda_max_W > -epsilon
    fprintf('❌ A''P+PA not strictly negative: λ_max = %.2e\n', metrics.lambda_max_W)
    status = 'FAIL';
else
    fprintf('✓ A''P+PA strictly negative: λ_max = %.2e\n', metrics.lambda_max_W)
end

% Check 3: Condition number
metrics.cond_P = cond(P);
if metrics.cond_P > 1e6
    fprintf('⚠ High condition number: κ(P) = %.2e (consider scaling)\n', metrics.cond_P)
else
    fprintf('✓ Condition number acceptable: κ(P) = %.2e\n', metrics.cond_P)
end

% Check 4: Residual
metrics.residual_P = norm(P - P', 'fro');  % 대칭인지 확인
if metrics.residual_P > eps(class(P))
    fprintf('❌ P is not symmetric: ||P - P''|| = %.2e\n', metrics.residual_P)
    status = 'FAIL';
else
    fprintf('✓ P is symmetric\n')
end

end
```

**사용 예**:
```matlab
A = [-1 0.5; 0 -2];
P = lyap(A', eye(2));  % 또는 Solver에서 반환된 P

[status, metrics] = verify_sdp_solution(P, A, 1e-6);
if strcmp(status, 'PASS')
    disp('해가 수치적으로 신뢰할 수 있음!')
end
```

### 3.7.2 Python: CVXPy 예제

```python
import numpy as np
import cvxpy as cp
from scipy.linalg import eigh

A = np.array([[-1.0, 0.5], [0.0, -2.0]])
B = np.array([[0.0], [1.0]])
n = A.shape[0]
m = B.shape[1]

# 상태궤환 제어: (A + BK)'P + P(A + BK) < 0
P = cp.Variable((n, n), symmetric=True)
Y = cp.Variable((m, n))

constraints = [
    P >= 1e-6 * np.eye(n),
    A.T @ P + P @ A + Y.T @ B.T + B @ Y <= -1e-6 * np.eye(n)
]

# Feasibility 문제
problem = cp.Problem(cp.Minimize(0), constraints)

# Solver 선택: 'SCS' (기본), 'ECOS' (중간), 'MOSEK' (상용)
problem.solve(solver=cp.SCS, verbose=True)

print(f"Status: {problem.status}")
print(f"P eigenvalues: {np.linalg.eigvalsh(P.value)}")

# 검증
W = A.T @ P.value + P.value @ A
lambda_max_W = np.max(np.linalg.eigvalsh(W))
print(f"λ_max(A'P+PA) = {lambda_max_W:.6e} (should be < -1e-6)")

if problem.status == 'optimal' and lambda_max_W < -1e-6:
    K = Y.value @ np.linalg.inv(P.value)
    print(f"Stabilizing gain: K = {K}")
```

---

## 3.8 복습 문제

1. **조건수의 의미**: $P$의 조건수가 $10^8$이면 수치해석에서 무엇을 의미하는가? 어떻게 개선할 것인가?

2. **Duality gap**: SDP 문제에서 duality gap이 $10^{-3}$이라면, 이 해를 신뢰할 수 있는가? 왜/왜 아닌가?

3. **Solver 선택**: 실시간 제어 루프에서 LMI를 매 100ms마다 풀어야 한다면, 어떤 Solver를 쓸 것인가? 이유는?

4. **정규화 기법**: 극단적으로 다른 시간상수를 가진 로봇 (관절 1: 0.1초, 관절 2: 10초)의 LMI를 풀 때, 어떻게 정규화할 것인가?

---

## 3.9 이 장의 핵심 요약

> **Solver는 증명의 도구이지, 진리의 판결자가 아니다.**
>
> "Optimal"은 수치적 의미일 뿐. 반환된 $P$는 항상 다음을 확인해야 한다:
> 1. 최소 고유값이 충분히 양수인가? ($> 10^{-6}$)
> 2. 실제로 $A^\top P + PA \prec 0$인가? (직접 계산으로 재검증)
> 3. 조건수가 안전한가? ($\kappa < 10^5$)
> 4. Duality gap이 충분히 작은가? ($< 10^{-8}$)
>
> 이제 **단일 시스템 $A$ 하나**가 아니라, **여러 개의 $A_i$가 불확실하게 섞여 있다면?** Part II에서 그 답을 찾는다.
