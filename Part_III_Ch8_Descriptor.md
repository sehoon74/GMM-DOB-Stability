# 8장. DOB Feedback과 Descriptor 오차동역학

> **이 장의 목표**: TDoF 제어 구조(FF + FB + DOB)에서 나타나는 가속도 항의 대수적 루프를 해결하기 위해 Descriptor 시스템 형태로 오차동역학을 정리한다.
>
> **선수 지식**: 4~5장 (TS Fuzzy 시스템, GMR 구조), 6장 (DOB 개요)
>
> **다음 장과의 연결**: 여기서 유도한 Descriptor 형태가 9장에서 Fuzzy Lyapunov 함수로 안정성을 증명하는 LMI 조건의 기초가 된다.

---

## 8.1 왜 Descriptor인가

### 8.1.1 가속도 항의 대수적 루프

앞서 TDoF 제어를 살펴보았다:

$$
\tau_c = \tau_{FF} + \tau_{FB} + \hat{d}
$$

여기서:
- $\tau_{FF} = Q(s) \sum_i h_i(\xi^d) (A_i \xi^d + B_i)$: 피드포워드
- $\tau_{FB} = -\hat{M}K_p e - \hat{M}K_d \dot{e}$: 피드백
- $\hat{d} = Q(s)[f(\xi) - \tau_c]$: DOB 추정

DOB가 피드백할 때, 내부 모델 $f(\xi)$에는 **가속도 항**이 포함된다:

$$
\hat{f}_{\mathrm{PPLM}}(\xi) = \sum_i h_i(A_i^{11}e + A_i^{12}\dot{e} + A_i^{13}\ddot{e}) + \cdots
$$

따라서 오차 동역학은:

$$
M\ddot{e} = -\sum_i h_i(A_i^{11}e + A_i^{12}\dot{e} + A_i^{13}\ddot{e}) - \hat{M}K_p e - \hat{M}K_d \dot{e} + \cdots
$$

**문제**: $\ddot{e}$가 양쪽에 나타난다! ($M$의 좌변과 $A_i^{13}$의 우변). 

> 만약 $\ddot{e} = (M + A_i^{13})^{-1}[\cdots]$ 형태로 정리하려 하면, **역행렬 $(M + A_i^{13})^{-1}$이 $h_i$에 의존**하게 되어 TS Fuzzy의 "볼록결합(convex combination)" 구조가 깨진다. 이는 LMI 기반 안정성 증명을 불가능하게 한다.

### 8.1.2 Descriptor 시스템의 우아한 해법

**Descriptor 형태** (또는 DAE, Differential-Algebraic Equation)는 이 문제를 풀기 위한 표준 도구이다:

$$
E^* \dot{\eta}^*(t) = \sum_{i=1}^C h_i \mathcal{A}_i \eta^*(t) + \mathcal{T}^*
$$

여기서:
- $E^* = \mathrm{blkdiag}(I, 0)$: **특이(singular)** 행렬
- $\eta^* = [\eta^\top, \dot{\eta}^\top]^\top = [e^\top, \dot{e}^\top, \dot{e}^\top, \ddot{e}^\top]^\top$: 확장된 상태
- 하단 블록: $0 \cdot \ddot{\eta}^* = \cdots$는 **대수적 제약조건**

**핵심 장점**:
1. $\ddot{e}$의 계수 $M + A_i^{13}$을 역행렬로 만들지 않아도 된다.
2. $h_i$의 convex combination 구조가 $\mathcal{A}_i$ 행렬에서 보존된다.
3. 따라서 LMI 기반 안정성 분석이 가능해진다.

---

## 8.2 TDoF 구조와 DOB의 역할

### 8.2.1 세 가지 토크 성분

TDoF 제어의 세 가지 성분을 정리하자:

$$
\begin{aligned}
\tau_{FF} &= Q(s) \sum_i h_i(\xi^d) (A_i \xi^d + B_i) \\
\tau_{FB} &= -\hat{M}K_p e - \hat{M}K_d \dot{e} \\
\hat{d} &= Q(s) \left[ \sum_i h_i(\xi) (A_i \xi + B_i) - \tau_c \right]
\end{aligned}
$$

여기서:
- $\xi^d = [q^d, \dot{q}^d, \ddot{q}^d]^\top$: 목표궤적의 상태
- $\xi = [q, \dot{q}, \ddot{q}^\top]^\top$: 실제 상태
- $e = q - q^d$: 위치 오차

**부호 주의**: $\tau_{FB}$는 **음의 부호**를 가진다. 오차를 줄이기 위해 오차 방향과 반대로 작용해야 하기 때문이다.

### 8.2.2 식물(Plant) 동역학

외란 $d$가 있는 상태에서:

$$
M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) = \tau_c + d
$$

또는 간단히:

$$
f(\xi) = \tau_c + d
$$

여기서 $f(\xi) := M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q)$는 **실제 역동역학**이다.

---

## 8.3 DOB의 핵심 등식

### 8.3.1 DOB 입력-출력 관계

DOB의 입력을 정리하면:

$$
\hat{f}_{\mathrm{PPLM}}(\xi) - \tau_c = \hat{f}_{\mathrm{PPLM}}(\xi) - [f(\xi) - d]
$$

$$
= \hat{f}_{\mathrm{PPLM}}(\xi) - f(\xi) + d = -\tilde{f}(\xi) + d
$$

여기서 $\tilde{f}(\xi) := f(\xi) - \hat{f}_{\mathrm{PPLM}}(\xi)$는 **실제 상태에서의 모델 오차**이다.

따라서 DOB 추정값은:

$$
\hat{d} = Q(s)[d - \tilde{f}(\xi)]
$$

**의미**: DOB는 외란 $d$와 모델 오차 $\tilde{f}$를 모두 보상하되, Q-필터를 통해 저주파 대역만 피드백한다.

### 8.3.2 유효 토크 균형

세 성분을 합하면:

$$
f(\xi) = \tau_{FF} + \tau_{FB} + (1-Q(s))d + Q(s)\tilde{f}(\xi)
$$

**해석**:
- $(1-Q(s))d$: 고주파 외란 (DOB가 보상 못 함)
- $Q(s)\tilde{f}(\xi)$: 저주파 모델 오차 (DOB가 보상)
- $Q(s) \approx 1$ 근처에서는 모델 오차가 대부분 취소된다.

---

## 8.4 오차동역학의 유도

### 8.4.1 PD 피드백과 DOB를 포함한 오차방정식

목표궤적 동역학을 참조점에서 빼면:

$$
\hat{f}_{\mathrm{PPLM}}(\xi) - \hat{f}_{\mathrm{PPLM}}(\xi^d) \approx \sum_i h_i A_i [e, \dot{e}, \ddot{e}]^\top
$$

(멤버십 함수 미스매치와 고계 항은 잔차에 흡수)

따라서 $Q(s) \approx 1$일 때:

$$
M\ddot{e} = -\sum_i h_i(A_i^{11}e + A_i^{12}\dot{e} + A_i^{13}\ddot{e}) - \hat{M}K_p e - \hat{M}K_d \dot{e} + \mathcal{T}_2
$$

여기서 $\mathcal{T}_2$는:
- Q-필터의 유한대역폭 잔차 (크기 $\mathcal{O}(s/L)$)
- 멤버십 함수 미스매치
- 모델링되지 않은 동역학

### 8.4.2 세 가지 DOB 기여도

DOB가 피드백하는 세 가지 항을 분리해서 보자:

$$
\sum_i h_i A_i [e, \dot{e}, \ddot{e}]^\top = \sum_i h_i(A_i^{11}e + A_i^{12}\dot{e} + A_i^{13}\ddot{e})
$$

**물리적 의미**:
- $A_i^{11}e$: **위치 기반 강성** (중력, 구성에 따른 효과)
- $A_i^{12}\dot{e}$: **속도 기반 감소** (코리올리, 마찰)
- $A_i^{13}\ddot{e}$: **관성 변형** (실제 관성 $M$에 학습된 항 $A_i^{13}$ 추가)

---

## 8.5 Local-Sum 형태

### 8.5.1 상태벡터 재정의

$\eta := [e, \dot{e}]^\top \in \mathbb{R}^{2n}$로 정의하고 $\dot{\eta} = [\dot{e}, \ddot{e}]^\top$로 표기하자.

오차방정식은:

$$
\sum_i h_i \mathcal{R}_i \dot{\eta} = \sum_i h_i \mathcal{S}_i \eta + \mathcal{T}
$$

### 8.5.2 행렬 $\mathcal{R}_i$, $\mathcal{S}_i$의 정의

$$
\mathcal{R}_i = \begin{bmatrix} I & 0 \\ 0 & M + A_i^{13} \end{bmatrix}
$$

$\mathcal{R}_i$는 두 부분으로 분해:

$$
\mathcal{R}_i = \underbrace{\begin{bmatrix} I & 0 \\ 0 & M \end{bmatrix}}_{\mathcal{R}^{\mathrm{plant}}} + \underbrace{\begin{bmatrix} 0 & 0 \\ 0 & A_i^{13} \end{bmatrix}}_{\mathcal{R}_i^{\mathrm{DOB}}}
$$

마찬가지로:

$$
\mathcal{S}_i = \begin{bmatrix} 0 & I \\ -(A_i^{11} + \hat{M}K_p) & -(A_i^{12} + \hat{M}K_d) \end{bmatrix}
$$

분해:

$$
\mathcal{S}_i = \underbrace{\begin{bmatrix} 0 & I \\ -\hat{M}K_p & -\hat{M}K_d \end{bmatrix}}_{\mathcal{S}^{\mathrm{PD}}} + \underbrace{\begin{bmatrix} 0 & 0 \\ -A_i^{11} & -A_i^{12} \end{bmatrix}}_{\mathcal{S}_i^{\mathrm{DOB}}}
$$

### 8.5.3 문제: 좌변의 $h_i$-의존 합

$$
\sum_i h_i \mathcal{R}_i \dot{\eta}
$$

이 형태는 **$\dot{\eta}$의 계수가 시간변화**하므로, 직접적인 LMI 조건을 세울 수 없다.

> **비교**: $\dot{\eta} = A\eta$ (고정 계수) → LMI 가능  
> $\sum_i h_i \mathcal{R}_i \dot{\eta} = \sum_i h_i \mathcal{S}_i \eta$ (변수 계수) → LMI 불가능

---

## 8.6 확장된 Descriptor 형태

### 8.6.1 상태 확장

**새로운 확장 상태** $\eta^* \in \mathbb{R}^{4n}$를 정의:

$$
\eta^* = \begin{bmatrix} \eta \\ \dot{\eta} \end{bmatrix} = \begin{bmatrix} e \\ \dot{e} \\ \dot{e} \\ \ddot{e} \end{bmatrix}
$$

이때 $\eta$와 $\dot{\eta}$를 **독립적인 변수**로 취급한다. 관계식 $\dot{\eta} = \frac{d}{dt}\eta$는 **대수적 제약조건**으로 인코딩된다.

### 8.6.2 Descriptor 시스템의 정의

$$
E^* \dot{\eta}^* = \sum_i h_i \mathcal{A}_i \eta^* + \mathcal{T}^*
$$

여기서:

$$
E^* = \mathrm{blkdiag}(I, 0) = \begin{bmatrix} I & 0 \\ 0 & 0 \end{bmatrix}
$$

(상단: $2n \times 2n$, 하단: $2n \times 2n$, 모두 0)

$$
\mathcal{A}_i = \begin{bmatrix} 0 & I \\ \mathcal{S}_i & -\mathcal{R}_i \end{bmatrix}
$$

$$
\mathcal{T}^* = \begin{bmatrix} 0 \\ \mathcal{T} \end{bmatrix}
$$

### 8.6.3 블록 구조 해석

**상단 블록** ($2n$ 행):

$$
I \cdot \dot{e} = I \cdot \dot{e}
$$

(항등식, 역동역학 아님)

**하단 블록** ($2n$ 행):

$$
0 \cdot \ddot{\eta} = \mathcal{S}_i \eta - \mathcal{R}_i \dot{\eta} + \mathcal{T}
$$

이를 정리하면:

$$
\mathcal{R}_i \dot{\eta} = \mathcal{S}_i \eta + \mathcal{T}
$$

이것이 원래 오차동역학! 하단 블록은 **대수적 제약조건** 역할을 한다.

### 8.6.4 왜 명시적 형태를 쓸 수 없는가

만약 $(\sum_i h_i \mathcal{R}_i)^{-1}$을 직접 계산하여 다음 형태로 만든다면:

$$
\dot{\eta} = \left(\sum_i h_i \mathcal{R}_i\right)^{-1} \sum_i h_i \mathcal{S}_i \eta + \cdots
$$

**문제**: $(\sum_i h_i \mathcal{R}_i)^{-1}$은 **각 $h_i$에 비선형적으로 의존**한다. (역행렬 함수는 비선형) 따라서 LMI 조건:

$$
\mathcal{A}^\top P + P^\top \mathcal{A} \prec 0
$$

에 대한 convex optimization이 불가능해진다.

**Descriptor 형태의 우점**: 역행렬을 없애고 **$h_i$를 선형으로 유지**하면서, LMI 프레임워크를 적용할 수 있다.

---

## 8.7 정칙성(Regularity) 가정

### 8.7.1 Descriptor 쌍의 정칙성 조건

Descriptor 시스템 $(E^*, \mathcal{A}_i)$가 **정칙(regular)**이려면, 모든 $i = 1, \ldots, C$에 대해:

$$
\det(sE^* - \mathcal{A}_i) \not\equiv 0 \quad \text{(다항식으로서)}
$$

이 조건은 다음과 동치:

$$
\det(I + M^{-1}(q) A_i^{13}) \neq 0 \quad \text{for all } i
$$

### 8.7.2 물리적 의미

잘 훈련된 PPLM에서 $A_i^{13} \approx M(q_i^*)$이다. (q_i^*$는 i번째 가우시안의 중심)

따라서:

$$
I + M^{-1} A_i^{13} \approx I + M^{-1}M = 2I
$$

이는 **큰 마진**을 가진다. Determinant:

$$
\det(I + M^{-1}A_i^{13}) \approx \det(2I) = 2^n \neq 0
$$

충분히 크기가 0에서 멀어 있다.

### 8.7.3 Neumann 급수를 이용한 검증

$A_i^{13} \approx M$일 때, 다음을 확인하면 정칙성이 보장된다:

$$
\|I - M^{-1}A_i^{13}\| < 1
$$

그러면 Neumann 급수:

$$
(I + M^{-1}A_i^{13})^{-1} = \sum_{k=0}^\infty (-M^{-1}A_i^{13})^k
$$

가 수렴한다.

> **흔한 실수**: $\|M^{-1}A_i^{13}\| < 1$을 확인하는 것. 이는 $A_i^{13} \approx M$과 모순이다!

### 8.7.4 수치적 검증 절차

1. PPLM 훈련 완료 후, 대표 구성(representative configurations) $q_1, q_2, \ldots$에서 $M(q)$와 $A_i^{13}$ 계산
2. 각 $i$마다 $\det(I + M^{-1}(q_j)A_i^{13})$ 계산
3. 모든 값이 0에서 충분히 멀면 (예: $> 0.5$) 정칙성 만족

---

## 8.8 예제: 1-DOF 로봇 팔

### 8.8.1 설정

- 질량 $m = 2$ kg, 길이 $L = 1$ m
- PPLM: 2개 가우시안 성분 ($C=2$)
- 학습된 파라미터 (예시):

**Component 1** (저속 영역):
$$
A_1 = \begin{bmatrix} 0 & 0 & 0.5 \\ 0 & 0 & 1.0 \\ 0 & 0 & 0.8 \end{bmatrix}, \quad B_1 = \begin{bmatrix} 0 \\ 0 \\ 0.1 \end{bmatrix}
$$

**Component 2** (고속 영역):
$$
A_2 = \begin{bmatrix} 0 & 0.2 & 0.6 \\ 0 & 0 & 1.2 \\ 0 & 0 & 0.9 \end{bmatrix}, \quad B_2 = \begin{bmatrix} 0 \\ 0 \\ 0.2 \end{bmatrix}
$$

PD 게인: $K_p = 20$, $K_d = 5$, $\hat{M} = 2$

### 8.8.2 행렬 계산

$\mathcal{R}_1$:

$$
\mathcal{R}_1 = \begin{bmatrix} 1 & 0 \\ 0 & 2 + 0.8 \end{bmatrix} = \begin{bmatrix} 1 & 0 \\ 0 & 2.8 \end{bmatrix}
$$

$\mathcal{S}_1$:

$$
\mathcal{S}_1 = \begin{bmatrix} 0 & 1 \\ -(0.5 + 2 \times 20) & -(1.0 + 2 \times 5) \end{bmatrix} = \begin{bmatrix} 0 & 1 \\ -40.5 & -11 \end{bmatrix}
$$

마찬가지로 $\mathcal{R}_2$, $\mathcal{S}_2$ 계산.

### 8.8.3 Descriptor 행렬 구성

$$
\mathcal{A}_1 = \begin{bmatrix} 0 & 1 & 0 & 0 \\ 0 & 0 & 1 & 0 \\ -40.5 & -11 & 0 & -1 \\ 0 & 0 & 0 & 0 \end{bmatrix}
$$

$(4 \times 4$ 블록 행렬)

### 8.8.4 정칙성 확인

$$
\det(I + M^{-1}A_i^{13}) = \det(1 + \frac{A_i^{13}}{2})
$$

$A_1^{13} = 0.8 \Rightarrow \det = 1 + 0.4 = 1.4 > 0$ ✓  
$A_2^{13} = 0.9 \Rightarrow \det = 1 + 0.45 = 1.45 > 0$ ✓

---

## 8.9 실습 코드

### 8.9.1 MATLAB: Descriptor 행렬 구성

```matlab
%% Ch8 예제: 1-DOF 로봇, 2-rule TS Fuzzy
clear; clc

% 파라미터
m = 2; L = 1;
Kp = 20; Kd = 5;
Mhat = 2;

% 학습된 A, B (2개 component)
A{1} = [0   0   0.5;
        0   0   1.0;
        0   0   0.8];
B{1} = [0; 0; 0.1];

A{2} = [0   0.2  0.6;
        0   0    1.2;
        0   0    0.9];
B{2} = [0; 0; 0.2];

% Descriptor 시스템 구성
n = 1;  % DOF
C = 2;  % 규칙 수

% R_i, S_i 계산
for i = 1:C
    R{i} = [eye(n)        zeros(n,n);
            zeros(n,n)    m + A{i}(3,3)];
    
    S{i} = [zeros(n,n)                  eye(n);
            -(A{i}(1,1) + Mhat*Kp)  -(A{i}(2,2) + Mhat*Kd)];
    
    % A_i 행렬 (Descriptor 시스템)
    A_desc{i} = [zeros(2*n)  eye(2*n);
                 S{i}       -R{i}];
end

% E* 행렬 (singular)
E_star = blkdiag(eye(2*n), zeros(2*n));

% 정칙성 확인
fprintf('=== Regularity Check ===\n');
for i = 1:C
    det_val = det(eye(n) + (1/m)*A{i}(3,3));
    fprintf('i=%d: det(I + M^{-1}A_i^{13}) = %.4f\n', i, det_val);
    if det_val <= 0
        error('Regularity violated for i=%d', i);
    end
end

fprintf('\nDescriptor matrices constructed successfully.\n');
fprintf('A_desc{1} size: %d x %d\n', size(A_desc{1}));
```

### 8.9.2 Python: Descriptor 시스템 시뮬레이션

```python
import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

# 파라미터
m = 2.0
Kp, Kd = 20, 5
Mhat = 2.0

# 학습된 A, B
A_list = [
    np.array([[0,   0,   0.5],
              [0,   0,   1.0],
              [0,   0,   0.8]]),
    np.array([[0,   0.2, 0.6],
              [0,   0,   1.2],
              [0,   0,   0.9]])
]

B_list = [np.array([0, 0, 0.1]), np.array([0, 0, 0.2])]

# 멤버십 함수 (예: 가우시안)
def membership(xi, mu=[0, 0, 0], sigma=0.5):
    """xi = [q, q_dot, q_ddot]"""
    dist_sq = np.sum(((xi - np.array(mu))/sigma)**2)
    h = np.exp(-0.5*dist_sq)
    return h

# Descriptor 행렬 구성 함수
def construct_descriptor_matrices(A, B, Kp, Kd, Mhat, m):
    """
    Returns: A_desc, E_star for eta^* = [eta; eta_dot]
    """
    n = 1  # DOF (scalar)
    
    # R_i, S_i
    R_i = np.array([[1,         0],
                    [0,  m + A[2,2]]])
    
    S_i = np.array([[0,                      1],
                    [-(A[0,0] + Mhat*Kp),  -(A[1,1] + Mhat*Kd)]])
    
    # A_desc_i = [0  I; S_i  -R_i]
    A_desc = np.vstack([
        np.hstack([np.zeros((2,2)),  np.eye(2)]),
        np.hstack([S_i,             -R_i])
    ])
    
    E_star = np.vstack([
        np.hstack([np.eye(2),    np.zeros((2,2))]),
        np.zeros((2,4))
    ])
    
    return A_desc, E_star, R_i, S_i

# 계산
A_desc_list = []
E_star_common = None

for i, (A, B) in enumerate(zip(A_list, B_list)):
    A_desc_i, E_star, R_i, S_i = construct_descriptor_matrices(
        A, B, Kp, Kd, Mhat, m
    )
    A_desc_list.append(A_desc_i)
    if E_star_common is None:
        E_star_common = E_star

print("Descriptor system matrices constructed.")
print(f"A_desc[0] shape: {A_desc_list[0].shape}")
print(f"E_star shape: {E_star_common.shape}")

# 정칙성 확인
print("\n=== Regularity Check ===")
for i, A in enumerate(A_list):
    det_val = 1 + A[2,2] / m
    print(f"i={i}: det(I + M^-1 * A_i^{{13}}) = {det_val:.4f}")

print("\nDescriptor form ready for LMI analysis (Chapter 9)")
```

### 8.9.3 정칙성 검증 함수 (일반화)

```matlab
function check_regularity(A_cell, M_nominal)
    % A_cell: cell array of A_i matrices (3x3 partition)
    % M_nominal: nominal inertia M(q*)
    
    C = length(A_cell);
    fprintf('=== Regularity Verification ===\n');
    
    all_regular = true;
    for i = 1:C
        A_i13 = A_cell{i}(3,3);
        M_inv = inv(M_nominal);
        
        % Check 1: det(I + M^{-1}A_i^{13}) != 0
        det_matrix = eye(size(M_nominal)) + M_inv * A_i13;
        det_val = det(det_matrix);
        
        % Check 2: Neumann series |I - M^{-1}A_i^{13}| < 1
        neumann_check = norm(eye(size(M_nominal)) - M_inv * A_i13);
        
        fprintf('Rule %d:\n', i);
        fprintf('  det(I + M^{-1}A_i^{13}) = %.6f', det_val);
        if abs(det_val) < 1e-6
            fprintf(' [FAIL]\n');
            all_regular = false;
        else
            fprintf(' [OK]\n');
        end
        
        fprintf('  ||I - M^{-1}A_i^{13}|| = %.6f', neumann_check);
        if neumann_check >= 1.0
            fprintf(' [WARNING: Neumann series may not converge]\n');
        else
            fprintf(' [OK]\n');
        end
    end
    
    if all_regular
        fprintf('\n✓ All regularity conditions satisfied.\n');
    else
        error('Regularity condition violated. System is singular.');
    end
end
```

---

## 8.10 복습 문제

1. **가속도 항의 대수적 루프**: 왜 명시적 형태 $\ddot{e} = (M + A_i^{13})^{-1}[\cdots]$을 쓸 수 없는가? (힌트: LMI와 convex structure)

2. **Descriptor 형태의 장점**: 다음 중 Descriptor 시스템의 가장 중요한 이점은?
   - (a) 계산 속도 향상
   - (b) $h_i$의 선형성 유지로 LMI 적용 가능
   - (c) 물리적 직관의 향상
   
   정답과 그 이유를 설명하라.

3. **정칙성 조건**: $A_i^{13} = M + 0.3$일 때 (같은 크기의 정사각 행렬), $\det(I + M^{-1}A_i^{13})$를 계산하라.

4. **예제 8.1 확장**: 예제의 파라미터를 다음과 같이 바꾸고, 정칙성이 여전히 만족되는지 확인하라.
   - $K_p = 50$ (더 큰 게인)
   - $A_1^{13} = 1.9$ (학습이 덜 정확)

---

## 8.11 핵심 요약

**핵심 내용**:
1. TDoF 제어에서 DOB가 피드백할 때, 가속도 항 $\ddot{e}$가 양쪽에 나타나는 **대수적 루프** 문제가 생긴다.
2. 이를 해결하기 위해 **Descriptor 형태** $E^* \dot{\eta}^* = \sum_i h_i \mathcal{A}_i \eta^*$를 도입한다.
3. Descriptor 시스템에서는 $h_i$가 **선형**으로 유지되므로, TS Fuzzy의 convex structure가 보존되고 LMI 기반 분석이 가능하다.
4. DOB의 세 가지 기여 (강성 $A_i^{11}$, 감수 $A_i^{12}$, 관성 $A_i^{13}$)가 $\mathcal{A}_i$ 행렬에 명시적으로 나타난다.
5. **정칙성 조건** $\det(I + M^{-1}A_i^{13}) \neq 0$이 만족되면, Descriptor 시스템이 well-posed이다.

> **다음 장으로의 연결**: Part I-II에서 배운 Lyapunov 정리, LMI, TS Fuzzy 도구들을 이제 여기 Descriptor 시스템에 적용한다. 9장에서 Fuzzy Lyapunov 함수 $V = \eta^{*\top} E^{*\top} (\sum_i h_i P_i) \eta^*$와 LMI 조건 $\mathcal{A}_i^\top P_j + P_j^\top \mathcal{A}_i \prec 0$ (및 Tanaka-Wang 교차항)을 이용하여 **점근 안정성** 또는 **균일궁극유계(UUB)**를 증명한다.

---

## 참고 문헌

- Tanaka, K., & Wang, H. O. (2001). *Fuzzy Control Systems Design and Analysis: A Linear Matrix Inequality Approach*. John Wiley & Sons.
- Chen, W., Ballance, D. J., Gawthrop, P. J., & O'Reilly, J. (2000). A nonlinear disturbance observer for robotic manipulators. *IEEE Trans. Industrial Electronics*, 47(4), 932–938.
- Boyd, S., El Ghaoui, L., Feron, E., & Balakrishnan, V. (1994). *Linear Matrix Inequalities in System and Control Theory*. SIAM.
- Khalil, H. K. (2002). *Nonlinear Systems* (3rd ed.). Prentice Hall.

---

**Word count**: ~550 lines (including equations, examples, and code blocks)
