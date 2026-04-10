# 7장. GMM/GMR 기초와 TS Fuzzy로의 재해석

> **이 장의 목표**: 기계학습으로 훈련한 가우시안 혼합 모델(GMM)에서 회귀(GMR)로 얻은 예측기의 구조가, **구성적으로(by construction) 그대로 TS fuzzy 시스템**임을 알게 된다. 근사(approximation)가 아닌 **정확한 동등성**이다.
>
> **선수 지식**: 4-5장의 TS fuzzy, 부분공간 선형 시스템, 가중평균 개념
>
> **다음 장과의 연결**: 이 장에서 보인 등가성이 8장의 DOB 폐루프 안정성 증명의 근간이 된다.

---

## 7.1 왜 이 장이 핵심 다리인가

Part II에서 우리는 TS fuzzy 시스템

$$
\dot{x} = \sum_{i=1}^{C} h_i(z) A_i x
$$

의 안정성을 LMI로 분석하는 방법을 배웠다 (4, 5장). Part III에서는 로봇 제어기에 **GMR을 사용한 역동역학 예측**을 넣으려고 한다:

$$
\hat{\tau} = f_{\text{PPLM}}(\xi) = \sum_{i=1}^{C} h_i(\xi) (A_i \xi + B_i)
$$

여기서 $\xi = [q, \dot{q}, \ddot{q}]^\top$이고, $h_i$는 i번째 가우시안의 사후확률(posterior), $A_i$와 $B_i$는 조건부 회귀 행렬이다.

**핵심 질문**: GMR 예측기는 단순히 "TS fuzzy처럼 보이는" 근사인가, 아니면 **정확한 TS fuzzy 시스템**인가?

**답**: **정확한 등가성**이다. GMM의 수학적 구조 자체가 TS fuzzy의 세 조건(normalization, non-negativity, smoothness)을 **자동으로 만족**한다. 따라서 GMR은 근사가 아니라 **구성적 동등성**이다. 이것이 이 장의 핵심 아이디어이고, 8장에서 DOB 폐루프의 안정성 증명이 강력해지는 이유이다.

---

## 7.2 GMM 기초

### 7.2.1 가우시안 혼합 모델의 정의

가우시안 혼합 모델(Gaussian Mixture Model)은 데이터의 결합 분포를 다음과 같이 표현한다:

$$
p(\zeta) = \sum_{i=1}^{C} \pi_i \mathcal{N}(\zeta; \mu_i, \Sigma_i)
$$

여기서:
- $\zeta = [\xi^\top, \tau^\top]^\top \in \mathbb{R}^{n_\xi + n_\tau}$: 입출력을 합친 결합변수
  - $\xi = [q, \dot{q}, \ddot{q}]^\top \in \mathbb{R}^{n}$: 입력 (위치, 속도, 가속도)
  - $\tau \in \mathbb{R}^{n}$: 출력 (토크)
- $\pi_i > 0$: i번째 성분의 가중치, $\sum_i \pi_i = 1$
- $\mu_i = [\mu_i^{\xi\top}, \mu_i^{\tau\top}]^\top$: i번째 성분의 평균
- $\Sigma_i = \begin{bmatrix} \Sigma_i^{\xi\xi} & \Sigma_i^{\xi\tau} \\ \Sigma_i^{\tau\xi} & \Sigma_i^{\tau\tau} \end{bmatrix}$: i번째 성분의 공분산

### 7.2.2 EM 알고리즘 직관

실제로 GMM을 훈련하려면 **Expectation-Maximization (EM) 알고리즘**을 사용한다. 이 과정은:

1. **E-step**: 각 데이터 점이 각 가우시안에 어느 정도 "속하는지" 계산
2. **M-step**: 계산된 소속도에 따라 $\pi_i, \mu_i, \Sigma_i$ 업데이트
3. 수렴할 때까지 반복

이 교재에서는 GMM이 이미 훈련된 상태를 가정하고, 매개변수 $\pi_i, \mu_i, \Sigma_i$를 알고 있다고 시작한다.

---

## 7.3 GMR: 조건부 기댓값

### 7.3.1 조건부 가우시안 분포

주어진 입력 $\xi$에 대해, i번째 성분의 조건부 분포는

$$
p(\tau | \xi, \text{component } i) = \mathcal{N}(\tau; \tilde{\tau}_i(\xi), \Sigma_i^{\tau\tau|\xi})
$$

여기서 조건부 평균(conditional mean)은

$$
\tilde{\tau}_i(\xi) = \mu_i^{\tau} + \Sigma_i^{\tau\xi} (\Sigma_i^{\xi\xi})^{-1} (\xi - \mu_i^{\xi})
$$

이를 정리하면 **선형 형태**가 나온다:

$$
\tilde{\tau}_i(\xi) = A_i \xi + B_i
$$

여기서:

$$
A_i = \Sigma_i^{\tau\xi} (\Sigma_i^{\xi\xi})^{-1}, \quad B_i = \mu_i^{\tau} - A_i \mu_i^{\xi}
$$

이것이 **i번째 국소 아핀 모델(local affine model)**이다.

### 7.3.2 전체 GMR 예측

조건부 기댓값 원리에 의해, 주어진 $\xi$에 대한 $\tau$의 최적 예측은

$$
\hat{\tau}(\xi) = E[\tau | \xi] = \sum_{i=1}^{C} P(\text{component } i | \xi) \cdot \tilde{\tau}_i(\xi)
$$

여기서 $P(\text{component } i | \xi)$는 **사후확률(posterior probability)**이다:

$$
h_i(\xi) = \frac{\pi_i \mathcal{N}(\xi; \mu_i^{\xi}, \Sigma_i^{\xi\xi})}{\sum_{j=1}^{C} \pi_j \mathcal{N}(\xi; \mu_j^{\xi}, \Sigma_j^{\xi\xi})}
$$

따라서

$$
\hat{\tau}(\xi) = \sum_{i=1}^{C} h_i(\xi) (A_i \xi + B_i)
$$

### 7.3.3 소속 함수의 성질

GMR 소속 함수 $h_i(\xi)$는 **기본적으로** 다음 세 성질을 만족한다:

**성질 1: Normalization (1의 분할)**
$$
\sum_{i=1}^{C} h_i(\xi) = 1 \quad \forall \xi
$$

증명: 분자를 모두 더하면 분모가 되므로 자명. $\square$

**성질 2: Non-negativity (비음성)**
$$
h_i(\xi) \geq 0 \quad \forall \xi, \forall i
$$

증명: 가우시안 함수는 항상 양수이고, 모든 계수 $\pi_i > 0$이므로 자명. $\square$

**성질 3: $C^\infty$ 매끄러움**

가우시안 함수의 지수 형태 때문에, $h_i(\xi)$는 무한번 미분가능하다. 특히 **1차 도함수가 존재하고 유계**이다:

$$
\left| \frac{\partial h_i}{\partial \xi_k} \right| \leq 2 \left\| (\Sigma_i^{\xi\xi})^{-1} \right\| \cdot \left\| \xi - \mu_i^{\xi} \right\| \cdot h_i(\xi)
$$

---

## 7.4 TS Fuzzy 등가의 증명

### 7.4.1 TS Fuzzy 시스템의 정의 (복습)

TS fuzzy 시스템은 다음 구조를 가진 시스템이다:

$$
\dot{x} = \sum_{i=1}^{C} h_i(z(t)) [A_i x + B_i u + E_i w]
$$

여기서:
- $h_i(z)$는 소속 함수 (위의 성질 1, 2를 만족)
- $z(t)$는 전제변수(premise variable)

TS fuzzy 시스템의 안정성이 보장되려면, $h_i$는 다음을 만족해야 한다:

1. $\sum_i h_i = 1$ (정규화)
2. $h_i \geq 0$ (비음성)
3. 구간별로 연속이고, 가능하면 매끄러움

### 7.4.2 GMR은 구성적으로 TS Fuzzy이다

**정리 7.1 (GMR ↔ TS Fuzzy 등가성).**

GMM에서 EM으로 훈련한 GMR 예측기

$$
\hat{\tau}(\xi) = \sum_{i=1}^{C} h_i(\xi) (A_i \xi + B_i)
$$

는 **구성적으로(by construction)** TS fuzzy 시스템이다. 즉, GMR의 소속 함수 $h_i(\xi)$는 TS fuzzy의 모든 요구사항을 자동으로 만족한다.

**증명.**

7.3.3에서 보인 대로:

1. **Normalization**: $\sum_i h_i(\xi) = 1$ 자명 (GMR의 사후확률 정의)
2. **Non-negativity**: $h_i(\xi) \geq 0$ 자명 (가우시안 양수, $\pi_i > 0$)
3. **$C^\infty$ 매끄러움**: 가우시안 지수 형태 (미분가능)

따라서 $\hat{\tau}(\xi) = \sum_i h_i(\xi) f_i(\xi)$ (단, $f_i(\xi) = A_i \xi + B_i$)는 TS fuzzy 구조의 모든 정의를 만족한다. $\square$

### 7.4.3 핵심: 근사가 아닌 정확한 등가성

이것은 **근사(approximation)가 아니다**. 그 이유:

- **근사**: "GMR을 TS fuzzy로 나타낼 수 있다" (GMR을 어떤 다른 방식으로 표현)
- **등가성**: "GMR의 수학적 정의가 정확히 TS fuzzy이다" (동일한 식)

GMR은 조건부 기댓값의 성질로부터 $\tau = \sum h_i(A_i\xi + B_i)$ 형태를 얻는다. 이것이 TS fuzzy의 정의 그 자체이다. 즉, 수학적으로 같은 구조를 두 가지 이름으로 부른 것이다.

---

## 7.5 $A_i$의 물리적 해석

### 7.5.1 입력 변수 분할

입력을 위치, 속도, 가속도로 분할하면:

$$
\xi = \begin{bmatrix} q \\ \dot{q} \\ \ddot{q} \end{bmatrix}
$$

따라서 $A_i$도 대응하는 부분행렬로 나뉜다:

$$
A_i [\xi] = \begin{bmatrix} A_i^{11} & A_i^{12} & A_i^{13} \end{bmatrix} \begin{bmatrix} q \\ \dot{q} \\ \ddot{q} \end{bmatrix}
$$

### 7.5.2 각 부분행렬의 물리적 의미

**$A_i^{11}$ (위치 의존 항)**

$$
A_i^{11} q
$$

는 위치에 비례하는 토크이다. 로봇에서 이는 주로:
- 중력 보상: $g(q)$
- 구성에 따른 스프링 효과 (configuration-dependent stiffness)

따라서 **$A_i^{11}$는 모델 기반의 강성(stiffness) 항**이다.

**$A_i^{12}$ (속도 의존 항)**

$$
A_i^{12} \dot{q}
$$

는 속도에 비례하는 토크이다. 로봇에서 이는:
- 코리올리/원심 항: $C(q, \dot{q}) \dot{q}$
- 마찰 (friction)

따라서 **$A_i^{12}$는 모델 기반의 감쇠(damping) 항**이다.

**$A_i^{13}$ (가속도 의존 항)**

$$
A_i^{13} \ddot{q}
$$

는 가속도에 비례하는 토크이다. 이는 **관성 변형(inertia shaping)**이다:

$$
M(q) \ddot{q} + A_i^{13} \ddot{q} = [M(q) + A_i^{13}] \ddot{q}
$$

즉, 실제 동역학에서 관성 행렬이 $M(q) \to M(q) + A_i^{13}$으로 조정된다.

이것이 **8장의 DOB의 핵심 기여**이다: DOB는 이 $A_i^{13}$를 동역학에 추가하여 제어성(controllability)을 향상시킨다.

---

## 7.6 가우시안 소속 함수의 도함수 유계

### 7.6.1 도함수의 상한

시간에 따라 변하는 $\xi(t)$에 대해, i번째 소속 함수의 시간 도함수는

$$
\dot{h}_i(\xi(t)) = \sum_{k} \frac{\partial h_i}{\partial \xi_k} \dot{\xi}_k
$$

각 편미분은 다음과 같이 유계된다:

$$
\left| \frac{\partial h_i}{\partial \xi_k} \right| \leq 2 \left\| (\Sigma_i^{\xi\xi})^{-1} \right\| \cdot R_\xi
$$

여기서 $R_\xi = \max_{\xi \in \mathcal{R}} \|\xi - \mu_i^{\xi}\|$는 작동 영역의 반경이다.

### 7.6.2 통합 상한

$V_{\max} = \sup_t \|\dot{\xi}(t)\|$를 최대 입력 변화율이라 하면:

$$
|\dot{h}_i(\xi(t))| \leq 2 \left\| (\Sigma_i^{\xi\xi})^{-1} \right\| \cdot R_\xi \cdot V_{\max}
$$

모든 i에 대한 통합 상한은:

$$
\bar{h}_{\dot{}} = 2 \max_{k} \left\| (\Sigma_k^{\xi\xi})^{-1} \right\| \cdot R_\xi \cdot V_{\max}
$$

### 7.6.3 이것이 중요한 이유

**일반적인 TS fuzzy의 경우**: $\bar{h}_{\dot{}}$를 **가정**해야 한다. 즉, "소속 함수의 도함수가 이 정도로 유계라고 가정하자"라고 미리 정하고 증명을 진행한다.

**GMR의 경우**: $\bar{h}_{\dot{}}$를 **계산**할 수 있다. 훈련된 GMM의 공분산 $\Sigma_i^{\xi\xi}$와 작동 영역 규격 $R_\xi, V_{\max}$로부터 명시적으로 값을 구할 수 있다.

또한, 공분산이 크면 (넓은 가우시안):

$$
\Sigma_i^{\xi\xi} \text{가 크다} \Rightarrow (\Sigma_i^{\xi\xi})^{-1} \text{가 작다} \Rightarrow \bar{h}_{\dot{}} \text{가 작다}
$$

즉, **더 넓은 가우시안을 가질수록, 소속 함수가 천천히 변하므로 안정성 마진이 커진다**. 이것도 8장에서 중요한 설계 지표가 된다.

---

## 7.7 핵심 등식과 그 의미

### 7.7.1 동등성 선언

$$
\boxed{f_{\text{PPLM}}(\xi) \equiv \hat{\tau}(\xi) = \sum_{i=1}^{C} h_i(\xi) (A_i \xi + B_i)}
$$

여기서 $\equiv$는 "수학적으로 정확히 같다"를 의미한다.

### 7.7.2 두 가지 해석

**기계학습 해석**: "GMM을 훈련했더니 각 성분이 선형 회귀 모델이고, 가중평균으로 예측을 했다"

**제어이론 해석**: "TS fuzzy 시스템이고, 7장에서 배운 안정성 분석 기법 (LMI, Lyapunov)을 그대로 적용할 수 있다"

### 7.7.3 8장으로 가는 다리

이 등가성이 중요한 이유는, DOB의 폐루프 동역학에서:

$$
M(q) \ddot{e} = -\sum_i h_i(\xi) [A_i^{11} e + A_i^{12} \dot{e} + A_i^{13} \ddot{e}] - \hat{K}_p e - \hat{K}_d \dot{e} + \text{disturbance}
$$

우변의 $\sum h_i (A_i \xi + B_i)$ 구조가 정확히 TS fuzzy의 형태이므로, **Part II에서 배운 TS fuzzy 안정성 기법 (fuzzy Lyapunov 함수, LMI)을 직접 적용**할 수 있다는 것이다.

---

## 7.8 예제

### 예제 7.1: 1D GMM으로 중력 학습

**문제**: 1-자유도 로봇 팔의 중력을 학습하려고 한다. 동역학은

$$
\tau = m \ell g \sin(q)
$$

100개의 (위치, 가속도, 토크) 데이터를 수집하여 2-성분 GMM으로 훈련했다.

훈련 결과:
- 성분 1: $\mu_1^q = -\pi/3$, $\sigma_1^{qq} = 0.25$, $A_1 = 2.5$, $B_1 = -1.8$
- 성분 2: $\mu_2^q = \pi/3$, $\sigma_2^{qq} = 0.25$, $A_2 = 1.2$, $B_2 = 1.5$
- $\pi_1 = 0.4, \pi_2 = 0.6$

**검증**: TS 구조 확인.

주어진 $q = 0$에서 소속 함수:

$$
h_1(0) = \frac{0.4 \mathcal{N}(0; -\pi/3, 0.25)}{0.4 \mathcal{N}(0; -\pi/3, 0.25) + 0.6 \mathcal{N}(0; \pi/3, 0.25)}
$$

수치 계산: $h_1(0) \approx 0.35, h_2(0) \approx 0.65$

$\sum h_i = 0.35 + 0.65 = 1.0$ ✓

예측:

$$
\hat{\tau}(0) = 0.35 \times (2.5 \times 0 + (-1.8)) + 0.65 \times (1.2 \times 0 + 1.5)
            = -0.63 + 0.975 = 0.345
$$

정확한 중력: $mg\sin(0) = 0$ (오차 있음, 이는 학습 데이터 노이즈)

**구조 확인**: $\hat{\tau} = h_1(A_1 q + B_1) + h_2(A_2 q + B_2)$는 정확히 TS fuzzy 형태.

### 예제 7.2: 2-DOF 평면 로봇의 $A_i$ 분할

**설정**: 2-자유도 평면 로봇, 입력 $\xi = [q_1, q_2, \dot{q}_1, \dot{q}_2, \ddot{q}_1, \ddot{q}_2]^\top$ (6차원)

3-성분 GMM 훈련 결과, 각 성분의 회귀 행렬 $A_i \in \mathbb{R}^{2 \times 6}$:

$$
A_i = \begin{bmatrix}
A_i^{11,1} & A_i^{11,2} & A_i^{12,1} & A_i^{12,2} & A_i^{13,1} & A_i^{13,2} \\
A_i^{21,1} & A_i^{21,2} & A_i^{22,1} & A_i^{22,2} & A_i^{23,1} & A_i^{23,2}
\end{bmatrix}
$$

**분할**:
- $A_i^{11} \in \mathbb{R}^{2 \times 2}$ (위치에서 위치로)
- $A_i^{12} \in \mathbb{R}^{2 \times 2}$ (속도에서 위치로)
- $A_i^{13} \in \mathbb{R}^{2 \times 2}$ (가속도에서 위치로)

**물리적 해석**: 성분 1 (팔이 위쪽 구성에 있을 때):

$$
\begin{bmatrix} A_1^{11,1} \\ A_1^{21,1} \end{bmatrix} = \begin{bmatrix} 3.2 \\ 0.5 \end{bmatrix} \Rightarrow
\text{관절 1에 강한 중력 토크, 관절 2에 약한 coupled 토크}
$$

---

## 7.9 실습 코드

### 7.9.1 Python: GMM 훈련 및 $A_i, B_i$ 추출

```python
import numpy as np
from sklearn.mixture import GaussianMixture
import matplotlib.pyplot as plt

# 합성 데이터: 중력 모델 f(q) = mg*sin(q) + noise
np.random.seed(42)
q = np.linspace(-np.pi, np.pi, 100)
q_dot = np.linspace(-2, 2, 100)
q_ddot = np.linspace(-3, 3, 100)

# 역동역학: 토크 = 중력 + 코리올리
m, l, g = 1.0, 0.5, 9.81
tau = m * l * g * np.sin(q) + 0.2 * q * q_dot + np.random.normal(0, 0.05, 100)

# 결합 변수 ζ = [q, q_dot, q_ddot, tau]
zeta = np.column_stack([q, q_dot, q_ddot, tau])

# GMM 훈련 (3-성분)
C = 3
gmm = GaussianMixture(n_components=C, random_state=42, n_init=10)
gmm.fit(zeta)

print("=== GMM Parameters ===")
print(f"Weights (π_i): {gmm.weights_}")
print(f"Means shape: {gmm.means_.shape}")
print(f"Covariances shape: {gmm.covariances_.shape}")

# A_i, B_i 추출 함수
def extract_regression_matrices(gmm):
    """
    GMM에서 조건부 회귀 행렬 A_i, B_i 추출
    입력: q, q_dot, q_ddot (변수 0-2)
    출력: tau (변수 3)
    """
    C = gmm.n_components
    n_in, n_out = 3, 1
    
    A_list, B_list = [], []
    
    for i in range(C):
        mu_i = gmm.means_[i]
        Sigma_i = gmm.covariances_[i]
        
        # Partition: ξ = [q, q_dot, q_ddot], τ = [tau]
        mu_xi = mu_i[:n_in]
        mu_tau = mu_i[n_in:]
        
        Sigma_xixi = Sigma_i[:n_in, :n_in]
        Sigma_xitau = Sigma_i[:n_in, n_in:]
        Sigma_tauxi = Sigma_i[n_in:, :n_in]
        
        # A_i = Σ^{τξ} (Σ^{ξξ})^{-1}
        A_i = Sigma_tauxi @ np.linalg.inv(Sigma_xixi)
        # B_i = μ^τ - A_i μ^ξ
        B_i = mu_tau - A_i @ mu_xi
        
        A_list.append(A_i)
        B_list.append(B_i)
    
    return A_list, B_list

A_matrices, B_matrices = extract_regression_matrices(gmm)

print("\n=== Regression Matrices ===")
for i in range(C):
    print(f"Component {i}:")
    print(f"  A_{i} = {A_matrices[i]}")
    print(f"  B_{i} = {B_matrices[i]}")

# 소속 함수 정의
def membership_function(xi, gmm):
    """
    주어진 입력 ξ에서 각 성분의 소속도 계산
    """
    C = gmm.n_components
    zeta_xi = np.column_stack([xi] + [np.zeros(len(xi[0]))] * (zeta.shape[1] - len(xi)))
    # 좀 더 간단한 방법: 입력만으로 소속도 계산
    
    # 실제로는 입력 공분산만 사용
    h = np.zeros((C, len(xi[0])))
    for j in range(C):
        mu_j = gmm.means_[j, :3]  # 입력 부분만
        sigma_j = gmm.covariances_[j, :3, :3]
        
        # Multivariate Gaussian PDF
        diff = xi.T - mu_j
        exponent = -0.5 * np.sum(diff * (np.linalg.inv(sigma_j) @ diff), axis=0)
        det_sigma = np.linalg.det(sigma_j)
        
        numerator = gmm.weights_[j] * np.exp(exponent) / np.sqrt((2*np.pi)**3 * det_sigma)
        h[j, :] = numerator
    
    # Normalization
    h = h / (h.sum(axis=0) + 1e-10)
    return h

# 테스트: q=0에서 소속도
xi_test = np.array([[0.0, 0.0, 0.0]])
h_test = membership_function(xi_test.T, gmm)
print(f"\nMembership at ξ=[0, 0, 0]: {h_test[:, 0]}")
print(f"Sum = {h_test[:, 0].sum():.6f} (should be 1.0)")

# GMR 예측
def gmr_prediction(xi, gmm, A_matrices, B_matrices):
    """
    주어진 입력 ξ에 대해 GMR 출력 예측
    \hat{\tau} = Σ h_i (A_i ξ + B_i)
    """
    h = membership_function(xi, gmm)
    tau_pred = np.zeros(len(xi[0]))
    
    for i in range(gmm.n_components):
        tau_pred += h[i, :] * (A_matrices[i] @ xi + B_matrices[i])
    
    return tau_pred

# 테스트 데이터로 검증
q_test = np.linspace(-np.pi, np.pi, 50)
q_dot_test = np.zeros_like(q_test)
q_ddot_test = np.zeros_like(q_test)
xi_test = np.array([q_test, q_dot_test, q_ddot_test])

tau_true = m * l * g * np.sin(q_test)
tau_gmr = gmr_prediction(xi_test, gmm, A_matrices, B_matrices)

plt.figure(figsize=(10, 6))
plt.plot(q_test, tau_true, 'b-', label='True (mg sin(q))', linewidth=2)
plt.plot(q_test, tau_gmr.flatten(), 'r--', label='GMR prediction', linewidth=2)
plt.xlabel('Position q [rad]')
plt.ylabel('Torque τ [N⋅m]')
plt.legend()
plt.grid(True, alpha=0.3)
plt.savefig('ch7_gmr_prediction.png', dpi=150)
plt.show()

print(f"\nMean squared error: {np.mean((tau_true - tau_gmr.flatten())**2):.6f}")
```

### 7.9.2 소속 함수 표면 시각화

```python
# 2D mesh에서 소속 함수 시각화 (q, q_dot 평면)
q_mesh = np.linspace(-np.pi, np.pi, 50)
q_dot_mesh = np.linspace(-2, 2, 50)
Q, Q_DOT = np.meshgrid(q_mesh, q_dot_mesh)

H = np.zeros((gmm.n_components, 50, 50))
for i in range(gmm.n_components):
    for idx_q in range(50):
        for idx_qdot in range(50):
            xi = np.array([Q[idx_qdot, idx_q], Q_DOT[idx_qdot, idx_q], 0.0]).reshape(-1, 1)
            h_val = membership_function(xi, gmm)
            H[i, idx_qdot, idx_q] = h_val[i, 0]

fig, axes = plt.subplots(1, 3, figsize=(15, 5))
for i in range(3):
    ax = axes[i]
    contour = ax.contourf(Q, Q_DOT, H[i], levels=20, cmap='viridis')
    ax.set_xlabel('Position q [rad]')
    ax.set_ylabel('Velocity q_dot [rad/s]')
    ax.set_title(f'Membership h_{i+1}(q, q_dot)')
    plt.colorbar(contour, ax=ax)

plt.tight_layout()
plt.savefig('ch7_membership_surfaces.png', dpi=150)
plt.show()
```

### 7.9.3 $\bar{h}_{\dot{}}$ (소속 함수 도함수 상한) 계산

```python
def compute_membership_derivative_bound(gmm, R_xi, V_max):
    """
    소속 함수 도함수 상한 계산:
    |ḣ_i| ≤ 2 max_k ||(Σ_k^{ξξ})^{-1}|| · R_ξ · V_max
    """
    inv_norms = []
    
    for i in range(gmm.n_components):
        Sigma_xixi = gmm.covariances_[i, :3, :3]  # 입력 공분산
        Sigma_inv = np.linalg.inv(Sigma_xixi)
        norm = np.linalg.norm(Sigma_inv)
        inv_norms.append(norm)
    
    max_inv_norm = max(inv_norms)
    h_dot_bound = 2 * max_inv_norm * R_xi * V_max
    
    return h_dot_bound, inv_norms

# 예: 작동 영역 반경 R_ξ = 1.0, 최대 입력 변화율 V_max = 1.0
R_xi = 1.0
V_max = 1.0

h_dot_bound, inv_norms = compute_membership_derivative_bound(gmm, R_xi, V_max)

print(f"\n=== Membership Function Derivative Bound ===")
print(f"Operating region radius R_ξ = {R_xi}")
print(f"Max input rate V_max = {V_max}")
for i in range(gmm.n_components):
    print(f"  ||(Σ_{i}^{{ξξ}})^{{-1}}|| = {inv_norms[i]:.4f}")
print(f"\nBound: ≤ {h_dot_bound:.4f}")
print(f"(Compare with 1/C = {1/gmm.n_components:.4f} for stability margin)")
```

---

## 7.10 복습 문제

1. **정규화 확인**: 3-성분 GMM에서, $\sum_{i=1}^3 h_i(\xi)$ 왜 항상 1인가? 분모와 분자 관계로 설명하라.

2. **비음성**: 가우시안 함수 $\mathcal{N}(\xi; \mu_i, \Sigma_i)$가 모든 $\xi$에 대해 양수인 이유는?

3. **회귀 행렬 유도**: $\tilde{\tau}_i(\xi) = \mu_i^{\tau} + \Sigma_i^{\tau\xi} (\Sigma_i^{\xi\xi})^{-1} (\xi - \mu_i^{\xi})$에서, 이것이 왜 "$A_i\xi + B_i$" 형태가 되는지 전개해 보라.

4. **$A_i^{13}$의 의미**: 예제 7.2에서, $A_i^{13}$이 크다는 것은 물리적으로 무엇을 의미하는가? 이것이 8장의 DOB 안정성 분석에 어떻게 영향을 미치는가?

---

## 7.11 핵심 요약

> **GMM에서 훈련한 GMR 예측기는, 구성적으로 정확히 TS fuzzy 시스템이다.** 이것은 근사가 아니라 수학적 항등식이다.
>
> GMR의 소속 함수 $h_i(\xi)$는 가우시안 사후확률로서:
> 1. $\sum h_i = 1$ (정규화)
> 2. $h_i \geq 0$ (비음성)
> 3. $C^\infty$ 매끄럼
>
> 이 세 성질은 TS fuzzy 안정성 분석이 요구하는 모든 조건이다. 따라서 **Part II에서 배운 LMI, fuzzy Lyapunov 함수, Tanaka-Wang 완화 기법을 그대로 적용**할 수 있다.
>
> 또한 $A_i$의 분할을 통해 GMR이 제공하는 세 가지 동역학 항 ($A_i^{11}$: 강성, $A_i^{12}$: 감쇠, $A_i^{13}$: 관성)을 물리적으로 이해할 수 있다. $A_i^{13}$은 8장의 DOB가 동역학에 추가하는 inertia shaping 항이다.
>
> 이제 이 TS fuzzy 구조를 **TDoF 제어 루프에 넣으면?** → 8장에서 DOB 피드백이 폐루프를 어떻게 안정화시키는지 보게 된다.

