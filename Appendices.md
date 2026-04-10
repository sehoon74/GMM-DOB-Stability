# 부록

---

## 부록 A. 세 가지 접근법 비교

이 교재는 최종 정정된 FLF + Descriptor 방법으로 검증되었으나, 개발 과정에서 세 가지 다른 접근법을 시도했다. 각각의 강점과 약점을 이해하면 안정성 분석의 맥락을 더 깊이 있게 파악할 수 있다.

### A.1 세 가지 접근법의 개요

| 측면 | 에너지 기반 (초안1) | CQLF (초안2) | FLF + Descriptor (최종) |
|---|---|---|---|
| **Lyapunov 함수** | $V = \frac{1}{2}\dot{e}^T M \dot{e} + \frac{1}{2}e^T K_p e$ | $V = x^T P x$ (단일 P) | $V = \eta_*^T E_*^T (\Sigma h_i P_i) \eta_*$ |
| **TS fuzzy 구조** | 증명에 미사용 | CQLF로 사용하지만 보수적 | 완전히 활용 (subsystem마다 $P_i$) |
| **Descriptor 형식** | 언급되었으나 미사용 | 없음 | 적분, DAE 대수 루프 처리 |
| **DOB 구조** | 손실됨 ($(1-Q)\tilde{f}$ 잔여만 남음) | 손실됨 (norm-bounded $\Delta_i$로 축약) | 보존됨 ($R_i, S_i$ 분해) |
| **보수성 정도** | 높음 (모델 구조 무시) | 중간 (CQLF가 과도하게 보수적) | 낮음 (Tanaka-Wang 이완) |
| **UUB 한계** | 모호한 $O(\bar{d})$ | 명시적이지만 부정확 | 명시적이고 정확 $r = \sqrt{\bar{\lambda}\beta/(\underline{\lambda}\alpha)}$ |
| **검증 가능성** | 아니오 (K에 대한 조건만) | 부분적 (LMI이지만 단일 P) | 예 (SDP, $C(C+1)/2$ LMI) |

### A.2 에너지 기반 접근법 (초안1)

**아이디어**: 로봇의 물리적 에너지를 직접 사용.

**Lyapunov 후보**:
$$
V = \frac{1}{2}\dot{e}^T M(q) \dot{e} + \frac{1}{2}e^T K_p e
$$

**강점**:
- 물리적 직관: 운동에너지 + 위치에너지 (스프링)
- 기하학적: 각 항의 의미가 명확

**약점**:
1. **TS fuzzy 구조 미활용**: Σ $h_i(ξ)$를 전혀 사용하지 않음 → 각 로컬 모델 $A_i$의 강점을 못 씀
2. **DOB 구조 손실**: 오직 $(1-Q(s))\tilde{f}(ξ)$만 잔여에 포함, 즉 DOB의 구체적 구조 불가시적
3. **Descriptor 시스템 미사용**: $A_i^{13}$으로 인한 대수 루프 처리 미흡
4. **UUB 한계 모호**: $\dot{V} \leq -\alpha_1 ||\dot{e}||^2 - \alpha_2 ||e||^2 + \beta \bar{d}^2$ 형태에서 정규화 부족 → $r$의 명시 공식 도출 어려움
5. **검증 불가**: LMI 없음, 오직 게인 선택 규칙만 존재

### A.3 CQLF 접근법 (초안2)

**아이디어**: 일반적인 TS fuzzy 시스템 분석. 모든 subsystem을 하나의 공통 Lyapunov 행렬 P로 처리.

**Lyapunov 함수**:
$$
V = x^T P x, \quad P \succ 0
$$

**시스템**:
$$
\dot{x} = \Sigma h_i (A_i + \Delta_i)x + d, \quad ||\Delta_i|| \leq \rho
$$

**LMI 조건** (Schur complement):
$$
\begin{bmatrix}
A_i^T P + PA_i + \alpha P & P \\
P & -\rho I
\end{bmatrix} \prec 0 \quad \forall i
$$

**강점**:
- 깔끔한 LMI 형식: SDP 솔버로 직접 풀 수 있음
- 모든 subsystem이 하나의 P를 만족 → 명확한 문제 정의

**약점**:
1. **CQLF 보수성**: 단일 P가 모든 $i$에 대해 $A_i^T P + PA_i \prec 0$을 만족해야 함 → 매우 제한적. 많은 시스템에서 LMI 해가 존재하지 않음 또는 매우 작은 $\alpha$만 가능
2. **Descriptor 시스템 부재**: $A_i^{13}$의 대수 루프 처리 못 함
3. **DOB 구조 축약**: 모든 불확실성을 norm-bounded $\Delta_i$로 매핑 → 실제 DOB 피드백 구조 (`$R_i, S_i` 분해`) 불가시적
4. **$\rho$ 정의 모호**: $\rho = c_1 + c_2 + \varepsilon_Q L_f$와 같은 형식은 DOB 대역폭 $L$과의 관계를 명확히 하지 못함
5. **수학적 오류**: Error Catalog의 Error 2 참조. Young 부등식 적용에서 $\bar{p}^2$이 양쪽에 나타나는 오류 가능

### A.4 FLF + Descriptor 접근법 (최종, 정정)

**아이디어**: 각 Gaussian 성분마다 독립적인 Lyapunov 행렬 $P_i$ 사용 + Descriptor 시스템으로 대수 루프 해결.

**Lyapunov 함수**:
$$
V(\eta_*) = \eta_*^T E_*^T \left(\sum_{i=1}^C h_i(ξ) P_i\right) \eta_*
$$

**Descriptor 시스템**:
$$
E_* \dot{\eta}_* = \sum_{i=1}^C h_i A_i \eta_* + T_*, \quad E_* = \text{blkdiag}(I, 0)
$$

**강점**:
1. **FLF 활용**: 각 $P_i$는 $i$번째 Gaussian 근처에서만 사용되므로 CQLF 보수성 회피
2. **Descriptor 구조**: $E_*$가 특이행렬이지만 정규성 조건 하에서 $\eta_*$는 선택적 차원에서만 자유. 대수 루프 자동 처리
3. **DOB 명시**: $R_i = [I, 0; 0, M+A_i^{13}]$, $S_i = [0, I; -(A_i^{11}+\hat{M}K_p), -(A_i^{12}+\hat{M}K_d)]$ 형태로 DOB 기여 분해 가능
4. **Tanaka-Wang 이완**: 교차 항 $A_i^T P_j + P_j^T A_i + A_j^T P_i + P_i^T A_j \preceq 0$ 추가로 추가 이완
5. **명시적 UUB 한계**: $r = \sqrt{\bar{\lambda} \beta / (\underline{\lambda} \alpha)}$ 공식 유도 가능
6. **LMI 검증**: $C(C+1)/2$개 조건 × 최대 C개 매개변수 집합 → 수치 해석기로 풀 수 있음

**복잡성**: 의사 결정 변수 많음 ($P_{11,i}, P_{21,i}, P_{22,i}$ per $i$), 4n×4n 크기.

### A.5 세 가지의 정성적 흐름도

```
에너지기반 ──→ 물리직관이지만 미통합
                 └─ 증명이 부정확 (모델구조무시)

CQLF ──→ 깔끔한 LMI이지만 과도보수
           └─ 많은 시스템에서 불가능

FLF+Descriptor ──→ 정확하고 통합적이지만 계산복잡
                    └─ SDP솔버 필수, 검증 가능
```

---

## 부록 B. 흔한 오류 카탈로그

다음 8가지 오류는 TS fuzzy descriptor 시스템의 안정성 증명에서 자주 발생한다. 각각에 대해 오류 형태, 정정 형태, 심각도, 근본 원인을 정리했다.

| # | 오류 | 정정 | 심각도 | 근본 원인 |
|---|---|---|---|---|
| **1** | $\alpha_1 = \min_i \lambda_{\min}(Q_i)$ | $\alpha_1 = (1/C) \min_i \lambda_{\min}(Q_i)$ | 🔴 높음 | Cauchy-Schwarz $\Sigma h_i^2 \geq 1/C$ 누락 |
| **2** | $\alpha_2 = \varepsilon \bar{p}^2$ AND $\beta_2 = \bar{p}^2\Delta^2/\varepsilon$ | $\alpha_2 = \varepsilon \bar{p}^2$, $\beta_2 = \Delta^2/\varepsilon$ | 🟡 중간 | Young 부등식 적용에서 $\bar{p}$이 $a = \bar{p}\|\eta\|$ 에만 흡수 |
| **3** | $\tau_{FB} = +\hat{M}K_p e + \hat{M}K_d \dot{e}$ | $\tau_{FB} = -\hat{M}K_p e - \hat{M}K_d \dot{e}$ | 🟢 낮음 | $e = q - q^d$ 이므로 음의 피드백이어야 함 |
| **4** | regularity: $\|\|M^{-1}A_i^{13}\|\| < 1$ | regularity: $\|\|I - M^{-1}A_i^{13}\|\| < 1$ | 🟢 낮음 | $A_i^{13} \approx M$ 일 때 Neumann 급수 정확히 적용 |
| **5** | $\|\dot{h}_i\| \leq f(\|\|\dot{\xi}(t)\|\|)$ (시변) | $\|\dot{h}_i\| \leq \bar{\dot{h}} = 2\max_k \|\|(\Sigma_k^{\xi\xi})^{-1}\|\| \cdot R_\xi \cdot V_{max}$ | 🟢 낮음 | 시간불변 한계 필요 (Lyapunov 정리 적용 위해) |
| **6** | $L_2 = 2\eta^T P T$ (P 대칭처럼) | $L_2 = T^T P \eta + \eta^T P^T T = 2\eta^T P^T T$ | 🟢 낮음 | Descriptor Lyapunov 행렬 $P_i$ 비대칭 |
| **7** | "$\dot{V} \leq -\alpha\|\|\eta\|\|^2 + \beta$ ⟹ UUB" (건너뜀) | $\dot{V} \leq -(\alpha/\bar{\lambda})V + \beta$ → 비교 보조정리 → $\limsup V \leq \bar{\lambda}\beta/\alpha$ | 🟡 중간 | 비교 보조정리 명시 누락 |
| **8** | "$\eta$ 유계 ⟹ $\eta_*$ 유계" (정당화 없음) | Descriptor 제약 $\Sigma h_i(S_i\eta - R_i\dot{\eta}) = -T_{*,lower}$ + 정규성 ⟹ $\dot{\eta} = (\Sigma h_i R_i)^{-1}[\Sigma h_i S_i \eta + T_{*,lower}]$, bounded $\eta, T \Rightarrow$ bounded $\dot{\eta} \Rightarrow$ bounded $\eta_*$ | 🟡 중간 | Descriptor 제약 다양체 상의 논증 생략 |

### B.1 각 오류의 상세 분석

#### **오류 1: 1/C 인수 누락** 🔴

**식**:
$$
(\Sigma h_i)^2 = 1 \leq C \Sigma h_i^2 \quad \Rightarrow \quad \Sigma h_i^2 \geq 1/C
$$

$\Sigma h_i h_j X_{ij}$에서 대각항만 추출할 때 $\Sigma h_i^2 X_{ii} \geq (1/C) X_{ii}$가 성립.

**영향**: $\alpha_1$을 $C$배 과대평가 → UUB 반경 $r$을 $\sqrt{C}$배 축소 → **안정성 여유 과다 평가**.

---

#### **오류 2: Young 부등식에서 $\bar{p}^2$ 중복** 🟡

**식**:
$$
2ab \leq \gamma a^2 + \frac{b^2}{\gamma}
$$

$a = \bar{p}\|\eta_*\|, b = \Delta$로 두면:
$$
2\bar{p}\|\eta_*\| \cdot \Delta \leq \varepsilon(\bar{p}\|\eta_*\|)^2 + \frac{\Delta^2}{\varepsilon} = \varepsilon \bar{p}^2\|\eta_*\|^2 + \frac{\Delta^2}{\varepsilon}
$$

**오류**: "$\alpha_2 = \varepsilon \bar{p}^2$ AND $\beta_2 = \bar{p}^2 \Delta^2/\varepsilon$" → 첫 항에만 $\bar{p}^2$ 나타남, 두 번째에는 $\bar{p}$가 없음.

**영향**: $r$ 저평가 (실제로는 더 크다).

---

#### **오류 3: τ_FB 부호** 🟢

**정의**: $e = q - q^d$ (추적 오차)

**반응**: PD 제어기는 오차를 **반대 부호**로 제거해야 함.
$$
\tau_{FB} = -\hat{M}(K_p e + K_d \dot{e})
$$

**오류**: 양수 부호 사용 → 피드백이 오히려 오차를 증폭 → 증명이 자기모순적이 될 수 있음.

**영향**: 논리적으로는 심각하나, 이후 식이 모두 부호를 잘못 유지하면 최종 결론은 여전히 맞을 수 있음 (저등급).

---

#### **오류 4: 정규성 판정** 🟢

**조건**: $\det(I + M^{-1}A_i^{13}) \neq 0$ (또는 equivalently, $\det(M + A_i^{13}) \neq 0$)

**Neumann 급수**: $\|\|M^{-1}A_i^{13}\|\| < 1$이면 $(I - M^{-1}A_i^{13})$이 가역.

**오류**: $A_i^{13} \approx M$일 때 $M^{-1}A_i^{13} \approx I$ → norm ≈ 1, NOT < 1.

**정확한 형식**: $(I - M^{-1}A_i^{13})$이 가역인지 확인. $A_i^{13} \approx M$이면 $I - M^{-1}A_i^{13} \approx 0$ → 문제. 대신 $\det(M + A_i^{13}) = \det(M(I + M^{-1}A_i^{13})) \approx \det(2M) \gg 0$ (큰 여유).

**영향**: 저등급 (정규성은 어차피 충분조건이고, 실제로는 $A_i^{13}$이 잘 학습되면 대부분 만족).

---

#### **오류 5: 시변 $\bar{\dot{h}}$ 한계** 🟢

**잘못된 형식**: $\|\dot{h}_i(ξ(t))\| \leq f(\|\dot{\xi}(t)\|)$ (시간에 따라 변함)

**올바른 형식**: 
$$
\|\dot{h}_i\| = \|\nabla h_i\|_\text{op} \cdot \|\dot{\xi}\| \leq \max_k \|(\Sigma_k^{\xi\xi})^{-1}\| \cdot R_\xi \cdot V_{\max}
$$

여기서:
- $R_\xi$ = 작동 영역 반경
- $V_{\max} = \sup \|\dot{\xi}\|$ = 명시된 제약

**영향**: 낮음. 시변 한계를 써도 적응 Lyapunov 이론으로 처리 가능하나, 표준 이론에서는 시간불변이 필요.

---

#### **오류 6: 비대칭 P와 L₂** 🟢

**Descriptor Lyapunov 행렬**:
$$
P_i = \begin{bmatrix} P_{11,i} & 0 \\ P_{21,i} & P_{22,i} \end{bmatrix}, \quad P_{11,i} = P_{11,i}^T, \quad P_{21,i}, P_{22,i} \text{ 자유}
$$

$P_i$ 자체는 비대칭. 따라서:
$$
L_2 = T_*^T P \eta_* + \eta_*^T P^T T_* \neq 2\eta_*^T P T_*
$$

**올바른 형식**: $T_*^T P \eta_* = (\eta_*^T P^T T_*)^T = \eta_*^T P^T T_*$ (스칼라는 전치해도 같음)
$$
L_2 = 2\eta_*^T P^T T_*
$$

**영향**: 낮음 (최종 bound는 같음, $\|P^T\| = \|P\|$이므로).

---

#### **오류 7: 비교 보조정리 누락** 🟡

**불완전한 증명**: "$\dot{V} \leq -\alpha\|\eta\|^2 + \beta$ ⟹ UUB"

**완전한 증명**:
$$
\dot{V} \leq -\frac{\alpha}{\bar{\lambda}_{11}} V + \beta
$$

**비교 보조정리 (Khalil)**: $\dot{u} \leq -cu + d$이면
$$
u(t) \leq u(0)e^{-ct} + \frac{d}{c}(1 - e^{-ct})
$$

따라서 $\limsup V \leq \bar{\lambda}_{11}\beta/\alpha$, 그러면 $\underline{\lambda}_{11}\|\eta\|^2 \leq V$로부터 $\|\eta\|$ 한계 도출.

**영향**: 중간. 논증이 끝나지 않음.

---

#### **오류 8: Descriptor 제약 다양체** 🟡

**문제**: Descriptor 시스템의 lower block:
$$
\Sigma h_i (S_i \eta - R_i \dot{\eta}) + T_{*,\text{lower}} = 0
$$

$\eta$와 $T_*$의 boundedness로부터 $\dot{\eta}$의 boundedness를 보장하려면?

**해법**: $\Sigma h_i R_i$의 가역성 (regularity) 사용:
$$
\dot{\eta} = \left(\Sigma h_i R_i\right)^{-1} \left[\Sigma h_i S_i \eta + T_{*,\text{lower}}\right]
$$

$\eta, T_*$ bounded ⟹ 우변 bounded ⟹ $\dot{\eta}$ bounded ⟹ $\eta_* = [\eta; \dot{\eta}]$ bounded.

**영향**: 중간. 생략하면 augmented state의 완전한 boundedness 증명이 불완전.

---

## 부록 C. 수학 도구 Quick Reference

### C.1 Cauchy-Schwarz: 멤버십에 적용

**명제**: 
$$
\left(\sum_{i=1}^C h_i\right)^2 \leq C \sum_{i=1}^C h_i^2
$$

**증명**: 
$$
1 = \left(\sum_{i=1}^C h_i \cdot 1\right)^2 \leq \left(\sum_{i=1}^C h_i^2\right) \left(\sum_{i=1}^C 1^2\right) = C\sum_{i=1}^C h_i^2
$$

**응용**: $\Sigma h_i h_j X_{ij} = \Sigma h_i^2 X_{ii} + \Sigma_{i \neq j} h_i h_j X_{ij}$ 경계.

---

### C.2 Young 부등식

**명제**: 임의 $a, b \in \mathbb{R}$와 $\gamma > 0$에 대해
$$
2ab \leq \gamma a^2 + \frac{b^2}{\gamma}
$$

**증명**: $(γ a - b/γ)^2 \geq 0$ 전개.

**응용**: $L_2$ 항 경계에서 $a = \bar{p}\|\eta\|, b = \Delta$.

---

### C.3 비교 보조정리 (Comparison Lemma)

**명제** (Khalil, Theorem 4.2): $u(t)$가 미분가능하고
$$
\dot{u}(t) \leq -c u(t) + d, \quad c, d > 0, \quad u(0) = u_0
$$
이면
$$
u(t) \leq e^{-ct} u_0 + \frac{d}{c}(1 - e^{-ct})
$$

따라서 $\limsup_{t \to \infty} u(t) \leq d/c$.

**증명**: $u(t) - d/c$의 Lyapunov 분석.

**응용**: $\dot{V} \leq -\alpha V + \beta$ → $\limsup V \leq \beta/\alpha$.

---

### C.4 Descriptor 대칭성 조건

**명제**: $E_* = \text{blkdiag}(I, 0)$이고 $E_*^T P_i = P_i^T E_*$이려면
$$
P_i = \begin{bmatrix} P_{11,i} & 0 \\ P_{21,i} & P_{22,i} \end{bmatrix}, \quad P_{11,i}^T = P_{11,i}
$$

**증명**: 
$$
\begin{bmatrix} I & 0 \\ 0 & 0 \end{bmatrix} \begin{bmatrix} P_{11,i} & P_{12,i}^T \\ P_{21,i} & P_{22,i} \end{bmatrix} = \begin{bmatrix} P_{11,i}^T & P_{21,i}^T \\ 0 & 0 \end{bmatrix}
$$

$(0,1)$ 성분: $P_{12,i} = 0$. $(1,0)$ 성분: $P_{11,i} = P_{11,i}^T$.

---

### C.5 Schur 여집합

**명제**:
$$
\begin{bmatrix} A & B \\ B^T & C \end{bmatrix} \succ 0 \quad \Longleftrightarrow \quad A \succ 0, \quad C - B^T A^{-1} B \succ 0
$$

(마지막 항을 $A$에 대한 **Schur 여집합**이라 함)

**응용**: LMI를 성분 부등식으로 변환.

---

### C.6 Grönwall 부등식

**명제**: $y(t)$가 연속이고
$$
y(t) \leq \alpha(t) + \int_0^t \beta(s) y(s) \, ds
$$
이면
$$
y(t) \leq \alpha(t) + \int_0^t \alpha(s) \beta(s) e^{\int_s^t \beta(\tau)d\tau} ds
$$

**특수 경우** (상수): $y(t) \leq c_0 + \int_0^t L y(s) ds$ ⟹ $y(t) \leq c_0 e^{Lt}$.

**응용**: $V(t) \leq V(0)e^{-\alpha t} + \ldots$ 유도.

---

## 부록 D. 검토 체크리스트

TS fuzzy descriptor 시스템의 안정성 증명을 작성하거나 검토할 때 다음 10가지 항목을 확인하시오. 모두 "예"이면 증명의 수학적 엄밀성이 높다.

### 안정성 증명 체크리스트

- [ ] **1. Fuzzy Lyapunov 함수 구조**: $V(\eta_*) = \eta_*^T E_*^T (\Sigma h_i P_i) \eta_*$가 $E_*$의 구조 ($\text{blkdiag}(I, 0)$)와 일치하는가? 특히 $E_*^T P_i = P_i^T E_*$ 조건이 만족되는가?

- [ ] **2. 1/C 인수**: $L_1$ bound에서 Cauchy-Schwarz 부등식 $\Sigma h_i^2 \geq 1/C$가 명시적으로 나타나는가?

- [ ] **3. P^T 처리**: $L_2$ bound에서 비대칭 $P_i$에 대해 정확히 $L_2 = 2\eta_*^T P^T T_*$로 표기되었는가 (또는 그 동등 형식)?

- [ ] **4. Young 부등식 정확성**: Young 부등식 $2ab \leq \gamma a^2 + b^2/\gamma$ 적용에서 $\bar{p}$이 한쪽에만 나타나는가? 특히 $\alpha_2 = \gamma \bar{p}^2$, $\beta_2 = \Delta^2/\gamma$ 형태인가?

- [ ] **5. 비교 보조정리**: $\dot{V} \leq -\alpha V + \beta$로부터 $\limsup V \leq \beta/\alpha$로 도출할 때 비교 보조정리를 명시적으로 호출하는가?

- [ ] **6. Descriptor 제약**: Descriptor 시스템의 lower block (대수 제약)에서 $\eta_*$ boundedness를 보장하기 위해 regularity와 제약 다양체 논증을 사용하는가?

- [ ] **7. ḣ_i 한계**: $|\dot{h}_i|$의 bound가 시간불변 상수 $\bar{\dot{h}}$로 주어졌는가? 또는 적응 Lyapunov 이론이 명시되었는가?

- [ ] **8. Neumann 급수 조건**: 정규성 조건을 $\|I - M^{-1}A_i^{13}\| < 1$로 표기하였는가 (NOT $\|M^{-1}A_i^{13}\| < 1$)?

- [ ] **9. 부호 일관성**: $\tau_{FB}$ 부호가 $e = q - q^d$의 정의와 일관되게 음수 피드백으로 설정되었는가?

- [ ] **10. α₁, α₂, α₃ 명시 정의**: 각 항 $\alpha_1, \alpha_2, \alpha_3$이 파라미터로 명시적으로 정의되고, 어느 LMI 또는 부등식으로부터 유래했는지 추적 가능한가?

### 검토 결과 해석

| "예" 개수 | 평가 |
|---|---|
| 10 | 매우 엄밀한 증명 ✓ |
| 8-9 | 약간의 세부사항 보강 필요 |
| 6-7 | 주요 논증 재검토 권장 |
| < 6 | 심각한 결함 존재, 전면 재작성 필요 |

---

## 부록 E. 추가 참고문헌

### 주요 교재 및 논문

1. **Takagi, T. and Sugeno, M.** (1985). "Fuzzy Identification of Systems and Its Applications to Modeling and Control." *IEEE Transactions on Systems, Man, and Cybernetics*, 15(1), 116-132.
   - TS fuzzy 시스템 최초 제안

2. **Tanaka, K. and Wang, H. O.** (2004). *Fuzzy Control Systems Design and Analysis: A Linear Matrix Inequality Approach*. Wiley-Interscience.
   - Tanaka-Wang 이완, CQLF vs FLF 비교

3. **Boyd, S., El Ghaoui, L., Feron, E., and Balakrishnan, V.** (1994). *Linear Matrix Inequalities in System and Control Theory*. SIAM.
   - LMI 기초 이론

4. **Khalil, H. K.** (2002). *Nonlinear Systems* (3rd ed.). Prentice Hall.
   - Lyapunov 정리, 비교 보조정리, UUB 정의 (Definition 4.5, Theorem 4.6)

5. **Khansari-Zadeh, S. M. and Billard, A.** (2011). "Learning Stable Nonlinear Dynamical Systems with Gaussian Mixture Models." *IEEE Transactions on Robotics*, 27(2), 252-270.
   - SEDS (Stable Estimator of Dynamical Systems), GMM/GMR 이론

6. **Chen, W.-H., Yang, J., Guo, L., and Li, S.** (2016). "Disturbance-Observer-Based Control and Related Methods — An Overview." *IEEE Transactions on Industrial Electronics*, 63(2), 1083-1095.
   - DOB 설계 및 튜닝 가이드

7. **Sariyildiz, E., Oboe, R., and Ohnishi, K.** (2019). "Disturbance Observer-Based Robust Control and Its Applications: 35th Anniversary Overview." *IEEE Transactions on Industrial Electronics*, 67(3), 2879-2889.
   - DOB의 최신 응용 (SEA, 회전 workspace 등)

### 온라인 자료

- **YALMIP** (Löfberg, J.): http://users.isy.liu.se/johanl/yalmip/
  - LMI 형식 최적화 (MATLAB)

- **CVX** (Boyd et al.): http://cvxr.com/cvx/
  - Convex 최적화 (MATLAB/Python)

- **SDP 솔버**: SeDuMi, SDPT3, Mosek, etc.

---

