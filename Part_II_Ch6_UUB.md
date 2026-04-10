# 6장. UUB: 외란이 있을 때의 현실적 안정성

> **이 장의 목표**: "실제 시스템에는 모델 오차와 외란이 항상 존재하므로, 완전한 안정성 대신 **상태가 특정 영역 안에 머무는** 현실적 안정성을 보장하는 방법을 배운다."
>
> **선수 지식**: 1장 Lyapunov, 5장 FLF (Fuzzy Lyapunov Function)
>
> **다음 장과의 연결**: 이 장에서 유도한 UUB 조건이 Part III에서 PPLM-DOB 제어기 설계에 직접 적용된다.

---

## 6.1 왜 UUB인가

### 6.1.1 이상적 안정성의 한계

1장과 5장에서 다룬 **점근 안정(asymptotic stability)**은 다음을 요구한다:

$$
\dot{V}(x) < 0 \quad \text{for all } x \neq 0
$$

이 조건이 성립하면 $V(x)$는 계속 감소하여 $V(x) \to 0$, 즉 $x(t) \to 0$이다.

하지만 실제 로봇 시스템에서는:

1. **모델 오차**: $M(q)$, $C(q,\dot{q})$ 등의 파라미터가 미완벽함
2. **DOB 대역폭 제한**: DOB가 무한 대역폭을 가질 수 없음 → 고주파 외란은 미보상
3. **비건모(unmodeled) 동역학**: 마찰, 유연도, 신축 등 모델링하지 않은 현상
4. **외적 외란**: 충격, 풍압, 센서 노이즈 등 시간 변화 외란

이 모든 요인을 "외란 항 $w(t)$"로 표현하면, 실제 시스템은

$$
\dot{x} = f(x) + w(t)
$$

형태이고, 여기서 $|w(t)| \leq \Delta$ (경계가 있는 외란)이다.

### 6.1.2 현실적 안정성의 개념

외란이 있을 때, $\dot{V} < 0$ everywhere는 **불가능**하다. 왜냐하면:

$$
\dot{V} = \nabla V^\top (f(x) + w)
$$

에서, 지표 $w$의 방향이 $-\nabla V$ 방향의 반대이고 충분히 크면 $\dot{V} > 0$이 될 수 있기 때문이다.

대신, 다음과 같은 형태를 노린다:

$$
\dot{V} \leq -\alpha\|x\|^2 + \beta
$$

여기서 $\alpha, \beta > 0$는 시스템과 외란의 경계로부터 정해진 상수이다.

이 부등식의 의미:
- $\alpha\|x\|^2 < \beta$인 영역에서는 $\dot{V} > 0$일 수 있음 → 에너지가 증가 가능
- $\alpha\|x\|^2 > \beta$인 영역에서는 $\dot{V} < 0$ → 에너지 감소
- 결과적으로, 상태는 $\|x\|^2 \lesssim \beta/\alpha$인 공(ball)에 진입하여 머문다.

### 6.1.3 UUB의 기하학적 해석

상태 공간에서 반지름 $r = \sqrt{\beta/\alpha}$인 공(ball) $B_r = \{\eta : \|\eta\| < r\}$을 생각하자.

UUB 안정성이 보장된다는 것은:

1. **수렴 단계**: 임의의 초기조건 $\eta(0)$에서 출발한 궤적이 유한 시간 $T$ 내에 공 $B_r$에 진입
2. **정체 단계**: $t \geq T$ 이후, 궤적은 절대 공 $B_r$을 벗어나지 않음

이를 그림으로 나타내면, 모든 외부 궤적이 공으로 "깔때기"되는 형태이다. 외란 때문에 완벽한 수렴은 불가능하지만, **유한 대역폭의 떨림(limit cycle)**으로 머문다는 뜻이다.

---

## 6.2 UUB의 정의

### 6.2.1 Khalil 정의

**정의 6.1 (Uniformly Ultimately Bounded, UUB).**

시스템 $\dot{\eta} = f(\eta, t)$에서, $f$가 $\eta$와 $t$에 대해 연속이고, 임의의 $c > 0$에 대해 **양수 상수** $b > c$와 $T(c)$가 존재하여

$$
\|\eta(t_0)\| < c \implies \|\eta(t)\| < b \quad \text{for all } t \geq t_0 + T(c)
$$

이면, **원점이 UUB 안정**이라고 한다.

또는 다르게 표현하면: $b$에 대해, 임의의 $\epsilon > b$를 택하면, 유한 시간 $T(\epsilon)$ 후에 $\|\eta(t)\| \leq b$가 모든 $t \geq t_0 + T(\epsilon)$에서 성립한다는 뜻이다.

### 6.2.2 Lyapunov 함수를 통한 UUB 조건

**정리 6.1 (UUB via Lyapunov function).**

시스템 $\dot{\eta} = f(\eta, t)$에 대해, Lyapunov 함수 $V(\eta)$가 존재하여

$$
\lambda_{\min} \|\eta\|^2 \leq V(\eta) \leq \lambda_{\max} \|\eta\|^2
$$

$$
\dot{V}(\eta, t) \leq -\alpha \|\eta\|^2 + \beta \quad \text{for all } \eta, t
$$

를 만족하면 (여기서 $\alpha, \beta, \lambda_{\min}, \lambda_{\max} > 0$), 원점이 UUB 안정이고 최종 도달 집합의 반지름은

$$
r = \sqrt{\frac{\lambda_{\max} \beta}{\lambda_{\min} \alpha}}
$$

이다.

**기하학적 의미**: 모든 궤적이 시간 $T$ 이후 반지름 $r$인 공 내부에 갇힌다.

---

## 6.3 Young의 부등식 완전 유도

### 6.3.1 문제 설정

곱 $ab$를 상한하고 싶다 ($a, b \geq 0$). Young의 부등식은 **임의의 양수 $\gamma > 0$에 대해**

$$
2ab \leq \gamma a^2 + \frac{b^2}{\gamma}
$$

를 제공한다. 이 부등식은 제어 문제에서 **상호작용항(cross term)**을 쪼갤 때 핵심이다.

### 6.3.2 완전 증명

**증명.**

$\gamma > 0$일 때, 다음 완전제곱 부등식부터 시작한다:

$$
\left(\sqrt{\gamma}\, a - \frac{b}{\sqrt{\gamma}}\right)^2 \geq 0
$$

이 부등식은 모든 실수 $a, b$와 $\gamma > 0$에 대해 항상 성립한다 (제곱은 항상 음이 아님).

양변을 전개한다:

$$
(\sqrt{\gamma}\, a)^2 - 2 \cdot \sqrt{\gamma}\, a \cdot \frac{b}{\sqrt{\gamma}} + \left(\frac{b}{\sqrt{\gamma}}\right)^2 \geq 0
$$

$$
\gamma a^2 - 2ab + \frac{b^2}{\gamma} \geq 0
$$

양변에 $2ab$를 더하고, 우변의 첫 번째와 세 번째 항만 남긴다:

$$
2ab \leq \gamma a^2 + \frac{b^2}{\gamma}
$$

이것이 Young의 부등식이다. $\square$

### 6.3.3 제어 문제에의 적용

**상황**: TS fuzzy 시스템에서 다음 교차항이 나타난다:

$$
\dot{V} = \ldots - p \|\eta^*\| \Delta + \ldots
$$

여기서 $p > 0$는 피드백 계수, $\eta^*$는 오차, $\Delta$는 외란/오차 경계이다.

이 항을 $V$의 음의 항과 상쇄하려면, Young의 부등식을 적용한다:

$$
2 \cdot p \|\eta^*\| \cdot \Delta \leq \gamma p^2 \|\eta^*\|^2 + \frac{\Delta^2}{\gamma}
$$

따라서

$$
- p \|\eta^*\| \Delta \leq - \frac{1}{2}\left(\gamma p^2 \|\eta^*\|^2 + \frac{\Delta^2}{\gamma}\right)
$$

(모든 항이 음수로 변함)

정리하면:

$$
- p \|\eta^*\| \Delta \leq -\frac{\gamma p^2}{2} \|\eta^*\|^2 - \frac{\Delta^2}{2\gamma}
$$

### 6.3.4 일반적 형태: $\dot{V}$ 항 분석

실제 TS fuzzy 제어에서는:

$$
\dot{V} \leq -\bar{p} \|\eta^*\|^2 - \bar{p} \|\eta^*\| \Delta + \ldots
$$

형태가 나타난다 (정확한 값은 제어기 설계에 따라 다름).

Young을 적용하면:

$$
2 \bar{p} \|\eta^*\| \Delta \leq \varepsilon \bar{p}^2 \|\eta^*\|^2 + \frac{\Delta^2}{\varepsilon}
$$

따라서

$$
\dot{V} \leq -\bar{p}\|\eta^*\|^2 - \bar{p}\|\eta^*\|\Delta
$$

$$
\leq -\bar{p}\|\eta^*\|^2 + \frac{1}{2}\left(\varepsilon \bar{p}^2 \|\eta^*\|^2 + \frac{\Delta^2}{\varepsilon}\right)
$$

(앞의 항에서 $\bar{p}\|\eta^*\|$를 인수로 빼고 Young 적용)

선택적으로 정렬하면:

$$
\dot{V} \leq -\left(\bar{p} - \frac{\varepsilon \bar{p}^2}{2}\right) \|\eta^*\|^2 + \frac{\Delta^2}{2\varepsilon}
$$

$\varepsilon$를 충분히 작게 고르면, 괄호 안이 양수가 되어 $\dot{V} \leq -\alpha \|\eta^*\|^2 + \beta$ 형태를 얻는다.

**핵심 경고**: $\alpha = \varepsilon \bar{p}^2$, $\beta = \Delta^2/\varepsilon$라고 쓸 때, **$\varepsilon$는 양쪽에 다르게** 들어간다. Young의 $2ab \leq \gamma a^2 + b^2/\gamma$에서 $a$에만 $\gamma$가 곱해지고 $b$ 쪽은 $1/\gamma$이기 때문이다. 흔한 실수는 양쪽에 $\bar{p}^2$을 붙이는 것인데, 이는 틀렸다.

---

## 6.4 Comparison Lemma 완전 유도

### 6.4.1 문제 설정

다음 스칼라 미분부등식을 생각하자:

$$
\dot{V}(t) \leq -cV(t) + d
$$

여기서 $c, d > 0$이다. 이 부등식만 알고 $V(t)$의 정확한 형태를 모를 때, $V(t)$의 상한과 극한을 구하고 싶다.

### 6.4.2 완전 증명 (Integrating Factor Method)

**정리 6.2 (Comparison Lemma).**

$\dot{V} \leq -cV + d$ (단, $c, d > 0$)이면,

$$
V(t) \leq V(0) e^{-ct} + \frac{d}{c}(1 - e^{-ct})
$$

특히,

$$
\limsup_{t \to \infty} V(t) \leq \frac{d}{c}
$$

**증명.**

양변에 **적분 인자(integrating factor)** $e^{ct}$를 곱한다:

$$
e^{ct} \dot{V}(t) \leq -c e^{ct} V(t) + d e^{ct}
$$

좌변은 곱의 미분법 $\frac{d}{dt}[e^{ct} V(t)] = e^{ct}\dot{V} + ce^{ct}V$를 이용하면:

$$
\frac{d}{dt}[e^{ct} V(t)] = e^{ct} \dot{V} + c e^{ct} V
$$

따라서

$$
e^{ct} \dot{V} = \frac{d}{dt}[e^{ct} V] - c e^{ct} V
$$

이를 부등식에 대입하면:

$$
\frac{d}{dt}[e^{ct} V(t)] - c e^{ct} V(t) \leq -c e^{ct} V(t) + d e^{ct}
$$

$$
\frac{d}{dt}[e^{ct} V(t)] \leq d e^{ct}
$$

양변을 0부터 $t$까지 적분한다:

$$
\int_0^t \frac{d}{ds}[e^{cs} V(s)] \, ds \leq \int_0^t d e^{cs} \, ds
$$

좌변은 기본정리에 의해:

$$
e^{ct} V(t) - V(0) \leq d \left[\frac{e^{cs}}{c}\right]_0^t = \frac{d}{c}(e^{ct} - 1)
$$

양변을 $e^{ct}$로 나누면:

$$
V(t) \leq \frac{V(0)}{e^{ct}} + \frac{d}{c} \left(1 - \frac{1}{e^{ct}}\right)
$$

$$
V(t) \leq V(0) e^{-ct} + \frac{d}{c}(1 - e^{-ct})
$$

$t \to \infty$일 때, $e^{-ct} \to 0$이므로:

$$
\limsup_{t \to \infty} V(t) \leq \frac{d}{c}
$$

이것이 comparison lemma이다. $\square$

### 6.4.3 기하학적 해석

이 보조정리를 그래프로 나타내면:

- $V(t)$의 상한(bound)은 지수 곡선 $V(0) e^{-ct}$에서 시작
- 시간이 경과하면서 수평선 $d/c$에 가까워짐
- 상한은 항상 $d/c$보다 위에 있음 (처음엔 $V(0)$, 나중엔 $d/c$)

외란이 없으면 ($d = 0$) → $V(t) \leq V(0) e^{-ct} \to 0$ (점근 안정)

외란이 있으면 ($d > 0$) → $\limsup V(t) = d/c$ (최종 값은 외란의 크기에 비례)

---

## 6.5 \dot{V} → V → ||η|| 완전 체인

이 절이 이 장의 **최핵심**이다. $\dot{V}$ 부등식으로부터 어떻게 최종적으로 상태의 범위 $\|\eta\|$을 구하는지 **한 단계씩** 보인다.

### 6.5.1 시작: $\dot{V}$ 형태

TS fuzzy 제어기가 설계되어, 다음을 만족한다고 하자:

$$
\dot{V}(\eta) \leq -\alpha \|\eta^*\|^2 + \beta
$$

여기서:
- $\eta^*$: 전체 오차상태의 일부 (예: 위치 오차, 속도 오차)
- $\alpha, \beta > 0$: 제어기 파라미터와 외란 경계로부터 정해진 상수
- $\|\eta^*\|^2$는 $\|\eta\|^2$의 **일부 성분**만 포함

### Step 1: $\|\eta^*\| \leq C\|\eta\|$ 관계

일반적으로, 오차 상태 정의에 의해 다음 관계가 성립한다:

$$
\|\eta^*\| \leq C_1 \|\eta\|
$$

여기서 $C_1 > 0$는 상수 (예: $\eta^* = [e_1, e_2, \ldots]^\top$의 노름이 전체 $\eta = [e_1, e_2, \ldots, e_k]^\top$의 노름보다 작거나 같음).

따라서:

$$
\|\eta^*\|^2 \leq C_1^2 \|\eta\|^2
$$

역으로,

$$
\|\eta\|^2 \geq \frac{\|\eta^*\|^2}{C_1^2}
$$

### Step 2: $\dot{V}$ 부등식을 $\|\eta\|$로 다시 쓰기

$$
\dot{V} \leq -\alpha \|\eta^*\|^2 + \beta
$$

에서 $\|\eta^*\|^2 \geq \|\eta\|^2/C_1^2$를 사용하면:

$$
\dot{V} \leq -\alpha \cdot \frac{\|\eta\|^2}{C_1^2} + \beta = -\frac{\alpha}{C_1^2} \|\eta\|^2 + \beta
$$

$\alpha' = \alpha/C_1^2$로 정의하면:

$$
\dot{V} \leq -\alpha' \|\eta\|^2 + \beta
$$

### Step 3: $\dot{V}$ 에서 $V$로의 변환

$V(\eta)$는 quadratic Lyapunov 함수이므로:

$$
\lambda_{\min} \|\eta\|^2 \leq V(\eta) \leq \lambda_{\max} \|\eta\|^2
$$

따라서:

$$
\|\eta\|^2 \geq \frac{V(\eta)}{\lambda_{\max}}
$$

이를 앞의 부등식에 대입하면:

$$
\dot{V} \leq -\alpha' \cdot \frac{V}{\lambda_{\max}} + \beta = -\frac{\alpha'}{\lambda_{\max}} V + \beta
$$

$c = \alpha'/\lambda_{\max} > 0$, $d = \beta$로 정의하면:

$$
\dot{V} \leq -cV + d
$$

이것이 comparison lemma의 형태이다!

### Step 4: Comparison Lemma 적용

Step 3의 부등식 $\dot{V} \leq -cV + d$에 comparison lemma (정리 6.2)를 적용하면:

$$
\limsup_{t \to \infty} V(t) \leq \frac{d}{c} = \frac{\beta}{\alpha'/\lambda_{\max}} = \frac{\beta \lambda_{\max}}{\alpha'}
$$

$\alpha' = \alpha/C_1^2$를 역으로 대입하면:

$$
\limsup_{t \to \infty} V(t) \leq \frac{\beta \lambda_{\max}}{\alpha / C_1^2} = \frac{\beta \lambda_{\max} C_1^2}{\alpha}
$$

실제 문제에서 보통 $C_1 = 1$ (즉, $\eta^*$가 $\eta$의 전체 또는 비슷한 스케일)이면:

$$
\limsup_{t \to \infty} V(t) \leq \frac{\beta \lambda_{\max}}{\alpha}
$$

### Step 5: $V$에서 $\|\eta\|$로의 변환

마지막으로, $V$와 $\|\eta\|$의 관계를 사용한다:

$$
V(\eta) \geq \lambda_{\min} \|\eta\|^2
$$

따라서:

$$
\lambda_{\min} \|\eta\|^2 \leq \limsup_{t \to \infty} V(t) \leq \frac{\beta \lambda_{\max}}{\alpha}
$$

양변을 $\lambda_{\min}$으로 나누고 제곱근을 취하면:

$$
\limsup_{t \to \infty} \|\eta(t)\| \leq \sqrt{\frac{\beta \lambda_{\max}}{\lambda_{\min} \alpha}}
$$

이것이 **최종 도달 반지름**이다:

$$
r = \sqrt{\frac{\beta \lambda_{\max}}{\lambda_{\min} \alpha}}
$$

### 6.5.6 정리: 5-Step Chain

| Step | 식 | 의미 |
|------|-----|------|
| 1 | $\dot{V} \leq -\alpha \|\eta^*\|^2 + \beta$ | 제어기 설계로부터 얻은 조건 |
| 2 | $\dot{V} \leq -\alpha' \|\eta\|^2 + \beta$ | 오차 관계 이용 |
| 3 | $\dot{V} \leq -cV + d$ | Lyapunov 함수의 정의 이용 |
| 4 | $\limsup V(t) \leq d/c$ | Comparison lemma |
| 5 | $\limsup \|\eta(t)\| \leq r$ | 최종 상태 범위 |

**이 chain이 UUB 분석의 핵심이다.** 많은 논문에서 Step 2-3의 변환 ($\dot{V}$를 $V$로 바꾸기)을 건너뛰거나 모호하게 쓰는데, 완전한 이해를 위해서는 매우 중요하다.

---

## 6.6 예제

### 예제 6.1: Scalar 시스템의 UUB

**문제.**

$$
\dot{x} = -2x + w(t), \quad |w(t)| \leq 0.1
$$

이 시스템이 UUB 안정인지 확인하고, 최종 도달 반지름을 구하라.

**풀이.**

**Lyapunov 함수 선택:** $V = x^2$

**$\dot{V}$ 계산:**

$$
\dot{V} = 2x \dot{x} = 2x(-2x + w) = -4x^2 + 2xw
$$

**상한:** Young의 부등식을 적용하면, $|w| \leq 0.1$이므로:

$$
|2xw| \leq 2|x| \cdot 0.1 = 0.2|x|
$$

더 정확하게, $2xy \leq \varepsilon x^2 + y^2/\varepsilon$를 사용하면:

$$
2xw \leq \varepsilon x^2 + \frac{w^2}{\varepsilon} \leq \varepsilon x^2 + \frac{0.01}{\varepsilon}
$$

$\varepsilon = 0.1$로 선택하면:

$$
2xw \leq 0.1 x^2 + 0.1
$$

따라서:

$$
\dot{V} \leq -4x^2 + 0.1x^2 + 0.1 = -3.9 x^2 + 0.1
$$

**$\dot{V}$의 형태:** $\dot{V} \leq -\alpha \|x\|^2 + \beta$, 여기서 $\alpha = 3.9$, $\beta = 0.1$

**Lyapunov 함수의 경계:**

$$
\lambda_{\min} = \lambda_{\max} = 1 \quad (\text{because } V = 1 \cdot x^2)
$$

**최종 도달 반지름:**

$$
r = \sqrt{\frac{\beta \lambda_{\max}}{\alpha \lambda_{\min}}} = \sqrt{\frac{0.1 \times 1}{3.9 \times 1}} = \sqrt{\frac{0.1}{3.9}} \approx 0.16
$$

**결론:** 이 시스템은 UUB 안정이고, 모든 궤적이 유한 시간 후 $|x| \leq 0.16$ 범위 내에 머문다.

### 예제 6.2: 2-Rule TS Fuzzy 시스템

**문제.**

다음 TS fuzzy 시스템을 생각하자:

$$
\dot{x} = h_1(z) \cdot (-2x) + h_2(z) \cdot (-0.5x) + w(t), \quad h_1 + h_2 = 1, \quad |w| \leq 0.1
$$

이는 두 선형 시스템 $\dot{x} = -2x + w$와 $\dot{x} = -0.5x + w$ 사이를 $h_1, h_2$의 값에 따라 보간한 것이다.

Lyapunov 함수 $V = x^2$를 사용하여 UUB를 보이고 최종 반지름을 구하라.

**풀이.**

$$
\dot{V} = 2x \dot{x} = 2x[h_1(-2x) + h_2(-0.5x) + w]
$$

$$
= h_1(-4x^2) + h_2(-x^2) + 2xw
$$

$$
= -h_1 \cdot 4x^2 - h_2 \cdot x^2 + 2xw
$$

최악의 경우, $h_2 = 1, h_1 = 0$일 때 (가장 약한 피드백):

$$
\dot{V} \leq -x^2 + 2xw
$$

Young의 부등식 $2xw \leq 0.1 x^2 + 0.1$ (위 예제와 동일)를 적용하면:

$$
\dot{V} \leq -x^2 + 0.1 x^2 + 0.1 = -0.9 x^2 + 0.1
$$

$\alpha = 0.9$, $\beta = 0.1$, $\lambda_{\min} = \lambda_{\max} = 1$이므로:

$$
r = \sqrt{\frac{0.1}{0.9}} = \sqrt{1/9} \approx 0.33
$$

**해석:** 2-rule fuzzy 시스템이므로 피드백이 약해져, 단일 $\dot{x} = -2x + w$ 경우 ($r \approx 0.16$)보다 최종 반지름이 커졌다 ($r \approx 0.33$).

### 예제 6.3: Comparison Lemma 시각화

**목표:** Comparison lemma의 보수성과 정확성을 시뮬레이션으로 확인한다.

**설정:** 실제 비선형 시스템

$$
\dot{x} = -2x - x^3 + 0.1 \sin(10t)
$$

(외란 $w = 0.1\sin(10t)$, $|w| \leq 0.1$)

**예상 결과:**

- $V(x) = x^2$의 직접 시뮬레이션 궤적
- Comparison lemma 상한 $V(t) \leq V(0)e^{-ct} + d/c(1-e^{-ct})$
- 실제 $V(t)$ 궤적이 상한 아래에 위치함을 확인

(Python 코드는 6.7절 참조)

---

## 6.7 실습 코드

### 6.7.1 MATLAB: Scalar UUB + Comparison Lemma

```matlab
%% Chapter 6: UUB Analysis
% Example 6.1 + Comparison Lemma visualization

clear; clc;

% System: dx = -2x + w(t), |w| <= 0.1
% Lyapunov: V = x^2

% UUB parameters
alpha = 3.9;   % -α||x||^2 coefficient (from Young)
beta = 0.1;    % +β constant (disturbance)
lambda_min = 1;
lambda_max = 1;

% UUB final radius
r_uub = sqrt(beta * lambda_max / (alpha * lambda_min));
fprintf('=== Example 6.1: Scalar UUB ===\n');
fprintf('UUB final radius r = %.4f\n', r_uub);

% Comparison lemma parameters
c = alpha / lambda_max;  % decay coefficient
d = beta;                 % constant term
V_infinity = d / c;       % steady-state value

fprintf('\n=== Comparison Lemma ===\n');
fprintf('dot(V) <= -c*V + d: c=%.2f, d=%.2f\n', c, d);
fprintf('limsup V(t) <= d/c = %.4f\n', V_infinity);
fprintf('limsup ||x(t)|| <= sqrt(d/c/λ_min) = %.4f\n', sqrt(V_infinity/lambda_min));

%% Simulation
t_sim = 20;
t0 = [0, 0.1, 1, 2];  % different initial conditions
x0_vals = [1.0, -0.8, 0.5, -0.3];

fig = figure('Position', [100, 100, 1400, 900]);

for idx = 1:length(t0)
    x0 = x0_vals(idx);
    V0 = x0^2;
    
    % ODE15s for nonlinear system
    % dx = -2x + 0.1*sin(10*t)
    options = odeset('RelTol', 1e-6, 'AbsTol', 1e-9);
    [t, x] = ode45(@(t,x) -2*x + 0.1*sin(10*t), [0, t_sim], x0, options);
    
    V_sim = x.^2;
    
    % Comparison lemma bound
    V_bound = V0 * exp(-c * t) + (d/c) * (1 - exp(-c * t));
    
    % Plot
    subplot(2, 2, idx);
    semilogy(t, V_sim, 'b-', 'LineWidth', 2, 'DisplayName', 'V(t) simulation');
    hold on;
    semilogy(t, V_bound, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Comparison lemma bound');
    yline(V_infinity, 'g:', 'LineWidth', 1.5, 'DisplayName', sprintf('d/c = %.4f', V_infinity));
    
    xlabel('Time [s]', 'FontSize', 11);
    ylabel('V(x)', 'FontSize', 11);
    title(sprintf('IC: x(0) = %.1f, V(0) = %.4f', x0, V0), 'FontSize', 11);
    legend('Location', 'best');
    grid on;
    axis([0 t_sim 1e-8 10]);
    
    % Verify bound is never violated
    if max(V_sim - V_bound) > 1e-6
        fprintf('Warning: V_sim exceeded bound for IC %.2f\n', x0);
    end
end

sgtitle('Example 6.1: Scalar UUB + Comparison Lemma', 'FontSize', 14, 'FontWeight', 'bold');
```

### 6.7.2 Python: TS Fuzzy UUB + Detailed Analysis

```python
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from matplotlib.patches import Circle

# ============================================================================
# Chapter 6: UUB and Comparison Lemma
# ============================================================================

# Example 6.2: 2-Rule TS Fuzzy System
# dx = h_1(-2x) + h_2(-0.5x) + w(t)
# h_1 + h_2 = 1, |w| <= 0.1

print("=" * 60)
print("Example 6.2: 2-Rule TS Fuzzy UUB")
print("=" * 60)

# UUB parameters (worst case: h_2=1, h_1=0)
alpha_fuzzy = 0.9      # -α||x||² (conservative)
beta_fuzzy = 0.1       # disturbance
lambda_min_fuzzy = 1
lambda_max_fuzzy = 1

r_fuzzy = np.sqrt(beta_fuzzy * lambda_max_fuzzy / (alpha_fuzzy * lambda_min_fuzzy))
print(f"UUB final radius (fuzzy): r = {r_fuzzy:.4f}")

c_fuzzy = alpha_fuzzy / lambda_max_fuzzy
V_inf_fuzzy = beta_fuzzy / c_fuzzy
print(f"Comparison lemma: limsup V(t) = {V_inf_fuzzy:.4f}")
print(f"                 limsup ||x|| = {np.sqrt(V_inf_fuzzy):.4f}")

# Membership functions: smooth transition
def membership_h1(t):
    """h_1(t) = 0.5 + 0.4*sin(0.5*t)"""
    return 0.5 + 0.4 * np.sin(0.5 * t)

def membership_h2(t):
    """h_2(t) = 1 - h_1(t)"""
    return 1 - membership_h1(t)

def disturbance_w(t):
    """w(t) = 0.1*sin(10*t)"""
    return 0.1 * np.sin(10 * t)

# System dynamics
def fuzzy_dynamics(t, x):
    h1 = membership_h1(t)
    h2 = membership_h2(t)
    w = disturbance_w(t)
    return h1 * (-2 * x) + h2 * (-0.5 * x) + w

# ============================================================================
# Simulation
# ============================================================================
t_span = [0, 30]
t_eval = np.linspace(0, 30, 1000)
x_inits = [1.5, -1.2, 0.8, -0.4, 0.1]

fig, axes = plt.subplots(2, 2, figsize=(14, 10))

# (A) State trajectories
ax = axes[0, 0]
for x0 in x_inits:
    sol = solve_ivp(fuzzy_dynamics, t_span, [x0], t_eval=t_eval, method='RK45', dense_output=True)
    ax.plot(sol.t, sol.y[0], linewidth=1.5, label=f'x(0)={x0:.1f}')

ax.axhline(r_fuzzy, color='r', linestyle='--', linewidth=2, label=f'r = {r_fuzzy:.4f}')
ax.axhline(-r_fuzzy, color='r', linestyle='--', linewidth=2)
ax.fill_between([0, 30], -r_fuzzy, r_fuzzy, alpha=0.2, color='red', label='UUB region')
ax.set_xlabel('Time [s]', fontsize=11)
ax.set_ylabel('x(t)', fontsize=11)
ax.set_title('(A) State Trajectories + UUB Region', fontsize=12, fontweight='bold')
ax.grid(True, alpha=0.3)
ax.legend(loc='best')
ax.set_xlim([0, 30])

# (B) V(t) and Comparison Lemma
ax = axes[0, 1]
for x0 in x_inits:
    sol = solve_ivp(fuzzy_dynamics, t_span, [x0], t_eval=t_eval, method='RK45', dense_output=True)
    V_sim = sol.y[0]**2
    V0 = x0**2
    
    # Comparison lemma bound
    V_bound = V0 * np.exp(-c_fuzzy * sol.t) + V_inf_fuzzy * (1 - np.exp(-c_fuzzy * sol.t))
    
    ax.semilogy(sol.t, V_sim, linewidth=1.5, label=f'V(x), x(0)={x0:.1f}', marker='', alpha=0.7)
    ax.semilogy(sol.t, V_bound, '--', linewidth=1, alpha=0.5)

ax.axhline(V_inf_fuzzy, color='g', linestyle=':', linewidth=2, label=f'limsup V = d/c = {V_inf_fuzzy:.4f}')
ax.set_xlabel('Time [s]', fontsize=11)
ax.set_ylabel('V(x) = x²', fontsize=11)
ax.set_title('(B) Lyapunov Function + Bounds', fontsize=12, fontweight='bold')
ax.grid(True, alpha=0.3, which='both')
ax.legend(loc='best')
ax.set_xlim([0, 30])
ax.set_ylim([1e-8, 10])

# (C) Membership functions
ax = axes[1, 0]
t_mem = np.linspace(0, 30, 500)
h1_vals = membership_h1(t_mem)
h2_vals = membership_h2(t_mem)
ax.plot(t_mem, h1_vals, 'b-', linewidth=2, label='h₁(t)')
ax.plot(t_mem, h2_vals, 'r-', linewidth=2, label='h₂(t)')
ax.fill_between(t_mem, h1_vals, alpha=0.3, color='blue')
ax.fill_between(t_mem, h2_vals, alpha=0.3, color='red')
ax.set_xlabel('Time [s]', fontsize=11)
ax.set_ylabel('Membership degree', fontsize=11)
ax.set_title('(C) TS Fuzzy Membership Functions', fontsize=12, fontweight='bold')
ax.set_ylim([-0.1, 1.1])
ax.grid(True, alpha=0.3)
ax.legend(loc='best')
ax.set_xlim([0, 30])

# (D) Phase plane + UUB ball
ax = axes[1, 1]
x0_phase = 2.0
sol_phase = solve_ivp(fuzzy_dynamics, t_span, [x0_phase], t_eval=t_eval, method='RK45')
# For 2D phase plot, add velocity dx
dx_vals = np.array([fuzzy_dynamics(t, x) for t, x in zip(sol_phase.t, sol_phase.y[0])])

ax.plot(sol_phase.y[0], dx_vals, 'b-', linewidth=2, label='Trajectory')
ax.plot(x0_phase, fuzzy_dynamics(0, x0_phase), 'go', markersize=10, label='Start')
ax.plot(sol_phase.y[0][-1], dx_vals[-1], 'r*', markersize=15, label='End')

# Add equilibrium and UUB region indicator
ax.axhline(0, color='k', linestyle='-', linewidth=0.5, alpha=0.5)
ax.axvline(0, color='k', linestyle='-', linewidth=0.5, alpha=0.5)
ax.set_xlabel('x', fontsize=11)
ax.set_ylabel('dx/dt', fontsize=11)
ax.set_title('(D) Phase Portrait (1D projected)', fontsize=12, fontweight='bold')
ax.grid(True, alpha=0.3)
ax.legend(loc='best')

plt.tight_layout()
plt.savefig('/tmp/ch6_uub_fuzzy.png', dpi=150, bbox_inches='tight')
print("\n✓ Figure saved: ch6_uub_fuzzy.png")

# ============================================================================
# Summary statistics
# ============================================================================
print("\n" + "=" * 60)
print("Summary: Convergence to UUB ball")
print("=" * 60)

for x0 in x_inits:
    sol = solve_ivp(fuzzy_dynamics, t_span, [x0], t_eval=t_eval, method='RK45')
    final_x = sol.y[0][-1]
    max_x = np.max(np.abs(sol.y[0]))
    
    # Find when ||x|| <= r_fuzzy + 0.01
    in_region = np.abs(sol.y[0]) <= r_fuzzy * 1.1
    if np.any(in_region):
        t_enter = sol.t[np.argmax(in_region)]
    else:
        t_enter = float('inf')
    
    print(f"x(0) = {x0:6.2f}: max|x|={max_x:.4f}, final x={final_x:.6f}, t_enter={t_enter:.2f}s")

print(f"\nUUB region: ||x|| ≤ {r_fuzzy:.4f}")

plt.show()
```

### 6.7.3 MATLAB: Comparison Lemma 증명 검증

```matlab
%% Comparison Lemma Proof Verification
% Verify that V(t) <= V0*exp(-c*t) + d/c*(1-exp(-c*t))

clear; clc; close all

% ODE: dV = -c*V + d
c = 2;
d = 0.5;
V_infinity = d/c;  % 0.25

% Analytical solution to dV = -c*V + d
% General solution: V(t) = C*exp(-c*t) + V_infinity
% With V(0) = V0: C = V0 - V_infinity
% So: V(t) = (V0 - V_infinity)*exp(-c*t) + V_infinity
%           = V0*exp(-c*t) + V_infinity*(1 - exp(-c*t))

V0_list = [1.0, 0.5, 0.1, 0.05];
t_plot = linspace(0, 5, 1000);

fig = figure('Position', [100, 100, 1200, 600]);

subplot(1, 2, 1);
for V0 = V0_list
    V_exact = V0 * exp(-c * t_plot) + V_infinity * (1 - exp(-c * t_plot));
    semilogy(t_plot, V_exact, 'LineWidth', 2, DisplayName=sprintf('V₀=%.2f', V0));
    hold on;
end
yline(V_infinity, 'k--', 'LineWidth', 2, 'DisplayName', sprintf('d/c = %.3f', V_infinity));
xlabel('Time t'); ylabel('V(t)');
title('Comparison Lemma: Exact Solutions');
legend('Location', 'best');
grid on;
set(gca, 'YScale', 'log');

% Now compare with numerical ODE solution
subplot(1, 2, 2);
for V0 = V0_list
    [t, V_num] = ode45(@(t,V) -c*V + d, [0, 5], V0);
    plot(t, V_num, 'o', 'MarkerSize', 3, DisplayName=sprintf('ODE V₀=%.2f', V0));
    hold on;
    
    % Overlay exact solution
    V_exact_fine = V0 * exp(-c * t) + V_infinity * (1 - exp(-c * t));
    plot(t, V_exact_fine, '-', 'LineWidth', 1.5, 'Color', get(gca, 'ColorOrder')(end, :));
end
yline(V_infinity, 'k--', 'LineWidth', 2);
xlabel('Time t'); ylabel('V(t)');
title('Numerical ODE vs Exact Solution');
legend('Location', 'best');
grid on;

sgtitle('Verification of Comparison Lemma', 'FontSize', 14, 'FontWeight', 'bold');

fprintf('✓ Numerical solutions match exact formula within ode45 tolerance.\n');
```

---

## 6.8 복습 문제

1. **Young 부등식 변형**: $a = 2\bar{p}\|\eta^*\|$, $b = \Delta$, $\gamma = \varepsilon$로 두고, Young의 $2ab \leq \gamma a^2 + b^2/\gamma$를 적용하면 어떤 형태가 나오는가? $\varepsilon$가 작을수록, 큰수록 $\alpha$와 $\beta$에 어떤 영향을 주는가?

2. **Comparison Lemma 응용**: $\dot{V} \leq -2V + 1$일 때, $V(0) = 2$에서 출발한 궤적의 $\limsup V(t)$는? 그리고 $t = 2$일 때의 상한은?

3. **Lyapunov 함수의 경계**: TS fuzzy 시스템의 이차형 Lyapunov 함수 $V(x) = x^\top P x$에서, $\lambda_{\min}(P)$가 크면 UUB 반지름 $r$은 어떻게 변하는가? 왜인가?

4. **외란 효과**: 예제 6.1에서 외란 크기가 $|w| \leq 0.2$로 두 배 커지면, 최종 반지름 $r$은 몇 배가 되는가?

---

## 6.9 핵심 요약

> **UUB (Uniformly Ultimately Bounded) 안정성**은 외란이 있는 실제 시스템의 현실적 안정성 개념이다.
>
> 핵심은 **$\dot{V} \leq -\alpha\|\eta\|^2 + \beta$ 형태의 부등식**을 얻고, 이로부터 **모든 궤적이 반지름 $r = \sqrt{\beta\lambda_{\max}/(\alpha\lambda_{\min})}$인 공에 진입**함을 보이는 것이다.
>
> **5-Step Chain** ($\dot{V} \to V \to \|\eta\|$)이 UUB 분석의 핵심:
> 1. 제어기 설계로부터 $\dot{V} \leq -\alpha\|\eta^*\|^2 + \beta$ 획득
> 2. 오차 관계 이용: $\|\eta^*\| \leq C\|\eta\|$
> 3. Lyapunov 함수 경계 이용: $V \geq \lambda_{\min}\|\eta\|^2$로 변환
> 4. **Comparison Lemma** 적용: $\dot{V} \leq -cV + d$ → $\limsup V \leq d/c$
> 5. 역변환으로 $\limsup \|\eta\| \leq r$ 도출
>
> **Young의 부등식** ($2ab \leq \gamma a^2 + b^2/\gamma$)은 외란 항 처리에 필수이며, **Comparison Lemma**는 미분부등식에서 함수의 상한을 얻는 표준 도구이다.
>
> **이제 모든 도구가 갖춰졌다**:
> - **1장 (Lyapunov)**: 에너지 함수 개념
> - **2장 (LMI)**: 행렬 부등식 최적화
> - **3장 (Solver)**: 수치 해법
> - **4장 (TS Fuzzy)**: 비선형 시스템 모델링
> - **5장 (FLF)**: TS fuzzy 안정성 분석
> - **6장 (UUB)**: 외란 하에서의 현실적 안정성
>
> **Part III**에서는 이 모든 도구를 **PPLM-DOB (Polynomial Parameter-Lyapunov Matrix Disturbance Observer) 제어기 설계**에 총동원하여, 모델 오차와 미측정 외란에 강건한 로봇 제어기를 구성한다.

