# GMM-DOB 안정성 해석 교재

## Stability Analysis of PPLM-DOB via TS Fuzzy Descriptor Systems

> GMM/GMR 기반 비선형 제어기의 안정성을 Lyapunov 이론, LMI, TS 퍼지 기술자 시스템을 통해 기초부터 완전 증명까지 다루는 대학원 교재

---

### 교재 구조

```
Part I — 기초 도구
├── Ch 1. Lyapunov 안정성
├── Ch 2. LMI (선형행렬부등식)
└── Ch 3. SDP 솔버 실습

Part II — TS 퍼지 안정성
├── Ch 4. Takagi-Sugeno 퍼지 시스템
├── Ch 5. Fuzzy Lyapunov Function (FLF)
└── Ch 6. Uniform Ultimate Boundedness (UUB)

Part III — GMM-DOB 응용
├── Ch 7. GMR → TS 퍼지 등가성
├── Ch 8. DOB 기술자 시스템 구성
├── Ch 9. 완전 안정성 증명
└── Ch 10. 설계 지침
```

### 의존성 다이어그램

```
Ch1 (Lyapunov) ──→ Ch2 (LMI) ──→ Ch3 (SDP)
       │                │              │
       ▼                ▼              ▼
     Ch4 (TS Fuzzy) → Ch5 (FLF) → Ch6 (UUB)
       │                │              │
       ▼                ▼              ▼
     Ch7 (GMR↔TS) → Ch8 (Descriptor) → Ch9 (Full Proof) → Ch10 (Design)
```

### 목차

| 파일 | 제목 | 핵심 내용 |
|------|------|-----------|
| [Part_I_Ch1_Lyapunov.md](Part_I_Ch1_Lyapunov.md) | Lyapunov 안정성 | $V > 0,\ \dot{V} < 0$ 기본 정리, 양의 정부호, LaSalle |
| [Part_I_Ch2_LMI.md](Part_I_Ch2_LMI.md) | 선형행렬부등식 | Schur 보조정리, 합동변환, BMI→LMI 변수치환 |
| [Part_I_Ch3_SDP_Solver.md](Part_I_Ch3_SDP_Solver.md) | SDP 솔버 실습 | YALMIP/CVXPy, SeDuMi/MOSEK 비교, 수치 검증 |
| [Part_II_Ch4_TS_Fuzzy.md](Part_II_Ch4_TS_Fuzzy.md) | TS 퍼지 시스템 | 섹터 비선형성, CQLF, 보수성 예제 |
| [Part_II_Ch5_FLF.md](Part_II_Ch5_FLF.md) | Fuzzy Lyapunov Function | FLF 이중합, Tanaka-Wang 완화, Cauchy-Schwarz $1/C$ |
| [Part_II_Ch6_UUB.md](Part_II_Ch6_UUB.md) | UUB 분석 | Young 부등식, 비교 보조정리, $\dot{V} \to V \to \|\eta\|$ 체인 |
| [Part_III_Ch7_GMR_TS.md](Part_III_Ch7_GMR_TS.md) | GMR ↔ TS 등가성 | GMM 기초, 조건부 기대값, $A_i$ 물리적 해석, $\dot{h}_i$ 바운드 |
| [Part_III_Ch8_Descriptor.md](Part_III_Ch8_Descriptor.md) | 기술자 시스템 구성 | DOB 피드백, 오차 동역학, $E^* = \text{blkdiag}(I,0)$ |
| [Part_III_Ch9_Full_Proof.md](Part_III_Ch9_Full_Proof.md) | 완전 안정성 증명 | 5단계 UUB 증명, $\mathcal{L}_1/\mathcal{L}_2/\mathcal{L}_3$ 바운드 |
| [Part_III_Ch10_Design.md](Part_III_Ch10_Design.md) | 설계 지침 | 파라미터→UUB 반경 매핑, DOB 이중 이점, $\varepsilon$ 최적화 |
| [Appendices.md](Appendices.md) | 부록 | 3가지 접근법 비교, 8개 오류 카탈로그, 체크리스트 |

### 웹 교재

| 파일 | 설명 |
|------|------|
| [index.html](index.html) | Apple 스타일 인터랙티브 교재 (KaTeX 수식, 다크모드, 접이식 증명) |
| [textbook_GMM_DOB_Stability.tex](textbook_GMM_DOB_Stability.tex) | LaTeX 소스 (컴파일용) |

> GitHub Pages에서 이 폴더의 `index.html`을 열면 교재 전체를 웹 브라우저에서 바로 볼 수 있습니다.

### 대상 독자

대학원 신입생 수준. 선형대수 기초와 미분방정식에 대한 이해가 있으면 충분합니다. 각 장은 MATLAB과 Python 코드를 포함하여 이론을 즉시 실습할 수 있도록 구성되어 있습니다.

### 핵심 결과 요약

PPLM-DOB 제어기의 오차 동역학이 TS 퍼지 기술자 시스템

$$
E^* \dot{\eta}^* = \sum_{i=1}^{C} h_i(\xi)\, A_i^* \eta^* + \mathcal{T}^*
$$

으로 표현되며, Fuzzy Lyapunov Function과 비교 보조정리를 통해 **Uniform Ultimate Boundedness (UUB)** 를 보장합니다:

$$
\limsup_{t\to\infty} \|\eta^*(t)\| \leq r = \sqrt{\frac{\bar{\lambda}\,\beta}{\underline{\lambda}\,\alpha}}
$$

DOB는 $A_i^*$를 풍부하게 하여 $\alpha$를 증가시키고, 잔여 외란 $\Delta$를 줄여 $\beta$를 감소시키는 **이중 이점**을 제공합니다.

---

### 라이선스

이 교재는 학술 및 교육 목적으로 작성되었습니다.

### 참고 문헌

- Tanaka & Wang, *Fuzzy Control Systems Design and Analysis* (Wiley, 2001)
- Boyd et al., *Linear Matrix Inequalities in System and Control Theory* (SIAM, 1994)
- Khalil, *Nonlinear Systems*, 3rd ed. (Prentice Hall, 2002)
