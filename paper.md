# 선택적 다방향 회전평가를 결합한 CPPN–NEAT 복셀 로봇 진화
## Introduction 및 구현 Methods 초안

> **작성 범위.** 본 초안은 제공된 Python/C++ 구현에서 실제로 활성화된 `RotEdge` 기반 이산 회전 경로를 중심으로 작성하였다. 연속 quaternion 회전 경로는 코드에 존재하지만 `ENABLE_QUAT_ROT=False`이므로 본 실험 방법에서는 제외하였다. 아래의 본문은 논문에 바로 편집해 넣을 수 있는 서술이고, 마지막의 **재현성 점검표**는 제출 전 코드·실행 로그와 대조해야 할 저자용 메모이다.

---

# 1. Introduction

## 1.1. 복셀 기반 소프트 로봇의 진화적 설계

복셀 기반 소프트 로봇은 다수의 변형 가능한 셀을 3차원 격자에 배치하여 하나의 몸체를 구성하며, 각 셀의 존재 여부, 재료 유형 및 작동 특성이 결합되어 전체 형태와 운동을 결정한다. 이와 같은 설계 공간은 개별 복셀의 수가 증가함에 따라 급격히 커지고, 형태와 작동 패턴의 변화가 접촉, 중력, 마찰 및 탄성 변형을 통해 비선형적으로 결합되기 때문에 수작업 또는 국소 최적화만으로 탐색하기 어렵다. Compositional Pattern-Producing Network(CPPN)는 공간 좌표를 입력받아 각 위치의 표현형 속성을 출력하는 간접 인코딩으로, 대칭, 반복 및 연속적인 재료 패턴을 압축적으로 표현할 수 있다. NeuroEvolution of Augmenting Topologies(NEAT)는 이러한 CPPN의 연결 가중치뿐 아니라 노드와 연결 구조 자체를 진화시킴으로써 단순한 초기 패턴에서 더 복잡한 형태로 점진적으로 확장할 수 있게 한다 [Stanley and Miikkulainen, 2002; Stanley, 2007].

CPPN 기반의 생성적 인코딩은 직접 인코딩보다 공간적으로 조직된 복셀 조직을 생성하는 경향이 있으며, 이러한 규칙성은 인접 작동 복셀의 협응과 다양한 이동 전략의 출현에 유리한 것으로 보고되어 왔다 [Cheney et al., 2013]. 그러나 소프트 바디의 물리 시뮬레이션은 계산 비용이 높고, 하나의 개체를 평가할 때마다 전체 격자를 디코딩하고 시간 적분을 수행해야 한다 [Hiller and Lipson, 2014]. 따라서 복셀 로봇 진화에서는 탐색의 질뿐 아니라 한정된 시뮬레이션 예산을 어디에 배분할 것인지가 핵심적인 방법론적 문제이다.

## 1.2. 초기 자세가 표현형 평가에 미치는 영향

복셀 로봇의 이동 성능은 형태와 작동 패턴만으로 결정되지 않는다. 동일한 몸체도 어느 면을 지면 쪽으로 두고 시작하는지에 따라 초기 접촉점, 무게중심의 높이, 안정적인 착지 여부 및 마찰력의 방향이 달라질 수 있다. 따라서 하나의 고정된 초기 자세만 평가하면, 잠재적으로 유망한 형태가 우연히 불리한 면으로 놓였다는 이유로 낮은 적합도를 받을 수 있다. 반대로 우연히 유리한 자세를 가진 개체는 형태 자체의 일반적 우수성보다 초기 배치의 이점을 통해 선택될 수 있다. 이 경우 진화 알고리즘은 형태와 제어의 질뿐 아니라 평가 좌표계에 대한 우연한 정렬까지 동시에 탐색하게 된다.

가장 직접적인 해결책은 모든 개체를 여러 초기 자세에서 시뮬레이션하고 가장 좋은 결과를 사용하는 것이다. 본 연구에서 고려한 이산 자세 집합은 원래 자세와 다섯 개의 90° 또는 180° 회전으로 구성된 여섯 개의 **face orientation**이다. 이 집합은 로봇의 서로 다른 여섯 면이 중력 및 지면과 마주하도록 만들지만, 수직축 주위의 모든 yaw를 열거하는 24개 정육면체 회전 전체를 포함하지는 않는다. 모든 개체를 여섯 방향에서 끝까지 시뮬레이션하면 초기 자세 편향은 줄일 수 있지만 계산 비용이 최대 여섯 배까지 증가할 수 있다. 또한 서로 다른 초기 회전이 settling 이후 동일한 면으로 착지하거나, 초기 몇 초의 운동만으로 성능이 낮은 방향이 명확해지는 경우에도 불필요한 중복 계산이 발생한다.

## 1.3. 선택적 회전평가와 유전형 정준화

본 연구는 다방향 평가의 장점을 유지하면서 비용을 제한하기 위해 세 가지 수준의 선택을 결합하였다. 첫째, 부모와 자식의 점유 복셀 형태가 충분히 달라진 경우에만 자식을 여섯 방향 회전평가 대상으로 지정하였다. 둘째, 회전평가 대상으로 지정된 개체에서도 settling 후 동일한 착지 상태로 수렴한 방향들을 중복으로 묶어 하나만 남겼다. 셋째, 중복 제거 후 남은 방향들을 짧은 추가 구간 동안 평가하고 초기 이동 거리가 가장 큰 하나만 전체 시뮬레이션 길이까지 진행하였다. 따라서 여섯 방향은 후보 생성 단계에서는 모두 고려되지만, 모든 방향이 전체 길이의 평가를 받지는 않는다.

회전평가에서 발견한 최적 방향은 일회성 평가 정보로 폐기하지 않고 CPPN의 좌표 입력 연결에 직접 반영하였다. 구체적으로, 좌표축 입력에서 나가는 연결의 가중치와 활성 상태를 회전행렬에 대응하는 signed permutation으로 교환하거나 부호 반전하였다. 이 연산은 물리 엔진에서 몸체를 다시 회전시키는 것이 아니라, 회전된 표현형이 이후부터 CPPN의 기준 자세에서 생성되도록 유전형을 재매개화한다. 평가 중 획득한 최적 방향이 게놈에 기록되어 후손에게 전달되므로, 본 연구에서는 이를 **Lamarckian orientation write-back** 또는 **genotypic canonicalization**으로 부른다. 이 과정은 경사하강이나 역전파가 아니라, 좌표계 변환에 대한 정확한 대수적 재표현이다.

## 1.4. 연령–적합도 기반 다양성 유지

본 구현에서는 전통적인 NEAT speciation을 실제 부모 선택에 사용하지 않고, fitness 최대화와 lineage age 최소화를 동시에 고려하는 Age–Fitness Pareto Optimization(AFPO)을 사용하였다. AFPO는 매 세대 새로운 무작위 게놈을 주입하고, 높은 적합도뿐 아니라 낮은 연령도 선택상 이점으로 인정함으로써 오래된 국소 최적해와 새로운 탐색 계통이 공존하도록 한다 [Schmidt and Lipson, 2010]. 이 선택 방식은 회전 정준화로 좌표 입력 가중치가 크게 바뀌더라도 유전적 거리 척도 자체가 교란되지 않는다는 장점이 있다.

## 1.5. 연구 목적

본 연구의 목적은 선택적 회전평가가 고정 자세 평가와 비교하여 (i) 동일한 시뮬레이션 예산에서 최고 성능 및 상위 fitness 분포를 개선하는지, (ii) 전복 또는 수치적 파손과 같은 물리적 실패를 줄이는지, (iii) 모든 개체를 여섯 방향으로 완전 평가하는 방식보다 훨씬 적은 추가 비용으로 이러한 효과를 달성하는지를 검증하는 것이다. 이를 위해 Rot과 NoRot 조건을 동일 seed 및 population size에서 대응시켜 실행하였으며, 성능 비교는 세대 수가 아니라 누적 시뮬레이션 비용을 기준으로 수행하였다. 구체적인 데이터 처리와 통계 분석은 다음의 분석 방법 절에서 기술한다.

---

# 2. Materials and Methods — 진화 시스템과 회전평가 구현

## 2.1. 전체 시스템 구조

진화 시스템은 Python 기반의 유전 알고리즘 계층과 C++ 기반의 물리 평가 계층으로 구성하였다. Python 계층은 CPPN 게놈의 초기화, 표현형 디코딩, 회전평가 대상 결정, AFPO 부모 선택, 교배와 돌연변이, 그리고 세대별 로깅을 담당하였다. C++ 계층은 Python에서 전달된 복셀 출력 배열을 물리 시뮬레이터의 개체로 변환하고, 형태 유효성 검사, 다방향 시간 적분, 착지 중복 제거, early pruning, 이동 거리 및 복합 fitness 계산을 수행하였다. 두 계층은 `ctypes` 구조체를 통해 연결되었으며, C++ DLL은 개체별 fitness, 선택된 방향 index 및 회전 파라미터를 Python으로 반환하였다.

DLL 초기화 시 Python은 최대 thread 수, 로봇 종류, 표현형 parser 종류, oscillator 수 및 총 시뮬레이션 step 수를 전달하였다. 반대로 DLL은 실제 복셀 격자의 세 축 크기, CPPN input 수 및 output 수를 반환하였다. 따라서 격자 크기와 CPPN 차원은 Python 파일에 하드코딩되지 않고 C++ 초기화 결과에 의해 결정되었다. 각 CPPN은 반환된 모든 격자 좌표 $(x,y,z)$에 대해 한 번씩 질의되었고, 결과는 `float32` 배열

$
\mathbf{Y}\in\mathbb{R}^{N_x\times N_y\times N_z\times C}
$

로 저장되었다. 여기서 ($N_x,N_y,N_z$)는 DLL이 반환한 격자 크기이고 ($C$)는 CPPN output channel 수이다.

### 조건 정의

본 연구의 두 실험 조건은 동일한 CPPN–NEAT/AFPO와 물리 모델을 사용하고 회전평가만 달리하였다.

| 조건 | 회전 전략 | 개체당 기본 평가 | 최적 방향의 게놈 반영 |
|---|---|---:|---|
| **NoRot** | `ENABLE_INPUT_ROTATION=False` | 현재 정준 자세 1개 | 없음 |
| **Rot** | `ENABLE_INPUT_ROTATION=True`, discrete `RotEdge` | 형태 변화가 임계값을 넘은 개체에 여섯 방향 후보 평가 | 최종 선택 방향을 CPPN 좌표 입력 연결에 write-back |

연속 quaternion 기반의 C++ 물리 회전 경로도 구현되어 있으나, 제공된 설정에서는 `ENABLE_QUAT_ROT=False`였으므로 본 연구의 Rot 조건은 이산 `RotEdge` 방식만을 사용하였다.

## 2.2. CPPN 유전형과 표현형 디코딩

### 2.2.1. 초기 네트워크 구조

초기 CPPN은 hidden node 없이 모든 input node가 모든 output node에 연결된 feed-forward network로 생성되었다. Input node는 음수 key, output node는 0부터 시작하는 nonnegative key로 구분하였다. 각 연결 가중치와 각 output node bias는 평균 0, 표준편차 1의 정규분포에서 초기화하였다. 초기 output activation은 sigmoid 또는 Gaussian 함수 중 하나를 무작위로 선택하였으므로 output은 현재 구현에서 주로 $[0,1]$ 범위에 놓였다.

Hidden node에 허용된 activation function은 sigmoid, hyperbolic tangent, ReLU, absolute value, sine, cosine, Gaussian ($\exp(-z^2/4)$), identity, $\sqrt{|z|}$, 그리고 코드에서 `square`로 명명된 square-wave형 함수 ($1+\lfloor\sin z\rfloor$)였다. Output node의 activation 후보는 sigmoid와 Gaussian으로 제한하였다. 모든 node는 incoming signal의 합에 bias를 더한 뒤 activation을 적용하였다.

### 2.2.2. 좌표 질의와 복셀 channel

각 격자 index는 C++의 `Mod_CPPN_Input` 함수를 통해 CPPN input vector로 변환되었다. 회전 코드가 input key (-1,-2,-3)을 각각 세 Cartesian coordinate 축으로 취급하므로, 이 세 input에서 나가는 연결만 이산 회전 시 교환 또는 부호 반전되었다. Radius 또는 기타 부가 input이 존재하는 경우에는 회전 연산의 영향을 받지 않았다. 제공된 파일에는 `Mod_CPPN_Input`의 본문이 포함되어 있지 않으므로, 좌표 정규화 범위와 추가 input의 정확한 구성은 최종 DLL source에서 확인해야 한다.

회전평가 대상 선정을 위한 Python의 SINE phenotype parser는 raw CPPN output을 다음과 같이 해석하였다.

| Channel | Python-side 해석 | 사용 방식 |
|---:|---|---|
| 0 | occupancy | output (>0.5)이면 occupied voxel |
| 1 | muscle indicator | occupied voxel 중 output (>0.5)이면 muscle |
| 2 | actuation amplitude | muscle voxel에서만 저장 |
| 3 | actuation phase | muscle voxel에서만 저장; 보조 phasor 거리에서는 ([0,1]\to[0,2\pi]) 변환 |

현재 회전 gate에서는 occupancy channel만 실제 거리 계산에 사용하였으며 muscle, amplitude 및 phase 기반 거리는 비활성화하였다. 물리 시뮬레이터에서 이 raw tensor를 실제 material 및 작동 파라미터로 변환하는 `Make_VoxParam_from_CPPN`의 본문은 제공된 파일에 포함되어 있지 않다. 따라서 논문 최종본의 재료 상수, 진폭 범위 및 phase-to-actuation 변환은 해당 C++ source와 함께 보완해야 한다.

## 2.3. 여섯 face orientation의 생성

### 2.3.1. 방향 집합

Rot 조건에서 사용하는 방향 index는 다음과 같다.

| 방향 index | 코드 label | `Rot_Edge` 연산의 핵심 | 코드 주석의 physical coordinate map |
|---:|---|---|---|
| 0 | identity | 변경 없음 | ((x,y,z)) |
| 1 | `y(180)` | (x) 및 (z) input edge weight 부호 반전 | ((-x,y,-z)) |
| 2 | `x(+90)` | (y,z) input edge 속성 교환 후 새 (y) 쪽 부호 반전 | ((x,-z,y)) |
| 3 | `x(-90)` | (y,z) input edge 속성 교환 후 새 (z) 쪽 부호 반전 | ((x,z,-y)) |
| 4 | `y(+90)` | (x,z) input edge 속성 교환 후 새 (z) 쪽 부호 반전 | ((z,y,-x)) |
| 5 | `y(-90)` | (x,z) input edge 속성 교환 후 새 (x) 쪽 부호 반전 | ((-z,y,x)) |

이 집합은 중력 방향에 대해 가능한 여섯 면을 탐색하기 위한 것이며, 같은 면을 아래로 둔 상태에서의 네 가지 yaw를 모두 탐색하지 않는다. 즉, 본 연구의 “6방향 회전”은 24개 proper cube rotation에 대한 완전한 orientation invariance가 아니라 **ground-facing face selection**이다.

### 2.3.2. 좌표 변환의 게놈 구현

CPPN의 한 node가 coordinate input vector ($\mathbf{q}=[x,y,z]^\top$)에 대해 선형 pre-activation

$
\mathbf{h}=W\mathbf{q}+\mathbf{b}
$

을 계산한다고 하자. 물리적으로 회전행렬 (R)만큼 회전된 표현형을 현재 좌표 ($\mathbf{q}$)에서 생성하려면 원래 CPPN을 inverse-rotated coordinate ($R^{-1}\mathbf{q}$)에서 질의하면 된다.

$
\mathbf{h}_{R}=W R^{-1}\mathbf{q}+\mathbf{b}.
$

따라서 coordinate input에 연결된 weight block을

$
W_R=W R^{-1}=W R^{\top}
$

로 바꾸면 동일한 결과를 얻는다. 본 연구의 회전은 90°와 180°의 signed permutation이므로 일반적인 행렬곱을 수행할 필요 없이 coordinate input edge들을 교환하고 필요한 축의 부호를 반전하면 된다. 이 치환은 모든 coordinate input의 outgoing edge에 적용되므로, 이후에 임의의 nonlinear activation이 존재하더라도 원래 CPPN을 회전된 coordinate에서 질의한 것과 같은 pre-activation을 재현한다.

이 연산은 손실함수의 gradient를 계산하는 학습이나 back-propagation이 아니다. 이미 결정된 직교 좌표변환을 게놈의 첫 coordinate coupling에 정확히 흡수하는 deterministic reparameterization이다.

### 2.3.3. Sparse connection과 crossover key 보존

`swap_edges`는 두 coordinate input에서 동일 destination node로 향하는 edge가 모두 존재할 경우 edge dictionary key 자체를 바꾸지 않고 weight와 active flag만 교환하였다. 이는 회전 때문에 homologous gene의 key가 불필요하게 바뀌어 crossover 정렬이 깨지는 것을 줄이기 위한 구현이다. 한쪽 coordinate input edge만 존재하는 sparse case에서는 해당 edge를 반대 input key로 이동시켰다. 또한 edge deletion 연산은 각 input의 마지막 outgoing edge와 각 output의 마지막 incoming edge를 삭제 후보에서 제외하였다. 초기 network가 fully connected이고 이러한 보호 규칙을 사용하므로, coordinate transformation이 적용될 수 있는 기본 연결성은 대부분 유지되었다.

## 2.4. 회전평가 대상의 선택적 지정

### 2.4.1. 전략 코드

Python과 C++ 사이에서 사용한 orientation strategy code는 다음과 같다.

| 값 | 의미 | 본 연구에서의 사용 |
|---:|---|---|
| `-2` | 여섯 개의 `RotEdge` phenotype을 생성하여 탐색 | Rot의 search 대상 |
| `-1` | 원 phenotype 하나를 보내고 C++에서 quaternion/physical rotation | 구현은 존재하나 비활성 |
| `0` | 현재 정준 자세만 평가 | NoRot 전체 및 Rot의 keep 대상 |
| `1–5` | 특정 이산 방향을 고정 평가 | active marking에서는 직접 사용하지 않음 |

첫 세대에서 `max_rot_kind`가 아직 비어 있으면 Rot 조건의 모든 개체는 기본적으로 `-2`로 지정되었다. 이후 세대에서는 reproduction 직후 부모–자식 morphology distance를 계산하여 다음 세대의 search 여부를 정하였다.

### 2.4.2. Bounding-box center alignment

부모와 자식의 occupancy mask를 비교하기 전에 각 mask의 occupied bounding box를 잘라낸 뒤 원래 크기의 3차원 canvas 중앙에 다시 배치하였다. 이는 CPPN 출력의 단순 평행이동이 morphology distance를 과도하게 키우는 것을 줄이기 위한 처리이다. 사용한 정렬은 center-of-mass 또는 PCA 정렬이 아니라 **axis-aligned occupied bounding-box centering**이다. Largest connected component 필터와 24개 직교 회전 중 최적 정렬은 구현되어 있으나 현재 설정에서는 모두 비활성화하였다.

### 2.4.3. Body Jaccard distance

중앙정렬된 부모 occupancy를 (A), 자식 occupancy를 (B)라 할 때 body Jaccard distance를

$d_J(A,B)=1-\frac{|A\cap B|}{|A\cup B|}$

로 계산하였다. 두 mask가 모두 비어 있으면 거리는 0으로 정의하였다. 현재 gate에서는 surface weighting, bottom-voxel weighting, full-grid Hamming distance, muscle Jaccard distance, muscle Hamming distance 및 actuation phasor distance를 모두 끄고 body Jaccard만 사용하였다. 따라서 활성 component의 정규화 weight는 1이었다.

Mutation-only offspring는 유일한 부모와 비교하였다. Crossover offspring는 두 부모 각각과의 거리를 구한 뒤 더 닮은 부모에 대한 최소 거리

$d_{\mathrm{child}}=\min\{d_J(P_1,C),d_J(P_2,C)\}$

를 사용하였다. 이 값이 (0.20)을 **초과**하면 여섯 방향 search((-2)), 그렇지 않으면 현재 orientation 유지((0))로 표시하였다. Body Jaccard만 활성화된 현재 설정에서 이는 선택된 부모와의 intersection-over-union이 0.8 미만일 때 회전 search를 수행하는 것과 같다.

Global elite는 거리와 관계없이 현재 orientation을 유지하도록 강제하였고, 부모가 없는 fresh random genome은 비교 대상이 없으므로 항상 full rotation search로 지정하였다. Parent phenotype을 찾지 못하거나 morphology-distance 계산에서 예외가 발생하면 보수적으로 search를 수행하거나 legacy ancestry rule로 fallback하였다. Legacy fallback에서는 elite와 crossover offspring는 keep, mutation-only 및 random offspring는 search로 지정하였다.

회전 search 표식은 fitness 자체가 아니며 부모 선택에 직접 사용되지 않았다. 이는 다음 세대의 물리 평가에 얼마의 orientation budget을 배정할지만 결정하는 계산 자원 gate이다.

## 2.5. 형태 유효성 검사와 정적 형태 지표

각 개체의 동적 평가 전에 C++에서 phenotype을 한 번 구성하여 형태 유효성과 fitness에 사용될 정적 지표를 계산하였다. `valiationCheck`의 주석과 반환값 사용 방식에 따르면 복셀 수가 5 미만이면 (-1000) sentinel을 반환하고, 형태가 연결되지 않았으면 음수 값을 반환한다. `Calc_Connectivity_Penalty`가 반환한 값은 코드 변수명과 달리 최종 fitness에 더해지는 connectivity score로 사용되었다. 개체가 유효하다고 판정되려면 이 값이 0.999보다 크고, occupied voxel과 muscle voxel이 각각 하나 이상이어야 했다. 이 조건을 충족하지 못한 개체는 `shape_fail`로 표시하여 최종 fitness를 0으로 설정하였다.

제공된 파일에는 `valiationCheck`와 `Calc_Connectivity_Penalty`의 함수 본문이 없으므로, 연결성의 정확한 neighborhood 정의와 oscillator–muscle/sensor connectivity 조건은 최종 C++ source에서 보완해야 한다. 다만 호출부 주석은 SINE phenotype에서 connectivity score가 통상 1이고, CE phenotype에서는 oscillator별 motor–sensor 연결을 확인하는 용도임을 나타낸다.

### 2.5.1. Occupancy-density scale

전체 격자에서 occupied voxel의 비율을

$
\rho=\frac{N_{\mathrm{vox}}}{N_xN_yN_z}
$

로 정의하고, density scale을

$
P(\rho)=\max\left\{0,\;1-\left[1.835(\rho-0.5)\right]^8\right\}
$

로 계산하였다. Source에는 먼저 4차식이 할당되지만 바로 다음 문장에서 8차식으로 덮어쓰기 때문에 실제 fitness에는 위의 8차식만 사용된다. 이 scale은 occupancy가 0.5 근처일 때 최대가 되며, 너무 희박하거나 너무 조밀한 형태에서 weighted performance block을 낮춘다.

### 2.5.2. Muscle 및 sensor sparsity score

Muscle과 sensor의 상대적 희소성을 각각

$
M=\operatorname{clip}\left(1-\frac{N_{\mathrm{muscle}}}{N_{\mathrm{vox}}},0,0.8\right),
\qquad
S=\operatorname{clip}\left(1-\frac{N_{\mathrm{sensor}}}{N_{\mathrm{vox}}},0,0.8\right)
$

로 계산하였다. 따라서 두 값은 해당 기능성 voxel 비율이 낮을수록 커지고 최대 0.8에서 포화된다. SINE fitness에서는 (S)가 사용되지 않고, CE fitness에서만 포함된다.

### 2.5.3. Convex-hull branching score

Occupied voxel의 convex-hull volume을 (V_{\mathrm{hull}})이라 할 때

$
B=\operatorname{clip}\left(1-\frac{N_{\mathrm{vox}}}{V_{\mathrm{hull}}},0,0.7\right)
$

을 계산하였다. Source에서는 이를 convex-hull ratio 또는 branching index로 지칭하였다. 동일한 외곽 hull 안에서 실질 점유가 낮은 형태일수록 값이 커지며, 0.7에서 상한을 두었다. 이 값은 최소 다섯 복셀을 갖고 기본 validation을 통과한 경우에 계산하였다.

## 2.6. 물리 시뮬레이션의 시간 구조와 이동 거리

### 2.6.1. Burn-in, flip horizon 및 총 길이

C++는 parser 종류에 따라 burn-in을 SINE에서 1 s, CE에서 3 s로 설정하고, flip 판정을 위한 duration parameter를 2 s로 설정하였다. 이 값들은 physics time step으로 나누어 integer step 수 (BURN\_DURATION)과 (FLIP\_DURATION)으로 변환되었다. C++ 내부에는 SINE 20 s 및 CE 13 s의 nominal total duration도 존재하지만, 계산 직후 Python에서 전달된 `DUR` 값으로 덮어쓰기 때문에 실제 총 평가 길이는 runtime configuration의 integer step 수가 결정하였다.

Rot search의 early checkpoint는 burn-in 종료 후 5,000 physics step에 위치하였다.

$
t_{\mathrm{early}}=BURN\_DURATION+5000.
$

따라서 실제 `DUR`는 이 checkpoint보다 커야 early pruning이 동작한다. 제공된 `g_config.py` snapshot의 `DURATION=1000`은 이 조건과 모순되므로, 결과를 생성한 실행본의 실제 duration을 로그 또는 launch configuration에서 반드시 확인해야 한다.

### 2.6.2. 이동 거리

모든 조건에서 burn-in 시점에 center of mass를 다시 기록하여 settling 과정의 변위를 fitness에서 제외하였다. 이후 이동 거리는 시작 및 현재 center-of-mass vector에서 (y) 성분을 0으로 만든 뒤 Euclidean norm을 계산하였다.

$
D(t)=\sqrt{[x(t)-x(t_b)]^2+[z(t)-z(t_b)]^2},
$

여기서 (t_b)는 burn-in 종료 시점이다. 즉, 현재 구현은 (y)축 변위를 무시한 평면 이동 거리를 사용한다. 좌표계에서 (y)축이 실제 수직축인지 여부는 simulator coordinate convention과 함께 최종본에서 명시해야 한다. NaN 거리는 0으로 대체하였다.

### 2.6.3. 물리적 실패

각 step에서 simulator update 함수의 반환값을 `is_blow`로 기록하고, `Check_Orient_Flip`의 결과를 `is_flip`으로 기록하였다. 또한 phenotype에 muscle voxel이 하나도 없으면 즉시 실패로 처리하였다. 실패가 발생하면 그 시점까지의 이동 거리를 저장하고 해당 simulation slot을 중단하였다. Rot search의 여섯 thread가 서로 다른 시점에 실패해도 synchronization barrier가 deadlock되지 않도록, 실패 slot은 아직 도달하지 못한 stage에 (-1) sentinel을 보고하였다. `Loop_All`의 numerical blow 기준과 `Check_Orient_Flip`의 각도/시간 threshold 본문은 제공된 파일에 없으므로 최종 simulator source에서 보완해야 한다.

## 2.7. 다단계 회전평가

Rot search로 표시된 한 개체에 대해서는 여섯 phenotype slot을 생성하고 각 slot을 별도 simulation으로 실행하였다. 평가 과정은 두 개의 atomic synchronization stage와 최종 장기 평가로 구성되었다.

### 2.7.1. Stage 0: settling과 착지 자세 기록

여섯 방향은 burn-in까지 병렬로 진행되었다. Burn-in 전에 blow, flip 또는 muscle 부재가 발생한 slot은 물리 실패로 표시하고 landing orientation을 기록하지 않았다. 정상 slot은 burn-in 시점에 `Check_Landing_Kind`를 호출하여 로봇의 local up orientation을 얻고, 해당 vector를 shared result에 기록하였다. 모든 여섯 slot이 stage 0에 도착하면 atomic counter를 이용한 barrier가 해제되고 착지 중복 처리를 수행하였다.

### 2.7.2. 중복 착지 자세 제거

중복 그룹화 전에 이미 실패한 slot과 기존에 redundant로 표시된 slot은 별도의 물리 실패로 남겨 두고 중복 후보에서 제외하였다. 정상적으로 착지한 두 방향 ($i,j$)는 local-up vector의 (z) 성분이

$
|u_{i,z}-u_{j,z}|<0.015
$

을 만족하면 동일 착지 그룹으로 간주하였다. 한 그룹에 둘 이상의 방향이 포함되면 `rand() % group_size`를 사용하여 한 방향을 균등하게 선택하고, 나머지를 redundant로 표시하여 즉시 중단하였다.

이 구현은 full 3D orientation 또는 yaw를 비교하는 것이 아니라 local-up의 단일 (z) 성분만으로 equivalence class를 구성한다. 따라서 논문에서는 이를 “완전히 동일한 6-DoF pose”가 아니라 **동일한 effective landing class**로 기술하는 것이 정확하다. 무작위 survivor를 사용한 이유는 같은 착지 class에서 항상 낮은 rotation index가 살아남는 결정론적 편향을 피하기 위함이다.

### 2.7.3. Stage 1: early performance pruning

중복 제거 후 남은 방향들은 burn-in 이후 추가 5,000 step 동안 진행하였다. 이 시점의 center-of-mass distance를 `early_dist`로 기록하고 두 번째 atomic barrier에서 비교하였다. Redundant slot과 음수 sentinel을 가진 실패 slot은 후보에서 제외하였고, (10^{-4})의 tolerance를 두어 가장 큰 early distance를 가진 방향을 선택하였다. 차이가 tolerance 이내이면 loop에서 먼저 등장한 작은 방향 index가 유지되었다.

모든 방향이 실패하여 유효 후보가 없으면 orientation 0을 fallback survivor로 되살리고 early distance를 0으로 설정하였다. 선택된 하나를 제외한 모든 방향은 redundant로 표시하고 중단하였다. Winner는 전체 `DUR`까지 계속 시뮬레이션되었으며, 구현상 중단된 다섯 방향에서 해제된 계산 자원을 활용하도록 내부 physics thread 수를 6으로 재설정하였다.

### 2.7.4. Final evaluation

전체 duration에 도달하거나 물리 실패로 종료되면 burn-in 이후 최종 center-of-mass distance를 저장하였다. Fitness aggregation에서는 redundant, blow 및 flip slot을 모두 제외하였다. SINE 조건에서는 살아남은 방향의 최대 거리를 locomotion score로 사용하였다. Early pruning 이후 일반적으로 유일한 survivor만 full length에 도달하므로, 이 값은 선택된 방향의 최종 이동 거리와 동일하다.

### Algorithm 1. 한 개체의 선택적 회전평가

```text
Input: CPPN genome g, strategy s ∈ {0, -2}
Output: fitness F, best orientation r*

if s = 0:
    evaluate current canonical phenotype once
else:
    generate six signed-permutation variants g0...g5
    run all six until burn-in or physical failure
    synchronize stage 0
    exclude failures
    group normal slots by |local_up_z(i)-local_up_z(j)| < 0.015
    randomly retain one slot from each duplicate group

    run remaining unique slots for another 5000 steps
    synchronize stage 1
    retain the slot with maximal early displacement
    stop all other slots
    run the winner to the full duration

exclude redundant/blow/flip slots
compute locomotion and shape-based fitness
return fitness and the maximal-distance orientation index
```

## 2.8. Fitness function

### 2.8.1. Orientation aggregation

한 개체에서 최종적으로 유효한 방향 집합을

$
\mathcal{V}=\{r:\mathrm{redundant}_r=0,\;\mathrm{blow}_r=0,\;\mathrm{flip}_r=0\}
$

로 정의하였다. (\mathcal{V})가 비어 있으면 locomotion score를 0으로 두었다. 그렇지 않으면

$
D_{\max}=\max_{r\in\mathcal V}D_r,
\qquad
\bar D=\frac{1}{|\mathcal V|}\sum_{r\in\mathcal V}D_r
$

를 계산하였다. SINE parser에서는 ($D=D_{\max}$), CE parser에서는 ($D=\bar D$)를 사용하였다. Best orientation index는 parser와 관계없이 ($D_r$)가 가장 큰 방향으로 정하였다. 최종 거리 차이가 ($10^{-6}$) 이하이면 먼저 검사된 작은 index가 유지되었다.

### 2.8.2. Competence-gated composite fitness

Connectivity score를 (Q), density scale을 (P), branching score를 (B), muscle sparsity를 (M), sensor sparsity를 (S)라 하였다. Shape term의 활성 여부는 orientation aggregation에 사용된 (D)가 아니라 원시 최대 거리 ($D_{\max}$)가 0.4를 초과하는지로 결정하였다.

SINE parser의 fitness는 다음과 같다.

$
F_{\mathrm{SINE}}=
\begin{cases}
P D + Q, & D_{\max}\le 0.4,\$4pt]
P\left(\dfrac{7D+2B+M}{10}\right)+Q, & D_{\max}>0.4.
\end{cases}
$

CE parser의 fitness는

$
F_{\mathrm{CE}}=
\begin{cases}
P D + Q, & D_{\max}\le 0.4,\$4pt]
P\left(\dfrac{6D+3B+M+S}{11}\right)+Q, & D_{\max}>0.4
\end{cases}
$

로 계산하였다. 따라서 ($D_{\max}>0.4$) threshold는 최소한의 이동 능력이 나타나기 전에는 branching 또는 material-sparsity term이 선택을 지배하지 못하게 하는 competence gate로 작동한다. (Q)는 weighted block에 곱해지는 것이 아니라 마지막에 더해지므로 fitness는 1을 초과할 수 있다.

복셀 수가 5 미만인 sentinel case는 최종 fitness를 0으로 강제하였다. `valiationCheck`가 disconnected morphology를 음수로 표시한 경우에는 위에서 계산한 fitness에 0.01을 곱하였다. `shape_fail`은 fitness 0이었다.

구현상 주의할 점은, 형태 validation을 통과했지만 모든 orientation이 blow/flip/redundant로 제외된 경우 ($D=0$)일 뿐 additive connectivity score (Q)는 남는다는 것이다. 즉, 이러한 개체의 fitness가 자동으로 0이 되지는 않는다. 이것이 연구 설계의 의도인지 최종 제출 전에 확인해야 하며, 의도와 다르다면 “유효 orientation이 없을 때 ($F=0$)” 규칙으로 수정한 뒤 결과를 다시 생성해야 한다.

## 2.9. 최적 방향의 Lamarckian write-back

C++ 평가가 끝나면 각 개체에 대해 fitness와 best orientation index ($r^*\in\{0,\ldots,5\}$)를 Python으로 반환하였다. 입력 strategy가 (-2)였던 개체에는

$
g\leftarrow\mathrm{RotEdge}(g,r^*)
$

를 즉시 적용하였다. 따라서 이번 평가에서 가장 잘 움직였던 방향의 phenotype이 이후에는 orientation 0에서 생성된다. 다음 세대에 이 개체가 elite로 복사되거나 자식의 부모가 되면 정준화된 coordinate frame이 그대로 상속된다. 이는 acquired orientation을 genotype에 기록한다는 operational sense에서 Lamarckian update이다.

이산 write-back은 별도의 angle gene을 추가하지 않으며, NEAT가 이미 진화시키는 coordinate input edge를 재배치한다. 반면 비활성 quaternion 경로에서는 best angle과 axis를 별도 genome field에 저장하도록 구현되어 있다. 본 연구 결과는 전자에만 해당한다.

`RotKindStrategy=-2`와 `best orientation`은 서로 다른 변수이다. 전자는 평가 전에 정한 “여섯 방향을 탐색할 것인가”를 뜻하고, 후자는 C++ 평가 후 0–5 중 선택된 방향을 뜻한다. 분석에서는 이 둘을 구분하여 search 대상 비율과 최종 선택 방향을 별도로 기록하였다.

## 2.10. AFPO 기반 세대 갱신

### 2.10.1. Age 정의와 Pareto dominance

모든 초기 genome의 age는 0이었다. 한 세대의 평가가 끝난 뒤 reproduction을 시작할 때 기존 population 전체의 age를 1 증가시켰다. Mutation-only 자식은 증가된 부모 age를 그대로 상속하였고, crossover 자식은 두 부모 age의 최댓값을 상속하였다. Fresh random genome만 age 0으로 시작하였다. 따라서 본 구현의 age는 개별 객체가 생성된 이후의 생존 세대 수라기보다, 마지막 random injection 이후 해당 계통에서 탐색이 지속된 기간을 나타내는 **lineage/genotypic age**이다.

개체 (a)가 (b)를 지배하려면

$
f_a\ge f_b,\qquad age_a\le age_b
$

를 모두 만족하고, 두 부등식 중 적어도 하나가 strict해야 했다. 즉, fitness는 최대화하고 age는 최소화하였다.

### 2.10.2. Elite, tournament 및 random injection

각 세대에서 fitness가 가장 높은 ($E=1$)개체를 global elite로 그대로 복사하였다. 나머지 부모 선택은 크기 5의 AFPO tournament로 수행하였다. Population에서 다섯 개체를 무작위 비복원 추출하고 그 안의 nondominated front를 계산한 뒤, front 구성원 중 하나를 균등 무작위로 선택하였다. Population이 다섯 이하이면 전체 population을 tournament로 사용하였다.

매 세대 한 개의 fresh random genome을 주입하였다. 이 genome은 기본 fully connected input–output CPPN으로 생성한 뒤 `mutate_()`를 10회 연속 호출하여 구조와 parameter를 충분히 교란하고 age 0을 부여하였다.

### 2.10.3. Mutation-only와 crossover offspring

Elite와 random injection을 제외한 offspring slot마다 0.5의 확률로 mutation-only reproduction을 수행하였다. 선택된 부모를 deep copy하고 새 genome ID를 부여한 뒤 한 번의 full mutation pass를 적용하였다. 나머지 0.5에서는 두 부모를 독립적으로 AFPO tournament에서 선택하였으며, 같은 부모가 선택되면 최대 다섯 번까지 재추출하였다.

Crossover에서는 fitness가 높은 부모를 primary parent로 두었다. Primary parent의 disjoint node와 edge gene은 그대로 자식에 전달하고, 양쪽에 동일 key가 있는 homologous gene의 weight, active flag, bias, activation 및 기타 속성은 부모 중 하나에서 0.5 확률로 선택하였다. Crossover 직후에도 한 번의 full mutation pass를 적용하였다.

전통적인 NEAT partition/speciation code와 compatibility parameter는 source에 남아 있지만 active `next_generation_afp`는 이를 부모 선택이나 population allocation에 사용하지 않았다. `tk_main.py`에서 초기 partition을 한 번 생성하는 것은 diagnostic/visualization 코드와의 호환을 위한 것이며 세대 갱신은 전적으로 AFPO 경로를 사용하였다.

### Algorithm 2. 한 세대의 갱신

```text
Input: evaluated population P with fitness and lineage age
Output: next population P'

increment age of every member of P
copy the single highest-fitness elite into P'
reserve one slot for a fresh random genome

while offspring slots remain:
    with probability 0.5:
        parent <- AFPO tournament(P, size=5)
        child <- deep copy(parent)
        mutate(child)
        age(child) <- age(parent)
    otherwise:
        p1, p2 <- two AFPO tournaments
        child <- homologous crossover(p1, p2), fitter parent primary
        mutate(child)
        age(child) <- max(age(p1), age(p2))
    add child to P'

create one fresh genome, apply 10 mutation passes, age=0
mark each member of P' as rotation-search or keep using morphology distance
return P'
```

## 2.11. Genetic operators와 hyperparameters

### 2.11.1. 활성 진화 hyperparameters

| 범주 | Parameter | 값 | 적용 단위/의미 |
|---|---|---:|---|
| AFPO | `AFP_TOURNAMENT_SIZE` | 5 | parent tournament당 개체 수 |
| AFPO | `RANDOM_INDIVIDUALS_PER_GEN` | 1 | 세대당 fresh genome 수 |
| Reproduction | `ELITISM` | 1 | global best의 무변이 복사 수 |
| Reproduction | `PERCENTAGE_OFFSPRING_WITHOUT_CROSSOVER` | 0.50 | mutation-only offspring 확률 |
| Topology | `NODE_ADD_PROB` | 0.12 | genome mutation pass당 node 추가 확률 |
| Topology | `NODE_DEL_PROB` | 0.01 | pass당 non-output node 삭제 확률 |
| Topology | `EDGE_ADD_PROB` | 0.20 | pass당 acyclic edge 추가 확률 |
| Topology | `EDGE_DEL_PROB` | 0.05 | pass당 edge 삭제 확률 |
| Edge parameter | `WEIGHT_MUTATE_RATE` | 0.80 | edge별 Gaussian perturbation 확률 |
| Edge parameter | `WEIGHT_REINIT_RATE` | 0.05 | edge별 weight 재초기화 확률 |
| Edge parameter | `WEIGHT_MUTATE_SCALE` | 0.25 | Gaussian perturbation 표준편차 |
| Edge parameter | `WEIGHT_INIT_SCALE` | 1.0 | weight 초기화 표준편차 |
| Edge state | `ACTIVE_MUTATE_RATE` | 0.01 | edge별 active flag toggle 확률 |
| Node parameter | `BIAS_MUTATE_RATE` | 0.70 | node별 bias perturbation 확률 |
| Node parameter | `BIAS_REINIT_RATE` | 0.05 | node별 bias 재초기화 확률 |
| Node parameter | `BIAS_MUTATE_SCALE` | 0.25 | bias perturbation 표준편차 |
| Node parameter | `BIAS_INIT_SCALE` | 1.0 | bias 초기화 표준편차 |
| Node function | `ACTIVATION_MUTATE_RATE` | 0.02 | node별 activation 교체 확률 |
| Node response | `RESPONSE_MUTATE_RATE` | 0.0 | response mutation 비활성 |

Weight와 bias perturbation 후 값은 ($[-30,30]$)으로 clip하였다. Node addition은 무작위 active edge를 비활성화하고 그 사이에 새 node를 삽입하며, incoming new edge의 weight는 1, outgoing new edge의 weight는 기존 edge weight로 두었다. Edge addition은 cycle을 만들거나 output node끼리 직접 연결하는 후보를 거부하였다. Edge deletion은 각 coordinate/input node의 마지막 outgoing edge와 각 output node의 마지막 incoming edge를 보호하였다.

별도 `EDGE_DISABLE_PROB`와 `EDGE_ENABLE_PROB`는 모두 (-1)로 설정되어 해당 genome-level operator는 실행되지 않았다. 다만 `ACTIVE_MUTATE_RATE=0.01`에 의해 각 edge의 active flag는 여전히 parameter mutation 과정에서 toggle될 수 있었다.

### 2.11.2. 현재 선택/분화에 사용되지 않은 parameter

`CUTOFF_PCT=0.2`, `COMPATIBILITY_THRESHOLD=3.0`, node/edge distance coefficient 및 disjoint coefficient는 legacy speciation/partition code에서 사용되지만 active AFPO generation update에는 사용되지 않았다. 논문 본문에서는 실제 선택에 사용된 parameter와 source에 남아 있는 비활성 parameter를 구분하였다.

### 2.11.3. 회전 gate hyperparameters

| Parameter | 값 |
|---|---:|
| Occupancy threshold | 0.5 |
| Bounding-box center alignment | On |
| Body Jaccard | On |
| Largest component filtering | Off |
| Best-of-24 rotation alignment | Off |
| Surface/bottom weighting | Off |
| Muscle/actuation distance | Off |
| Crossover parent reduction | minimum distance |
| Rotation-search threshold | ($d_J>0.20$) |
| Elite forced keep | On |
| Random genome forced search | On |

## 2.12. 병렬 실행과 synchronization

C++ population evaluation은 OpenMP를 사용하였다. Nested parallelism을 허용하고 dynamic thread adjustment를 끈 상태에서 simulation slot들을 병렬로 실행하였다. 각 세대 시작 시 shape validation을 개체 단위로 병렬 수행하고, 회전평가의 stage 0 및 stage 1에서는 개체별 atomic counter를 사용하여 여섯 orientation slot이 모두 결과를 보고할 때까지 barrier를 유지하였다. 대기 thread는 acquire/release memory ordering과 `_mm_pause()`를 사용하는 spin wait를 수행하였다.

NoRot 개체는 runtime thread budget을 개체 수에 나누어 내부 physics thread를 할당하도록 구현되어 있다. Rot search의 winner는 early pruning 후 내부 thread 수를 6으로 증가시킨다. Windows에서는 processor group affinity를 설정하는 코드도 포함되어 있다. 그러나 제공된 `Eval_FuncDLL.cpp` snapshot에서 `ENABLE_ROTATION_TEST`는 `false`로 하드코딩되어 있어 mixed Rot/NoRot thread-group scheduling branch가 비활성일 수 있다. 이 flag는 `max_rot_kind=-2`에 따른 회전평가 자체와는 별개로 scheduling policy를 제어하지만, 실제 benchmark에서 사용한 build의 값을 최종 methods에 명시해야 한다.

Python configuration snapshot의 `NUM_THR`는 10이며 DLL 초기화에 전달된다. 실제 실행 hardware, 논리 core 수 및 OpenMP runtime 설정은 제공된 파일만으로 확정할 수 없으므로 재현성 정보에 추가해야 한다.

## 2.13. 난수와 paired experimental design

Python entry point는 Python `random`과 NumPy random generator에 동일 seed를 설정하였다. 본 연구의 paired design에서는 seed 40–51을 사용하고, 동일 seed와 population size에 대해 Rot과 NoRot을 대응시켰다. Population size는 20, 30 및 50이었다. 두 조건은 회전평가의 활성 여부를 제외한 CPPN, AFPO, mutation, physics 및 logging 설정을 동일하게 유지하도록 설계하였다.

다만 duplicate landing group의 survivor 선택에는 C++ 표준 `rand()`가 사용되며, 제공된 source에서는 이 generator의 seed 초기화가 확인되지 않았다. 따라서 완전한 seed-level 재현성을 위해 C++ RNG seed를 Python seed와 동기화하거나 실행 metadata에 별도로 기록해야 한다.

## 2.14. 로그와 분석 단계로의 연결

각 세대의 population fitness와 누적 simulation cost, 개체별 age와 rotation strategy, orientation slot별 실행 step, redundancy, blow 및 flip flag를 CSV로 기록하였다. Generation-level log는 maximum, mean 및 standard-deviation fitness를 저장하였고, individual/evaluation log는 후속 분석에서 Top-5, P90, age–fitness Pareto front, slot-level failure 및 실제 simulation budget을 재구성할 수 있도록 사용하였다.

Rot은 선택적 다방향 평가로 인해 한 세대당 simulator step이 NoRot보다 많을 수 있으므로, 결과 비교에서는 세대 index가 아니라 누적 `CumTotalSimTime`을 계산 예산으로 사용하였다. Seed–population pair마다 두 조건의 최종 누적 비용 중 작은 값을 common budget으로 정의하는 구체적인 절차는 다음의 데이터 분석 및 통계 방법 절에서 기술한다.

---

---

# 3. Materials and Methods — 데이터 분석 및 통계 방법

## 3.1. 분석 설계와 자료 구성

회전평가 조건(rotation-aware evaluation; **Rot**)과 비회전 조건(**NoRot**)의 진화 성능을 비교하기 위해 random seed 40–51의 12개 독립 반복을 분석하였다. 각 seed에는 population size 20, 30, 50의 두 조건이 모두 포함되었으므로, 전체 자료는 72개 run과 36개의 seed-matched Rot–NoRot pair로 구성되었다. 통계 분석의 기본 독립 단위는 개별 세대나 개체가 아니라 **동일 seed와 동일 population size로 대응된 한 쌍의 run**이었다. 따라서 모든 조건 차이는 먼저 seed별 paired difference로 계산한 뒤 12개 seed에 걸쳐 요약하였다.

각 run에서는 다음 세 종류의 CSV 로그를 사용하였다. `neat_generation_log.csv`는 세대별 fitness와 누적 시뮬레이션 비용을, `neat_generation_log_individual.csv`는 개체별 fitness, age, Pareto-front 표식 및 평가 슬롯 정보를, `neat_generation_log_eval.csv`는 개별 평가 슬롯의 시뮬레이션 시간, 전복·파손 여부와 중복 여부를 제공하였다.

입력 열은 수치형으로 강제 변환하였으며 변환할 수 없는 값은 결측값으로 유지하였다. `CumTotalSimTime`이 없을 때에는 세대별 `GenTotalSimTime`의 누적합으로 복원하였다. 일부 로그에서 signed age가 `Age`로 저장된 경우 이를 `AgeSigned`로 사용하였고, age 기반 분석에는 부호 표식을 제거한

$
\mathrm{AgeAbs}=|\mathrm{AgeSigned}|
$

를 사용하였다. 명시적인 `IsFront` 열이 없을 때에는 로그의 음수 age 표기 규칙에 따라 `AgeSigned < 0`인 개체를 age–fitness Pareto front 구성원으로 간주하였다. 평가 슬롯 분석에서는 실제 시뮬레이션이 수행된 `Time > 0` 행만 포함하였다. 결측값은 별도로 대체하지 않았으며, 계산할 수 없는 관측치는 해당 지표의 분석에서만 제외하였다.

## 3.2. 시뮬레이션 비용에 의한 matched-budget 비교

Rot은 일부 개체를 여러 방향에서 평가하므로 동일 세대 수의 비교는 두 조건에 서로 다른 계산량을 허용할 수 있다. 따라서 진화 비용은 wall-clock time이나 generation count가 아니라 모든 평가 슬롯에서 소비된 simulated timestep의 누적값인 `CumTotalSimTime`으로 정의하였다.

각 seed ($s$)와 population ($p$)에 대해 Rot과 NoRot의 최종 누적 비용을 각각 ($T_{R,s,p}$)와 ($T_{N,s,p}$)라 하고, paired comparison의 공통 예산을 다음과 같이 정의하였다.

$
T_{c,s,p}=\min(T_{R,s,p},T_{N,s,p}).
$

따라서 어느 조건도 상대 조건이 실제로 사용하지 않은 추가 예산으로 평가되지 않았다. 세대별 측정 시점이 두 조건에서 정확히 일치하지 않으므로, 공통 예산 ($T_c$)에서의 지표값은 `CumTotalSimTime` 축의 인접한 두 관측점 사이에서 선형 보간하였다. 모든 endpoint effect는 동일 seed·population의 Rot 값에서 NoRot 값을 뺀 paired difference로 계산하였다.

Fitness endpoint의 상대 변화율은

$
\Delta M_{s,p}(\%)=100\times
\frac{M_{R,s,p}(T_c)-M_{N,s,p}(T_c)}{M_{N,s,p}(T_c)}
$

로 정의하였다. Best-so-far, Top-5 mean 및 P90에서는 양의 값이 Rot 우위를 뜻한다. Failure rate는 절대차

$
\Delta F_{s,p}=F_{R,s,p}-F_{N,s,p}
$

를 사용하였으므로 음의 값이 Rot의 실패 감소를 의미한다. 본문에서는 필요할 때 이 값을 100배 하여 percentage-point 단위로 보고하였다. Failure의 상대 감소율은 보조적 기술통계로서 population별 조건 평균의 비

$
100\times\left(1-\frac{\bar F_R}{\bar F_N}\right)
$

로 계산하였으며, 추론 통계에는 seed별 paired absolute difference를 사용하였다.

## 3.3. 세대별 진화 성능 지표

### 3.3.1. Best-so-far fitness

세대 ($g$)의 best-so-far fitness는 해당 세대까지 관찰된 세대별 최대 fitness의 누적 최댓값으로 정의하였다.

$
B_g=\max_{j\leq g}(\mathrm{MaxFit}_j).
$

이 지표는 AFPO의 세대별 평균 fitness가 개체 주입과 age–fitness 선택으로 인해 단조롭게 증가하지 않을 수 있다는 점을 고려하여, 진화 과정이 발견한 최고 성능을 보존하는 지표로 사용하였다.

### 3.3.2. Top-5 mean과 P90

각 세대의 개체를 fitness 내림차순으로 정렬한 뒤 상위 5개체의 산술평균을 `Top5Mean`으로 계산하였다. Population이 5보다 작은 경우에는 존재하는 전체 개체를 사용하도록 구현하였으나, 본 실험의 population은 모두 20 이상이었다. `P90`은 해당 세대 fitness 분포의 90번째 백분위수로 계산하였으며, 표본 분위수 사이에는 선형 보간을 사용하였다. Top-5 mean은 소수 엘리트의 질을, P90은 population 상위 약 10%의 분포적 품질을 나타내는 상호보완적 지표로 사용하였다.

### 3.3.3. Endpoint와 예산 구간 평균

Best-so-far, Top-5 mean 및 P90에 대해 (i) 공통 예산 종료 시점의 보간값과 (ii) 공통 예산 구간의 budget-normalized area under the curve를 계산하였다. 지표 $M(t)$의 시간평균은 세대별 관측점과 ($T_c$)에서의 보간값에 사다리꼴 적분을 적용하여

$
\overline{M}_{s,p,c}=\frac{1}{T_{c,s,p}}
\operatorname{Trapz}\left\{(t_g,M_g):t_g<T_c\right\}
\cup \left\{(T_c,M(T_c))\right\}
$

로 계산하였다. 따라서 보고된 `AUC(best)` 등은 단위가 누적된 raw area가 아니라 공통 예산으로 정규화된 **예산 구간 평균값**이다. 구현상 적분은 첫 번째 기록 세대부터 시작하였으며 ($t=0$)의 가상 관측값은 추가하지 않았다. 동일 방식으로 Top-5, P90, age–fitness hypervolume 및 failure-rate trajectory의 예산 구간 평균을 계산하였다.

## 3.4. Age–fitness Pareto hypervolume

AFPO가 fitness 최대화와 age 최소화를 동시에 수행한다는 점을 반영하기 위해 세대별 age–fitness hypervolume(HV)을 계산하였다. 각 run 전체에서 관찰된 최대 absolute age와 최대 fitness를 각각 $A_{\max}$, $F_{\max}$라 하고, 개체 $i$의 값을

$
a_i=\operatorname{clip}\left(\frac{\mathrm{AgeAbs}_i}{A_{\max}},0,1\right),
\qquad
f_i=\operatorname{clip}\left(\frac{\mathrm{Fitness}_i}{F_{\max}},0,1\right)
$

로 정규화하였다. Age는 최소화하고 fitness는 최대화하는 2차원 목적공간에서 개체들을 age 오름차순으로 정렬한 뒤, age가 증가할 때 이전 최고 fitness를 엄격히 초과하는 점만 남겨 nondominated envelope를 구성하였다. HV는 이 계단형 envelope가 정규화된 기준영역 ($[0,1]\times[0,1]$)에서 지배하는 면적으로 계산하였다.

$
HV=\sum_{k=1}^{K-1}(a_{k+1}-a_k)f_k+(1-a_K)f_K.
$

세대별 HV trajectory는 공통 예산까지 사다리꼴 적분하고 $T_c$로 나누어 `AUC(HV)`로 요약하였다. 다만 $A_{\max}$와 $F_{\max}$가 각 run 내부에서 별도로 정해졌으므로, HV의 조건 간 비교는 절대 목적공간의 공통 reference point를 사용한 비교가 아니라 run-normalized 탐색 구조에 대한 보조적 비교로 해석하였다.

## 3.5. 전복·파손 실패 분석

평가 로그에서 `Time > 0`인 슬롯을 실제 수행된 평가로 정의하였다. 슬롯 $i$의 failure indicator는

$
I_{\mathrm{fail},i}=I(\mathrm{IsBlow}_i>0\;\lor\;\mathrm{IsFlip}_i>0)
$

로 정의하였다. 각 세대의 전체 슬롯 실패율(`FailRateSlots_All`)은 해당 세대의 실제 평가 슬롯에서 이 지표의 평균으로 계산하였다. 중복 평가 표식이 제공된 경우 `IsRedun <= 0`인 슬롯만을 사용한 unique-slot failure rate도 별도로 계산하여 run-level 진단 그림에 사용하였으나, 조건 간 주 비교에는 전체 수행 슬롯의 실패율을 사용하였다.

공통 예산 동안의 failure outcome은 세대별 `FailRateSlots_All` trajectory를 누적 시뮬레이션 시간에 대해 적분한 budget-normalized mean으로 정의하였다. 따라서 이 값은 모든 원시 슬롯을 단순 합산한 비율이 아니라, **세대별 슬롯 실패율을 시뮬레이션 예산 축에서 시간평균한 값**이다. Rot−NoRot 차이가 음수이면 Rot에서 failure가 감소한 것으로 해석하였다.

## 3.6. 목표 fitness 도달 시간

Best-so-far trajectory에 대해 fitness threshold 1.5, 1.6, 1.7, 1.8, 1.9 및 2.0의 time-to-attainment(TTA)를 계산하였다. 연속된 두 세대 $(t_{g-1},B_{g-1})$와 $(t_g,B_g)$ 사이에서 threshold $h$가 처음 교차된 경우 도달 시간은

$
TTA(h)=t_{g-1}+\frac{h-B_{g-1}}{B_g-B_{g-1}}
(t_g-t_{g-1})
$

으로 선형 보간하였다. 첫 관측 세대에서 이미 threshold 이상이면 첫 세대의 누적 시뮬레이션 시간을 사용하였다. Run 종료까지 threshold에 도달하지 못한 경우는 결측값이 아니라 **right-censored observation**으로 분류하였다.

각 threshold와 population에서 Rot와 NoRot이 모두 도달한 pair에 한해

$
\Delta TTA=TTA_R-TTA_N
$

를 계산하였다. 음의 값은 Rot이 더 빨리 도달했음을 뜻한다. 또한 `Rot only`, `NoRot only`, `both attained`, `neither attained`의 수를 별도로 집계하였다. 정식 survival model이나 censoring-aware rank test를 적용하지 않았기 때문에, jointly attained pair에 대한 평균차와 sign test는 탐색적 분석으로 해석하였다. TTA 그림에서는 threshold 1.6에 도달하지 못한 run이 표시되지 않으므로 해당 그림 역시 기술적 시각화로 사용하였다.

## 3.7. 계산 비용과 평가 효율

계산 비용의 주 지표는 각 run의 최종 `CumTotalSimTime`이었다. Seed별 비용비는

$
R_{\mathrm{cost},s,p}=\frac{T_{R,s,p}}{T_{N,s,p}}
$

로 계산한 뒤 population별로 평균, 중앙값, 범위 및 bootstrap 신뢰구간을 요약하였다. 성능은 공통 예산에서 비교하였지만, 비용비는 두 run이 실제로 소비한 전체 누적 시뮬레이션 비용을 사용하였다.

세대별 평가 구조를 정량화하기 위해 다음 지표를 계산하였다.

$
\mathrm{SimMult}_g=\frac{\sum_i \mathrm{EvalSlots}_{i,g}}{N},
$

$
\mathrm{UniqueRatio}_g=\frac{\sum_i \mathrm{UniqueSlots}_{i,g}}
{\sum_i \mathrm{EvalSlots}_{i,g}},
$

$
\mathrm{FullFracInd}_g=\frac{\sum_i \mathrm{FullSlots}_{i,g}}
{\sum_i \mathrm{EvalSlots}_{i,g}}.
$

여기서 $N$은 population size이다. `SimMult`는 개체당 실제 평가 슬롯 수, `UniqueRatio`는 실행 슬롯 중 비중복 슬롯의 비율을 나타낸다. Slot-level 진단에서는 run 전체의 최대 `Time`의 99% 이상 실행된 평가를 full-length slot으로 간주하였다. 또한 실제 수행 슬롯 중 `IsRedun > 0`인 비율과 blow-or-flip 비율을 run 전체에서 별도로 계산하였다. Run-level 값은 우선 세대별 지표를 산술평균하고, population-level 결과는 12개 run의 평균으로 요약하였다.

## 3.8. 선택적 회전 탐색 메커니즘

Rot 로그의 `RotKindStrategy = -2`를 선택적 회전 탐색 대상으로 표시된 집단으로 정의하였다. 각 세대에서 이 집단의 population 비율을

$
\mathrm{Rot2Frac}_g=\frac{n(\mathrm{RotKindStrategy}=-2)}{N}
$

로 계산하였다. 해당 집단이 AFPO front 또는 상위 fitness 집단에 과대표집되는지를 평가하기 위해 다음 enrichment ratio를 사용하였다.

$
\mathrm{FrontEnrich}_g=
\frac{P(\mathrm{Rot2}\mid\mathrm{Pareto\ front})}
{P(\mathrm{Rot2}\mid\mathrm{population})},
$

$
\mathrm{Top5Enrich}_g=
\frac{P(\mathrm{Rot2}\mid\mathrm{Top5})}
{P(\mathrm{Rot2}\mid\mathrm{population})}.
$

값이 1보다 크면 해당 집단이 기준 population 비율보다 front 또는 Top-5에서 과대표집되었음을 의미한다. 집단의 직접적인 fitness 차이는

$
\mathrm{Rot2MedianAdv}_g=
\operatorname{median}(Fitness\mid Rot2)-
\operatorname{median}(Fitness\mid non\mbox{-}Rot2)
$

로 계산하였다. 이들 지표는 먼저 run 내 세대에 걸쳐 산술평균한 뒤, 동일 population의 12개 Rot run에 걸쳐 평균하였다. 이는 선택 표식과 진화적 위치의 연관성을 보는 관찰적 메커니즘 분석이며 인과적 효과로 해석하지 않았다.

명목상 한 기본 방향과 다섯 추가 방향을 모든 개체에 적용하는 all-rotation scheme을 기준으로, `Rot2Frac`에서 다음 bookkeeping estimate를 도출하였다.

$
N_{search}=N\times\mathrm{Rot2Frac},\qquad
N_{keep}=N-N_{search},
$

$
\mathrm{SavedExtraSlots}=5N(1-\mathrm{Rot2Frac}),
\qquad
\mathrm{SavedFraction}=1-\mathrm{Rot2Frac}.
$

이는 실제 raw slot count를 다시 합산한 값이 아니라, `RotKindStrategy=-2` 표식과 “개체당 추가 5방향”이라는 명목 구조에 근거한 절감량 추정치이다.

## 3.9. 진화 궤적 시각화

Population별 median trajectory를 만들기 위해 해당 population의 Rot 및 NoRot 24개 run이 모두 포함되는 최대 공통 예산, 즉 모든 run의 최종 누적 시뮬레이션 시간 중 최솟값까지를 시각화 구간으로 사용하였다. 이 구간을 200개의 등간격 budget grid로 나누고, 각 run의 Best-so-far, Top-5 mean 및 P90을 선형 보간하였다. 첫 기록 시점보다 이른 grid point에는 첫 관측값을 사용하였다. 각 grid에서 조건별 12개 seed의 pointwise median을 곡선으로, 25번째와 75번째 백분위수를 음영으로 표시하였다. 따라서 음영은 seed 간 interquartile range이며 통계적 신뢰구간이 아니다.

Pairwise-delta 그림에는 각 seed의 matched-budget difference를 개별 점으로 표시하고, population 평균을 diamond marker로 표시하였다. Fitness와 HV에서는 0보다 큰 값이, failure에서는 0보다 작은 값이 Rot 우위를 의미하도록 축의 해석을 명시하였다. 이러한 곡선과 산점도는 기술적 시각화를 위한 것이며, 통계 검정은 pointwise graph 값이 아니라 seed-level paired endpoint 또는 budget-averaged value에 대해 수행하였다.

## 3.10. 통계 분석

### 3.10.1. 기술통계와 bootstrap confidence interval

각 population과 metric에서 12개의 seed-level paired difference에 대해 평균, 중앙값, 표본표준편차, Rot 우세 수, NoRot 우세 수 및 tie 수를 보고하였다. 평균차의 95% confidence interval은 paired difference 벡터 자체를 seed 단위로 복원추출하는 nonparametric percentile bootstrap으로 계산하였다. 각 bootstrap 표본은 원래와 동일한 12개 seed를 복원추출하여 평균을 계산하였고, 10,000회 반복 후 bootstrap mean distribution의 2.5번째와 97.5번째 백분위수를 사용하였다. 자동 분석 파이프라인의 bootstrap random seed는 12,345로 고정하였다.

### 3.10.2. Exact sign test

효과 방향의 seed 간 일관성을 평가하기 위해 two-sided exact sign test를 사용하였다. Tie는 제외하고 Rot 우세 수를 $W$, NoRot 우세 수를 $L$, $n=W+L$이라 하였을 때, 귀무가설 $P(\mathrm{Rot\ win})=0.5$ 아래의 p-value를

$
p=\min\left[1,
2\sum_{k=0}^{\min(W,L)}{n\choose k}0.5^n
\right]
$

로 계산하였다. Best-so-far, Top-5, P90, AUC(best) 및 AUC(HV)는 양의 차이를 Rot win으로, AUC(failure)와 TTA는 음의 차이를 Rot win으로 정의하였다.

### 3.10.3. Wilcoxon signed-rank test와 paired effect size

Paired difference의 크기와 순위를 함께 고려하는 민감도 분석으로 two-sided Wilcoxon signed-rank test를 적용하였다. 결측 pair는 metric별로 제외하고, 0인 차이는 순위 계산에서 제외하였다. SciPy의 자동 exact/asymptotic 선택과 기본 no-continuity-correction 설정을 사용하였다. 효과크기는 benefit-oriented paired Cohen-type standardized difference

$
d_z=\frac{\overline{D}_{benefit}}{s_D}
$

로 계산하였다. 여기서 $s_D$는 paired difference의 표본표준편차이다. Failure에서는 Rot의 감소가 양의 benefit가 되도록 원 차이에 −1을 곱한 뒤 계산하였다.

### 3.10.4. 다중비교 보정

주요 18개 population-by-metric 비교는 세 population과 여섯 지표—Best@budget, Top5@budget, P90@budget, AUC(best), AUC(HV), AUC(failure)—의 조합으로 정의하였다. Exact sign-test p-value 18개와 Wilcoxon p-value 18개에 대해 각각 별도의 Benjamini–Hochberg false-discovery-rate correction을 적용하여 q-value를 계산하였다. Budget-averaged Top-5 및 P90, 여러 threshold의 TTA와 interaction analysis는 사전에 독립적인 확인 검정으로 설계된 것이 아니라 추가 탐색 분석이었으므로, 이들의 unadjusted p-value는 보조적 증거로만 해석하였다. 모든 검정은 two-sided였고 nominal significance level은 0.05로 두었다.

### 3.10.5. Condition × population interaction

Population에 따라 Rot 효과가 달라지는지를 탐색하기 위해 seed를 subject로 하고 condition(Rot, NoRot)과 population(20, 30, 50)을 모두 within-subject factor로 둔 2×3 repeated-measures ANOVA를 별도로 수행하였다. 분석 outcome은 공통 예산의 Best-so-far, Top-5 mean, P90과 공통 예산 구간의 normalized AUC(best), AUC(HV), AUC(failure)였다. 조건별 raw value를 long format으로 구성하고 `statsmodels.stats.anova.AnovaRM`을 사용하여 condition main effect, population main effect 및 condition × population interaction을 산출하였다. 보고된 F test는 sphericity correction을 적용하지 않은 기본 출력이며, 표본 수가 12이고 일부 outcome의 분포가 비정규적일 수 있으므로 이 분석은 paired nonparametric 결과를 보완하는 탐색적 분석으로만 사용하였다.

## 3.11. 분석 소프트웨어와 재현성

모든 분석은 custom Python script로 수행하였다. 데이터 처리와 요약에는 pandas와 NumPy, 통계 검정에는 SciPy와 statsmodels, 시각화에는 Matplotlib을 사용하였다. Run-level 분석은 세대별 성능, failure, cost, age–fitness HV 및 회전 탐색 메커니즘을 산출하였고, dataset-level 분석은 seed-matched common-budget comparison, bootstrap confidence interval, sign test, threshold attainment 및 population-level 집계를 수행하였다. Bootstrap 난수 seed와 interpolation grid 크기를 고정하여 동일 입력에서 재현 가능한 결과가 생성되도록 하였다.

---

## 분석 지표 요약표

| 범주 | 지표 | 계산 단위 | Rot 우위 방향 |
|---|---|---|---|
| 최고 성능 | Best-so-far | 세대별 누적 최대 fitness; 공통 예산 endpoint 및 normalized AUC | 큼 |
| 엘리트 품질 | Top-5 mean | 세대별 상위 5개체 평균 | 큼 |
| 상위 분포 | P90 | 세대별 fitness의 90th percentile | 큼 |
| AFPO 다양성 | HV | run-normalized age–fitness nondominated area | 큼 |
| 안정성 | Failure rate | `Time>0` 슬롯 중 `IsBlow OR IsFlip` 비율의 budget average | 작음 |
| 도달 속도 | TTA(h) | Best-so-far가 threshold를 처음 넘는 simulated time | 작음 |
| 총 비용 | Cost ratio | 최종 `CumTotalSimTime_Rot / CumTotalSimTime_NoRot` | 1에 가까울수록 추가비용 작음 |
| 슬롯 비용 | SimMult | 세대 평가 슬롯 합 / population size | 기술통계 |
| 중복 억제 | UniqueRatio | unique slots / evaluated slots | 기술통계 |
| 회전 선택 | Rot2Frac | `RotKindStrategy=-2` 개체 비율 | 기술통계 |
| Front 연관 | FrontEnrich | front 내 Rot2 비율 / 전체 Rot2 비율 | 1보다 크면 과대표집 |
| Elite 연관 | Top5Enrich | Top-5 내 Rot2 비율 / 전체 Rot2 비율 | 1보다 크면 과대표집 |
| 집단 fitness | Rot2MedianAdv | median fitness(Rot2) − median fitness(others) | 큼 |

---

# 저자용 재현성 점검표 — 논문 제출 전 반드시 확인

아래 항목은 제공된 source snapshot만으로 실제 결과 생성 조건을 확정할 수 없거나, source 간 불일치가 발견된 부분이다. 본 표는 논문 본문에 그대로 넣기보다 최종 실행본과 대조하여 Methods의 숫자를 확정하는 용도로 사용한다.

| 항목 | 제공 source에서 확인된 내용 | 확인이 필요한 이유 |
|---|---|---|
| Voxel task flag | `g_config.py`에는 `NEAT_KIND=0`이지만 voxel evaluation은 C++에서 `NEAT_KIND=1`일 때 초기화됨 | 결과 생성 launch에서 실제 값 확인 필요 |
| Total duration | Python snapshot은 `DURATION=1000`; C++ early checkpoint는 burn-in+5000 step | snapshot 값이면 early pruning이 작동하지 않으므로 실제 run 값이 달랐을 가능성이 큼 |
| Nominal seconds | C++에 SINE 20 s/CE 13 s가 있으나 즉시 Python `DUR`로 override | 논문에는 실제 step 수와 physics time step으로 환산한 초를 보고해야 함 |
| Generation count | snapshot `MAX_GENERATION=100`; 로그는 generation counting 방식상 101개 record가 생길 수 있고 분석 자료의 run 길이도 다를 수 있음 | 실제 stop rule과 기록된 generation 범위를 명시해야 함 |
| Grid/input/output dimensions | DLL 초기화 반환값으로 결정 | 실행 metadata 또는 DLL source에서 정확한 값 필요 |
| Coordinate normalization | `Mod_CPPN_Input` 본문 미제공 | (x,y,z)의 범위와 추가 radial input을 명시해야 함 |
| Material/actuation mapping | Python gate의 channel 의미는 확인되나 `Make_VoxParam_from_CPPN` 본문 미제공 | 재료 탄성, amplitude, phase 변환을 보고해야 함 |
| Physics constants | time step, gravity, friction, damping 및 material constants 미제공 | 물리 재현성에 필수 |
| Blow/flip criterion | 호출 위치만 확인되고 함수 본문 미제공 | failure의 정확한 operational definition 필요 |
| Connectivity score | `Calc_Connectivity_Penalty` 본문 미제공 | (Q)의 의미와 범위 확인 필요 |
| No valid orientation fitness | 구현은 (D=0) 후 additive (Q)를 유지 | 연구 의도와 일치하는지 확인 필요 |
| Duplicate-pose RNG | C++ `rand()` seed 초기화가 보이지 않음 | paired seed 재현성에 영향 가능 |
| Thread scheduling flag | `ENABLE_ROTATION_TEST=false` in supplied C++ snapshot | 결과 생성 binary의 실제 scheduling branch 확인 필요 |
| Hardware | source에 없음 | CPU/model/core/RAM/compiler/OpenMP version 보고 필요 |

---

# 구현 근거 source map

| 내용 | Source 위치 |
|---|---|
| DLL 초기화, grid/input/output 반환, CPPN tensor 생성 | `Run_NEAT.py`, lines 148–278 |
| 여섯 방향 mapping, strategy별 slot 생성, C++ 결과 수집, write-back | `Run_NEAT.py`, lines 285–437 |
| CPPN activation, 초기 fully connected 구조 | `neat_original.py`, lines 27–223 |
| Mutation/crossover 연산 | `neat_original.py`, lines 281–548 and 938–958 |
| Coordinate-edge rotation | `neat_original.py`, lines 552–652 |
| AFPO dominance와 generation update | `neat_next_gen.py`, lines 12–192 |
| Center alignment와 Jaccard | `neat_next_gen_morphdist.py`, lines 82–295 |
| Rotation gate 설정과 threshold | `neat_next_gen_morphdist.py`, lines 580–798 |
| Shape validation descriptor와 fitness | `Evaluate_Robots_RP_Thread.cpp`, lines 95–305 |
| Burn-in, duplicate landing, early pruning, atomic barrier | `Evaluate_Robots_RP_Thread.cpp`, lines 397–653 |
| Active simulation loop와 failure handling | `Evaluate_Robots_RP_Thread.cpp`, lines 728–924 |
| OpenMP population scheduling | `Eval_FuncDLL.cpp`, lines 22–380 |
| Main evolution loop와 logging | `tk_main.py`, lines 557–708 |
| Python/NumPy seed initialization | `tk_main.py`, lines 892–901 |
| Hyperparameter values | `g_config.py`, lines 2–91 |

---

# Suggested references

- Cheney, N., MacCurdy, R., Clune, J., & Lipson, H. (2013). *Unshackling evolution: Evolving soft robots with multiple materials and a powerful generative encoding*. Proceedings of GECCO.
- Hiller, J., & Lipson, H. (2014). Dynamic simulation of soft multimaterial 3D-printed objects. *Soft Robotics, 1*(1), 88–101.
- Schmidt, M. D., & Lipson, H. (2010). Age-fitness Pareto optimization. *Proceedings of GECCO*, 543–544.
- Stanley, K. O. (2007). Compositional pattern producing networks: A novel abstraction of development. *Genetic Programming and Evolvable Machines, 8*(2), 131–162.
- Stanley, K. O., & Miikkulainen, R. (2002). Evolving neural networks through augmenting topologies. *Evolutionary Computation, 10*(2), 99–127.

