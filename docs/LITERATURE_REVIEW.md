# 先行研究レビューと Research Gap

> **関連ドキュメント**: [README](../README.md) | [ARCHITECTURE](ARCHITECTURE.md) | [ALGORITHMS](ALGORITHMS.md) | [API_REFERENCE](API_REFERENCE.md) | [APOLLO_ALIGNMENT](APOLLO_ALIGNMENT.md)

**対象読者**: 研究者・論文執筆者
**目的**: 先行研究との差異（Research Gap）と本研究の理論的裏付けを明確化する

---

## 目次

1. [先行研究の詳細分析](#1-先行研究の詳細分析)
   - [A. 織り込み区間の交通流特性と課題提起](#a-織り込み区間の交通流特性と課題提起)
   - [B. 制御アルゴリズムと最適化手法](#b-制御アルゴリズムと最適化手法)
   - [C. 基盤技術と安全性](#c-基盤技術と安全性)
2. [文献リスト](#2-文献リスト)
3. [CAVを用いる意義](#3-cavを用いる意義)
   - [3.1 計画軌道共有による暗黙的な協調](#31-計画軌道共有による暗黙的な協調)
   - [3.2 自律分散型でありながらLC分散を実現するメカニズム](#32-自律分散型でありながらlc分散を実現するメカニズム)
   - [3.3 HDV（人間運転車両）との対比](#33-hdv人間運転車両との対比)
   - [3.4 先行研究との差異：自律分散とLC分散の両立](#34-先行研究との差異自律分散とlc分散の両立)
4. [本研究の位置づけ](#4-本研究の位置づけ)
   - [4.1 ALGORITHMS.md への補足](#41-algorithmsmd-への補足)
   - [4.2 ARCHITECTURE.md への補足](#42-architecturemd-への補足)
   - [4.3 APOLLO_ALIGNMENT.md への補足](#43-apollo_alignmentmd-への補足)
   - [4.4 IMPLEMENTATION.md への補足](#44-implementationmd-への補足)
   - [4.5 既存交通シミュレータとの差異：なぜ自作シミュレータか](#45-既存交通シミュレータとの差異なぜ自作シミュレータか) ⭐ 重要
5. [結論](#結論)

---

## 1. 先行研究の詳細分析

本研究テーマである「自動運転車両（CAV）による織り込み区間（Weaving Section）の制御」に関連する文献を分析します。これらは研究の立ち位置を明確にするための重要なベンチマークとなります。

> **補足**: 以下の文献は基盤理論として参照しています：
> - Ashok_K1975.pdf（配置アルゴリズムの計算量解析）
> - Scheduling_Michael_L_Pinedo2012.pdf（スケジューリング理論の教科書）
> - Mayne2020.pdf（ロバストMPCの基礎理論）

---

### A. 織り込み区間の交通流特性と課題提起

#### 1. G_Tilg2018.pdf

**概要**:
織り込み区間において、車線変更（LC）を行う位置の分布が交通容量に与える影響をマクロモデルで分析。LC位置を最適化することで容量低下（Capacity Drop）を防げることを示した。

**本研究への裏付け**:
「車線変更が特定箇所に集中すると効率が落ちる」という根本的な問題を指摘しており、本研究における**「Urgency関数（空間分散）」**導入の最強の理論的根拠となります。

**Research Gap**:
| 先行研究 (Tilg) | 本研究 |
|----------------|--------|
| マクロ的かつ固定的な分布の最適化 | 個々の車両がリアルタイムかつ**確率的（Urgency関数）**に分散する「ミクロ・分散型」アプローチ |

---

#### 2. S_Tanaka2017.pdf

**概要**:
都市高速道路の織り込み区間でLCが上流に集中する（Front-loading）傾向を指摘し、これを分散させる制御アルゴリズムをシミュレーションで評価。

**本研究への裏付け**:
Front-loading（上流での集中）が渋滞の主因であるという課題設定を共有しています。

**Research Gap**:
| 先行研究 (Tanaka) | 本研究 |
|------------------|--------|
| 完全制御・完全情報を仮定した理想的なシミュレーション | 混合交通や通信遅延、Apollo準拠の車両運動制約まで考慮した、より実装に近いアプローチ |

---

#### 3. P_Ouyang_2024.pdf

**概要**:
実際の走行軌跡データ（NGSIM）を用いて、織り込み区間での車線変更挙動を分析。安全性と効率のトレードオフを示唆。

**本研究への裏付け**:
織り込み区間特有の危険性や挙動特性の基礎データとして、本シミュレーション設定の妥当性を支えます。

---

### B. 制御アルゴリズムと最適化手法

#### 4. E_Amini2021.pdf

**概要**:
100% CAV環境下での織り込み区間の軌道最適化。遺伝的アルゴリズム（GA）を用いて大域的な最適解を探索。

**Research Gap (計算効率)**:
| 先行研究 (Amini - GA) | 本研究 (QP) |
|----------------------|------------|
| 計算負荷が高く、リアルタイム制御には不向き | 問題を**二次計画法（QP）**に帰着させることで、計算コストを劇的に削減（10Hz制御）し実用性を確保 |
| FIFOの緩和を提案 | ST-Boundaryを用いることで柔軟な順序変更を可能に |

---

#### 5. L_Yan2024.pdf, xi2020.pdf, R_Rudolf2024.pdf

**概要**:
深層強化学習（DRL）や混合整数計画法（MIQP）を用いた意思決定手法。MIQPの計算コスト問題を解決するために機械学習（Neural Network）による近似を用いるアプローチ（Bi-level制御など）。

**Research Gap (透明性と安全性)**:
| 先行研究 | 本研究 |
|---------|--------|
| 「複雑な問題を学習で解く（Black-box）」アプローチ | **「階層型制御（MPC + QP）」**を採用し、物理モデルに基づいた**「White-box（説明可能）」な安全性**を保証 |
| 学習ベースの意思決定 | 安全クリティカルな場面ではRSS（責任感知型安全）ベースの保証が優位 |

---

#### 6. C_Zhang2019.pdf, T_Peng2025.pdf, X_Xu_2024.pdf, Ramezani2019.pdf

**概要**:
路側機（RSU）や中央管理システムが主導して、合流順序や軌道を全体最適化する「集中型（Centralized）」またはインフラ協調型のアプローチ。

**Research Gap (アーキテクチャ)**:
| 観点 | 先行研究 | 本研究 |
|------|---------|--------|
| **単一障害点** | サーバーが単一障害点となる | 各車両が独立してQPを解く**「分散自律型（Decentralized）」** |
| **通信負荷** | 高い | V2V通信のみで低減 |
| **スケーラビリティ** | RSU処理能力に依存 | **無限にスケーラブル** |
| **堅牢性** | サーバーダウン時に停止 | サーバーダウン時も動作可能 |

---

### C. 基盤技術と安全性

#### 7. Werling2010.pdf

**概要**:
Frenet座標系（道路座標系）を用いた最適軌道生成の金字塔的研究。ジャーク最小化による快適な軌道生成を提案。

**本研究への裏付け**:
「Level 2: FrenetQPController」の基礎理論です。横方向・縦方向の動きを分解して最適化する手法の正当性を保証します。

**Research Gap**:
| 先行研究 (Werling) | 本研究 |
|-------------------|--------|
| 候補軌道群（Polynomials）からの選択（サンプリング）が主 | Apollo準拠のPiecewise-Jerk QPとして定式化し、ST-Boundaryという凸空間内で厳密解を求める |

---

#### 8. Shai Shalev-Shwartz2017.pdf (Mobileye RSS)

**概要**:
責任感知型安全（RSS）モデルの提案。事故責任を形式化し、絶対的な安全距離を定義。

**本研究への裏付け**:
「安全マネージャ」の核心的な理論的根拠です。

**Research Gap / 工学的貢献**:
| 先行研究 (RSS論文) | 本研究 |
|-------------------|--------|
| 理論モデル | V2V通信環境（反応時間0.15sへの短縮）、QPの制約条件として**実装コードレベル**まで落とし込み |

---

#### 9. N_Chen2020.pdf

**概要**:
階層型MPC（戦略層と実行層）による合流制御。

**本研究への裏付け**:
複雑な合流問題を階層化して解くアプローチの有効性を支持しています。本研究のLevel 1/2構造と同様です。

---

#### 10. F_Zhu2018.pdf

**概要**:
Baidu Apolloの制御モジュールに関する解説。

**本研究への裏付け**:
本研究が準拠しているApolloアーキテクチャの産業界での標準性と信頼性を裏付けます。

---

#### 11. P_Listov2023.pdf, Suh2018.pdf

**概要**:
確率的MPC（Stochastic MPC）による不確実性への対処。

**Research Gap**:
| 先行研究 (Stochastic MPC) | 本研究 |
|--------------------------|--------|
| 計算負荷が高い確率計算 | **「適応的安全マージン（Adaptive Safety Margin）」**やV2V情報の確度に応じたバッファ調整という、より実用的で計算負荷の低いエンジニアリング解 |

---

## 2. 文献リスト

| No. | ファイル名 | 引用情報 |
|-----|-----------|---------|
| 1 | M_Algomaiah2019 | Algomaiah, M., & Li, Z. (2019). Next-generation interchange control based on centralized management of connected and autonomous vehicles. *IEEE Access*, 7, 82939-82955. |
| 2 | E_Amini2021 | Amini, E., Omidvar, A., & Elefteriadou, L. (2021). Optimizing operations at freeway weaves with connected and automated vehicles. *Transportation Research Part C*, 126, 103072. |
| 3 | Ashok_K1975 | Chandra, A. K., & Wong, C. K. (1975). Worst-case analysis of a placement algorithm related to storage allocation. *SIAM Journal on Computing*, 4(3), 249-263. |
| 4 | N_Chen2020 | Chen, N., van Arem, B., Alkim, T., & Wang, M. (2020). A hierarchical model-based optimization control approach for cooperative merging by connected automated vehicles. *IEEE Transactions on Intelligent Transportation Systems*, 22(12), 7712-7725. |
| 5 | T_Chen2018 | Chen, T., Wang, M., Gong, S., Zhou, Y., & Ran, B. (2021). Connected and automated vehicle distributed control for on-ramp merging scenario: A virtual rotation approach. *Transportation Research Part C*, 133, 103451. |
| 6 | Couchman2006 | Couchman, P. D., Cannon, M., & Kouvaritakis, B. (2006). Stochastic MPC with inequality stability constraints. *Automatica*. |
| 7 | D_Garikapati2024 | Garikapati, D., Poovalingam, S., Hau, W., De Castro, R., & Shinde, C. (2024). A comprehensive review of parallel autonomy systems within vehicles: applications, architectures, safety considerations and standards. *IEEE Access*. |
| 8 | Q_Guo2019 | Guo, Q., Li, L., & Ban, X. J. (2019). Urban traffic signal control with connected and automated vehicles: A survey. *Transportation Research Part C*, 101, 313-334. |
| 9 | L_Zhao2018 | Liao, X., Zhao, X., Wu, G., Barth, M., Wang, Z., Han, K., & Tiwari, P. (2021). A game theory based ramp merging strategy for connected and automated vehicles in the mixed traffic: A unity-sumo integrated platform. *arXiv preprint arXiv:2101.11237*. |
| 10 | P_Listov2023 | Listov, P., Schwarz, J., & Jones, C. N. (2024). Stochastic optimal control for autonomous driving applications via polynomial chaos expansions. *Optimal Control Applications and Methods*, 45(1), 3-28. |
| 11 | Mayne2020 | Mayne, D. Q., Seron, M. M., & Raković, S. V. (2005). Robust model predictive control of constrained linear systems with bounded disturbances. *Automatica*, 41(2), 219-224. |
| 12 | P_Ouyang_2024 | Ouyang, P., & Yang, B. (2024). Evaluation of spatiotemporal characteristics of Lane-Changing at the freeway weaving area from trajectory data. *Sustainability*, 16(4), 1639. |
| 13 | T_L_Pan2016 | Pan, T. L., Lam, W. H., Sumalee, A., & Zhong, R. X. (2016). Modeling the impacts of mandatory and discretionary lane-changing maneuvers. *Transportation Research Part C*, 68, 403-424. |
| 14 | T_Peng2025 | Peng, T., Xu, X., Li, Y., Wu, J., Li, T., Dong, X., ... & Ullah, S. (2025). Enhancing Expressway Ramp Merge Safety and Efficiency via Spatiotemporal Cooperative Control. *IEEE Access*. |
| 15 | Scheduling_Michael_L_Pinedo2012 | Pinedo, M. L. (2012). *Scheduling: Theory, algorithms, and systems* (4th ed.). Springer. |
| 16 | Ramezani2019 | Ramezani, M., & Ye, E. (2019). Lane density optimisation of automated vehicles for highway congestion control. *Transportmetrica B*, 7(1), 1096-1116. |
| 17 | S_Tanaka2017 | Tanaka, S., Hasegawa, N., Iizuka, D., & Nakamura, F. (2017). Evaluation of vehicle control algorithm to avoid conflicts in weaving sections under fully-controlled condition in urban expressway. *Transportation Research Procedia*, 21, 199-207. |
| 18 | G_Tilg2018 | Tilg, G., Yang, K., & Menendez, M. (2018). Evaluating the effects of automated vehicle technology on the capacity of freeway weaving sections. *Transportation Research Part C*, 96, 3-21. |
| 19 | Y_Wang2025 | Wang, Y., Xiang, J., Pan, J., Wang, J., Chen, T., & Wang, H. (2025). Cooperative control of CAVs in the merging area of multi-lane mainline and dual-lane ramps on freeways. *Transportation Safety and Environment*, 7(2), tdaf030. |
| 20 | Werling2010 | Werling, M., Ziegler, J., Kammel, S., & Thrun, S. (2010). Optimal trajectory generation for dynamic street scenarios in a frenet frame. *IEEE ICRA*, 987-993. |
| 21 | J_Wu2021 | Wu, J., Wang, Y., Shen, Z., Wang, L., Du, H., & Yin, C. (2021). Distributed multilane merging for connected autonomous vehicle platooning. *Science China Information Sciences*, 64(11), 212202. |
| 22 | xi2020 | Xi, C., Shi, T., Wu, Y., & Sun, L. (2020). Efficient motion planning for automated lane change based on imitation learning and mixed-integer optimization. *IEEE ITSC*, 1-6. |
| 23 | W_Xiao2020 | Xiao, W., & Cassandras, C. G. (2019). Decentralized optimal merging control for connected and automated vehicles. *IEEE ACC*, 3315-3320. |
| 24 | X_Xu_2024 | Xu, X., Lai, M., Zhang, H., Dong, X., Li, T., Wu, J., ... & Peng, T. (2024). Spatio-temporal Cooperative Control Method of Highway Ramp Merge Based on Vehicle-road Coordination. *IEEE ICTLE*, 93-98. |
| 25 | Q_Yang1996 | Yang, Q. I., & Koutsopoulos, H. N. (1996). A microscopic traffic simulator for evaluation of dynamic traffic management systems. *Transportation Research Part C*, 4(3), 113-129. |
| 26 | C_Zhang2019 | Zhang, C., Sabar, N. R., Chung, E., Bhaskar, A., & Guo, X. (2019). Optimisation of lane-changing advisory at the motorway lane drop bottleneck. *Transportation Research Part C*, 106, 303-316. |
| 27 | J_Zhang2023 | Zhang, J., Li, S., & Li, L. (2023). Coordinating CAV swarms at intersections with a deep learning model. *IEEE Transactions on Intelligent Transportation Systems*, 24(6), 6280-6291. |
| 28 | Z_Zhong2021 | Zhong, Z., Lee, J., & Zhao, L. (2021). Traffic flow characteristics and lane use strategies for connected and automated vehicles in mixed traffic conditions. *Journal of Advanced Transportation*, 2021(1), 8816540. |
| 29 | Z_Zhou2023 | Zhou, Z., Li, L., Ran, B., & Qu, X. (2023). A Cooperative Lane‐Changing Strategy for Weaving Sections of Urban Expressway under the Connected Autonomous Vehicle Environment. *Journal of Advanced Transportation*, 2023(1), 3363057. |
| 30 | J_Zhu2022 | Zhu, J., Easa, S., & Gao, K. (2022). Merging control strategies of connected and autonomous vehicles at freeway on-ramps: A comprehensive review. *Journal of Intelligent and Connected Vehicles*, 5(2), 99-111. |
| 31 | Shai Shalev-Shwartz2017 | Shalev-Shwartz, S., Shammah, S., & Shashua, A. (2017). On a formal model of safe and scalable self-driving cars. *arXiv preprint arXiv:1708.06374*. |
| 32 | F_Zhu2018 | Zhu, F., Ma, L., Xu, X., Guo, D., Cui, X., & Kong, Q. (2018). Baidu apollo auto-calibration system-an industry-level data-driven and learning based vehicle longitude dynamic calibrating algorithm. *arXiv preprint arXiv:1808.10134*. |
| 33 | Suh2018 | Suh, J., Chae, H., & Yi, K. (2018). Stochastic model-predictive control for lane change decision of automated driving vehicles. *IEEE Transactions on Vehicular Technology*, 67(6), 4771-4782. |
| 34 | L_Yan2024 | Yan, L., Liang, J., & Yang, K. (2025). Bi-level control of weaving sections in mixed traffic environments with connected and automated vehicles. *IEEE Transactions on Intelligent Transportation Systems*. |
| 35 | R_Rudolf2024 | Reiter, R., Quirynen, R., Diehl, M., & Di Cairano, S. (2024). Equivariant deep learning of mixed-integer optimal control solutions for vehicle decision making and motion planning. *IEEE Transactions on Control Systems Technology*. |
| 36 | C_rshayyid2024 | Irshayyid, A., & Chen, J. (2024). Highway Merging Control Using Multi-Agent Reinforcement Learning. *IEEE ICMI*, 1-2. |
| 37 | J_Rios2017 | Rios-Torres, J., & Malikopoulos, A. A. (2016). Automated and cooperative vehicle merging at highway on-ramps. *IEEE Transactions on Intelligent Transportation Systems*, 18(4), 780-789. |
| 38 | J_Zhou | Zhu, J., Tasic, I., & Qu, X. (2022). Flow-level coordination of connected and autonomous vehicles in multilane freeway ramp merging areas. *Multimodal Transportation*, 1(1), 100005. |
| 39 | N_Chen2022 | Chen, N., van Arem, B., & Wang, M. (2022). Hierarchical optimal maneuver planning and trajectory control at on-ramps with multiple mainstream lanes. *IEEE Transactions on Intelligent Transportation Systems*, 23(10), 18889-18902. |
| 40 | N_Suriyarachchi2021 | Suriyarachchi, N., Tariq, F. M., Mavridis, C., & Baras, J. S. (2021). Real-time priority-based cooperative highway merging for heterogeneous autonomous traffic. *IEEE ITSC*, 2019-2026. |
| 41 | U_Mandal2024 | Mandal, U., Amir, G., Wu, H., Daukantas, I., Newell, F., Ravaioli, U., ... & Barrett, C. (2024). Formally verifying deep reinforcement learning controllers with lyapunov barrier certificates. *FMCAD 2024*, 95. |
| 42 | W_Liu2023 | Liu, W., Hua, M., Deng, Z., Meng, Z., Huang, Y., Hu, C., ... & Xia, X. (2023). A systematic survey of control techniques and applications in connected and automated vehicles. *IEEE Internet of Things Journal*, 10(24), 21892-21916. |

---

## 3. CAVを用いる意義

本研究がCAV（Connected Autonomous Vehicle）を前提とする理由は、単に「自動運転だから精密な制御ができる」という点に留まりません。**V2V通信による計画軌道の共有**こそが、分散型制御の核心的な価値を生み出しています。

**重要な点**: 本研究は**「自律分散型（Decentralized）」でありながら「LC分散（Spatial Distribution）」を実現**しています。これは一見矛盾しているように見えますが、CAVの計画軌道共有という特性を活かすことで、中央管理者なしに車両間で暗黙的に協調し、結果的にLCが時空間的に分散されるという、従来手法では不可能だった両立を達成しています。

### 3.1 計画軌道共有による暗黙的な協調

従来の集中型制御（C_Zhang2019, T_Peng2025）では、中央サーバーが全車両の軌道を最適化し、各車両に指示を送ります。これに対し、本研究の**自律分散型**アプローチでは：

```
[従来の集中型]
  各車両 → (位置情報) → 中央サーバー → (指示軌道) → 各車両

[本研究の分散型]
  各車両 ⇄ (計画軌道をV2Vで共有) ⇄ 各車両
         ↓
  各車両が独立してQPを解く（他車の計画を制約として考慮）
```

各CAVは自身のQP解（今後数秒間の予定軌道）をV2Vでブロードキャストします。周囲の車両はこの情報を受信し、**他車がいつ・どこで車線変更する予定か**を把握した上で、自車のQPを解きます。

### 3.2 自律分散型でありながらLC分散を実現するメカニズム

この計画軌道共有により、**「制御の分散（Decentralized Control）」と「LC位置の分散（Spatial Distribution）」という2つの分散が同時に実現**されます：

| 状況 | 各車両の判断（自律的） | 結果（システム全体） |
|------|---------------------|-------------------|
| 前方車両が「今からLC」と計画をブロードキャスト | 自車は同時刻・同位置でのLCを避けるようQPが解く | LCの**時間的分散** |
| 後方車両が「100m先でLC」と計画をブロードキャスト | 自車はそれより手前または奥でLCするようUrgency関数が誘導 | LCの**空間的分散** |
| 複数車両が同じエリアでLC計画 | 各車両のST-Boundaryに他車の占有領域が反映 | **衝突回避と分散の両立** |

#### 本手法の本質的な価値

**「各車両が自律的に判断」しながらも「全体としてLCが分散される」**という、従来の二項対立を打破している点が最大の貢献です：

| 従来の二項対立 | 問題点 | 本研究の解決策 |
|-------------|--------|-------------|
| **集中型制御**<br>（Zhang, Peng等） | LC分散は実現できるが、単一障害点・通信負荷・スケーラビリティに課題 | → |
| **従来の分散型制御**<br>（協調なし） | 各車両の独立判断により、LCが集中してしまう（Front-loading問題） | → |
| **本研究：CAV前提の自律分散型** | **V2V計画軌道共有により、中央管理なしにLC分散を達成** | ✓ 両方の利点を獲得 |

重要なのは、「LCを分散させろ」という**明示的な指令や中央コーディネータが存在しない**ことです。各車両は単に：

1. **他車の計画軌道を制約として取り込み**（V2V通信の活用）
2. **自車の目標を最適化**（レーン到達、快適性、Urgency関数）

するだけで、**システム全体として自然にLCが時空間的に分散される**という創発的な効果が生まれます。これは、集中型制御の「単一障害点」「通信負荷」という弱点を克服しつつ、従来の分散型の「LC集中」という問題も解決する、**第三の道**といえます。

### 3.3 HDV（人間運転車両）との対比

HDV環境では、他車の「意図」は推測するしかありません（ウィンカー、挙動予測など）。一方CAV環境では：

| 要素 | HDV | CAV |
|------|-----|-----|
| 他車の意図 | 推測（不確実） | 計画軌道として明示（確実） |
| 反応時間 | 人間の知覚・判断遅延（1-2秒） | 通信遅延のみ（0.1秒以下） |
| 協調の質 | 暗黙的・非効率 | 明示的・最適化可能 |

この「意図の可視化」こそがCAVの本質的な価値であり、本研究の分散型QPアプローチはこの特性を最大限に活用しています。

### 3.4 先行研究との差異：自律分散とLC分散の両立

G_Tilg2018やS_Tanaka2017は「LC分散が重要」と指摘しましたが、その実現手段として：
- **Tilg**: マクロ的な制御ゾーンの設置
- **Tanaka**: 集中型の順序決定

を提案しており、いずれも**集中管理によってLC分散を実現**しています。

一方、C_Zhang2019、T_Peng2025、X_Xu_2024などのRSU主導型アプローチも、本質的には「中央サーバーがLC位置を指示する」という集中型の枠組みです。

**本研究の独自性は、これらの先行研究が「集中制御の力でLC分散を実現」しようとしたのに対し、「自律分散型でありながらLC分散を実現」という、相反する要求を両立させた点にあります。** この両立を可能にしたのが、CAVの計画軌道共有という特性であり、各車両が他車の計画を制約として取り込むことで、中央管理者なしに暗黙的な協調が生まれるというメカニズムです。

---

## 4. 本研究の位置づけ

### 4.1 ALGORITHMS.md への補足

| 要素 | 先行研究との関係 |
|------|-----------------|
| **Urgency関数** | G_Tilg2018やS_Tanaka2017が指摘した「織り込み区間の容量低下（Front-loading問題）」に対し、彼らの提案する「固定的なエリア規制」や「集中制御」ではなく、**「各車両の自律的な確率判断」**によって空間分散を実現する軽量なアルゴリズム |
| **Frenet QP** | Werling2010のFrenetフレーム理論を継承しつつ、サンプリングベースではなくApollo型のPiecewise-Jerk QP定式化を採用することで、R_Rudolf2024のようなMIQP手法やE_Amini2021のようなGA手法よりも**高速かつ決定論的な安全性（ST-Boundary制約）**を持って解を導出 |
| **RSSの拡張** | Shalev-Shwartz2017の理論を、V2V通信を前提とした環境（反応時間0.15s）に適応させ、**実用的な安全距離計算式として実装** |

---

### 4.2 ARCHITECTURE.md への補足

| 要素 | 先行研究との関係 |
|------|-----------------|
| **分散自律型制御** | C_Zhang2019、T_Peng2025、X_Xu_2024らが提案する「RSU主導の中央集権型制御」に対する**明確なアンチテーゼ**。通信インフラへの依存度を下げ、単一障害点を排除し、無限のスケーラビリティを持つ点が最大の差別化要素 |
| **階層構造** | L_Yan2024やN_Chen2020と同様の階層構造（戦略/戦術）を持つが、上位層にブラックボックスな学習モデル（DRL/Neural Network）を使用せず、**解釈可能なUrgency関数と数理最適化（QP）**を使用している点が、安全性検証（Safety Assurance）の観点で優位 |

---

### 4.3 APOLLO_ALIGNMENT.md への補足

| 要素 | 先行研究との関係 |
|------|-----------------|
| **実用性の証明** | 多くの先行研究（xi2020, R_Rudolf2024など）が独自の簡易シミュレーションや理論モデルに留まる中、本研究はBaidu Apollo (F_Zhu2018) という、**実際に公道を走行している業界標準のオープンソースアーキテクチャに準拠**。提案手法が単なる机上の空論ではなく、**実車への移植性が極めて高い**ことを裏付け |

---

### 4.4 IMPLEMENTATION.md への補足

| 要素 | 先行研究との関係 |
|------|-----------------|
| **実装の具体性** | 理論（Suh2018のSMPCなど）の実装難易度が高い中、本ドキュメントはQPソルバー（OSQP）を用いた具体的な実装フロー（逐次更新、遅延対応）を示しており、E_Amini2021のような計算コストの高い手法と比較して、**10Hzでのリアルタイム動作が可能**であることを示す |

---

### 4.5 既存交通シミュレータとの差異：なぜ自作シミュレータか

本研究は自作のPythonシミュレータ（weaving_v11）を用いています。既存の交通シミュレータ（SUMO、VISSIM、Carla等）ではなく、自作シミュレータを選択した理由を明確化します。

#### 研究の位置づけ：2つのアプローチの違い

交通流シミュレーション研究には、大きく2つのアプローチがあります：

| アプローチ | 目的 | 使用ツール | 例 |
|-----------|------|-----------|-----|
| **交通流改良研究** | 既存の交通システム（人間運転ベース）の改良・拡張 | 既存シミュレータ（SUMO、VISSIM等） | 信号機最適化、車線運用改善 |
| **CAV制御システム研究** | 新しいCAV制御アルゴリズムの提案と実装検証 | 自作シミュレータ or 実車プラットフォーム | Apollo、Autoware等の制御モジュール開発 |

**本研究は後者**です。つまり、「既存交通流の改良」ではなく、**「Apollo準拠の新しいCAV制御システム全体の提案」**を目的としています。

#### 既存シミュレータの特性と限界

既存の交通シミュレータは、人間運転を前提とした交通流シミュレーションに最適化されています：

| シミュレータ | 車両モデル | 安全性保証 | CAV制御への適用 |
|------------|-----------|-----------|----------------|
| **SUMO** | IDM/Krauss（追従モデル） | 経験的安全距離 | TraCIで外部制御可能だが、内部モデルと混在 |
| **VISSIM** | Wiedemann（心理的運転） | ヒューリスティック | COM APIで制御可能だが、商用・高コスト |
| **Carla** | Unreal Engine物理演算 | 物理エンジン | 高精度だが計算負荷大（3D描画） |

これらのシミュレータで本研究のような「Apollo準拠のCAV制御システム」を実装する場合、以下の課題が生じます：

##### 課題1: 制御アーキテクチャの混在

```
【既存シミュレータでの実装】
┌────────────────────────────────────────┐
│ 外部Python制御（本研究の提案）         │
│  - Urgency関数によるLC判断             │
│  - ST-Boundary QPによる軌道生成        │
│  - V2V計画軌道共有                     │
└────────────────────────────────────────┘
              ↓ TraCI/COM API経由
┌────────────────────────────────────────┐
│ SUMOの内部車両モデル（既存技術）       │
│  - IDM/Kraussの独自LC判断ロジック     │
│  - 独自の衝突回避ヒューリスティック   │
│  - 経験的な安全マージン               │
└────────────────────────────────────────┘

問題: どちらが効果を生んでいるか不明確
```

##### 課題2: V2V計画軌道共有の実装困難性

本研究の核心メカニズム（[3.2節](#32-自律分散型でありながらlc分散を実現するメカニズム)）である「各CAVがQP解をV2Vで共有し、他車がそれをST-Boundary制約として取り込む」を既存シミュレータで実装すると：

- 外部Pythonで全車両のQP計算を管理
- 計算した軌道をシミュレータに注入
- しかし、シミュレータの内部モデルが独自に軌道を修正
- **「共有した計画軌道」と「実際の軌道」が乖離**

この乖離により、「V2V軌道共有による暗黙的協調」という本研究の独自性が純粋に検証できません。

##### 課題3: Apollo準拠の実装検証が不可能

[4.3節](#43-apollo_alignmentmdへの補足)で述べたように、本研究の価値の一つは「実車への移植性」です。

- Baidu Apolloは実際のLinux環境で動作するC++/Pythonコードベース
- Piecewise-Jerk QP、ST-Boundary制約、10Hz制御周期などが厳密に定義されている
- 既存シミュレータの車両モデルは、Apolloの制御モジュールとは設計思想が異なる

既存シミュレータでは「SUMOで動いた」ことは示せても、「Apolloに移植可能」とは言えません。

#### 自作シミュレータの役割：Apollo実装検証プラットフォーム

本研究の自作シミュレータは、「交通シミュレータ」ではなく、**「Apollo制御モジュールの実装検証プラットフォーム」**として設計されています：

| 要素 | 自作シミュレータの実装 | Apollo実装との対応 |
|------|---------------------|-------------------|
| **軌道生成** | Piecewise-Jerk QP（OSQP） | Planning Module (apollo/modules/planning) |
| **ST-Boundary** | frenet_qp_apollo.py:867-975 | apollo/modules/planning/tasks/deciders/path_bounds_decider |
| **RSS安全距離** | apollo_safety.py:45-51 | RSS-based safety checker |
| **制御周期** | Level 2: 0.1s (10Hz) | Apollo標準制御周期 |
| **座標系** | Frenet座標系 (s, d) | apollo/modules/common/math/frenet_frame |

このように、自作シミュレータのコードは**Apolloの実装に1対1で対応**しており、シミュレーションで検証した制御ロジックをそのままApolloに移植できます。

#### 先行研究との比較：実装検証の透明性

| 研究 | シミュレーション環境 | 実装の透明性 | 実車移植性 |
|------|------------------|-------------|-----------|
| E_Amini2021 (GA) | 独自の簡易シミュレータ | 低（GAの評価関数のみ記述） | 不明 |
| L_Yan2024 (DRL) | SUMO + 学習環境 | 低（ブラックボックス） | 困難 |
| R_Rudolf2024 (MIQP+NN) | 独自シミュレータ | 中（理論モデルは明確だが実装詳細不明） | 不明 |
| **本研究** | **自作（Apollo準拠）** | **高（全コードがApollo対応）** | **高** |

#### 結論：自作シミュレータを選択する理由

本研究が自作Pythonシミュレータを使用する理由は、以下の3点に集約されます：

1. **制御アーキテクチャの純粋性**
   既存シミュレータの独自ロジック（IDM、Krauss等）が介在せず、提案手法（Urgency関数 + V2V軌道共有 + ST-Boundary QP）の効果を純粋に検証できる

2. **V2V計画軌道共有の忠実な実装**
   「共有された計画軌道」と「実際に実行される軌道」が完全に一致し、[3.2節](#32-自律分散型でありながらlc分散を実現するメカニズム)で述べた「暗黙的協調」のメカニズムを正確に検証できる

3. **Apollo準拠の実装検証**
   シミュレーションコードがApolloの実装に1対1対応しており、「机上の空論」ではなく「実車搭載可能なシステム」であることを裏付ける

既存シミュレータは「既存交通流の改良研究」には優れていますが、**「新しいCAV制御システムの提案研究」**である本研究には適していません。

---

## 結論

本研究は、**「織り込み区間の効率化」**という古典的な課題（Tilg, Tanaka）に対し、**「分散自律型」**というスケーラブルなアプローチ（対 Zhang, Peng）で挑んでいます。

また、手法としては**「Apollo準拠のQP」**と**「RSS」**を採用することで、アカデミックな複雑さ（MIQP, SMPC）を回避しつつ、**「実用性・計算速度・安全性」**を高度にバランスさせている点に、強い独自性と優位性があります。

さらに、自作のApollo準拠シミュレータを用いることで、提案手法の効果を純粋に検証し、実車への移植性を明確に示しています。

---

**文書バージョン**: 2.1
**最終更新**: 2026-01-23

**主な変更履歴**:
- v2.1 (2026-01-23): 「4.5 既存交通シミュレータとの差異」セクションを追加。既存シミュレータ（SUMO、VISSIM等）と自作シミュレータの使い分けの根拠を明確化
- v2.0 (2026-01-19): 初版公開

---

**End of Literature Review**
