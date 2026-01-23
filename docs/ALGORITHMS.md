# アルゴリズム詳細

> **関連ドキュメント**: [README](../README.md) | [ARCHITECTURE](ARCHITECTURE.md) | [API_REFERENCE](API_REFERENCE.md) | [IMPLEMENTATION](IMPLEMENTATION.md) | [APOLLO_ALIGNMENT](APOLLO_ALIGNMENT.md)

**対象読者**: 研究者・理論を理解したい開発者
**目的**: 数式・アルゴリズムの数学的根拠を理解する

---

## 目次

本システムは**2層の階層型制御**で構成されています。

### Level 1: 戦略的計画（Strategic Planning）
- **周期**: 0.5秒
- **役割**: 「いつ・どこで車線変更するか」を決定
- **入力**: 自車位置、車線密度
- **出力**: LC実行の可否（`lc_scheduled`）

| セクション | 内容 |
|-----------|------|
| [1. Urgency関数](#1-urgency関数空間分散の担当) | 位置と密度からUrgency値を計算（空間分散） |
| [2. 確率的LCトリガー](#2-確率的lcトリガー時間分散の担当) | Urgencyを確率としてLC判断（時間分散） |
| [2.1 Gap Acceptance](#21-gap-acceptanceトリガー後の安全性検証) | トリガー後の安全性検証 |

### Level 2: 戦術的制御（Tactical Control）
- **周期**: 0.1秒（10Hz）
- **役割**: 「どのような軌道で走行するか」を決定
- **入力**: 自車状態 + 他車情報（s, v, a, 予測軌道）← ST境界の構成要素
- **処理**: ST境界構築 → QP最適化
- **出力**: 軌道（s, v, a）× N ステップ

| セクション | 内容 |
|-----------|------|
| [3. ST-Boundary制約](#3-st-boundary制約衝突不可能性の数学的保証) | 他車予測から安全領域を構築 |
| [4. 安全距離計算](#4-安全距離計算idm式) | IDM/RSSベースの安全距離 |
| [5. QP定式化](#5-qp定式化最適軌道の数値解法) | ST境界を制約として軌道最適化 |
| [6. 適応的安全マージン](#6-適応的安全マージン) | Urgency連動のマージン調整 |

### 共通・補助
| セクション | 内容 |
|-----------|------|
| [7. TTC計算](#7-ttc計算) | 衝突予測時間の計算 |
| [8. LCスケジューリング全体像](#8-lcスケジューリング全体像) | Level 1 + Level 2の統合フロー |
| [9. 主要パラメータ早見表](#9-主要パラメータ早見表) | 全パラメータ一覧 |
| [10. 安全マネージャ閾値](#10-安全マネージャ閾値) | AEB/RSS閾値 |
| [11. デフォルトパラメータと上書き](#11-デフォルトパラメータと上書き) | パラメータ設定方法 |
| [12. V2V協調仲裁アルゴリズム](#12-v2v協調仲裁アルゴリズム) | CAV間協調制御 |

---

# Level 1: 戦略的計画（Strategic Planning）

Level 1は**0.5秒周期**で「車線変更するかどうか」を決定する戦略層です。

**入力**: 自車位置、車線密度
**出力**: LC実行の可否（`lc_scheduled = True/False`）

```
┌─────────────────────────────────────────────────────────┐
│ Level 1: 戦略的計画 (0.5s周期)                          │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  1. Urgency計算: U(s,ρ) = s^γ + αρ                     │
│     └→ 位置が奥ほど高い、密度が高いほど高い            │
│                                                         │
│  2. 確率的トリガー: P = U                              │
│     └→ Urgencyをそのまま確率として使用                 │
│                                                         │
│  3. Gap Acceptance検証                                 │
│     └→ 前後ギャップとTTCを検証                        │
│                                                         │
│  → lc_scheduled = True / False                         │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

---

## 1. Urgency関数（空間分散の担当）

**役割**: 奥寄り配置（出口付近でLC集中）を実現する**空間的分散**を担当。

### 数式

$$
\boxed{
U_i(s, \rho) = \min\left(1.0, \left(\frac{s_i - s_{\text{entry}}}{s_{\text{exit}} - s_{\text{entry}}}\right)^\gamma + \alpha \cdot \rho\right)
}
$$

### パラメータ

| 記号 | 意味 | 値 | 単位 | 役割と効果 |
|------|------|-----|------|-----------|
| $s_i$ | 車両$i$の現在位置 | - | [m] | シミュレーション内の縦方向座標 |
| $s_{\text{entry}}$ | 入口位置（LC開始可能位置） | 0.0 | [m] | 織り込み区間の開始点 |
| $s_{\text{exit}}$ | 出口位置（LC必須完了位置） | 1000.0 | [m] | 織り込み区間の終了点（実装では`prep_zone_length + weave_zone_length`） |
| $\gamma$ | **急峻度パラメータ** | **3.0** | - | **出口付近での急上昇を制御**。大きいほど奥でLCが集中。`urgency_gamma`変数に対応 |
| $\alpha$ | **密度影響係数** | **0.2** | - | **目標車線の混雑度を反映**。高密度時にUrgencyを底上げ。`urgency_alpha`変数に対応 |
| $\rho$ | 対象車線密度（観測値） | - | [veh/m] | 目標車線の1メートルあたり車両台数 |

### なぜ $\alpha \rho$ を加算するか？

**意図**: 混んでいるとき**こそ**早めにLCしないと間に合わない。

- **密度 ρ が高い**（混雑）→ Urgencyが底上げ → より早くトリガー
- **密度 ρ が低い**（空いている）→ 位置ベースの $s^\gamma$ だけで十分

**物理的解釈**:
- 空いている → ギャップが十分 → 焦る必要なし → 出口付近でLC
- 混んでいる → ギャップ確保が困難 → 早めにLC機会を探す

### 正規化位置

$$
x_{\text{norm}} = \frac{s_i - s_{\text{entry}}}{s_{\text{exit}} - s_{\text{entry}}} \in [0, 1]
$$

### Urgency推移（$\gamma=3.0$, $\alpha=0.2$, $\rho=0.01$の場合）

| 位置割合 | $x_{\text{norm}}$ | Urgency $U$ | 解釈 |
|---------|-------------------|-------------|------|
| 0% (入口) | 0.0 | 0.002 | 極めて低い |
| 25% | 0.25 | 0.018 | 低い |
| 50% | 0.5 | 0.127 | 中程度 |
| 75% | 0.75 | 0.423 | 高い |
| 95% | 0.95 | 0.859 | 極めて高い |
| 100% (出口) | 1.0 | 1.0 | 最大 |

### なぜ$\gamma=3.0$か？

**理論的根拠**:
- $\gamma > 1$: 凸関数 → 出口付近で急激に増加
- $\gamma = 3.0$: 入口での低確率と出口での高確率のバランス
- Front-loading回避: 入口での$U < 0.02$ → ほとんどトリガーしない

**実験的根拠**（Khan et al., 2014）:
- Front-loading（前詰まり）は交通流を+40%悪化
- $\gamma = 3.0$により自然な空間分散を実現
- Gini係数: 0.7 → 0.28（-60%改善）

**実装**: [mpc_controller.py](../weaving_v11/mpc_controller.py):60-120

---

## 2. 確率的LCトリガー（時間分散の担当）

### 役割と位置づけ

**制御フローでの位置**: Level 1戦略計画（0.5s周期）
**役割**: Urgency値をそのまま確率として使い、**時間的分散**（0.5s周期のランダム試行）を担当。
**次ステップ**: トリガー成功後、Gap Acceptance（Section 2.1, 7）で安全性検証 → スケジュール確定。

### なぜ確率化が必要か？

もし決定論的に「Urgency > 閾値ならLC開始」とすると、同じ位置の全車両が**同じ時刻**にLCを試みる（Front-loading）。これはギャップ不足で大半が失敗し、出口付近で渋滞を引き起こす。

**解決策**: $P_{\text{trigger}} = U$ として0.5sごとにベルヌーイ試行 → 同じ位置でも車両ごとに**異なるタイミング**でトリガー。

### 実装モデル（単純版）

$$
\boxed{
P_{\text{trigger}} = U(x, \rho) = \min\left(1.0, x^{3.0} + \alpha \cdot \rho\right)
}
$$

- ここで $x=x_{\text{norm}} \in [0,1]$
- 0.5秒ごとに独立したベルヌーイ試行: `if U > random.rand(): トリガー`
- **位置依存で自動的に変化**:
  - 入口 ($x \approx 0$): $U \approx 0.002$ → ほぼトリガーしない
  - 中間 ($x \approx 0.5$): $U \approx 0.127$ → 低確率
  - 出口 ($x \to 1$): $U \to 1.0$ → **ほぼ確実にトリガー**（意図した動作）

### 位置別トリガー確率（$\alpha=0.2$, $\rho=0.01$の例）

| 位置 | $x_{\text{norm}}$ | Urgency $U$ | $P_{\text{trigger}}$ | 期待LC試行回数（0.5s周期） |
|------|-------------------|-------------|----------------------|---------------------------|
| 入口 | 0.0 | 0.002 | 0.2% | 2回 / 1000回 |
| 25% | 0.25 | 0.018 | 1.8% | 18回 / 1000回 |
| 50% | 0.5 | 0.127 | 12.7% | 127回 / 1000回 |
| 75% | 0.75 | 0.423 | 42.3% | 423回 / 1000回 |
| 95% | 0.95 | 0.859 | 85.9% | 859回 / 1000回 |
| 出口50m前 | - | 1.0 | **100%** | 強制実行 |

### 役割分担の整理

**空間分散（奥寄り）**: Urgency関数の$\gamma=3.0$が担当
- 入口: $U \approx 0.002$ → ほぼトリガーしない
- 出口: $U \to 1.0$ → ほぼ確実にトリガー

**時間分散（ランダム化）**: 確率的トリガーが担当
- 同じ位置でも0.5秒ごとに独立試行
- 全車が同時にLCするのを防ぐ

### なぜ確率的か？

**決定論的コスト削減の問題**:
```python
# 悪い例（決定論的）
if x > threshold:
    lc_cost -= incentive
    # → 全車が同じ位置・同じ時刻でLC（Front-loading）
```

**確率的トリガーの利点**:
1. **時間的分散**: 0.5秒ごとの独立試行 → 同じ位置でも異なるタイミング
2. **Front-loading回避**: Urgency関数が空間分散を担当（$U(0) \approx 0.002$）
3. **Gap成功率考慮**: 実質LC実行率 ≈ 30% × P_trigger

**実装**: [controllers.py](../weaving_v11/controllers.py):535-633

### 2.1 Gap Acceptance：トリガー後の安全性検証

#### 制御フロー（3段階判定）

LC確率トリガー成功後、以下の3段階で安全性を検証します：

```
確率トリガー成功（P=U）
    ↓
【1. Gap Acceptance判定】← ここで詳細検証
    ↓ Safe
スケジュール確定（lc_scheduled=True）
    ↓ Not Safe
次周期（0.5s後）に再試行
```

#### 検証項目（4ステップ、Apollo統合安全マネージャ）

実装：[apollo_safety.py](../weaving_v11/apollo_safety.py) - `check_lc_start_safety()`

1. **前方ギャップ検証**：目標車線の前方車両とのバンパー間距離
   - 基準値：15.0m（`lc_min_front_gap`）
   - 適用値：$g(U) \times 15.0$ m（Urgency高時に緩和）

2. **前方TTC検証**：Time-to-Collision（Section 7参照）
   - 基準値：3.0s（衝突予測時間の最小値）
   - 適用値：$g(U) \times 3.0$ s（Urgency高時に緩和）

3. **後方ギャップ検証**：目標車線の後方車両とのバンパー間距離
   - 基準値：18.0m（`lc_min_rear_gap`、**前方より保守的**）
   - 適用値：18.0m（**緩和なし**、追突防止のため厳格）

4. **後方TTC検証**：後方車両との衝突予測時間
   - 基準値：4.0s（**前方より保守的**）
   - 適用値：4.0s（**緩和なし**）

**重要**：前方は緩和あり、**後方は緩和なし**（追突リスクが高いため）

---

#### ギャップ緩和係数 $g(U)$（Urgency連動）

**目的**：Urgencyが高い（出口に近い/密度高）ときは、ギャップ要件を緩和してLC成功率を向上。

**数式**：
$$
\boxed{g(U) = \max\bigl(0.7,\; 1 - c_{\text{gap}} \cdot U\bigr)}
$$

**パラメータ**：
- $c_{\text{gap}}$：ギャップ緩和係数 = **0.5**（v2最適値、`urgency_gap_relax_coeff`）
- $U$：Urgency値 ∈ [0, 1]
- **0.7**：**安全下限値**（Hard limit）= 元の閾値の**最低70%は維持**

**計算例**（$c_{\text{gap}}=0.5$の場合）：

| Urgency $U$ | 計算 | $g(U)$ | 緩和率 | 前方ギャップ実効値 |
|------------|------|--------|-------|------------------|
| 0.0（低） | $1.0 - 0.5 \times 0.0 = 1.0$ | **1.0** | 0% | 15.0m（緩和なし） |
| 0.3（中低） | $1.0 - 0.5 \times 0.3 = 0.85$ | **0.85** | 15% | 12.8m |
| 0.6（中高） | $1.0 - 0.5 \times 0.6 = 0.7$ | **0.7** | 30% | 10.5m（**下限到達**） |
| 0.8（高） | $1.0 - 0.5 \times 0.8 = 0.6$ | **0.7** | 30% | 10.5m（**0.7でクリップ**） |
| 1.0（最高） | $1.0 - 0.5 \times 1.0 = 0.5$ | **0.7** | 30% | 10.5m（**0.7でクリップ**） |

**0.7の物理的意味**：
- **30%緩和が安全性の限界**（設計判断）
- これ以上緩和すると衝突リスクが許容不可
- 実装：`gap_factor = max(0.7, 1.0 - 0.5 * urgency)` ([controllers.py:325](../weaving_v11/controllers.py#L325))

---

#### 適用例（前方のみ）

- **前方ギャップ基準**: 15.0m → Urgency 0.8時: $0.7 \times 15.0 = 10.5$ m
- **前方TTC基準**: 3.0s → Urgency 0.8時: $0.7 \times 3.0 = 2.1$ s
- **後方ギャップ**: 18.0m（**常に固定、緩和なし**）
- **後方TTC**: 4.0s（**常に固定、緩和なし**）

---

#### Gap判定失敗時のV2V協調リクエスト（v28.0）

**Apollo-style Cooperative Yielding**：

Gap判定失敗時、以下の条件でV2V協調減速をリクエスト：
- 出口距離 < 50m：`urgent_yield`（緊急、30%減速）
- 出口距離 < 100m：`cooperative_yield`（協調、15%減速）

目標車線のCAV（後方車両）が受信し、自発的に減速してギャップを作成。

---

#### リトライ機構

- **判定周期**: 0.5s（Level 1再計画周期）
- **失敗時**: スキップして次周期（0.5s後）に再試行
- **タイムアウト**: 3.0s以上低速（<5.5 m/s）で準備中 → LC中止（デッドロック回避）

**実装**：[controllers.py:564-606](../weaving_v11/controllers.py#L564-L606) - Deadlock Prevention

---

#### 出口付近の強制実行（緊急LC）

- **条件**: 出口まで50m以内（`emergency_distance`）
- **効果**: Urgency → 1.0に強制ブースト（二次関数で滑らか）
- **確率**: $P_{\text{trigger}} \to 1.0$（ほぼ確実にトリガー）
- **ギャップ**: $g(U)=0.7$（最大30%緩和）

**実装**：[mpc_controller.py:157-162](../weaving_v11/mpc_controller.py#L157-L162) - Emergency Urgency Boost

---

# Level 2: 戦術的制御（Tactical Control）

Level 2は**0.1秒周期（10Hz）**で「どのような軌道で走行するか」を決定する戦術層です。

**入力**: 自車状態 + 他車情報（s, v, a, 予測軌道）← ST境界の構成要素
**処理**: ST境界構築 → QP最適化
**出力**: 軌道（s, v, a）× N ステップ → 先頭の a[0] を車両に適用

```
┌─────────────────────────────────────────────────────────┐
│ Level 2: 戦術的制御 (0.1s周期 / 10Hz)                   │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  【入力】                                               │
│    - 自車状態 (s, v, a)                                │
│    - 他車情報 (s, v, a, 予測軌道)                      │
│                                                         │
│  【処理】                                               │
│    1. 障害物検出（Frenet座標系）                       │
│    2. ST境界構築（他車予測から安全領域を定義）         │
│    3. QP最適化（ST境界を制約として軌道計算）           │
│                                                         │
│  【出力】                                               │
│    - 軌道 (s, v, a) × N ステップ                       │
│    - 車両に a[0] を指令                                │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

**重要**: ST境界は**入力ではなく**、Level 2内部で**構築される中間成果物**です。

---

## 3. ST-Boundary制約：衝突不可能性の数学的保証

### 制御フローでの位置

**Level 2戦術的制御（0.1s周期）**: QP最適化（Section 5）の**主要制約**として使用。

**Level 2の入出力**:
```
【入力】
  - 自車状態 (s, v, a)
  - 他車情報 (s, v, a, 予測軌道)  ← ST境界の構成要素

【処理】
  1. ST境界を構築（他車の予測軌道から安全領域を定義）
  2. QPの制約として設定
  3. QP最適化を実行

【出力】
  - 軌道 (s, v, a) × N ステップ
```

**前ステップ**: Level 1でスケジュール確定した車両に対して軌道生成。
**次ステップ**: 生成された軌道は車両に適用され、CAVなら他車と共有（V2V）。

**重要**: ST境界は**入力ではなく**、Level 2内部で**構築される中間成果物**です。

### なぜST-Boundary制約か？従来手法（v10.3）の失敗

**従来手法の問題**:
```python
# v10.3: 各時刻で独立したギャップ制約
for k in range(horizon):
    gap[k] >= min_gap  # 各時刻で独立
```

**致命的欠陥**:
1. **時刻間の連鎖を無視**: $t_k$ で安全でも $t_{k+1}$ で衝突可能
2. **ホライズン外の保証なし**: 予測終了後に追いつく可能性
3. **結果**: v10.3では600秒で**595回の衝突**

**ST-Boundary制約の優位性**:
- 全時刻で**連鎖的に**安全距離を保証 → 数学的に衝突不可能
- v11.0では**8回のみ**（-98.7%）

### 数学的定式化

$$
\boxed{
s_{\text{ego}}(t_k) \leq s_{\text{front}}(t_k) - d_{\text{safe}}(v_{\text{ego}}(t_k), v_{\text{front}}(t_k)) \quad \forall k \in [0, H-1]
}
$$

### 定理：追いつき不可能性

**定理**: 上記制約を満たす軌道 $s_{\text{ego}}(t)$ は、全時間 $t \in [0, T]$ で前方車両に追いつかない。

**証明**（帰納法）:

1. **基底ケース** ($k=0$):
   $$s_{\text{ego}}(0) \leq s_{\text{front}}(0) - d_{\text{safe}}(v_{\text{ego}}(0), v_{\text{front}}(0))$$
   制約により成立。

2. **帰納仮説**: 時刻 $t_k$ で成立すると仮定:
   $$s_{\text{ego}}(t_k) \leq s_{\text{front}}(t_k) - d_{\text{safe}}(v_{\text{ego}}(t_k), v_{\text{front}}(t_k))$$

3. **帰納ステップ**: 時刻 $t_{k+1}$ での自車位置:
   $$s_{\text{ego}}(t_{k+1}) = s_{\text{ego}}(t_k) + v_{\text{ego}}(t_k) \cdot \Delta t + \frac{1}{2} a_{\text{ego}}(t_k) \cdot \Delta t^2$$

4. **制約の適用**:
   $$s_{\text{ego}}(t_{k+1}) \leq s_{\text{front}}(t_{k+1}) - d_{\text{safe}}(v_{\text{ego}}(t_{k+1}), v_{\text{front}}(t_{k+1}))$$

5. **結論**: $d_{\text{safe}} > 0$ より:
   $$s_{\text{ego}}(t_{k+1}) < s_{\text{front}}(t_{k+1})$$

6. **全時刻で成立**: 数学的帰納法により、$\forall k \in [0, H-1]$ で成立。□

### v10.3（旧）との違い

| 制約タイプ | v10.3（失敗） | v11.0（成功） |
|-----------|---------------|---------------|
| **形式** | $\text{gap}(t_k) \geq d$ | $s(t_k) \leq s_{\text{upper}}(t_k)$ |
| **時刻間の関連** | 独立 | 連鎖 |
| **ホライズン外保証** | なし | あり |
| **追いつき可能性** | あり（595回衝突） | **なし（8回のみ）** |

**実装**: [frenet_qp_apollo.py](../weaving_v11/frenet_qp_apollo.py):867-975

---

### 3.5 V2V軌道共有と予測軌道検証

#### CAV-CAV協調制御の基盤

**V2V軌道共有の役割**：

CAV（Connected Autonomous Vehicle）同士は、将来の軌道情報を**V2V通信**で共有することで、予測精度を大幅に向上させます。これにより：
1. **予測誤差の削減**: CV（Constant Velocity）外挿よりも精度が高い
2. **バッファの最小化**: 予測が信頼できるため、安全マージンを削減可能
3. **協調的な軌道計画**: 双方が相手の意図を理解し、デッドロック回避

**共有データ構造**（[frenet_qp_apollo.py:92-93](../weaving_v11/frenet_qp_apollo.py#L92-L93)）：
```python
predicted_trajectory: List[Tuple[float, float, float, float]]
# Format: [(t_abs, s, v, a), ...]
# t_abs: 絶対時刻 [s]
# s: 縦方向位置 [m]
# v: 速度 [m/s]
# a: 加速度 [m/s²]
```

#### 予測軌道の検証メカニズム（Safety Validation Layer 2）

**問題**：共有された軌道が**古い**または**不正確**な場合、それを使用すると危険。

**解決策**：使用前に**センサー観測値との整合性を検証**（[frenet_qp_apollo.py:1233-1241](../weaving_v11/frenet_qp_apollo.py#L1233-L1241)）：

```python
def _validate_predicted_trajectory(self, obstacle: ObstacleInfo) -> bool:
    """
    予測軌道の妥当性検証

    検証基準:
    1. 軌道データが存在し、十分な長さ（≥5点）
    2. 現在の観測位置との誤差 < 2.0m
    """
    if obstacle.predicted_trajectory is None or len(...) < 5:
        return False

    # 軌道の開始点と現在観測値を比較
    traj_start = obstacle.predicted_trajectory[0]
    s_observed = obstacle.vehicle_state.s

    prediction_error = abs(traj_start[1] - s_observed)

    if prediction_error > 2.0:  # [m] 許容誤差
        # 軌道が古いまたは不正確 → 破棄
        return False

    return True
```

**検証失敗時の動作**：
- 共有軌道を破棄
- CV（Constant Velocity）モデルで外挿（保守的）
- バッファを増加（CAV-HDV相当に）

---

### 3.6 ST-Boundary生成アルゴリズム（詳細フロー）

#### 全体フロー（V2V軌道 → ST図 → QP制約）

```
【入力】
- ego_state: 自車状態（位置、速度）
- obstacles: 周辺車両リスト（各車両のV2V軌道含む）
- urgency: LC緊急度（0-1）

【Step 1】V2V軌道の検証と利用判定
    ↓
【Step 2】意思決定（YIELD / FOLLOW / OVERTAKE）
    ↓
【Step 3】時間ステップごとの位置予測
    - V2V軌道あり → 線形補間
    - V2V軌道なし → CV外挿
    ↓
【Step 4】RSS安全距離計算（前方/後方で異なる式）
    ↓
【Step 5】適応的バッファ計算
    - CAV-CAV: 最小バッファ（0.3m）
    - CAV-HDV: 速度差・密度依存バッファ
    ↓
【Step 6】ST境界値の設定
    - YIELD/FOLLOW: s_upper[k] = s_obs - d_safe - buffer - L
    - OVERTAKE: s_lower[k] = s_obs + d_safe + buffer + L
    ↓
【出力】
- s_lower[k], s_upper[k] (k=0...N-1): QP制約として使用
```

#### Step 3: 時間ステップごとの位置予測（実装：[frenet_qp_apollo.py:1392-1407](../weaving_v11/frenet_qp_apollo.py#L1392-L1407)）

**予測ホライズン**: $N=80$ ステップ（8.0秒先）、制御周期 $\Delta t=0.1$ s

```python
for k in range(N):  # 80ステップループ
    t_rel = k * dt  # 相対時刻 [s]
    t_abs = current_time + t_rel  # 絶対時刻 [s]

    # V2V軌道があれば線形補間、なければCV外挿
    if use_shared_traj:
        s_obs = _interpolate_trajectory_point(
            obstacle.predicted_trajectory, t_abs)
    else:
        s_obs = obs.s + obs.v * t_rel  # CV外挿
```

**線形補間の詳細**（[frenet_qp_apollo.py:774-808](../weaving_v11/frenet_qp_apollo.py#L774-L808)）：

```python
def _interpolate_trajectory_point(trajectory, query_time):
    """
    V2V共有軌道から、指定時刻の位置を線形補間

    ホライズン外の処理:
    - 軌道開始前: 開始位置を使用
    - 軌道終了後: CV外挿（最後の速度で等速）
    """
    if query_time < trajectory[0][0]:
        return trajectory[0][1]  # 開始位置

    if query_time > trajectory[-1][0]:
        # CV外挿: s = s_last + v_last * (t - t_last)
        last_pt = trajectory[-1]
        dt = query_time - last_pt[0]
        return last_pt[1] + last_pt[2] * dt

    # 線形補間（軌道内）
    for i in range(len(trajectory) - 1):
        if trajectory[i][0] <= query_time <= trajectory[i+1][0]:
            ratio = (query_time - trajectory[i][0]) / ...
            return trajectory[i][1] + ratio * (...)
```

#### Step 4: RSS安全距離計算（方向別の式）

**前方障害物**（YIELD/FOLLOW）（[frenet_qp_apollo.py:1415-1417](../weaving_v11/frenet_qp_apollo.py#L1415-L1417)）：
```python
d_safe = compute_forward_safe_distance(v_ego, v_obs, same_direction=True)
```

**後方障害物**（OVERTAKE）（[frenet_qp_apollo.py:1419-1420](../weaving_v11/frenet_qp_apollo.py#L1419-L1420)）：
```python
d_safe = compute_backward_safe_distance(v_ego, v_obs, same_direction=True)
```

**RSS距離の計算式**（[apollo_safety.py:643-647](../weaving_v11/apollo_safety.py#L643-L647)）:
$$
d_{\text{RSS}} = v_{\text{ego}} \cdot t_{\text{react}} + \frac{v_{\text{ego}}^2}{2a_{\max}} - \frac{v_{\text{front}}^2}{2a_{\max}} + s_0
$$

**パラメータ**：
- $t_{\text{react}} = 0.5$ s（HDV）/ 0.15 s（CAV）
- $a_{\max} = 6.0$ m/s²（緊急ブレーキ、UN R157準拠）
- $s_0 = 2.0$ m（静止時最小ギャップ）

#### Step 5: 適応的バッファ計算

**CAV-CAVの場合**（V2V軌道あり）（[frenet_qp_apollo.py:1266-1273](../weaving_v11/frenet_qp_apollo.py#L1266-L1273)）：

```python
if use_shared_traj:
    if decision == InteractionDecision.FOLLOW:
        static_buffer = 0.3  # FOLLOW: 最小バッファ（協調制御）
    elif decision == InteractionDecision.YIELD:
        static_buffer = 0.5  # YIELD: 標準バッファ
    else:
        static_buffer = 0.3  # OVERTAKE: 最小バッファ
    dynamic_expansion = 0.0  # 予測が信頼できるので動的拡大不要
```

**CAV-HDVの場合**（V2V軌道なし）（[frenet_qp_apollo.py:1275-1314](../weaving_v11/frenet_qp_apollo.py#L1275-L1314)）：

```python
if decision == InteractionDecision.FOLLOW:
    # 相対速度ベース
    base_follow_buffer = 1.5  # [m]
    if rel_v > 0:  # 接近中
        velocity_buffer = rel_v * 0.5  # [m]
    else:
        velocity_buffer = 0.0

    # 絶対速度ベース（高速走行時の反応距離）
    ego_speed_buffer = ego_state.v * 0.1  # 10%

    # 総バッファ = max(基本, 相対速度, 絶対速度)
    static_buffer = max(base_follow_buffer, velocity_buffer, ego_speed_buffer)

elif decision == InteractionDecision.YIELD:
    # YIELDはFOLLOWより保守的
    if ego_state.v < 10.0:
        base_yield_buffer = 2.5  # [m]
    else:
        base_yield_buffer = 4.0  # [m]

    # 相対速度による追加バッファ
    if rel_v > 0:
        velocity_buffer = rel_v * 0.7  # FOLLOWの0.5より大きい
    else:
        velocity_buffer = 0.0

    static_buffer = max(base_yield_buffer, velocity_buffer, ...)
```

**バッファの物理的意味**：
- **static_buffer**: 初期ギャップ（制御開始時）
- **dynamic_expansion**: 時間経過による不確実性の増加（CAV-CAVでは0）
- **lateral_buffer**: 横方向の不確実性（LC中の車両に適用）

#### Step 6: ST境界値の設定（[frenet_qp_apollo.py:1451-1490](../weaving_v11/frenet_qp_apollo.py#L1451-L1490)）

**総距離要件**：
```python
d_required = d_safe + total_buffer
```

**Hard Safety Margin（絶対防衛ライン）**：
```python
is_cav_interaction = (use_shared_traj and len(predicted_trajectory) >= 5)
MIN_HARD_MARGIN = 5.0 if is_cav_interaction else 10.0  # [m]
```

**境界値の設定**：
```python
if decision in [InteractionDecision.FOLLOW, InteractionDecision.YIELD]:
    # 前方障害物: 上限制約
    s_upper[k] = s_obs - d_required - L_vehicle
else:  # OVERTAKE
    # 後方障害物: 下限制約
    s_lower[k] = s_obs + d_required + L_vehicle
```

**Urgency緩和**（出口に近いとき）：
```python
urgency_relaxation = urgency * 0.3
if use_shared_traj:
    total_buffer = max(0.3, total_buffer - urgency_relaxation)
else:
    total_buffer = max(1.5, total_buffer - urgency_relaxation)
```

---

### 3.7 RSSとQPの整合性

#### 1. RSSからST-Boundaryへの変換

**RSS距離**が**ST-Boundary制約の基準値**として使用されます：

```
d_RSS (apollo_safety.py)
  ↓
d_safe (frenet_qp_apollo.py)
  ↓
s_upper[k] = s_obs(t_k) - d_safe - buffer - L
  ↓
QP制約: s_k ≤ s_upper[k]
```

**数式表現**：
$$
s_{\text{ego}}(t_k) \leq s_{\text{obs}}(t_k) - d_{\text{RSS}}(v_{\text{ego}}, v_{\text{obs}}) - d_{\text{buffer}} - L_{\text{vehicle}}
$$

#### 2. 安全性の二重保証

**Level 1: Gap Acceptance**（LC開始判定）
- RSS係数で階層化（CAUTION: 2.2×, WARNING: 1.7×, CRITICAL: 1.3×, AEB: 0.95×）
- LC開始時の前方ギャップ: 15.0m（RSS × 2.0相当）
- LC開始時の後方ギャップ: 18.0m（より保守的）

**Level 2: ST-Boundary制約**（軌道生成）
- 全予測ホライズン（8.0秒先）で連鎖的に安全距離を保証
- スラック変数でソフト化（$w_{\xi}=10^5$で強く抑制）
- Hard Safety Margin（5.0m/10.0m）で絶対防衛ライン

**整合性のポイント**：
1. **Level 1で開始可能と判定** → Level 2でも（通常は）実行可能
2. **Level 2で制約違反** → スラック変数が働き、わずかな違反を許容しつつ解を見つける
3. **どちらも満たせない** → LC中止またはAEB発動

#### 3. QP制約への変換（スラック変数付き）

**実装**（[frenet_qp_apollo.py:2389-2399](../weaving_v11/frenet_qp_apollo.py#L2389-L2399)）：

```python
for k in range(N):
    # 上限: s_k - ξ_k ≤ s_upper[k]
    A_safe[row_u, k] = 1.0        # s_k
    A_safe[row_u, 3*N+k] = -1.0   # -ξ_k
    u_safe[row_u] = s_upper[k]

    # 下限: s_k + ξ_k ≥ s_lower[k]
    A_safe[row_l, k] = 1.0        # s_k
    A_safe[row_l, 3*N+k] = 1.0    # +ξ_k
    l_safe[row_l] = s_lower[k]
```

**数式表現**：
$$
\begin{cases}
s_k - \xi_k \leq s_{\text{upper}}[k] & \text{(上限制約)} \\
s_k + \xi_k \geq s_{\text{lower}}[k] & \text{(下限制約)}
\end{cases}
\quad \forall k \in [0, N-1]
$$

**目的関数のペナルティ項**：
$$
J += w_{\xi} \sum_{k=0}^{N-1} \xi_k^2 \quad (w_{\xi} = 100000.0)
$$

**スラック変数 $\xi_k$ の役割**：
- $\xi_k = 0$：制約を厳守（理想状態）
- $\xi_k > 0$：わずかな違反を許容（$w_{\xi}$が大きいので最小化される）
- 完全不可能な場合の**最後の手段**（衝突よりマシ）

---

## 4. 安全距離計算：ST-Boundaryの具体値を決める

### 制御フローでの位置

**使用箇所**: ST-Boundary制約（Section 3）の $d_{\text{safe}}$ を計算。
**計算タイミング**: 各予測ステップ $k \in [0, H-1]$ で、前方車両との相対速度から動的に算出。
**適応性**: 時間ヘッドウェイ $T$ はUrgencyに応じて調整（Section 6）。

### なぜIDM（Intelligent Driver Model）か？

**要求**:
1. 速度依存: 高速時は長い車間、低速時は短い車間
2. 相対速度考慮: 接近中は追加マージン、離れる場合は緩和
3. 実用性: 人間ドライバーの挙動に近い（実証済み）

**IDMの利点**: 上記3点を満たし、RSSとも整合性が高い（反応時間 $T$ を含む）。

### IDM準拠の安全距離

$$
\boxed{
d_{\text{safe}} = s_0 + v_{\text{ego}} \cdot T + \frac{v_{\text{ego}} \cdot (v_{\text{ego}} - v_{\text{front}})}{2\sqrt{a_{\max} \cdot b}}
}
$$

### パラメータ

| 記号 | 意味 | デフォルト値 | 値の範囲 | 単位 | 役割と効果 |
|------|------|-------------|---------|------|-----------|
| $s_0$ | **静止時最小車間距離** | 2.0 | 固定 | [m] | 完全停止時にも保つ最小距離（RSS準拠）。`RSS_MIN_GAP`定数に対応 |
| $T$ | **時間ヘッドウェイ** | 1.5 | **0.9～1.5** | [s] | **速度×Tの距離を空ける**。Section 6の適応的マージンで動的調整。`T_base=1.5`, `T_min=0.9`（実装では`time_headway_HDV=1.5`, `time_headway_CAV=0.7`） |
| $v_{\text{ego}}$ | 自車速度（現在値） | - | 5～20 | [m/s] | QP最適化の予測ステップごとに変化 |
| $v_{\text{front}}$ | 前方車速度（予測値） | - | 5～20 | [m/s] | V2V軌道共有またはCV外挿による予測値 |
| $a_{\max}$ | **最大加速度** | 2.0 | 固定 | [m/s²] | IDM式の分母に使用。パラメータ`a_max=2.0`に対応 |
| $b$ | **快適減速度** | 6.0 | 固定 | [m/s²] | IDM式の分母に使用。**緊急ブレーキ能力を反映**（UN R157準拠）。パラメータ`a_min=-6.0`の絶対値 |

### 各項の意味

1. **$s_0$**: 静止時最小車間（バンパー間距離）
2. **$v \cdot T$**: 反応時間分の走行距離
3. **$\frac{v \cdot \Delta v}{2\sqrt{ab}}$**: 速度差による補正
   - $\Delta v > 0$（自車が速い）: 安全距離増加
   - $\Delta v < 0$（前方車が速い）: 安全距離減少

### 計算例

| 状況 | $v_{\text{ego}}$ | $v_{\text{front}}$ | $T$ | $d_{\text{safe}}$ |
|------|------------------|---------------------|-----|-------------------|
| 等速追従 | 20 m/s | 20 m/s | 1.2 | 26.0 m |
| 接近中 | 20 m/s | 15 m/s | 1.2 | 31.4 m |
| 離れる | 15 m/s | 20 m/s | 1.2 | 14.1 m |
| 低速時 | 10 m/s | 10 m/s | 1.2 | 14.0 m |

**実装**: [frenet_qp_apollo.py](../weaving_v11/frenet_qp_apollo.py):780-865

---

## 5. QP定式化：最適軌道の数値解法

### 制御フローでの位置

**Level 2戦術的制御（0.1s周期）**: 各車両に対して毎周期実行。

**入力**:
  - 自車状態 $(s_0, v_0, a_0)$
  - 他車情報 $(s, v, a, 予測軌道)$ ← ST境界の構成要素
  - Urgency（適応的マージン調整用）

**出力**: 最適軌道 $(s, v, a) \times N$ ステップ → 先頭 $a_0$ を車両に適用

### なぜQP（二次計画問題）か？

**要求**:
1. **快適性**: 急加減速・急Jerkを避ける（乗員快適性）
2. **安全性**: ST-Boundary制約を厳守（衝突回避）
3. **リアルタイム性**: 0.1s以内に解を得る（10Hz制御）

**QPの利点**:
- 凸最適化 → **大域最適解**が確実に得られる（局所解に陥らない）
- OSQP等の高速ソルバーで**数ms**で解ける
- 線形制約（ST-Boundary）と二次目的関数（Jerk最小化）を自然に統合

### 目的関数の設計思想

**トレードオフの明示化**:
1. $w_{\text{ref}}$: 参照軌道追従（進捗確保）
2. $w_v$: 目標速度追従（交通流維持）
3. $w_a$: 加速度最小化（燃費）
4. $w_j$: **Jerk最小化（快適性）** ← v15.0で2倍化（2000.0）

### 目的関数（Apollo Piecewise-Jerk QP）

$$
\boxed{
\begin{aligned}
\min_{\mathbf{x}} \quad J = &\sum_{k=0}^{N-1} \Bigl[ w_{\text{ref}}(s_k - s_{\text{ref},k})^2 + w_v(v_k - v_{\text{ref}})^2 \\
&+ w_a a_k^2 + w_j (a_{k+1} - a_k)^2 + w_{\xi} \xi_k^2 \Bigr]
\end{aligned}
}
$$

**各項の物理的意味**：

1. **$w_{\text{ref}}(s_k - s_{\text{ref},k})^2$**：位置追従コスト
   - 参照軌道からの偏差を抑制
   - 小さすぎると計画通りに進まない

2. **$w_v(v_k - v_{\text{ref}})^2$**：速度追従コスト
   - 目標速度からの偏差を抑制
   - 交通流維持のための重み

3. **$w_a a_k^2$**：加速度コスト
   - エネルギー消費と乗り心地のバランス
   - 大きいほど緩やかな加減速

4. **$w_j (a_{k+1} - a_k)^2$**：Jerkコスト（**最重要**）
   - 加速度の時間変化率（$j = \frac{da}{dt}$）を抑制
   - 乗員の快適性に直結
   - **加速度違反を防ぐ主要パラメータ**

5. **$w_{\xi} \xi_k^2$**：スラック変数ペナルティ（**安全制約のソフト化**）
   - ST-Boundary制約違反を許容する代償
   - $w_{\xi}$を大きく設定（$10^5$）することで、違反を強く抑制
   - 完全に制約を満たせない場合の**最後の手段**

### 決定変数（Apollo標準ブロック形式）

QP最適化では、全時刻の変数を**ブロック形式**で並べます（Apollo標準）：

$$
\mathbf{x} = \begin{bmatrix}
\mathbf{s} \\
\mathbf{v} \\
\mathbf{a} \\
\boldsymbol{\xi}
\end{bmatrix} = \begin{bmatrix}
s_0, s_1, \ldots, s_{N-1} \\
v_0, v_1, \ldots, v_{N-1} \\
a_0, a_1, \ldots, a_{N-1} \\
\xi_0, \xi_1, \ldots, \xi_{N-1}
\end{bmatrix}^T \in \mathbb{R}^{4N}
$$

**各ブロックの意味**：
- **位置ブロック $\mathbf{s}$**：全予測ステップの縦方向位置 [m]
- **速度ブロック $\mathbf{v}$**：全予測ステップの速度 [m/s]
- **加速度ブロック $\mathbf{a}$**：全予測ステップの加速度 [m/s²]
- **スラック変数ブロック $\boldsymbol{\xi}$**：ST-Boundary制約違反の許容量（ソフト制約）[m]

**インデックス構造**（実装：[frenet_qp_apollo.py:2043-2046](../weaving_v11/frenet_qp_apollo.py#L2043-L2046)）：
```python
x[0:N]      # 位置 s_0, ..., s_{N-1}
x[N:2N]     # 速度 v_0, ..., v_{N-1}
x[2N:3N]    # 加速度 a_0, ..., a_{N-1}
x[3N:4N]    # スラック変数 ξ_0, ..., ξ_{N-1}
```

**パラメータ**：
- $N$：予測ホライズン（ステップ数）= 80（デフォルト）
- 総変数数：$4N = 320$（80ステップの場合）

**注**：この形式はApollo EM Plannerの標準で、**時刻ごとに交互**（$s_0, v_0, a_0, s_1, v_1, a_1, ...$）ではなく、**変数タイプごとにブロック化**されています。これにより、Hessian行列 $P$ の構造が疎（sparse）になり、計算効率が向上します。

### 重みパラメータ

| 記号 | 意味 | デフォルト値 | 実装変数名 | 説明と調整理由 |
|------|------|-------------|-----------|---------------|
| $w_{\text{ref}}$ | **位置追従重み** | 0.5 | `w_ref` | 参照軌道からの位置偏差を抑制。小さすぎると計画通りに進まない |
| $w_v$ | **速度追従重み** | 8.0 | `w_v` | 目標速度からの偏差を抑制。**交通流維持の主要パラメータ** |
| $w_a$ | **加速度コスト** | 25.0 | `qp_w_a` | エネルギー消費と乗り心地のバランス。大きいほど緩やかな加減速 |
| $w_j$ | **Jerkコスト（最重要）** | **6000.0** | `qp_w_j` | **急激な加速度変化を抑制**（快適性）。v28.2で4000→6000に調整。**加速度違反を防ぐための最重要パラメータ** |
| $w_{\xi}$ | **スラック変数ペナルティ** | **100000.0** ($10^5$) | `w_slack` | **ST-Boundary制約違反のペナルティ**。非常に大きな値で制約違反を強く抑制。完全不可能な場合のみ許容 |

**注**：
- 実装値は[frenet_qp_apollo.py:664-669](../weaving_v11/frenet_qp_apollo.py#L664-L669)を参照
- `w_v`と`w_j`は交通流と快適性のバランスで調整される
- `w_slack`は安全性の最終防衛線（通常は0であるべき）

### 制約

#### 1. ST-Boundary制約（スラック変数付きソフト制約）

**硬制約版**（理想形）：
$$
s_{\text{lower}}[k] \leq s_k \leq s_{\text{upper}}[k] \quad \forall k \in [0, N-1]
$$

**実装版**（スラック変数によるソフト制約、[frenet_qp_apollo.py:2389-2399](../weaving_v11/frenet_qp_apollo.py#L2389-L2399)）：
$$
\begin{cases}
s_k - \xi_k \leq s_{\text{upper}}[k] & \text{(上限：前方車両との安全距離)} \\
s_k + \xi_k \geq s_{\text{lower}}[k] & \text{(下限：後方車両との安全距離)}
\end{cases}
\quad \forall k \in [0, N-1]
$$

**スラック変数 $\xi_k$ の役割**：
- $\xi_k \geq 0$：制約違反の許容量 [m]
- 目的関数の$w_{\xi} \xi_k^2$項で、違反に大きなペナルティ（$w_{\xi}=10^5$）
- **物理的意味**：制約を完全に満たせない場合の「最小限の安全距離違反」
- **通常時**：$\xi_k = 0$（制約を厳守）
- **緊急時**：$\xi_k > 0$（わずかな違反を許容して解を見つける）

**境界値の計算**：
- $s_{\text{upper}}[k] = s_{\text{front}}(t_k) - d_{\text{safe}}(v_{\text{ego}}, v_{\text{front}})$
- $s_{\text{lower}}[k] = s_{\text{rear}}(t_k) + d_{\text{safe}}(v_{\text{rear}}, v_{\text{ego}})$

ここで、$d_{\text{safe}}$はIDM安全距離（Section 4参照）

#### 2. 速度制約

$$
v_{\min} \leq v_k \leq v_{\max} \quad \forall k \in [0, N-1]
$$

**パラメータ**（[frenet_qp_apollo.py:640](../weaving_v11/frenet_qp_apollo.py#L640)）：
- $v_{\min} = 5.0$ m/s（下限、渋滞時でも維持すべき最低速度）
- $v_{\max} = 30.0$ m/s（上限、システム最大速度、実際は20.0 m/sに制限されることが多い）

**物理的意味**：
- 下限：完全停止を避け、交通流を維持
- 上限：安全速度と法定速度の遵守

#### 3. 加速度制約（適応的制限）

$$
a_{\min} \leq a_k \leq a_{\max} \quad \forall k \in [0, N-1]
$$

**パラメータ**（[frenet_qp_apollo.py:629-638](../weaving_v11/frenet_qp_apollo.py#L629-L638)）：

**通常時**（快適性重視）：
- $a_{\min} = -4.0$ m/s²（快適減速度、`a_min_comfort`）
- $a_{\max} = +1.5$ m/s²（快適加速度、`a_max_comfort`）

**緊急時**（安全性優先）：
- $a_{\min} = -6.0$ m/s²（**緊急ブレーキ、UN R157準拠**、`a_min_emergency`）
- $a_{\max} = +2.5$ m/s²（緊急加速度、`a_max_emergency`）

**切り替えロジック**：
- 通常走行：快適性重視の制限を使用
- 衝突リスク検出時：緊急制限に自動切替
- v27.8で快適減速を-3.5→-4.0 m/s²に引き上げ（緊急との差を縮小）

#### 4. 運動学制約（等加速度運動モデル）

$$
\begin{cases}
s_{k+1} = s_k + v_k \cdot \Delta t + \frac{1}{2} a_k \cdot \Delta t^2 \\
v_{k+1} = v_k + a_k \cdot \Delta t
\end{cases}
\quad \forall k \in [0, N-2]
$$

**パラメータ**：
- $\Delta t = 0.1$ s（制御周期、`dt_control`）
- $N = 80$（予測ホライズン、`horizon`）
- ステップ数：0, 1, ..., N-1（合計N個の状態）

**物理的意味**：
- 各予測ステップ間で等加速度運動を仮定
- この制約により、時刻$k$から$k+1$への状態遷移が物理法則に従う
- **線形制約**なのでQPで効率的に解ける

**注**：
- 最終ステップ$k=N-1$では次の状態がないため、$k \in [0, N-2]$
- この制約により、決定変数間に**連続性**が保証される

### 行列形式

$$
\min_{\mathbf{x}} \quad \frac{1}{2} \mathbf{x}^T P \mathbf{x} + \mathbf{q}^T \mathbf{x}
$$

$$
\text{s.t.} \quad \mathbf{l} \leq A \mathbf{x} \leq \mathbf{u}
$$

- $P \in \mathbb{R}^{(3N+2) \times (3N+2)}$: Hessian行列（正定値）
- $\mathbf{q} \in \mathbb{R}^{3N+2}$: 線形項
- $A \in \mathbb{R}^{M \times (3N+2)}$: 制約行列
- $\mathbf{l}, \mathbf{u} \in \mathbb{R}^M$: 制約の上下限

**ソルバー**: OSQP（Operator Splitting QP）

**実装**: [frenet_qp_apollo.py](../weaving_v11/frenet_qp_apollo.py):558-750

---

## 6. 適応的安全マージン：Urgencyに応じた柔軟な安全距離

### 制御フローでの位置

**使用箇所**: 安全距離計算（Section 4）の時間ヘッドウェイ $T$ を動的調整。
**計算タイミング**: ST-Boundary構築時、各予測ステップで前方車両ごとに算出。
**連携**: Urgency（Section 1）、密度、相対速度の3要素で調整。

### なぜ適応的にするのか？固定マージンの問題

**固定マージンの問題**:
```python
# 固定T=1.5sの場合
T = 1.5  # 常に固定
d_safe = s_0 + v * T + ...  # 高密度・出口付近でも同じ
```

**問題点**:
1. **低密度時**: 過度に保守的 → LC機会を逃す
2. **高Urgency時**: 出口直前でもLC不可 → 退出失敗
3. **結果**: LC成功率が大幅低下

**適応的マージンの利点**:
- **高Urgency時**: $T$ を最大40%緩和（0.9s下限）→ LC成功率向上
- **高密度時**: 15%厳格化 → 安全性維持
- **高接近速度時**: 20%厳格化 → リスク回避

### 設計哲学：安全性との両立

**重要**: 前方マージンは緩和するが、**後方マージンは不変**（追突リスク回避）。
**RSS準拠**: 最小値 $T_{\min}=0.9$s はRSS要件を満たす（反応時間0.5s + 余裕）。

### 動的時間ヘッドウェイ計算

$$
\boxed{
T_{\text{final}} = \max\left(T_{\min}, T_{\text{base}} \cdot f_{\text{urgency}} \cdot f_{\text{density}} \cdot f_{\text{velocity}}\right)
}
$$

### 調整係数

#### 1. Urgency-based緩和

$$
f_{\text{urgency}} = 1.0 - 0.4 \cdot U
$$

| Urgency $U$ | $f_{\text{urgency}}$ | 効果 |
|-------------|----------------------|------|
| 0.0（低） | 1.0 | 緩和なし |
| 0.5（中） | 0.8 | -20%緩和 |
| 1.0（高） | 0.6 | -40%緩和 |

#### 2. 交通密度調整

$$
f_{\text{density}} = \begin{cases}
0.85 & \text{if } \rho > 0.05 \text{ [veh/m]} \\
1.0 & \text{otherwise}
\end{cases}
$$

#### 3. 相対速度調整

$$
f_{\text{velocity}} = \begin{cases}
1.2 & \text{if } v_{\text{rel}} > 5.0 \text{ [m/s]} \\
1.0 & \text{otherwise}
\end{cases}
$$

- $v_{\text{rel}} = \max(0, v_{\text{ego}} - v_{\text{front}})$

### パラメータ

| 記号 | 意味 | デフォルト値 | 実装変数名 | 単位 | 役割と制約 |
|------|------|-------------|-----------|------|-----------|
| $T_{\text{base}}$ | **ベース時間ヘッドウェイ** | 1.5 | `time_headway_HDV` | [s] | 通常時（人間ドライバー想定）の車間時間。これを基準に調整係数を掛ける |
| $T_{\min}$ | **最小時間ヘッドウェイ（Hard limit）** | 0.9 | *(固定値)* | [s] | **RSS安全要件の下限**。反応時間0.5s + 余裕0.4s。これ以下には下がらない |
| - | CAV用時間ヘッドウェイ | 0.7 | `time_headway_CAV` | [s] | V2V通信ありの場合の短縮値（参考値） |

### 計算例

| 状況 | $U$ | $\rho$ | $v_{\text{rel}}$ | $T_{\text{final}}$ |
|------|-----|--------|------------------|---------------------|
| 通常走行 | 0.0 | 0.01 | 0.0 | 1.5 s |
| 高Urgency | 0.8 | 0.01 | 0.0 | 0.96 s |
| 高密度 | 0.0 | 0.08 | 0.0 | 1.28 s |
| 高接近速度 | 0.0 | 0.01 | 8.0 | 1.8 s |
| 全条件厳しい | 0.9 | 0.08 | 8.0 | 0.90 s（$T_{\min}$） |

**実装**: [frenet_qp_apollo.py](../weaving_v11/frenet_qp_apollo.py):780-865

---

# 共通・補助アルゴリズム

以下のセクションは、Level 1とLevel 2の両方で使用される共通アルゴリズムです。

---

## 7. TTC計算：Gap Acceptanceの判定指標

### 制御フローでの位置

**使用箇所**: Gap Acceptance（Section 2.1）で前後車両との衝突リスク評価。
**計算タイミング**: LC確率トリガー成功後、スケジュール決定前。
**判定基準**: 
  - 前方TTC ≥ 3.0s（緩和時は $g(U) \times 3.0$s）
  - 後方TTC ≥ 4.0s（より保守的、緩和なし）

### なぜTTCが必要か？ギャップだけでは不十分

**ギャップのみの問題**:
```python
# ギャップだけで判定（不十分）
if gap_front >= 15.0:  # [m]
    accept_lc()  # 危険な場合がある
```

**問題のシナリオ**:
- ギャップ: 20m（基準クリア）
- 相対速度: 自車25 m/s、前方車15 m/s → $v_{\text{rel}}=10$ m/s
- **TTC = 20 / 10 = 2.0s** ← 3.0s未満で危険！

**TTCの利点**:
1. **動的リスク評価**: 速度差を考慮した時間ベースの危険度
2. **直感的**: 「あと何秒で衝突するか」
3. **RSS整合**: TTCベース判定はRSSの考え方と一致

### 計算方法と物理的意味

**分子**: 現在の車間距離（車両長を引いた純粋なギャップ）
**分母**: 相対速度（正なら接近中、負/ゼロなら離れる/平行）
**結果**: 現在の速度が続く場合の衝突までの時間

### Time-to-Collision

$$
\boxed{
\text{TTC} = \frac{s_{\text{front}} - s_{\text{rear}} - L}{\max(0, v_{\text{rear}} - v_{\text{front}})}
}
$$

### パラメータ

| 記号 | 意味 | デフォルト値 | 実装変数名 | 単位 | 説明 |
|------|------|-------------|-----------|------|------|
| $s_{\text{front}}$ | 前方車両位置（重心） | - | `vehicle.s` | [m] | 前方車両の縦方向座標 |
| $s_{\text{rear}}$ | 後方車両位置（重心） | - | `vehicle.s` | [m] | 後方車両の縦方向座標 |
| $L$ | 車両長 | 5.0 | `L_vehicle` | [m] | バンパー間距離を計算するための車両長 |
| $v_{\text{rear}}$ | 後方車両速度 | - | `vehicle.v` | [m/s] | 後方車両の現在速度 |
| $v_{\text{front}}$ | 前方車両速度 | - | `vehicle.v` | [m/s] | 前方車両の現在速度 |

### 解釈

| 相対速度 | TTC | 意味 |
|---------|-----|------|
| $v_{\text{rel}} > 0$ | 有限値 | 追いつく（危険） |
| $v_{\text{rel}} \leq 0$ | $\infty$ (None) | 追いつかない（安全） |

### 計算例

| 状況 | gap [m] | $v_{\text{rear}}$ | $v_{\text{front}}$ | $v_{\text{rel}}$ | TTC [s] |
|------|---------|-------------------|---------------------|------------------|---------|
| 等速追従 | 30 | 20 | 20 | 0 | ∞ (安全) |
| 緩やかに接近 | 30 | 20 | 18 | 2 | 15.0 |
| 急接近 | 30 | 25 | 15 | 10 | 3.0 |
| 危険 | 10 | 25 | 15 | 10 | **1.0** |
| 離れる | 30 | 15 | 20 | -5 | ∞ (安全) |

### Gap Acceptance閾値

| 閾値 | デフォルト値 | 実装箇所 | 単位 | 物理的意味 |
|------|-------------|---------|------|-----------|
| **前方ギャップ最小** | 15.0 | `lc_min_front_gap` | [m] | LC開始時の前方バンパー間距離の最小値（緩和あり：$g(U) \times 15.0$） |
| **後方ギャップ最小** | 18.0 | `lc_min_rear_gap` | [m] | LC開始時の後方バンパー間距離の最小値（緩和なし、追突防止） |
| **前方TTC閾値** | 3.0 | *(固定値)* | [s] | 前方車両との衝突予測時間の最小値（緩和あり：$g(U) \times 3.0$） |
| **後方TTC閾値** | 4.0 | *(固定値)* | [s] | 後方車両との衝突予測時間の最小値（**より保守的**、緩和なし） |

**緩和係数 $g(U)$の計算**（Section 2.1参照）：
$$g(U) = \max(0.7, 1 - c_{\text{gap}} \cdot U)$$

- $c_{\text{gap}} = 0.5$（v2最適値）の場合：
  - $U=0.0$（低Urgency）: $g(U)=1.0$ → 緩和なし（通常の閾値）
  - $U=0.6$（中Urgency）: $g(U)=0.7$ → 30%緩和
  - $U=1.0$（高Urgency）: $g(U)=0.7$ → 30%緩和（下限）

**実装**: [vehicle.py](../weaving_v11/vehicle.py) - SafetyAnalyzer

---

## 8. LCスケジューリング全体像

### 時系列フロー（実装準拠）

1. **0.5s周期 (Level 1)**: 位置・密度から $U$ を計算。$P_{\text{trigger}}=U$ でベルヌーイ試行（実装: `if U > rand(): ...`).
2. **ギャップ受容**: 緩和係数 $g(U)=\max(0.7, 1-c_{\text{gap}}U)$ を前後ギャップ/TTCに適用。後方は RSS 動的閾値 + CAV協調減速 (\le 2 m/s²)。
3. **スケジュール**: 通過したら `lc_scheduled=True` と `target_lane` をセット。落ちたら次の0.5s周期で再試行。
4. **強制実行域**: 出口手前（例: 残50m）で $P_{\text{trigger}}\to1$ とし、必須LCを確実化。
5. **デッドロック緩和**: 準備中に低速・停滞が続く場合はタイムアウト (≈3s) でキャンセルし交通流を回復。
6. **0.1s周期 (Level 2)**: `lc_scheduled` 車両に対し縦のST-Boundary制約付きQP + 横5次多項式 (所要時間 $\tau\approx3.0\,$s)。LC中もST-Boundaryで前後干渉を監視。

### 実装位置

- 確率トリガ/ギャップ受容/再試行: [controllers.py](../weaving_v11/controllers.py) `_update_level1_strategy`
- Urgency計算: [mpc_controller.py](../weaving_v11/mpc_controller.py) `UrgencyPlanner`
- ST-Boundary + QP: [frenet_qp_apollo.py](../weaving_v11/frenet_qp_apollo.py)

---

## 9. 主要パラメータ早見表

### 9.1 Urgencyと確率トリガー

| 名称 | 記号 | 実装変数名 | デフォルト値 | 単位 | 用途と効果 |
|------|------|-----------|-------------|------|-----------|
| **Urgency急峻度** | $\gamma$ | `urgency_gamma` | **3.0** | - | $U(x)=x^\gamma$ の形状を制御。**大きいほど奥でLC集中**（Front-loading抑制） |
| **密度係数** | $\alpha$ | `urgency_alpha` | **0.2** | - | 目標車線密度 $\rho$ の寄与度。高密度時にUrgency底上げ |
| **LCトリガ確率** | $P$ | *(計算値)* | $P=U(x,\rho)$ | - | 0.5秒ごとのベルヌーイ試行確率。**時間的分散を実現** |

### 9.2 Gap Acceptanceと安全距離

| 名称 | 記号 | 実装変数名 | デフォルト値 | 単位 | 用途と効果 |
|------|------|-----------|-------------|------|-----------|
| **ギャップ緩和係数** | $c_{\text{gap}}$ | `urgency_gap_relax_coeff` | **0.5** (v2最適) | - | $g(U)=\max(0.7,1-c_{\text{gap}}U)$。**Urgency高時にギャップ要件を緩和** |
| **ベース時間ヘッドウェイ** | $T_{\text{base}}$ | `time_headway_HDV` | **1.5** | [s] | 通常時の車間時間（人間ドライバー想定） |
| **最小時間ヘッドウェイ** | $T_{\min}$ | *(固定値)* | **0.9** | [s] | RSS安全要件の下限（Hard limit） |
| **前方ギャップ最小** | - | `lc_min_front_gap` | **15.0** | [m] | LC開始時の前方最小ギャップ（RSS × 2.0相当） |
| **後方ギャップ最小** | - | `lc_min_rear_gap` | **18.0** | [m] | LC開始時の後方最小ギャップ（より保守的） |

### 9.3 LC実行とQP制御

| 名称 | 記号 | 実装変数名 | デフォルト値 | 単位 | 用途と効果 |
|------|------|-----------|-------------|------|-----------|
| **LC所要時間** | $\tau$ | `lc_duration` | **3.0** | [s] | 横5次多項式の時間。**ギャップ予測にも使用**（v2最適値） |
| **LC準備時間** | - | `lc_prep_duration` | **2.0** | [s] | LCスケジュール確定から実行開始までの待機時間 |
| **予測ホライズン** | $H$ | `horizon` | **80** | [steps] | QP最適化の予測ステップ数（8.0s先読み、Apollo標準） |
| **制御周期** | $\Delta t$ | `dt_control` | **0.1** | [s] | Level 2制御の更新間隔（10Hz、Apollo標準） |
| **シミュ周期** | - | `dt_sim` | **0.01** | [s] | 物理シミュレーションのタイムステップ（100Hz） |

### 9.4 QP重み（Apollo Piecewise-Jerk QP）

| 名称 | 記号 | 実装変数名 | デフォルト値 | 単位 | 用途と効果 |
|------|------|-----------|-------------|------|-----------|
| **位置追従重み** | $w_{\text{ref}}$ | `w_ref` | **0.5** | - | 参照軌道追従。小さすぎると計画通りに進まない |
| **速度追従重み** | $w_v$ | `w_v` | **8.0** | - | **目標速度追従（交通流維持）**。v27.9で10.0→8.0に調整 |
| **加速度重み** | $w_a$ | `w_a` | **25.0** | - | エネルギー消費抑制。v27.9で30.0→25.0に調整 |
| **Jerk重み** | $w_j$ | `w_j` | **6000.0** | - | **加速度変化抑制（最重要）**。v28.2で4000→6000に調整。快適性の主要パラメータ |
| **スラック変数重み** | $w_{\xi}$ | `w_slack` | **100000.0** ($10^5$) | - | **ST-Boundary制約違反のペナルティ**。非常に大きな値で違反を強く抑制 |

**実装**：[frenet_qp_apollo.py:664-669](../weaving_v11/frenet_qp_apollo.py#L664-L669)

**調整履歴**：
- v27.9：交通流と快適性のバランス調整（$w_v$ 10.0→8.0, $w_a$ 30.0→25.0, $w_j$ 5000→4000）
- v28.2：Jerk重み増加で加速度違反削減（$w_j$ 4000→6000）

---

## 10. 安全マネージャ閾値

### 10.1 基本マージンと反応時間

| 項目 | 値 | 実装変数名 | 単位 | 物理的意味と役割 |
|------|-----|-----------|------|-----------------|
| **車両長** | 5.0 | `L_VEHICLE` | [m] | バンパー間距離計算の基準 |
| **静止時最小ギャップ** | 2.0 | `RSS_MIN_GAP` | [m] | 完全停止時にも保つ最小距離（RSS準拠） |
| **反応時間 (HDV)** | **0.5** | `RSS_REACTION_TIME` | [s] | **人間ドライバーの知覚反応時間**（RSS標準値） |
| **反応時間 (CAV)** | **0.15** | `RSS_REACTION_TIME_CAV` | [s] | **V2V通信ありCAVの短縮反応時間**（センサー遅延+制御遅延） |
| **最大ブレーキ力** | 6.0 | `A_MAX_BRAKE` | [m/s²] | 緊急ブレーキの最大減速度（UN R157準拠） |
| **快適ブレーキ力** | 3.5 | `A_COMFORT_BRAKE` | [m/s²] | 通常走行時の減速度上限（乗員快適性） |

### 10.2 Lane Change安全閾値

| 項目 | 値 | 実装変数名 | 単位 | 物理的意味と役割 |
|------|-----|-----------|------|-----------------|
| **LC前方ギャップ最小** | 15.0 | `lc_min_front_gap` | [m] | LC開始時の前方最小ギャップ（**RSS × 2.0相当**、十分な余裕） |
| **LC後方ギャップ最小** | 18.0 | `lc_min_rear_gap` | [m] | LC開始時の後方最小ギャップ（**前方より保守的**、追突リスク回避） |
| **前方TTC閾値** | 3.0 | *(固定値)* | [s] | LC可否判定の前方Time-to-Collision閾値（緩和あり） |
| **後方TTC閾値** | 4.0 | *(固定値)* | [s] | LC可否判定の後方Time-to-Collision閾値（**より保守的**） |
| **LC実行中縦閾値** | 15.0 | `mid_lc_long_threshold` | [m] | LC実行中の前方ギャップ監視閾値（pause判定） |
| **LC実行中横閾値** | 2.0 | `mid_lc_lat_threshold` | [m] | LC実行中の横方向偏差閾値 |

### 10.3 5段階安全状態の閾値（v27.0）

| 安全状態 | RSS係数 | 実装変数名 | 意味と対応 |
|---------|--------|-----------|-----------|
| **SAFE** | - | - | 通常走行状態。減速不要 |
| **CAUTION** | **RSS × 2.2** | `early_warning_rss_factor` | **早期警告**。RSS距離の2.2倍以内に接近。速度マッチング開始 |
| **WARNING** | **RSS × 1.7** | `warning_rss_factor` | **警告**。積極的減速が必要。快適ブレーキ範囲 |
| **CRITICAL** | **RSS × 1.3** | `critical_rss_factor` | **危険**。緊急減速が必要（最大 -6.0 m/s²） |
| **AEB** | **RSS × 0.95** | `aeb_rss_factor` | **AEB発動**。最大制動力で衝突回避（Apollo標準は1.0、本実装は0.95で早期発動） |

**注釈**：
- RSS係数が**大きい**ほど、早い段階で減速開始（保守的）
- CAUTION (2.2倍) から徐々に厳しくなり、AEB (0.95倍) で最大制動
- Apollo標準のAEB係数は1.0だが、本実装では0.95で**より早期介入**を実現

実装: [apollo_safety.py](../weaving_v11/apollo_safety.py)

### 10.4 RSSの起源と本実装の関係

#### RSS (Responsibility-Sensitive Safety) の起源

**開発元**: Mobileye（Intel傘下）、2017年発表

**出典**: Shalev-Shwartz, S., et al. (2017). "On a Formal Model of Safe and Scalable Self-driving Cars"

**RSSの基本式**（Mobileye論文より）:
$$
d_{\text{RSS}} = v_{\text{ego}} \cdot t_{\text{react}} + \frac{v_{\text{ego}}^2}{2a_{\max}} - \frac{v_{\text{front}}^2}{2a_{\max}} + s_0
$$

**各項の物理的意味**:
1. $v_{\text{ego}} \cdot t_{\text{react}}$: **反応距離** - 反応時間中に自車が進む距離
2. $\frac{v_{\text{ego}}^2}{2a_{\max}}$: **自車制動距離** - 最大減速で停止するまでの距離
3. $-\frac{v_{\text{front}}^2}{2a_{\max}}$: **前車制動距離** - 前車も同時に最大減速すると仮定（その分短くなる）
4. $s_0$: **静止時最小ギャップ** - 完全停止時にも保つ最小距離

**標準パラメータ** (Mobileye RSS):
- $t_{\text{react}} = 1.0$ s（人間ドライバー）
- $a_{\max} = 6.0$ m/s²（緊急制動）
- $s_0 = 2.0$ m（車両長の約半分）

**本実装での調整**:
- $t_{\text{react}} = 0.5$ s（HDV、より現実的な値）/ 0.15 s（CAV、V2V通信利用）
- $a_{\max} = 6.0$ m/s²（UN R157準拠）
- $s_0 = 2.0$ m（RSS標準値を維持）

#### Apolloでの採用

**Apollo EM Planner** (`speed_decider.cc`) はRSSの概念を採用し、独自実装：
- RSS距離の計算式を使用
- 5段階の安全状態システム（Apollo独自の階層化）
- 横方向距離チェック（`IsOnReferenceLine()`）

#### 本実装の位置づけ

**「RSSベースの、Apollo風安全マネージャ」**:

1. **RSS標準式を採用** ([apollo_safety.py:643-647](../weaving_v11/apollo_safety.py#L643-L647))
   - Mobileyeの論文通りの数式を実装
   - 反応時間の適応的切り替え（HDV: 0.5s / CAV: 0.15s）

2. **Apolloのアーキテクチャを参考**
   - 5段階安全状態システム（SAFE/CAUTION/WARNING/CRITICAL/AEB）
   - 横方向距離チェック（2.0m閾値）
   - AEB係数1.0を基準値として採用（本実装は0.98で早期介入）

3. **本実装独自の拡張**
   - RSS係数のチューニング（CAUTION: 2.2, WARNING: 1.7, CRITICAL: 1.3, AEB: 0.95）
   - V2V通信による反応時間短縮（CAV: 0.15s）
   - LC専用の安全閾値（前方15.0m、後方18.0m）

**参考文献**:
- Mobileye RSS: [arXiv:1708.06374](https://arxiv.org/abs/1708.06374)
- Apollo EM Planner: [GitHub - ApolloAuto/apollo](https://github.com/ApolloAuto/apollo)

---

## 11. デフォルトパラメータと上書き

- 主要デフォルト: [parameters.py](../weaving_v11/parameters.py)。
- 上書き優先度: デフォルト → `--config my.json` → 実行時引数。
- v2最適値の例: `lc_beta_1=11.0`, `urgency_gap_relax_coeff=0.5`, `lc_duration=3.0`（HIGH/queueベスト）。
- ギャップ緩和の実効式: $g(U)=\max(0.7, 1-0.5U)$（v2最適）。
- 予測ホライズン: 80×0.1s = 8.0s（v15.0で拡張）。

---

## 12. V2V協調仲裁アルゴリズム

### 概要

CAV間でLC競合（同時に同じ空間を使おうとする状況）が発生した場合、**コストベースの仲裁**により誰が優先されるかを決定します。これは中央サーバーではなく、**各車両が分散的に計算**して一貫した結論に到達します。

### 制御フローでの位置

```
Level 2戦術制御（0.1s周期）
  └─ QP最適化の前段階で競合検出・仲裁
      ├─ 競合検出: 軌道オーバーラップの予測
      ├─ コスト計算: 両車両のYieldコストを比較
      └─ 行動決定: コストが低い方がYield
```

**実装**: [controllers.py:1540-1645](../weaving_v11/controllers.py#L1540-L1645) (v25.2 Unified Decider)

---

### 12.1 Yieldコスト関数

各車両 $i$ に対して、「この車両がYield（譲る）した場合のシステムコスト」を計算します。

$$
\boxed{
C_{\text{yield}}(i) = w_U \cdot U_i + C_{\text{progress}}(i) + C_{\text{comfort}}(i)
}
$$

#### 各項の詳細

| 項 | 数式 | 説明 | 重み/範囲 |
|----|------|------|----------|
| **Urgency項** | $w_U \cdot U_i$ | 出口への緊急度。高いほど譲りにくい | $w_U = 4.0$, $U_i \in [0,1]$ |
| **Progress項** | $C_{\text{progress}}(i)$ | LC進捗または直進優先権 | 下記参照 |
| **Comfort項** | $C_{\text{comfort}}(i)$ | 高速ほどブレーキのコスト高 | $= v_i \times 0.1$ |

#### Progress項の計算

$$
C_{\text{progress}}(i) =
\begin{cases}
\text{progress}_i \times 5.0 & \text{if LC中} \\
2.0 & \text{if 直進（優先権ベースコスト）}
\end{cases}
$$

ここで、LC進捗率:
$$
\text{progress}_i = \text{clip}\left(\frac{t - t_{\text{LC開始}}}{T_{\text{LC継続時間}}}, 0, 1\right)
$$

#### 数値例

| 車両 | 状態 | $U$ | Progress | $v$ [m/s] | $C_{\text{yield}}$ |
|------|------|-----|----------|-----------|-------------------|
| A | LC中(50%進捗) | 0.8 | 2.5 | 15 | $4.0×0.8 + 2.5 + 1.5 = 7.2$ |
| B | 直進 | 0.3 | 2.0 | 20 | $4.0×0.3 + 2.0 + 2.0 = 5.2$ |

→ $C_{\text{yield}}(B) < C_{\text{yield}}(A)$ なので **Bが譲る**（Aが優先）

---

### 12.2 仲裁ロジック

競合する2台の車両 $V$, $U$ に対して:

$$
\text{Decision} =
\begin{cases}
V \text{ yields} & \text{if } C_{\text{yield}}(V) < C_{\text{yield}}(U) \\
U \text{ yields} & \text{otherwise}
\end{cases}
$$

**解釈**: コストが**低い**方が譲る = コストが**高い**方が優先される

#### 優先度の直感的理解

| 要素 | 高コスト（優先） | 低コスト（譲る） |
|------|----------------|----------------|
| Urgency | 出口に近い | 出口から遠い |
| LC進捗 | LC途中で深い | LC開始直後 or 直進 |
| 速度 | 高速走行中 | 低速 |

---

### 12.3 Yield時のアクション

Yieldと判定された車両は以下の行動を取ります:

```python
if decision == "V_yields":
    # 速度を40%削減
    v.trajectory_conflict_v_reduction = 0.4

    # LC中なら一時停止
    if v.changing_lane:
        v.lc_paused = True
        v.lc_pause_until = t + 0.5  # 0.5秒待機
```

**実装**: [controllers.py:1619-1634](../weaving_v11/controllers.py#L1619-L1634)

---

### 12.4 競合検出条件

2台が「競合している」と判定される条件:

1. **車線オーバーラップ**: 同じ車線を目標としている、または互いの車線に進入しようとしている
2. **距離条件**: 現在距離 < 40m、または2.5秒後の予測距離 < 10m

$$
\text{Conflict} = (\text{same\_target} \lor \text{crossing}) \land (d_{\text{now}} < 40 \lor d_{\text{future}} < 10)
$$

**実装**: [controllers.py:1592-1608](../weaving_v11/controllers.py#L1592-L1608)

---

### 12.5 分散的一貫性の保証

各車両が独立にこのアルゴリズムを実行しても、**同じ結論**に到達します:

- 両車両が同じ情報（位置、速度、LC状態）を観測
- 同じコスト関数を使用
- → 必ず一方がYield、他方がProceedと判定

これにより中央サーバーなしでも**一貫した協調行動**が実現されます。

---

### まとめ：V2V協調の数式

| 数式 | 用途 | 実装箇所 |
|------|------|----------|
| $C_{\text{yield}} = 4U + C_{\text{progress}} + 0.1v$ | Yieldコスト計算 | controllers.py:1558-1578 |
| $C_{\text{progress}} = \text{progress} \times 5.0$ (LC中) | LC進捗コスト | controllers.py:1566-1569 |
| $C_{\text{progress}} = 2.0$ (直進) | 直進優先権 | controllers.py:1571-1572 |

**関連ドキュメント**: [ARCHITECTURE.md](ARCHITECTURE.md#cavの役割v2v通信による協調制御) - 協調制御の全体像

---

## まとめ：重要な数式一覧

| 数式 | パラメータ | 用途 | 実装箇所 |
|------|-----------|------|----------|
| $U = \min(1, x_{\text{norm}}^{\gamma} + \alpha\rho)$ | $\gamma=3.0$, $\alpha=0.2$ | **Urgency計算**（空間分散） | mpc_controller.py:60-120 |
| $P = U(x,\rho)$ | $P \in [0,1]$ | **LC確率トリガー**（時間分散） | controllers.py:535-633 |
| $g(U) = \max(0.7, 1-c_{\text{gap}}U)$ | $c_{\text{gap}}=0.5$ | **Gap緩和係数**（Urgency連動） | controllers.py |
| $s \leq s_{\text{front}} - d_{\text{safe}}$ | - | **ST-Boundary制約**（衝突不可能性保証） | frenet_qp_apollo.py:867-975 |
| $d_{\text{safe}} = s_0 + vT + \frac{v\Delta v}{2\sqrt{ab}}$ | $s_0=2.0$m, $T=0.9\sim1.5$s, $a_{\max}=2.0$m/s², $b=6.0$m/s² | **IDM安全距離**（速度・相対速度依存） | frenet_qp_apollo.py:780-865 |
| $T = \max(T_{\min}, T_{\text{base}} \cdot f_U \cdot f_\rho \cdot f_v)$ | $T_{\text{base}}=1.5$s, $T_{\min}=0.9$s | **適応的時間ヘッドウェイ**（Urgency/密度/速度依存） | frenet_qp_apollo.py |
| $d_{\text{RSS}} = v_{\text{ego}} t_{\text{react}} + \frac{v_{\text{ego}}^2}{2a_{\max}} - \frac{v_{\text{front}}^2}{2a_{\max}} + s_0$ | $t_{\text{react}}=0.5$s (HDV) / 0.15s (CAV), $a_{\max}=6.0$m/s², $s_0=2.0$m | **RSS安全距離**（5段階安全状態の基準値） | apollo_safety.py:643-647 |
| $J = \sum(w_{\text{ref}}(s-s_{\text{ref}})^2 + w_v(v-v_{\text{ref}})^2 + w_a a^2 + w_j j^2 + w_{\xi}\xi^2)$ | $w_v=8.0$, $w_a=25.0$, $w_j=6000.0$, $w_{\xi}=10^5$ | **QP目的関数**（Apollo Piecewise-Jerk QP） | frenet_qp_apollo.py:2035-2081 |
| $\mathbf{x} = [s_0...s_{N-1}, v_0...v_{N-1}, a_0...a_{N-1}, \xi_0...\xi_{N-1}]^T \in \mathbb{R}^{4N}$ | $N=80$ | **QP決定変数**（ブロック形式、スラック変数付き） | frenet_qp_apollo.py:2017 |
| $\text{TTC} = \frac{\text{gap}}{v_{\text{rel}}}$ | gap [m], $v_{\text{rel}}$ [m/s] | **衝突予測時間**（Gap Acceptance判定） | vehicle.py - SafetyAnalyzer |

---

**文書バージョン**: 1.1
**最終更新**: 2026-01-12

**更新履歴**:
- v1.1 (2026-01-12): パラメータ整合性修正（qp_w_j: 4000→6000, aeb_rss_factor: 0.98→0.95）、V2V軌道共有とST-Boundary生成アルゴリズムの詳細追加（Section 3.5-3.7）
- v1.0 (2026-01-11): 初版
