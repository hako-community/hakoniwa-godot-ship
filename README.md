# hakoniwa-godot-ship

自動運航船シミュレータ（MEGURI2040 / DFFAS+ 対応）の **工学・高速検証ビューア**（SHP-23）。
Godot 4.7.1 (mono / C#) 上で、箱庭 PDU から自船・他船の姿勢・舵角・プロペラを同期し、
決定論的 Gerstner 波面、レーダ RCS ローブ、AIS ベクトル、OZT 危険領域を重畳表示する。

写実描画・船載カメラ PDU は姉妹リポジトリ [`hakoniwa-unreal-ship`](https://github.com/hako-community/hakoniwa-unreal-ship)（UE 5.8）が担当する。

## 前提環境

| 項目 | バージョン / パス |
|---|---|
| Godot | 4.7.1 stable mono（既定: `D:\Godot_v4.7.1-stable_mono_win64`、`$env:GODOT_BIN` で上書き可） |
| .NET SDK | 8.0 以上 |
| 箱庭 SDK | `shipnavi/scripts/env_ship.ps1`（存在すれば自動読込、共有メモリ `Z:/mmap_ship`） |
| 箱庭 Godot addon | `viewer/hakoniwa-godot` を `shipnavi/scripts/build_godot_ext.ps1` で Windows ビルド（参照リポジトリは無改変） |
| 船舶モデル | `images/image2sim-framework/output/<model>/`（glb ＋ `ship_sync.profile.json`） |

## セットアップ

```powershell
# 1) hakoniwa-godot addon を out-of-tree ビルド（初回のみ・十数分）→ shipnavi/build/hakoniwa-godot/addon
pwsh -File ../scripts/build_godot_ext.ps1
# 2) addon と船舶モデルをプロジェクトへ取り込む（addons/ は git 管理外、Models/ はコミット対象）
pwsh -File tools/sync_assets.ps1
# 3) fleet.json / pdu_config_ship.json を変更したらビューア設定を再生成
python tools/gen_viewer_config.py
```

## 使い方

```powershell
# エディタで開く
pwsh -File tools/run_editor.ps1

# ビューア起動（初回は import + C# ビルドを自動実行）
pwsh -File tools/run_viewer.ps1

# 起動スモークテスト（CI 用）。船隊生成・関節ノード解決・シーン向きを検証し HAKONIWA_GODOT_SHIP_BOOT_OK / exit 0
pwsh -File tools/run_viewer.ps1 -Headless -Smoke -Build

# 箱庭に接続（conductor 起動済み）。-HakoStart で開始要求も行う
pwsh -File tools/run_viewer.ps1 -Hako

# PDU 同期統合テスト: conductor → Python 船隊パブリッシャ → Godot（姿勢・関節・波浪パラメータ）。
# HAKONIWA_GODOT_SHIP_SYNC_OK / exit 0
pwsh -File tools/run_sync_test.ps1

# 波面の単体テスト（CPU と Python/C++ 参照の一致、GPU シェーダと CPU の一致）。-SkipGpu で GPU 不要分のみ
pwsh -File tools/run_unit_tests.ps1
```

> 注: Godot 4.7.1 の `--build-solutions --quit` は headless で終了しないため、ビルドは `--import` ＋ `dotnet build` で行う。

## 座標系と PDU 規約

| 項目 | 規約 |
|---|---|
| 位置 | 箱庭 FLU（ワールドは ENU）→ Godot: x→−Z, y→−X, z→+Y（`hakoniwa_robot_sync` の `ros_position_to_godot` と同一、`ship_fleet.gd` の `FLU_TO_GODOT`） |
| 姿勢 `pos.angular` | `(roll, asin(R20), yaw)` [rad]。`hakoniwa-mujoco-runtime` の `physics_impl.hpp` と同じで、y は右手系ピッチの符号反転（船首下げ＝負） |
| 関節 `joint_states` | `ship_sync.profile.json` の軸（FLU）を glb ローカル軸へ変換: Z→y(+1), X/X_FLU_Y_BLENDER→z(−1), Y→x(−1) |
| glb | image2sim 出力は Godot 座標（船首 −Z、上 +Y）。`Adapter` ノード（逆変換）で FLU 系に戻す |
| `.tscn` | `Transform3D(...)` は**行優先**で記述される（列ベクトルではない） |

## 海面（決定論的 Gerstner 波）

物理（`hakoniwa-mujoco-ship` の `wave_spectrum.hpp`）と同じ波面を描画する。

| 層 | 実装 | 精度検証 |
|---|---|---|
| 成分テーブル・CPU 評価 | `Scripts/wave_spectrum.gd`（C++/Python と同一アルゴリズム・Xorshift32） | `tests/unit/wave_parity`: 参照値との最大誤差 3.4e-15 |
| GPU 評価 | `Shaders/wave_common.gdshaderinc`（中心位置・時刻を CPU 倍精度で位相に畳み込み、単精度は相対座標のみ） | `tests/unit/wave_shader_parity`: 1 時間後・中心 5.8 km 地点・±1.1 km で最大 1.7e-5 m |
| 海面メッシュ | `Scripts/ocean.gd` ＋ `Shaders/ocean_gerstner.gdshader`（2400 m 四方・4 m 格子、カメラ追従は格子スナップ） | 外周 950〜1180 m で潮位面へフェード（検証範囲は中心 950 m 以内） |
| パラメータ | `Scripts/disturbance_sync.gd`: `OwnShip/maritime_disturbance` の `d_user_custom[2]=[Hs,Tp,波向,γ,seed]`, `[1][4]=水深` | `run_sync_test.ps1` で受信・反映を検証 |

- 物理プラグインと同じく 32 成分、Hs > 0.01 m で有効、潮位は波面に含めない（`tide_level_m = 0`）。
- 時刻は箱庭シミュレーション時刻（`simulation_step` の simtime）。オフライン時は `fleet.json` の `preview_wave` をローカル時刻で再生。
- `gerstner_displacement`（シェーダ uniform、既定 0）を上げると波頭が尖るが、頂点が水平移動するため物理の標高とは一致しなくなる（見た目用）。

## 船隊と PDU

`config/fleet.json` が唯一の手書き設定。`tools/gen_viewer_config.py` が `../pdu_config_ship.json` と
`Models/*/ship_sync.profile.json` から `config/comm/`・`config/robot_sync/`・`config/codec_manifest.json` を生成する。

| robot | model | 購読 PDU | 同期ノード |
|---|---|---|---|
| OwnShip | container_749gt | pos (ch0), joint_states (ch3), maritime_disturbance (ch4) | `HakoniwaRobotSyncController`（addon）、`disturbance_sync.gd` |
| TargetShip01 | container_700teu | pos (ch21) | `ship_pose_sync.gd` |
| TargetShip02 | ferry_seto | pos (ch22) | `ship_pose_sync.gd` |
| TargetShipFishing | fishing_boat | pos (ch23) | `ship_pose_sync.gd` |

## 構成

```text
hakoniwa-godot-ship/
├─ project.godot              # Forward+, C#, 4.7
├─ hakoniwa_godot_ship.csproj # Godot.NET.Sdk/4.7.1, net8.0
├─ Scenes/Main.tscn           # 海面・空・太陽・カメラ・HUD、Ships/Codec/Sim/Session
├─ Scripts/
│  ├─ ShipViewerMain.cs       # HUD
│  ├─ ship_viewer_session.gd  # 起動モード（--smoke / --hako / --hako-start / --sync-test）と自己検証
│  ├─ ship_fleet.gd           # fleet.json から船舶生成、同期ノード付与
│  ├─ ship_pose_sync.gd       # pos のみの他船用姿勢同期
│  ├─ wave_spectrum.gd        # JONSWAP/Gerstner 決定論的波面（wave_spectrum.hpp の移植）
│  ├─ ocean.gd                # 海面メッシュ・uniform 更新・カメラ追従
│  └─ disturbance_sync.gd     # Disturbance PDU → 波浪パラメータ
├─ config/                    # fleet.json（手書き）＋ 生成物
├─ Models/                    # image2sim 生成 glb（sync_assets.ps1 で取込）
├─ Shaders/                   # wave_common.gdshaderinc / ocean_gerstner / ocean_far
├─ tests/
│  ├─ integration/pdu_sync    # expected.json ＋ publish_fleet.py
│  └─ unit/                   # wave_parity（reference.json ＋ gen_reference.py）、wave_shader_parity
└─ tools/                     # run_editor / run_viewer / run_sync_test / run_unit_tests / sync_assets / gen_viewer_config
```

## ロードマップ（SHP-23）

- [x] Step 1: プロジェクト基盤（project.godot、起動スクリプト、スモークテスト）
- [x] Step 2: 船舶 glb 配置と `ship_sync.profile.json` / PDU 姿勢同期（`viewer/hakoniwa-godot` addon を無改変参照）
- [x] Step 3: `wave_spectrum.hpp` 互換の決定論的 Gerstner 波面シェーダ
- [ ] Step 4: 工学オーバレイ（レーダ RCS ローブ、AIS ベクトル、OZT 扇形、航跡）
