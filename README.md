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

# PDU 同期統合テスト: conductor → Python 船隊パブリッシャ → Godot。HAKONIWA_GODOT_SHIP_SYNC_OK / exit 0
pwsh -File tools/run_sync_test.ps1
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

## 船隊と PDU

`config/fleet.json` が唯一の手書き設定。`tools/gen_viewer_config.py` が `../pdu_config_ship.json` と
`Models/*/ship_sync.profile.json` から `config/comm/`・`config/robot_sync/`・`config/codec_manifest.json` を生成する。

| robot | model | 購読 PDU | 同期ノード |
|---|---|---|---|
| OwnShip | container_749gt | pos (ch0), joint_states (ch3) | `HakoniwaRobotSyncController`（addon） |
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
│  └─ ship_pose_sync.gd       # pos のみの他船用姿勢同期
├─ config/                    # fleet.json（手書き）＋ 生成物
├─ Models/                    # image2sim 生成 glb（sync_assets.ps1 で取込）
├─ Shaders/                   # Gerstner 波面シェーダ（Step 3）
├─ tests/integration/pdu_sync # expected.json ＋ publish_fleet.py
└─ tools/                     # run_editor / run_viewer / run_sync_test / sync_assets / gen_viewer_config
```

## ロードマップ（SHP-23）

- [x] Step 1: プロジェクト基盤（project.godot、起動スクリプト、スモークテスト）
- [x] Step 2: 船舶 glb 配置と `ship_sync.profile.json` / PDU 姿勢同期（`viewer/hakoniwa-godot` addon を無改変参照）
- [ ] Step 3: `wave_spectrum.hpp` 互換の決定論的 Gerstner 波面シェーダ
- [ ] Step 4: 工学オーバレイ（レーダ RCS ローブ、AIS ベクトル、OZT 扇形、航跡）
