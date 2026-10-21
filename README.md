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

## 使い方

```powershell
# エディタで開く
pwsh -File tools/run_editor.ps1

# ビューア起動（初回は import + C# ビルドを自動実行）
pwsh -File tools/run_viewer.ps1

# 起動スモークテスト（CI 用）。成功時 HAKONIWA_GODOT_SHIP_BOOT_OK を出力し exit 0
pwsh -File tools/run_viewer.ps1 -Headless -Smoke -Build
```

> 注: Godot 4.7.1 の `--build-solutions --quit` は headless で終了しないため、ビルドは `--import` ＋ `dotnet build` で行う。

## 座標系

箱庭 / MuJoCo 側は ENU（x=East, y=North, z=Up）。Godot 側への変換は
`ShipViewerMain.ToGodot()`（East→+X, Up→+Y, North→−Z）に集約する。

## 構成

```text
hakoniwa-godot-ship/
├─ project.godot              # Forward+, C#, 4.7
├─ hakoniwa_godot_ship.csproj # Godot.NET.Sdk/4.7.1, net8.0
├─ Scenes/Main.tscn           # 海面・空・太陽・カメラ・HUD、Ships/Overlays ルート
├─ Scripts/ShipViewerMain.cs  # エントリ、スモーク判定、ENU→Godot 変換
├─ Models/                    # image2sim 生成 glTF（Step 2）
├─ Shaders/                   # Gerstner 波面シェーダ（Step 3）
└─ tools/                     # run_editor.ps1 / run_viewer.ps1 / godot_env.ps1
```

## ロードマップ（SHP-23）

- [x] Step 1: プロジェクト基盤（project.godot、起動スクリプト、スモークテスト）
- [ ] Step 2: 船舶 glTF 配置と `ship_sync.profile.json` / PDU 姿勢同期（`viewer/hakoniwa-godot` addon を無改変参照）
- [ ] Step 3: `wave_spectrum.hpp` 互換の決定論的 Gerstner 波面シェーダ
- [ ] Step 4: 工学オーバレイ（レーダ RCS ローブ、AIS ベクトル、OZT 扇形、航跡）
