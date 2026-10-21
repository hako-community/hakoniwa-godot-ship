# sync_assets.ps1 - 外部生成物を hakoniwa-godot-ship へ取り込む
#   1) addons: shipnavi/build/hakoniwa-godot/addon（scripts/build_godot_ext.ps1 の出力）→ addons/
#      （hakoniwa / hakoniwa_msgs / hakoniwa_robot_sync。viewer/hakoniwa-godot 由来のため git 管理外）
#   2) models: images/image2sim-framework/output/<model>/ → Models/<model>/（glb ＋ ship_sync.profile.json）
param(
    [string[]]$Models = @("container_749gt", "container_700teu", "ferry_seto", "fishing_boat")
)
$ErrorActionPreference = "Stop"
$PROJECT_ROOT = Split-Path -Parent $PSScriptRoot
$SHIP = Split-Path -Parent $PROJECT_ROOT
$ADDON_SRC = Join-Path $SHIP "build\hakoniwa-godot\addon"
$MODEL_SRC = Join-Path (Split-Path -Parent $SHIP) "images\image2sim-framework\output"

if (-not (Test-Path (Join-Path $ADDON_SRC "addons\hakoniwa\bin\hakoniwa_godot_native.dll"))) {
    throw "addon が未ビルドです。先に pwsh -File $SHIP\scripts\build_godot_ext.ps1 を実行してください: $ADDON_SRC"
}
foreach ($a in "hakoniwa", "hakoniwa_msgs", "hakoniwa_robot_sync") {
    $to = Join-Path $PROJECT_ROOT "addons\$a"
    if (Test-Path $to) { Remove-Item -Recurse -Force $to }
    New-Item -ItemType Directory -Force (Split-Path -Parent $to) | Out-Null
    Copy-Item -Recurse (Join-Path $ADDON_SRC "addons\$a") $to
    Write-Host "[addon] $a"
}
Copy-Item (Join-Path $ADDON_SRC "godot_addon_manifest.json") (Join-Path $PROJECT_ROOT "addons\godot_addon_manifest.json") -Force

foreach ($m in $Models) {
    $from = Join-Path $MODEL_SRC $m
    $to = Join-Path $PROJECT_ROOT "Models\$m"
    New-Item -ItemType Directory -Force $to | Out-Null
    foreach ($f in "$m.glb", "ship_sync.profile.json") {
        $src = Join-Path $from $f
        if (-not (Test-Path $src)) { throw "モデルがありません: $src" }
        Copy-Item $src $to -Force
    }
    Write-Host "[model] $m"
}
