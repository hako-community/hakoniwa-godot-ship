# run_viewer.ps1 - hakoniwa-godot-ship ビューア起動
#   -Headless : 画面なし実行（CI 用）
#   -Smoke    : 数フレーム描画後に HAKONIWA_GODOT_SHIP_BOOT_OK を出力して終了
#   -Build    : 起動前に C# ソリューションをビルド
param(
    [switch]$Headless,
    [switch]$Smoke,
    [switch]$Build
)
. (Join-Path $PSScriptRoot "godot_env.ps1")

if ($Build -or -not (Test-Path (Join-Path $PROJECT_ROOT ".godot\mono\temp\bin\Debug\hakoniwa_godot_ship.dll"))) {
    Invoke-GodotBuild
}

$godotArgs = @("--path", $PROJECT_ROOT)
if ($Headless) { $godotArgs = @("--headless") + $godotArgs }
if ($Smoke) { $godotArgs += @("--", "--smoke") }

if ($Headless -or $Smoke) {
    & $GODOT_CONSOLE @godotArgs
} else {
    & $GODOT_GUI @godotArgs
}
exit $LASTEXITCODE
