# run_viewer.ps1 - hakoniwa-godot-ship ビューア起動
#   -Headless  : 画面なし実行（CI 用）
#   -Smoke     : オフラインで船隊生成・関節解決・シーン向きを検証し HAKONIWA_GODOT_SHIP_BOOT_OK で終了
#   -Hako      : 箱庭に接続（conductor 起動済みであること）
#   -HakoStart : 箱庭に接続し、シミュレーション開始を要求
#   -SyncTest  : 箱庭に接続・開始し、tests/integration/pdu_sync の既知値受信を検証して終了
#   -Build     : 起動前に import ＋ C# ビルドを実行
param(
    [switch]$Headless,
    [switch]$Smoke,
    [switch]$Hako,
    [switch]$HakoStart,
    [switch]$SyncTest,
    [switch]$Build
)
. (Join-Path $PSScriptRoot "godot_env.ps1")

if (-not (Test-Path (Join-Path $PROJECT_ROOT "addons\hakoniwa\bin\hakoniwa_godot_native.dll"))) {
    & (Join-Path $PSScriptRoot "sync_assets.ps1")
    $Build = $true
}
if ($Build -or -not (Test-Path (Join-Path $PROJECT_ROOT ".godot\mono\temp\bin\Debug\hakoniwa_godot_ship.dll"))) {
    Invoke-GodotBuild
}

$godotArgs = @("--path", $PROJECT_ROOT)
if ($Headless) { $godotArgs = @("--headless") + $godotArgs }
$userArgs = @()
if ($Smoke) { $userArgs += "--smoke" }
if ($Hako) { $userArgs += "--hako" }
if ($HakoStart) { $userArgs += "--hako-start" }
if ($SyncTest) { $userArgs += "--sync-test" }
if ($userArgs.Count -gt 0) { $godotArgs += @("--") + $userArgs }

if ($Headless -or $Smoke -or $SyncTest) {
    & $GODOT_CONSOLE @godotArgs
} else {
    & $GODOT_GUI @godotArgs
}
exit $LASTEXITCODE
