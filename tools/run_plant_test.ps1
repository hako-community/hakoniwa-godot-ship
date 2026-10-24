# run_plant_test.ps1 - 実物理との結合テスト（conductor → ship_plant ＋ 外乱パブリッシャ → Godot ビューア）
#
#   pwsh -File tools/run_plant_test.ps1 [-TimeoutSec 240] [-Screenshot <png> [-CameraDistance <m>]]
#   pwsh -File tools/run_plant_test.ps1 -Watch [-TimeoutSec 900] [-WinPos -2720,630] [-ViewerArgs '--no-overlays --camera-distance=400']
#     検証せずに見るだけ（窓を閉じると終わる）。-ViewerArgs はビューアの引数（例：オーバレイを消して船に寄る外観の確認）
#
# ship_plant（hakoniwa-mujoco-ship、事前に cmake --build build --config Release）が全船の pos・OwnShip の
# joint_states / radar_points を書き、tools/publish_disturbance.py が同じシナリオの波浪を書く。
# ビューアは --plant-test で時刻・海面と船体の上下動・レーダ物標・関節・運動を検証する。
# -Screenshot を付けるとウィンドウ表示で実行し、終了時の画面を保存する（目視確認用）。
#
# 成功時: Godot が HAKONIWA_GODOT_SHIP_PLANT_OK を出力し exit 0。ログは build/plant_test/ に保存。
param(
    [int]$TimeoutSec = 240,
    [string]$Screenshot = "",
    [string]$CameraDistance = "",
    [string]$Scenario = "",
    [switch]$Watch,
    [string]$WinPos = "",
    [string]$ViewerArgs = ""
)
$ErrorActionPreference = "Stop"
. (Join-Path $PSScriptRoot "godot_env.ps1")

$SHIP = Split-Path -Parent $PROJECT_ROOT
$PLANT_REPO = Join-Path $SHIP "hakoniwa-mujoco-ship"
$PLANT = Join-Path $PLANT_REPO "build\Release\ship_plant.exe"
if (-not $Scenario) { $Scenario = Join-Path $PLANT_REPO "config\plant\fleet_encounter.json" }
$ENV_PUB = Join-Path $PLANT_REPO "tools\publish_disturbance.py"
$MUJOCO_BIN = Join-Path $PLANT_REPO "deps\mujoco-3.9.0\bin"
$CONDUCTOR = Join-Path $SHIP "build\hakoniwa-godot\build\tools\hako_conductor_runner.exe"
$PYTHON = Join-Path $SHIP "sdk\windows-msvc-x64\python-venv\Scripts\python.exe"
$PDU_CONFIG = Join-Path $SHIP "pdu_config_ship.json"
$LOG_DIR = Join-Path $PROJECT_ROOT "build\plant_test"
foreach ($p in $PLANT, $Scenario, $ENV_PUB, $MUJOCO_BIN, $CONDUCTOR, $PYTHON, $PDU_CONFIG) { if (-not (Test-Path $p)) { throw "not found: $p" } }
New-Item -ItemType Directory -Force $LOG_DIR | Out-Null
$env:PATH = "$MUJOCO_BIN;" + $env:PATH

$procs = @()
function Start-Logged($name, $exe, $argList) {
    $p = Start-Process $exe -ArgumentList $argList -PassThru -NoNewWindow `
        -RedirectStandardOutput (Join-Path $LOG_DIR "$name.log") -RedirectStandardError (Join-Path $LOG_DIR "$name.err.log")
    $script:procs += $p
    return $p
}

$exitCode = 1
try {
    Invoke-GodotBuild | Out-Null
    Start-Logged "conductor" $CONDUCTOR @("--delta-usec", "20000", "--max-delay-usec", "100000", "--sleep-sec", "$TimeoutSec") | Out-Null
    Start-Sleep -Seconds 2
    Start-Logged "plant" $PLANT @($Scenario, "--trace", (Join-Path $LOG_DIR "plant_trace.csv")) | Out-Null
    Start-Logged "env" $PYTHON @("-u", $ENV_PUB, $PDU_CONFIG, $Scenario) | Out-Null
    Start-Sleep -Seconds 3
    if ($Watch) {
        $viewArgs = @("--path", $PROJECT_ROOT)
        if ($WinPos) { $viewArgs += @("--position", $WinPos) }
        $viewArgs += @("--", "--hako-start")
        if ($ViewerArgs) { $viewArgs += $ViewerArgs.Trim() -split ' +' }
        $viewer = Start-Logged "viewer" $GODOT_GUI $viewArgs
    } elseif ($Screenshot) {
        $viewArgs = @("--path", $PROJECT_ROOT, "--", "--plant-test", "--screenshot=$Screenshot")
        if ($CameraDistance) { $viewArgs += "--camera-distance=$CameraDistance" }
        $viewer = Start-Logged "viewer" $GODOT_CONSOLE $viewArgs
    } else {
        $viewer = Start-Logged "viewer" $GODOT_CONSOLE @("--headless", "--path", $PROJECT_ROOT, "--", "--plant-test")
    }
    if (-not $viewer.WaitForExit($TimeoutSec * 1000)) {
        Write-Host "[plant-test] viewer timeout"
    } else {
        $exitCode = $viewer.ExitCode
    }
} finally {
    foreach ($p in $procs) { if (-not $p.HasExited) { Stop-Process -Id $p.Id -Force -ErrorAction SilentlyContinue } }
}

foreach ($name in "conductor", "plant", "env") {
    Write-Host "----- $name -----"
    Get-Content (Join-Path $LOG_DIR "$name.log"), (Join-Path $LOG_DIR "$name.err.log") -ErrorAction SilentlyContinue |
        Where-Object { $_ -and $_ -notmatch '\x1b\[' -and $_ -notmatch 'DEBUG|Log message truncated' } | Select-Object -Last 8
}
Write-Host "----- viewer -----"
Get-Content (Join-Path $LOG_DIR "viewer.log"), (Join-Path $LOG_DIR "viewer.err.log") -ErrorAction SilentlyContinue |
    Where-Object { $_ -match 'PLANT_TEST|HAKONIWA_GODOT_SHIP|^  |ERROR|SCRIPT' }
Write-Host "[plant-test] exit=$exitCode"
exit $exitCode