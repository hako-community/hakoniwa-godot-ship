# run_sync_test.ps1 - PDU 同期統合テスト（conductor → Python 船隊パブリッシャ → Godot ビューア）
#
#   pwsh -File tools/run_sync_test.ps1 [-TimeoutSec 90]
#
# 成功時: Godot が HAKONIWA_GODOT_SHIP_SYNC_OK を出力し exit 0。ログは build/sync_test/ に保存。
param(
    [int]$TimeoutSec = 90
)
$ErrorActionPreference = "Stop"
. (Join-Path $PSScriptRoot "godot_env.ps1")

$SHIP = Split-Path -Parent $PROJECT_ROOT
$CONDUCTOR = Join-Path $SHIP "build\hakoniwa-godot\build\tools\hako_conductor_runner.exe"
$PYTHON = Join-Path $SHIP "sdk\windows-msvc-x64\python-venv\Scripts\python.exe"
$TEST_DIR = Join-Path $PROJECT_ROOT "tests\integration\pdu_sync"
$PDU_DEF = Join-Path $PROJECT_ROOT "config\comm\pdu_def.json"
$LOG_DIR = Join-Path $PROJECT_ROOT "build\sync_test"
foreach ($p in $CONDUCTOR, $PYTHON) { if (-not (Test-Path $p)) { throw "not found: $p" } }
New-Item -ItemType Directory -Force $LOG_DIR | Out-Null

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
    $conductor = Start-Logged "conductor" $CONDUCTOR @("--delta-usec", "20000", "--max-delay-usec", "100000", "--sleep-sec", "$TimeoutSec")
    Start-Sleep -Seconds 2
    $publisher = Start-Logged "publisher" $PYTHON @("-u", (Join-Path $TEST_DIR "publish_fleet.py"), $PDU_DEF, (Join-Path $TEST_DIR "expected.json"))
    Start-Sleep -Seconds 3
    $viewer = Start-Logged "viewer" $GODOT_CONSOLE @("--headless", "--path", $PROJECT_ROOT, "--", "--sync-test")
    if (-not $viewer.WaitForExit($TimeoutSec * 1000)) {
        Write-Host "[sync-test] viewer timeout"
    } else {
        $exitCode = $viewer.ExitCode
    }
} finally {
    foreach ($p in $procs) { if (-not $p.HasExited) { Stop-Process -Id $p.Id -Force -ErrorAction SilentlyContinue } }
}

foreach ($name in "conductor", "publisher", "viewer") {
    Write-Host "----- $name -----"
    Get-Content (Join-Path $LOG_DIR "$name.log"), (Join-Path $LOG_DIR "$name.err.log") -ErrorAction SilentlyContinue |
        Where-Object { $_ -and $_ -notmatch '\x1b\[' } | Select-Object -Last 15
}
Write-Host "[sync-test] exit=$exitCode"
exit $exitCode
