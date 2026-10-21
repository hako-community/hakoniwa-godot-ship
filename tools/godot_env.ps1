# godot_env.ps1 - hakoniwa-godot-ship 共通設定（dot-source して使用）
# GODOT_BIN を環境変数で上書き可能。既定は D:\Godot_v4.7.1-stable_mono_win64。
$PROJECT_ROOT = Split-Path -Parent $PSScriptRoot
$GODOT_HOME = "D:\Godot_v4.7.1-stable_mono_win64"

if ($env:GODOT_BIN) {
    $GODOT_GUI = $env:GODOT_BIN
    $GODOT_CONSOLE = $env:GODOT_BIN
} else {
    $GODOT_GUI = Join-Path $GODOT_HOME "Godot_v4.7.1-stable_mono_win64.exe"
    $GODOT_CONSOLE = Join-Path $GODOT_HOME "Godot_v4.7.1-stable_mono_win64_console.exe"
}

if (-not (Test-Path $GODOT_CONSOLE)) {
    Write-Error "Godot not found: $GODOT_CONSOLE (set `$env:GODOT_BIN)"
    exit 1
}

# 船舶シミュレータ共通環境（SDK / Z:/mmap_ship）があれば読み込む
$SHIP_ENV = Join-Path (Split-Path -Parent $PROJECT_ROOT) "scripts\env_ship.ps1"
if (Test-Path $SHIP_ENV) { . $SHIP_ENV }

# アセット import と C# アセンブリのビルド（.godot/ 生成）。失敗時は終了コードを返す
# 注: Godot 4.7.1 の `--build-solutions --quit` は headless で終了しないため dotnet build を使う
function Invoke-GodotBuild {
    & $GODOT_CONSOLE --headless --path $PROJECT_ROOT --import
    if ($LASTEXITCODE -ne 0) {
        Write-Error "Godot import failed (exit $LASTEXITCODE)"
        exit $LASTEXITCODE
    }
    dotnet build (Join-Path $PROJECT_ROOT "hakoniwa_godot_ship.csproj") -nologo -v q
    if ($LASTEXITCODE -ne 0) {
        Write-Error "C# build failed (exit $LASTEXITCODE)"
        exit $LASTEXITCODE
    }
}
