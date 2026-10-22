# gen_reference.ps1 - cpa_ozt.hpp（hakoniwa-mujoco-ship）を MSVC でビルドし reference.json を再生成する
#   pwsh -File tests/unit/ozt_parity/gen_reference.ps1
# 参照値が変わるのは cpa_ozt.hpp を変更したときだけ。生成物 reference.json はコミット対象。
$ErrorActionPreference = "Stop"
$HERE = $PSScriptRoot
$PROJECT_ROOT = (Resolve-Path (Join-Path $HERE "..\..\..")).Path
$SHIP_INCLUDE = (Resolve-Path (Join-Path $PROJECT_ROOT "..\hakoniwa-mujoco-ship\include")).Path
$OUT_DIR = Join-Path $PROJECT_ROOT "build\ozt_parity"
New-Item -ItemType Directory -Force $OUT_DIR | Out-Null
# build/ をプロジェクト内に置くため、Godot が .obj をメッシュとして取り込まないよう除外する
New-Item -ItemType File -Force (Join-Path $PROJECT_ROOT "build\.gdignore") | Out-Null

$vswhere = Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\Installer\vswhere.exe"
$vs = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $vs) { throw "MSVC (VC.Tools.x86.x64) not found" }
$vcvars = Join-Path $vs "VC\Auxiliary\Build\vcvars64.bat"
$exe = Join-Path $OUT_DIR "gen_reference.exe"

$compile = "`"$vcvars`" >nul && cl /nologo /utf-8 /std:c++17 /EHsc /O2 /fp:precise /I`"$SHIP_INCLUDE`" /Fo`"$OUT_DIR\\`" /Fe`"$exe`" `"$HERE\gen_reference.cpp`""
cmd /c $compile
if ($LASTEXITCODE -ne 0) { throw "compile failed" }

$json = & $exe
if ($LASTEXITCODE -ne 0) { throw "gen_reference.exe failed" }
$utf8 = New-Object System.Text.UTF8Encoding($false)
[System.IO.File]::WriteAllText((Join-Path $HERE "reference.json"), (($json -join "`n") + "`n"), $utf8)
Write-Host "wrote: tests/unit/ozt_parity/reference.json"
