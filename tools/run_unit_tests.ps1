# run_unit_tests.ps1 - 波面の単体テスト
#   1) wave_parity        : wave_spectrum.gd（倍精度）と Python/C++ 参照値の一致（headless）
#   2) wave_shader_parity : GPU シェーダ（単精度）と CPU の一致（GPU ウィンドウが必要）
#   -SkipGpu で 2) を省略（GPU の無い CI 用）
param(
    [switch]$SkipGpu
)
. (Join-Path $PSScriptRoot "godot_env.ps1")

$tests = @(@{ name = "wave_parity"; args = @("--headless", "--path", $PROJECT_ROOT, "--script", "res://tests/unit/wave_parity/run_wave_parity.gd") })
if (-not $SkipGpu) {
    $tests += @{ name = "wave_shader_parity"; args = @("--path", $PROJECT_ROOT, "--script", "res://tests/unit/wave_shader_parity/run_wave_shader_parity.gd") }
}
$failed = 0
foreach ($t in $tests) {
    $out = & $GODOT_CONSOLE @($t.args) 2>&1
    $code = $LASTEXITCODE
    $out | Where-Object { $_ -match "WAVE_" } | ForEach-Object { Write-Host "  $_" }
    Write-Host "[unit] $($t.name): $(if ($code -eq 0) { 'PASS' } else { "FAIL (exit $code)" })"
    if ($code -ne 0) { $failed++ }
}
exit $failed
