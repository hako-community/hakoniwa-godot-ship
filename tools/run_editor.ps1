# run_editor.ps1 - hakoniwa-godot-ship を Godot 4.7.1 エディタで開く
. (Join-Path $PSScriptRoot "godot_env.ps1")
& $GODOT_GUI --editor --path $PROJECT_ROOT
