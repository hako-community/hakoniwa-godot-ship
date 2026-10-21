extends MeshInstance3D
## 物理と同一の決定論的波面を表示する海面（SHP-23 Step 3）。
##
## - 波浪パラメータは set_wave_params()（Disturbance PDU または fleet.json の preview_wave）で受け取り、
##   ShipHydroPlugin::update_wave_engine_if_needed() と同じ規則（Hs > 0.01 で有効、32 成分）で展開する。
## - 時刻は set_time()（箱庭シミュレーション時刻 [s]）。物理の浮力計算と同じ t を使うこと。
## - メッシュはカメラ追従するが、格子間隔で位置をスナップするので頂点は常に同じ世界座標を標本化する。

const WaveSpectrum = preload("res://Scripts/wave_spectrum.gd")
## 物理プラグイン（ship_hydro_plugin.cpp）と同じ成分数・有効化閾値
const NUM_COMPONENTS := 32
const ACTIVE_HS_THRESHOLD := 0.01

@export var follow_path: NodePath
@export var mesh_size_m: float = 2400.0
@export var subdivisions: int = 599
@export var far_size_m: float = 40000.0

var wave = WaveSpectrum.new()
var wave_active := false
var sim_time_sec := 0.0
var _center_enu := Vector2.ZERO  # (東, 北) [m]、倍精度で保持
var _material: ShaderMaterial
var _far: MeshInstance3D

func _ready() -> void:
	var plane := PlaneMesh.new()
	plane.size = Vector2(mesh_size_m, mesh_size_m)
	plane.subdivide_width = subdivisions
	plane.subdivide_depth = subdivisions
	_material = ShaderMaterial.new()
	_material.shader = preload("res://Shaders/ocean_gerstner.gdshader")
	plane.material = _material
	mesh = plane
	# 波高ぶんの変位でカリングされないよう余裕を持たせる
	extra_cull_margin = 50.0

	_far = MeshInstance3D.new()
	_far.name = "Far"
	var far_plane := PlaneMesh.new()
	far_plane.size = Vector2(far_size_m, far_size_m)
	var far_material := ShaderMaterial.new()
	far_material.shader = preload("res://Shaders/ocean_far.gdshader")
	far_plane.material = far_material
	_far.mesh = far_plane
	_far.position.y = -0.05
	add_child(_far)
	set_wave_params({"hs_m": 0.0})

func grid_spacing() -> float:
	return mesh_size_m / float(subdivisions + 1)

## Disturbance 由来の波浪パラメータ（wave_spectrum.gd の WaveParams と同じキー）を適用する。
func set_wave_params(p: Dictionary) -> void:
	var merged := WaveSpectrum.default_params()
	merged.merge(p, true)
	merged["num_components"] = NUM_COMPONENTS
	wave_active = float(merged["hs_m"]) > ACTIVE_HS_THRESHOLD
	wave.initialize(merged)
	var tide := float(merged["tide_level_m"])
	_material.set_shader_parameter("wave_count", wave.component_count() if wave_active else 0)
	_material.set_shader_parameter("wave_k_amp", wave.shader_k_amp())
	_material.set_shader_parameter("wave_tide", tide)
	_far.position.y = tide - 0.05

func get_wave_params() -> Dictionary:
	return wave.params

func set_time(t_sec: float) -> void:
	sim_time_sec = t_sec

## 物理と同じ CPU 倍精度の標高 [m]（ENU 位置）。波浪無効時は潮位。
func elevation_at(east: float, north: float) -> float:
	if not wave_active:
		return float(wave.params.get("tide_level_m", 0.0))
	return wave.get_elevation(east, north, sim_time_sec)

func _process(_delta: float) -> void:
	var follow := get_node_or_null(follow_path) as Node3D
	if follow != null:
		var step := grid_spacing()
		var gp := follow.global_position
		global_position = Vector3(snappedf(gp.x, step), 0.0, snappedf(gp.z, step))
	# Godot (x, z) → ENU (東 = -z, 北 = -x)
	_center_enu = Vector2(-global_position.z, -global_position.x)
	if wave_active:
		_material.set_shader_parameter("wave_phase", wave.shader_phases(_center_enu.x, _center_enu.y, sim_time_sec))
