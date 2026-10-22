#!/usr/bin/env python3
"""config/fleet.json と shipnavi/pdu_config_ship.json からビューア用の箱庭設定を生成する。

生成物（config/ 配下、コミット対象）:
  comm/pdu_def.json, comm/pdutypes/<robot>.json  ... compact PDU 定義（pdu_config_ship.json の写し）
  comm/shm_poll_comm.json                        ... ビューアが購読する PDU（pos / joint_states / 海上外乱 / レーダ物標）
  cache/buffer.json, endpoint_shm_with_pdu.json  ... HakoniwaSimNode 内部 SHM endpoint 設定
  robot_sync/<robot>.profile.json                ... hakoniwa_robot_sync 用プロファイル（joint_states を持つ船のみ）
  codec_manifest.json                            ... HakoniwaCodecNode 用（ビルド済み codec のみ列挙）

usage: python tools/gen_viewer_config.py [--check]
"""
import argparse
import json
import sys
from pathlib import Path

PROJECT = Path(__file__).resolve().parent.parent
CONFIG = PROJECT / "config"

# shipnavi/scripts/build_godot_ext.ps1 の -Packages 既定値と一致させる
CODEC_PACKAGES = ["builtin_interfaces", "geometry_msgs", "hako_msgs", "sensor_msgs", "std_msgs"]

# image2sim ship_sync.profile.json の軸（FLU 基準）→ glTF/Godot ローカル軸と符号
# FLU→Godot: x→-Z, y→-X, z→+Y（hakoniwa_robot_sync の ros_position_to_godot と同一）
AXIS_FLU_TO_GODOT = {
    "X": ("z", -1.0),
    "X_FLU_Y_BLENDER": ("z", -1.0),
    "Y": ("x", -1.0),
    "Z": ("y", 1.0),
}


def load_json(path: Path):
    with path.open(encoding="utf-8") as f:
        return json.load(f)


def build_outputs(fleet: dict) -> dict:
    pdu_config = load_json((CONFIG / fleet["pdu_config"]).resolve())
    robots = {r["name"]: r for r in pdu_config["robots"]}
    out = {}

    pdu_def = {"paths": [], "robots": []}
    comm_robots = []
    for ship in fleet["ships"]:
        name = ship["robot"]
        if name not in robots:
            raise SystemExit(f"robot '{name}' not found in pdu_config")
        entries = []
        for key in ("shm_pdu_readers", "shm_pdu_writers"):
            for p in robots[name].get(key, []):
                entries.append({
                    "channel_id": p["channel_id"],
                    "pdu_size": p["pdu_size"],
                    "name": p["org_name"],
                    "type": p["type"],
                })
        entries.sort(key=lambda e: e["channel_id"])
        out[f"comm/pdutypes/{name}.json"] = entries
        pdu_def["paths"].append({"id": name, "path": f"pdutypes/{name}.json"})
        pdu_def["robots"].append({"name": name, "pdutypes_id": name})

        subscribed = [ship["pos_pdu"]] + ([ship["joint_states_pdu"]] if "joint_states_pdu" in ship else [])
        if fleet.get("disturbance", {}).get("robot") == name:
            subscribed.append(fleet["disturbance"]["pdu"])
        radar = fleet.get("overlays", {}).get("radar", {})
        if radar.get("robot") == name and "pdu" in radar:
            subscribed.append(radar["pdu"])
        known = {e["name"] for e in entries}
        for pdu in subscribed:
            if pdu not in known:
                raise SystemExit(f"{name}: PDU '{pdu}' not found in pdu_config")
        comm_robots.append({"name": name, "pdu": [{"name": p, "notify_on_recv": False} for p in subscribed]})

        if "joint_states_pdu" in ship:
            out[f"robot_sync/{name}.profile.json"] = build_robot_sync_profile(ship)

    out["codec_manifest.json"] = {
        "message_script_roots": ["res://addons/hakoniwa_msgs/"],
        "extensions": [f"res://addons/hakoniwa/codecs/{p}_codec.gdextension" for p in CODEC_PACKAGES],
    }
    out["comm/pdu_def.json"] = pdu_def
    out["comm/shm_poll_comm.json"] = {
        "protocol": "shm",
        "impl_type": "poll",
        "asset_name": fleet["asset_name"],
        "name": "ship_viewer_shm",
        "direction": "inout",
        "io": {"robots": comm_robots},
    }
    out["cache/buffer.json"] = {"type": "buffer", "name": "ship_viewer_latest_buffer", "store": {"mode": "latest"}}
    out["endpoint_shm_with_pdu.json"] = {
        "name": "ship_viewer_endpoint",
        "pdu_def_path": "comm/pdu_def.json",
        "cache": "cache/buffer.json",
        "comm": "comm/shm_poll_comm.json",
    }
    return out


def build_robot_sync_profile(ship: dict) -> dict:
    model = ship["model"]
    sync = load_json(PROJECT / "Models" / model / "ship_sync.profile.json")
    mappings = []
    for joint in sync["joints"]:
        if joint["axis"] not in AXIS_FLU_TO_GODOT:
            raise SystemExit(f"{model}: unsupported joint axis '{joint['axis']}'")
        axis, sign = AXIS_FLU_TO_GODOT[joint["axis"]]
        mappings.append({
            "joint_name": joint["name"],
            # ship_fleet.gd の階層: <ShipRoot>/Adapter/Model(glb instance)/<glb root>/<node>
            "node_path": f"Adapter/Model/{model}/{joint['node_name']}",
            "axis": axis,
            "sign": sign,
            "offset_rad": 0.0,
            "apply_mode": "basis_delta",
        })
    return {
        "version": 1,
        "robot_name": ship["robot"],
        "base_link_pdu_name": ship["pos_pdu"],
        "joint_states_pdu_name": ship["joint_states_pdu"],
        "base_node_path": "Adapter",
        "coordinate_system": {"position_rule": "hakoniwa_to_godot", "rotation_rule": "hakoniwa_to_godot"},
        "joint_mappings": mappings,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--check", action="store_true", help="生成物が最新か確認のみ（差分があれば exit 1）")
    args = parser.parse_args()

    outputs = build_outputs(load_json(CONFIG / "fleet.json"))
    stale = []
    for rel, data in outputs.items():
        path = CONFIG / rel
        text = json.dumps(data, indent=2, ensure_ascii=False) + "\n"
        if path.exists() and path.read_text(encoding="utf-8") == text:
            continue
        stale.append(rel)
        if not args.check:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text, encoding="utf-8")
    for rel in stale:
        print(f"{'stale' if args.check else 'wrote'}: config/{rel}")
    return 1 if (args.check and stale) else 0


if __name__ == "__main__":
    sys.exit(main())
