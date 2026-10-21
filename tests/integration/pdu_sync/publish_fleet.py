#!/usr/bin/env python3
"""PDU 同期統合テスト用の船隊パブリッシャ（箱庭アセット）。

expected.json の既知値を、ビューアと同じ PDU 定義（config/comm/pdu_def.json）で
OwnShip（pos / joint_states）と他船（pos）へ 20 ms ごとに書き込む。

usage: python publish_fleet.py <pdu_def.json> <expected.json>
"""
import json
import sys

import hakopy
from hakoniwa_pdu.impl.shm_communication_service import ShmCommunicationService
from hakoniwa_pdu.pdu_manager import PduManager
from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_conv_Twist import py_to_pdu_Twist
from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_pytype_Twist import Twist
from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_conv_JointState import py_to_pdu_JointState
from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_pytype_JointState import JointState

ASSET_NAME = "ShipFleetPublisher"
DELTA_USEC = 20000

_pdu_manager = None
_frames = []  # (robot, pdu_name, raw_bytes)


def _twist(pose: dict) -> Twist:
    msg = Twist()
    for part in ("linear", "angular"):
        src = pose.get(part, {})
        dst = getattr(msg, part)
        dst.x, dst.y, dst.z = (float(src.get(k, 0.0)) for k in ("x", "y", "z"))
    return msg


def _joint_state(values: dict) -> JointState:
    msg = JointState()
    msg.name = list(values.keys())
    msg.position = [float(v) for v in values.values()]
    msg.velocity = [0.0] * len(values)
    msg.effort = [0.0] * len(values)
    return msg


def build_frames(expected: dict) -> list:
    frames = []
    for robot, data in expected.items():
        if robot.startswith("_"):
            continue
        frames.append((robot, "pos", py_to_pdu_Twist(_twist(data["pos"]))))
        if "joint_states" in data:
            frames.append((robot, "joint_states", py_to_pdu_JointState(_joint_state(data["joint_states"]))))
    return frames


def on_manual_timing_control(_context):
    step = 0
    while True:
        for robot, pdu_name, raw in _frames:
            if not _pdu_manager.flush_pdu_raw_data_nowait(robot, pdu_name, raw):
                print(f"SHIP_FLEET_PUB_WRITE_FAILED:{robot}/{pdu_name}", flush=True)
                return 0
        if step == 0:
            print(f"SHIP_FLEET_PUB_FIRST_WRITE:{len(_frames)} pdus", flush=True)
        step += 1
        if not hakopy.usleep(DELTA_USEC):
            break
    print(f"SHIP_FLEET_PUB_DONE:steps={step}", flush=True)
    return 0


def _noop(_context):
    return 0


def main() -> int:
    global _pdu_manager, _frames
    if len(sys.argv) != 3:
        print(__doc__)
        return 1
    pdu_def_path, expected_path = sys.argv[1], sys.argv[2]
    with open(expected_path, encoding="utf-8") as f:
        _frames = build_frames(json.load(f))

    _pdu_manager = PduManager()
    _pdu_manager.initialize(config_path=pdu_def_path, comm_service=ShmCommunicationService())
    _pdu_manager.start_service_nowait()

    callbacks = {
        "on_initialize": _noop,
        "on_simulation_step": None,
        "on_manual_timing_control": on_manual_timing_control,
        "on_reset": _noop,
    }
    if not hakopy.asset_register(ASSET_NAME, pdu_def_path, callbacks, DELTA_USEC, hakopy.HAKO_ASSET_MODEL_PLANT):
        print("SHIP_FLEET_PUB_REGISTER_FAILED", flush=True)
        return 1
    print("SHIP_FLEET_PUB_REGISTERED", flush=True)
    ret = hakopy.start()
    print(f"SHIP_FLEET_PUB_START_RETURN:{ret}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
