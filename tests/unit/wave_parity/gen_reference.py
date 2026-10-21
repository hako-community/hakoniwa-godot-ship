#!/usr/bin/env python3
"""wave_spectrum.gd 検証用の参照値を hakoniwa-mujoco-ship の Python 実装から生成する。

Python 実装（python/hakoniwa_wave）は SHP-03 で C++ 版とビット一致を検証済み。
usage: python -I gen_reference.py   →  reference.json（コミット対象）
"""
import json
import sys
from dataclasses import asdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
WAVE_PY = HERE.parents[3] / "hakoniwa-mujoco-ship" / "python"
sys.path.insert(0, str(WAVE_PY))
from hakoniwa_wave.wave_spectrum import WaveParams, WaveSpectrumEngine  # noqa: E402

CASES = [
    # 物理プラグインの既定（num_components=32）と同条件
    {"hs_m": 2.5, "tp_s": 8.0, "wave_dir_rad": 0.0, "gamma": 3.3, "seed": 42, "water_depth_m": 100.0, "tide_level_m": 0.0, "num_components": 32},
    # 浅水（ニュートン法分岐）・seed=0（state=1 置換）・64 成分
    {"hs_m": 4.0, "tp_s": 11.0, "wave_dir_rad": 2.1, "gamma": 1.0, "seed": 0, "water_depth_m": 15.0, "tide_level_m": 0.0, "num_components": 64},
    # 深水分岐・大きな seed・潮位オフセット・16 成分
    {"hs_m": 0.8, "tp_s": 5.0, "wave_dir_rad": -0.7, "gamma": 7.0, "seed": 123456789, "water_depth_m": 300.0, "tide_level_m": 1.25, "num_components": 16},
]
POINTS = [(0.0, 0.0, 0.0), (37.5, -12.25, 3.7), (-850.0, 1200.0, 61.3), (4000.0, -2500.0, 3600.0)]


def main() -> int:
    out = []
    for case in CASES:
        engine = WaveSpectrumEngine(WaveParams(**case))
        out.append({
            "params": case,
            "components": [asdict(c) for c in engine.components],
            "samples": [{"x": x, "y": y, "t": t, **engine.get_surface(x, y, t)} for x, y, t in POINTS],
        })
    path = HERE / "reference.json"
    path.write_text(json.dumps(out, indent=1) + "\n", encoding="utf-8")
    print(f"wrote {path} ({len(CASES)} cases x {len(POINTS)} points)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
