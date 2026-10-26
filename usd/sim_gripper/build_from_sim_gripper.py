"""
build_from_sim_gripper.py
=========================
Hall_Sensor/Code/Sim_Gripper.py 가 그리는 핀어레이 그리퍼 CAD 를 Isaac Sim 용
메시로 옮긴다. 핀은 움직이지 않으므로(정지 자세) 판 하나를 색깔별 4덩어리로
합쳐서, 브리지가 매 프레임 판 두 개의 X 이동만 바꾸면 되게 한다.

입력 (Hall_Sensor/Code/Simulation/)
  Gripper_Body.stl                 몸통 (윗면 Z=0 이 로봇 플랜지 쪽 장착면)
  Left_Plate_v2.stl                +X 판 (하우징 + PCB + 센서 + 핀)
  Left_Plate_without_pin_v2.stl    +X 판 (핀 없음)

출력 (이 폴더, 단위 mm, Sim_Gripper 좌표계)
  body.stl
  plate_p_house.stl / plate_p_pcb.stl / plate_p_pins.stl / plate_p_mag.stl   +X 판
  plate_n_*.stl                                                             -X 판 (거울상)
  meta.json   닫힘 기준, 판 offset, 잡는 점 등

판 메시는 '닫힘(양쪽 핀 끝이 X=0 에서 만남)' 자세로 미리 옮겨 둔다. 브리지는
+X 판을 +(열림 + PLATE_OFFSET_MM), -X 판을 -(열림 + PLATE_OFFSET_MM) 만큼 민다.
Sim_Gripper._apply() 와 같은 규칙이다.

    python build_from_sim_gripper.py
    python build_from_sim_gripper.py --code C:/Users/oskwo/Desktop/Hall_Sensor/Code
"""
import argparse
import json
import os
import sys

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CODE = os.path.normpath(os.path.join(HERE, "..", "..", "..", "Hall_Sensor", "Code"))

p = argparse.ArgumentParser()
p.add_argument("--code", default=DEFAULT_CODE, help="Hall_Sensor/Code 폴더")
args = p.parse_args()

sys.path.insert(0, args.code)
import Sim_Gripper as SG  # noqa: E402  (CAD 경로·핀 추출 규칙을 그대로 쓴다)


def cat(meshes):
    meshes = [m for m in meshes if m is not None]
    return trimesh.util.concatenate(meshes) if len(meshes) > 1 else meshes[0]


def _magnet(x_back, y, z):
    """Sim_Gripper._magnet_mesh 와 같다: 핀 뒤끝에 붙는 자석 원통, 축은 X."""
    cyl = trimesh.creation.cylinder(radius=SG.MAGNET_D_MM / 2.0, height=SG.MAGNET_H_MM, sections=32)
    cyl.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
    cyl.apply_translation([x_back + SG.MAGNET_H_MM / 2.0, y, z])
    return cyl


full = SG.load(SG.FULL_STL)
house = SG.load(SG.HOUSE_STL)
body = SG.load(SG.BODY_STL)
pins, _, shafts = SG.extract_pins(full)
if full.centroid[0] < 0:
    raise SystemExit("[error] 판 STL 이 -X 쪽이다. 이 변환기는 +X 판을 기준으로 쓴다.")

tip0 = float(min(c.bounds[0][0] for c in shafts))     # STL 그대로의 핀 끝 X
stl_open = abs(tip0)
P = np.array([c.centroid for c in shafts])
grasp = [0.0, float((P[:, 1].min() + P[:, 1].max()) / 2), float((P[:, 2].min() + P[:, 2].max()) / 2)]

parts = house.split(only_watertight=False)
groups = {
    "house": cat([c for c in parts if not SG.is_pcb(c)]),
    "pcb": cat([c for c in parts if SG.is_pcb(c)]),
    "pins": cat([m for st, pk in pins for m in (st, pk)]),
    "mag": cat([_magnet(max(c.bounds[1][0] for c in shafts), sh.centroid[1], sh.centroid[2])
                for sh in shafts]),
}

shift = trimesh.transformations.translation_matrix([-stl_open, 0.0, 0.0])
n_tri = len(body.faces)
body.export(os.path.join(HERE, "body.stl"))
for name, m in groups.items():
    mp = m.copy()
    mp.apply_transform(shift)                         # 닫힘 자세: 핀 끝이 X=0
    mp.export(os.path.join(HERE, f"plate_p_{name}.stl"))
    SG.mirror_x(mp).export(os.path.join(HERE, f"plate_n_{name}.stl"))
    n_tri += 2 * len(mp.faces)

meta = {
    "source": os.path.relpath(SG.FULL_STL, args.code).replace("\\", "/"),
    "units": "mm",
    "frame": "Sim_Gripper: body top (mount face) Z=0, fingers -Z, plates open along +-X",
    "plate_offset_mm": SG.PLATE_OFFSET_MM,      # 닫힘에서도 판 하나가 벌어져 있는 양
    "plate_travel_mm": SG.MAX_OPEN_MM,          # 판 하나 최대 이동 (모터 행정)
    "grasp_center_mm": grasp,                   # 두 판 핀 격자의 가운데 (닫힘 자세)
    "colors": {"body": SG.MAT_BODY[0], "house": SG.MAT_BODY[0], "pcb": SG.MAT_PCB[0],
               "pins": SG.MAT_PIN[0], "mag": SG.MAT_MAG[0]},
    "triangles": int(n_tri),
}
json.dump(meta, open(os.path.join(HERE, "meta.json"), "w"), indent=1)
print(f"삼각형 {n_tri:,} 개, 잡는 점 {np.round(grasp, 2).tolist()} mm, 원래 열림 {stl_open:.2f} mm")
print("->", HERE)
