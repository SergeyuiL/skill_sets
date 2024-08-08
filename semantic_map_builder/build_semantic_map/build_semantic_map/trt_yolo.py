import os

from ultralytics import YOLO

ckpt_dir = "/home/nvidia/skillsets_ws/src/skill_sets/semantic_map_builder/checkpoint"
        
pt_path = os.path.join(ckpt_dir, "best.pt")
engine_path = os.path.join(ckpt_dir, "best.engine")

raw_model = YOLO(pt_path)
raw_model.export(format="engine", dynamic=True, batch=1) 