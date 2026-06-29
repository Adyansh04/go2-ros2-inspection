#!/usr/bin/env python3
"""detect_utils -- open-vocabulary object detection helpers (YOLOE) for the inspection scan.

Shared by zone_inspector (live detection during the viewpoint spin). Detection + crop only; no reading.

YOLOE text-prompt mode (the correct ultralytics 8.x API, verified):
    from ultralytics import YOLOE
    model = YOLOE("yoloe-11s-seg.pt")
    model.set_classes(names, model.get_text_pe(names))     # encode the open-vocab text prompts
The model returned by predict() emits class ids that index into `names` (== PROMPTS order here).

`get_text_pe` needs a CLIP text backend (ultralytics' CLIP fork). If it is absent, load_model raises a
clear error and the caller degrades gracefully (scan still runs, captures nothing). Weights are NOT
auto-downloaded in the sim path: set YOLOE_WEIGHTS=/abs/path or place ~/weights/yoloe-11s-seg.pt.
"""
import os
import numpy as np
import cv2

# Open-vocabulary prompts for the inspection_arena props (drop the old gauge/goggles/helmet set).
PROMPTS = [
    "fire", "person", "fire extinguisher", "fire hydrant", "barrel", "pallet", "cardboard box",
    "wooden crate", "storage rack", "metal cabinet", "electrical box", "valve", "pump", "exit sign",
    "office chair", "desk", "table", "trash can", "traffic cone", "concrete barrier",
]

DEFAULT_WEIGHTS = os.environ.get("YOLOE_WEIGHTS", os.path.expanduser("~/weights/yoloe-11s-seg.pt"))

# stable BGR colour per semantic group (hazards red, people orange, safety yellow, else green)
_RED = (40, 40, 220); _ORANGE = (0, 140, 255); _YELLOW = (0, 220, 220); _GREEN = (60, 200, 60)


def color_for(name):
    n = name.lower()
    if "fire" in n or "barrel" in n or "electrical" in n:
        return _RED
    if "person" in n:
        return _ORANGE
    if "exit" in n or "extinguisher" in n or "hydrant" in n or "cone" in n or "barrier" in n:
        return _YELLOW
    return _GREEN


def _pe_cache_path(weights, prompts):
    import hashlib
    h = hashlib.md5("|".join(prompts).encode()).hexdigest()[:10]
    return os.path.join(os.path.dirname(weights) or ".", f"yoloe_pe_{h}.pt")


def load_model(weights=None, device="", prompts=PROMPTS):
    """Load YOLOE and set the open-vocab text prompts. The prompt embeddings are computed ONCE via the
    CLIP/MobileCLIP text backend and cached next to the weights; subsequent loads reuse the cache, so a
    runtime scan never needs the (large) text backend on disk. Raises (caller degrades gracefully) if
    ultralytics or the weights file are unavailable and no embedding cache exists."""
    weights = os.path.expanduser(weights or DEFAULT_WEIGHTS)
    if not os.path.exists(weights) and not os.environ.get("YOLOE_ALLOW_DOWNLOAD"):
        raise FileNotFoundError(
            f"YOLOE weights not found: {weights} (set YOLOE_WEIGHTS=/abs/path or YOLOE_ALLOW_DOWNLOAD=1)")
    from ultralytics import YOLOE                       # ImportError if ultralytics missing
    import torch
    names = list(prompts)
    model = YOLOE(weights)
    cache = _pe_cache_path(weights, names)
    pe = None
    if os.path.exists(cache):
        try:
            pe = torch.load(cache, map_location="cpu")
        except Exception:
            pe = None
    if pe is None:
        pe = model.get_text_pe(names)                  # needs the CLIP/MobileCLIP text backend (once)
        try:
            torch.save(pe.cpu(), cache)
        except Exception:
            pass
    model.set_classes(names, pe)                       # YOLOE open-vocab API (embeddings, not just names)
    if device:
        try:
            model.to(device)
        except Exception:
            pass
    return model


def infer(model, img_bgr, conf=0.10, imgsz=640, device="", prompts=PROMPTS):
    """Run YOLOE on one BGR frame -> [(class_name, conf, [x0,y0,x1,y1]), ...] in pixel coords."""
    H, W = img_bgr.shape[:2]
    imgsz = int(min(1280, max(640, ((max(W, H) + 31) // 32) * 32)))
    kw = {"conf": conf, "imgsz": imgsz, "verbose": False}
    if device:
        kw["device"] = device
    res = model.predict(img_bgr, **kw)
    out = []
    if not res or res[0].boxes is None or len(res[0].boxes) == 0:
        return out
    r = res[0]
    boxes = r.boxes.xyxy.cpu().numpy()
    cls_ids = r.boxes.cls.cpu().numpy().astype(int)
    confs = r.boxes.conf.cpu().numpy()
    for i, b in enumerate(boxes):
        x0, y0, x1, y1 = [int(v) for v in b]
        x0, y0 = max(0, x0), max(0, y0); x1, y1 = min(W, x1), min(H, y1)
        if x1 <= x0 or y1 <= y0:
            continue
        name = prompts[cls_ids[i]] if 0 <= cls_ids[i] < len(prompts) else str(cls_ids[i])
        out.append((name, float(confs[i]), [x0, y0, x1, y1]))
    return out


def contact_sheet(zone_dir, objects, ch=160):
    """Montage of one crop per unique object (objects: dicts with 'class' + 'crop' relative path)."""
    rows = [o for o in objects if o.get("crop")]
    if not rows:
        return None
    sheet = np.full((ch + 22, ch * len(rows), 3), 40, np.uint8)
    for k, o in enumerate(rows):
        c = cv2.imread(os.path.join(zone_dir, o["crop"]))
        if c is None:
            continue
        s = ch / max(c.shape[:2])
        c = cv2.resize(c, (max(1, int(c.shape[1] * s)), max(1, int(c.shape[0] * s))))
        sheet[:c.shape[0], k * ch:k * ch + c.shape[1]] = c
        cv2.putText(sheet, o["class"].split()[0][:9], (k * ch + 4, ch + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_for(o["class"]), 1)
    out = os.path.join(zone_dir, "objects_contact_sheet.png")
    cv2.imwrite(out, sheet)
    return out
