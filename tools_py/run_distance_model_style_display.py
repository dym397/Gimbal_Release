
#!/usr/bin/env python3
# Distance-model style YOLO pure-video runner for RK3588:
# full-frame YOLO -> highest confidence bbox -> physics/MLP/GRU ranging -> HUD.
import math
import os
import time
from pathlib import Path

import cv2
import numpy as np

from gimbal_vision_ranging import _DistanceRuntime, ROI_SIZE

ROOT = Path('/home/linaro/gimbal')
VIDEO = ROOT / '20~250m.mkv'
if not VIDEO.is_file():
    mkvs = sorted(ROOT.glob('*.mkv'))
    VIDEO = mkvs[0] if mkvs else None
if VIDEO is None or not VIDEO.is_file():
    raise SystemExit('No mkv video found in /home/linaro/gimbal')

ONNX = ROOT / 'distance model' / 'best_2k.onnx'
if not ONNX.is_file():
    raise SystemExit(f'No ONNX model: {ONNX}')

CONF = float(os.getenv('GIMBAL_VISION_CONFIDENCE', '0.25'))
NMS_IOU = float(os.getenv('VIDEO_NMS_IOU', '0.45'))
MAX_FRAMES = int(os.getenv('VIDEO_RANGING_MAX_FRAMES', '800'))
START_FRAME = int(os.getenv('VIDEO_RANGING_START_FRAME', '0'))
FRAME_STRIDE = max(1, int(os.getenv('VIDEO_FRAME_STRIDE', '1')))
DISPLAY_SCALE = float(os.getenv('VIDEO_RANGING_DISPLAY_SCALE', '0.5'))
DETAIL_EVERY = max(1, int(os.getenv('VIDEO_DETAIL_LOG_EVERY', '1')))
SNAPSHOT_DIR = ROOT / 'distance_model_style_snapshots'
SNAPSHOT_DIR.mkdir(exist_ok=True)
SNAPSHOT_FRAMES = {1, 2, 3, 25, 50, 100, 150, 200}

print(f'[DMStyle] video={VIDEO}', flush=True)
print(f'[DMStyle] model={ONNX}', flush=True)
print(f'[DMStyle] conf={CONF}, nms={NMS_IOU}, frame_stride={FRAME_STRIDE}', flush=True)

cap = cv2.VideoCapture(str(VIDEO))
if not cap.isOpened():
    raise SystemExit(f'Cannot open video: {VIDEO}')
if START_FRAME > 0:
    cap.set(cv2.CAP_PROP_POS_FRAMES, START_FRAME)
fps = float(cap.get(cv2.CAP_PROP_FPS))
if not math.isfinite(fps) or fps <= 0:
    fps = 25.0
print(f'[DMStyle] video_fps={fps:.3f}', flush=True)

net = cv2.dnn.readNetFromONNX(str(ONNX))
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
runtime = _DistanceRuntime()
print(f'[DMStyle] ranging_backend={runtime.gru_backend}', flush=True)

cv2.namedWindow('distance_model_original_yolo_ranging', cv2.WINDOW_NORMAL)
cv2.resizeWindow('distance_model_original_yolo_ranging', 1280, 720)

def letterbox(frame, new_shape=640, color=(114, 114, 114)):
    h, w = frame.shape[:2]
    r = min(new_shape / w, new_shape / h)
    nw, nh = int(round(w * r)), int(round(h * r))
    resized = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((new_shape, new_shape, 3), color, dtype=np.uint8)
    dw = (new_shape - nw) // 2
    dh = (new_shape - nh) // 2
    canvas[dh:dh + nh, dw:dw + nw] = resized
    return canvas, r, dw, dh

def detect_full_frame(frame):
    h, w = frame.shape[:2]
    gain = 1.0
    pad_x = 0.0
    pad_y = 0.0
    blob = cv2.dnn.blobFromImage(frame, 1.0 / 255.0, (w, h), swapRB=True, crop=False)
    net.setInput(blob)
    out = np.asarray(net.forward())
    if out.ndim == 3:
        out = out[0]
    boxes = []
    scores = []
    cls_ids = []
    for row in out:
        obj = float(row[4])
        if row.shape[0] > 5:
            cls_scores = row[5:]
            cls_id = int(np.argmax(cls_scores))
            score = obj * float(cls_scores[cls_id])
        else:
            cls_id = 0
            score = obj
        if score < CONF:
            continue
        cx, cy, bw, bh = [float(v) for v in row[:4]]
        x1 = (cx - bw / 2.0 - pad_x) / gain
        y1 = (cy - bh / 2.0 - pad_y) / gain
        x2 = (cx + bw / 2.0 - pad_x) / gain
        y2 = (cy + bh / 2.0 - pad_y) / gain
        x1 = max(0.0, min(float(w - 1), x1)); y1 = max(0.0, min(float(h - 1), y1))
        x2 = max(0.0, min(float(w - 1), x2)); y2 = max(0.0, min(float(h - 1), y2))
        if x2 <= x1 or y2 <= y1:
            continue
        boxes.append([x1, y1, x2 - x1, y2 - y1])
        scores.append(score)
        cls_ids.append(cls_id)
    if not boxes:
        return [], 0.0
    idxs = cv2.dnn.NMSBoxes(boxes, scores, CONF, NMS_IOU)
    dets = []
    for idx in np.array(idxs).reshape(-1):
        x, y, bw, bh = boxes[int(idx)]
        dets.append({
            'box': [x, y, x + bw, y + bh],
            'confidence': float(scores[int(idx)]),
            'class_id': int(cls_ids[int(idx)]),
        })
    dets.sort(key=lambda d: d['confidence'], reverse=True)
    return dets, gain

proc_count = 0
last_log = time.time()
last_out = None
try:
    while True:
        ok, frame = cap.read()
        if not ok:
            print('[DMStyle] video ended', flush=True)
            break
        proc_count += 1
        video_pos = int(cap.get(cv2.CAP_PROP_POS_FRAMES))
        curr_time_s = video_pos / fps

        t0 = time.time()
        dets, gain = detect_full_frame(frame)
        infer_ms = (time.time() - t0) * 1000.0

        measurement_valid = False
        reason = 'TARGET_MISSING'
        best = None
        out = None
        if dets:
            # distance_model_gui.py YOLO mode sorts by confidence and uses pred[0].
            best = dets[0]
            out = runtime.update(best, curr_time_s)
            last_out = out
            measurement_valid = bool(out.get('valid'))
            reason = out.get('reason', '')
        else:
            runtime.mark_missing(curr_time_s)

        show = frame.copy()
        for det in dets[:10]:
            x1, y1, x2, y2 = det['box']
            cv2.rectangle(show, (int(x1), int(y1)), (int(x2), int(y2)), (80, 80, 80), 1)
            cv2.putText(show, f'{det["confidence"]:.2f}', (int(x1), max(20, int(y1) - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 180), 1, cv2.LINE_AA)
        if best is not None:
            x1, y1, x2, y2 = best['box']
            color = (0, 255, 0) if out and out.get('valid') else (0, 255, 255)
            cv2.rectangle(show, (int(x1), int(y1)), (int(x2), int(y2)), color, 3)
            label = f'BEST conf={best["confidence"]:.2f}'
            if out and out.get('valid'):
                label += f' Final={out["distance"]:.2f}m {out.get("source")}'
            elif out:
                label += f' warmup={out.get("warmup_count", 0)}/25 {out.get("reason", "")}'
            cv2.putText(show, label, (int(x1), max(24, int(y1) - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2, cv2.LINE_AA)

        # HUD, matching distance_model_gui spirit.
        overlay = show.copy()
        cv2.rectangle(overlay, (15, 15), (650, 245), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.62, show, 0.38, 0, show)
        font = cv2.FONT_HERSHEY_SIMPLEX
        if out and out.get('valid'):
            state = 'TRACKING'
            state_color = (0, 255, 0)
            raw_phy = f'{out.get("physics_distance", float("nan")):.2f} m'
            mlp = f'{out.get("mlp_distance", float("nan")):.2f} m'
            final = f'{out.get("distance", float("nan")):.2f} m'
            gru = final if out.get('source') == 'gimbal_yolo_gru' else out.get('source', '')
            warm = f'{out.get("warmup_count", 0)}/25'
        elif out:
            state = 'WARMING_UP' if not out.get('reason') else 'INVALID'
            state_color = (0, 255, 255) if state == 'WARMING_UP' else (0, 0, 255)
            raw_phy = f'{out.get("physics_distance", float("nan")):.2f} m' if 'physics_distance' in out else 'N/A'
            mlp = f'{out.get("mlp_distance", float("nan")):.2f} m' if 'mlp_distance' in out else 'N/A'
            final = 'N/A'
            gru = f'WARMING_UP {out.get("warmup_count", 0)}/25'
            warm = f'{out.get("warmup_count", 0)}/25'
            reason = out.get('reason', '')
        else:
            state = 'LOST'
            state_color = (0, 0, 255)
            raw_phy = mlp = final = gru = warm = 'N/A'
        cv2.putText(show, 'Distance Model YOLO mode (full-frame best box)', (25, 40), font, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(show, f'Time: {curr_time_s:.2f}s | Frame: {video_pos} | Infer: {infer_ms:.0f}ms | det={len(dets)}', (25, 68), font, 0.58, (220, 220, 220), 1, cv2.LINE_AA)
        cv2.putText(show, f'Tracking State: {state}', (25, 96), font, 0.58, state_color, 2, cv2.LINE_AA)
        cv2.putText(show, f'Robust Physics: {raw_phy}', (25, 124), font, 0.58, (0, 165, 255), 1, cv2.LINE_AA)
        cv2.putText(show, f'MLP: {mlp}', (25, 152), font, 0.58, (255, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(show, f'GRU: {gru}', (25, 180), font, 0.58, (255, 192, 203), 1, cv2.LINE_AA)
        cv2.putText(show, f'Final Stable: {final} | warmup={warm} | reason={reason}', (25, 208), font, 0.58, (0, 255, 255), 1, cv2.LINE_AA)

        if proc_count % DETAIL_EVERY == 0:
            if best is not None:
                bx = best['box']; bw = bx[2]-bx[0]; bh = bx[3]-bx[1]
                if out and out.get('valid'):
                    dist = f'distance={out.get("distance"):.2f}m,source={out.get("source")}'
                elif out:
                    dist = f'warmup={out.get("warmup_count",0)}/25,reason={out.get("reason","")}'
                else:
                    dist = 'no_out'
                print(f'[DMStyleDetail] proc={proc_count},video_pos={video_pos},det={len(dets)},best_conf={best["confidence"]:.2f},box=({bx[0]:.0f},{bx[1]:.0f},{bw:.0f}x{bh:.0f}),infer_ms={infer_ms:.1f},{dist}', flush=True)
            else:
                print(f'[DMStyleDetail] proc={proc_count},video_pos={video_pos},det=0,infer_ms={infer_ms:.1f},TARGET_MISSING', flush=True)

        if video_pos in SNAPSHOT_FRAMES:
            cv2.imwrite(str(SNAPSHOT_DIR / f'dmstyle_frame_{video_pos:05d}.jpg'), show)
        h, w = show.shape[:2]
        disp = cv2.resize(show, (int(w * DISPLAY_SCALE), int(h * DISPLAY_SCALE))) if DISPLAY_SCALE != 1.0 else show
        cv2.imshow('distance_model_original_yolo_ranging', disp)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord('q'), ord('Q')):
            print('[DMStyle] stopped by key', flush=True)
            break
        for _ in range(FRAME_STRIDE - 1):
            if not cap.grab():
                break
        if MAX_FRAMES > 0 and proc_count >= MAX_FRAMES:
            print('[DMStyle] max frames reached', flush=True)
            break
finally:
    cap.release()
    cv2.destroyAllWindows()
    print('[DMStyle] exit', flush=True)
