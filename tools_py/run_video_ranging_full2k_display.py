
#!/usr/bin/env python3
import math
import os
import time
from pathlib import Path

import cv2
import numpy as np

from gimbal_vision_ranging import _CpuYoloDetector, _DistanceRuntime, ROI_SIZE

ROOT = Path('/home/linaro/gimbal')
VIDEO = ROOT / '20~250m.mkv'
if not VIDEO.is_file():
    mkvs = sorted(ROOT.glob('*.mkv'))
    VIDEO = mkvs[0] if mkvs else None
if VIDEO is None or not VIDEO.is_file():
    raise SystemExit('No mkv video found in /home/linaro/gimbal')

CONF = float(os.getenv('GIMBAL_VISION_CONFIDENCE', '0.25'))
MAX_FRAMES = int(os.getenv('VIDEO_RANGING_MAX_FRAMES', '120'))
START_FRAME = int(os.getenv('VIDEO_RANGING_START_FRAME', '0'))
DISPLAY_SCALE = float(os.getenv('VIDEO_RANGING_DISPLAY_SCALE', '0.5'))
TILE_OVERLAP = int(os.getenv('VIDEO_RANGING_TILE_OVERLAP', '160'))
NMS_IOU = float(os.getenv('VIDEO_RANGING_FULL_NMS_IOU', '0.45'))
ASSOC_PX = float(os.getenv('VIDEO_RANGING_ASSOC_PX', '120'))

print(f'[Full2K] video={VIDEO}', flush=True)
print(f'[Full2K] conf={CONF}, max_frames={MAX_FRAMES}, overlap={TILE_OVERLAP}', flush=True)

cap = cv2.VideoCapture(str(VIDEO))
if not cap.isOpened():
    raise SystemExit(f'Cannot open video: {VIDEO}')
if START_FRAME > 0:
    cap.set(cv2.CAP_PROP_POS_FRAMES, START_FRAME)

detector = _CpuYoloDetector(ROOT / 'distance model' / 'best.pt', CONF)
tracks = {}
next_track_id = 1

cv2.namedWindow('gimbal_yolo_full2k_ranging', cv2.WINDOW_NORMAL)
cv2.resizeWindow('gimbal_yolo_full2k_ranging', 1280, 720)

def tile_positions(length, tile, overlap):
    if length <= tile:
        return [0]
    step = max(1, tile - overlap)
    positions = list(range(0, max(1, length - tile + 1), step))
    last = length - tile
    if positions[-1] != last:
        positions.append(last)
    return positions

def iou_xyxy(a, b):
    x1 = max(a[0], b[0]); y1 = max(a[1], b[1])
    x2 = min(a[2], b[2]); y2 = min(a[3], b[3])
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    area_a = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
    area_b = max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])
    return inter / max(1e-6, area_a + area_b - inter)

def global_nms(dets, threshold):
    dets = sorted(dets, key=lambda d: d['confidence'], reverse=True)
    kept = []
    for det in dets:
        if all(iou_xyxy(det['box'], old['box']) < threshold for old in kept):
            kept.append(det)
    return kept

frame_count = 0
last_log_t = time.time()
try:
    while True:
        ok, frame = cap.read()
        if not ok:
            print('[Full2K] video ended', flush=True)
            break
        frame_count += 1
        h, w = frame.shape[:2]
        xs = tile_positions(w, ROI_SIZE, TILE_OVERLAP)
        ys = tile_positions(h, ROI_SIZE, TILE_OVERLAP)

        crops = []
        origins = []
        for y0 in ys:
            for x0 in xs:
                crops.append(frame[y0:y0 + ROI_SIZE, x0:x0 + ROI_SIZE])
                origins.append((x0, y0))

        t0 = time.time()
        rows_by_crop = detector.detect_batch(crops)
        infer_ms = (time.time() - t0) * 1000.0

        detections = []
        for rows, (x0, y0) in zip(rows_by_crop, origins):
            for row in rows:
                x1 = float(row[0] + x0); y1 = float(row[1] + y0)
                x2 = float(row[2] + x0); y2 = float(row[3] + y0)
                conf = float(row[4])
                if x2 <= x1 or y2 <= y1:
                    continue
                detections.append({'box': [x1, y1, x2, y2], 'confidence': conf})
        detections = global_nms(detections, NMS_IOU)

        assigned = set()
        now = time.time()
        for det in detections:
            x1, y1, x2, y2 = det['box']
            center = np.array([(x1 + x2) / 2.0, (y1 + y2) / 2.0], dtype=np.float32)
            best_id = None
            best_dist = ASSOC_PX
            for tid, tr in tracks.items():
                if tid in assigned:
                    continue
                dist = float(np.linalg.norm(center - tr['center']))
                if dist < best_dist:
                    best_id = tid
                    best_dist = dist
            if best_id is None:
                best_id = next_track_id
                next_track_id += 1
                tracks[best_id] = {
                    'center': center,
                    'runtime': _DistanceRuntime(),
                    'last_seen': frame_count,
                    'last_out': None,
                }
            tr = tracks[best_id]
            assigned.add(best_id)
            tr['center'] = 0.7 * tr['center'] + 0.3 * center
            tr['last_seen'] = frame_count
            out = tr['runtime'].update(det, now)
            tr['last_out'] = out

            color = (0, 220, 0) if out.get('valid') else (0, 180, 255)
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            label = f'ID{best_id} conf={det["confidence"]:.2f}'
            if out.get('valid'):
                label += f' {out["distance"]:.1f}m'
            else:
                label += f' warmup={out.get("warmup_count", 0)}/25'
                if out.get('reason'):
                    label += f' {out.get("reason")}'
            cv2.putText(frame, label, (int(x1), max(24, int(y1) - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv2.LINE_AA)

        for tid in list(tracks.keys()):
            if frame_count - tracks[tid]['last_seen'] > 20:
                del tracks[tid]

        # draw tile grid lightly so we know YOLO covered the full 2K frame
        for y0 in ys:
            for x0 in xs:
                cv2.rectangle(frame, (x0, y0), (x0 + ROI_SIZE, y0 + ROI_SIZE), (80, 80, 80), 1)

        cv2.putText(frame, f'FULL 2K tiled YOLO: {len(crops)} tiles, det={len(detections)}, infer={infer_ms/1000:.2f}s, frame={int(cap.get(cv2.CAP_PROP_POS_FRAMES))}',
                    (20, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, 'No frame-diff. Full frame is covered by 640x640 YOLO tiles. Press q/Esc to exit.',
                    (20, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.68, (255, 220, 120), 2, cv2.LINE_AA)

        show = cv2.resize(frame, (int(w * DISPLAY_SCALE), int(h * DISPLAY_SCALE))) if DISPLAY_SCALE != 1.0 else frame
        cv2.imshow('gimbal_yolo_full2k_ranging', show)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord('q'), ord('Q')):
            print('[Full2K] stopped by key', flush=True)
            break

        if time.time() - last_log_t > 5:
            print(f'[Full2K] frame={frame_count}, tiles={len(crops)}, detections={len(detections)}, infer_s={infer_ms/1000:.2f}, tracks={len(tracks)}', flush=True)
            last_log_t = time.time()
        if MAX_FRAMES > 0 and frame_count >= MAX_FRAMES:
            print('[Full2K] max frame reached', flush=True)
            break
finally:
    cap.release()
    cv2.destroyAllWindows()
    print('[Full2K] exit', flush=True)
