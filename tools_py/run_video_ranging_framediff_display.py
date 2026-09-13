
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
MAX_FRAMES = int(os.getenv('VIDEO_RANGING_MAX_FRAMES', '500'))
FRAME_STRIDE = max(1, int(os.getenv('VIDEO_FRAME_STRIDE', '10')))
DETAIL_LOG_EVERY = max(1, int(os.getenv('VIDEO_DETAIL_LOG_EVERY', '3')))
START_FRAME = int(os.getenv('VIDEO_RANGING_START_FRAME', '0'))
DISPLAY_SCALE = float(os.getenv('VIDEO_RANGING_DISPLAY_SCALE', '0.5'))
DIFF_THRESHOLD = int(os.getenv('VIDEO_DIFF_THRESHOLD', '16'))
MIN_MOTION_AREA = float(os.getenv('VIDEO_MIN_MOTION_AREA', '8'))
MAX_ROIS = int(os.getenv('VIDEO_MAX_ROIS', '8'))
ASSOC_PX = float(os.getenv('VIDEO_RANGING_ASSOC_PX', '120'))
NMS_IOU = float(os.getenv('VIDEO_RANGING_NMS_IOU', '0.45'))
ROI_MARGIN = int(os.getenv('VIDEO_ROI_MARGIN', '80'))

print(f'[FrameDiff] video={VIDEO}', flush=True)
print(f'[FrameDiff] conf={CONF}, max_frames={MAX_FRAMES}, frame_stride={FRAME_STRIDE}, diff_threshold={DIFF_THRESHOLD}, min_area={MIN_MOTION_AREA}, max_rois={MAX_ROIS}', flush=True)

cap = cv2.VideoCapture(str(VIDEO))
if not cap.isOpened():
    raise SystemExit(f'Cannot open video: {VIDEO}')
if START_FRAME > 0:
    cap.set(cv2.CAP_PROP_POS_FRAMES, START_FRAME)

VIDEO_FPS = float(cap.get(cv2.CAP_PROP_FPS))
if not math.isfinite(VIDEO_FPS) or VIDEO_FPS <= 0:
    VIDEO_FPS = 25.0
print(f'[FrameDiff] video_fps={VIDEO_FPS:.3f}', flush=True)

detector = _CpuYoloDetector(ROOT / 'distance model' / 'best.pt', CONF)
tracks = {}
next_track_id = 1
prev_gray = None
last_motion_centers = []

cv2.namedWindow('gimbal_yolo_framediff_ranging', cv2.WINDOW_NORMAL)
cv2.resizeWindow('gimbal_yolo_framediff_ranging', 1280, 720)

def clamp_roi_origin(cx, cy, w, h):
    x0 = int(round(cx - ROI_SIZE / 2))
    y0 = int(round(cy - ROI_SIZE / 2))
    x0 = max(0, min(max(0, w - ROI_SIZE), x0))
    y0 = max(0, min(max(0, h - ROI_SIZE), y0))
    return x0, y0

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

def build_motion_rois(frame, prev_gray):
    h, w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    if prev_gray is None:
        return gray, [], []

    diff = cv2.absdiff(gray, prev_gray)
    _, mask = cv2.threshold(diff, DIFF_THRESHOLD, 255, cv2.THRESH_BINARY)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1)
    mask = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    motion_boxes = []
    for c in contours:
        area = float(cv2.contourArea(c))
        if area < MIN_MOTION_AREA:
            continue
        x, y, bw, bh = cv2.boundingRect(c)
        if bw < 3 or bh < 3:
            continue
        # avoid huge global lighting/scene changes from consuming all ROIs
        if bw * bh > 0.20 * w * h:
            continue
        motion_boxes.append({'box': [x, y, x + bw, y + bh], 'area': area})
    motion_boxes.sort(key=lambda item: item['area'], reverse=True)

    rois = []
    centers = []
    for item in motion_boxes:
        x1, y1, x2, y2 = item['box']
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        # enlarge ROI center slightly toward motion bbox center; margin is visual/for small drift only.
        x0, y0 = clamp_roi_origin(cx, cy, w, h)
        roi = [x0, y0, x0 + ROI_SIZE, y0 + ROI_SIZE]
        duplicate = False
        for old in rois:
            if iou_xyxy(roi, old) > 0.65:
                duplicate = True
                break
        if duplicate:
            continue
        rois.append(roi)
        centers.append((cx, cy))
        if len(rois) >= MAX_ROIS:
            break
    return gray, motion_boxes, rois

frame_count = 0
last_log_t = time.time()
try:
    while True:
        ok, frame = cap.read()
        if not ok:
            print('[FrameDiff] video ended', flush=True)
            break
        frame_count += 1
        h, w = frame.shape[:2]
        gray, motion_boxes, rois = build_motion_rois(frame, prev_gray)
        prev_gray = gray

        # If there is a brief no-motion gap, keep the latest track centers alive for a few frames.
        if not rois and tracks:
            recent = sorted(tracks.items(), key=lambda kv: kv[1]['last_seen'], reverse=True)[:MAX_ROIS]
            for _, tr in recent:
                cx, cy = tr['center']
                x0, y0 = clamp_roi_origin(cx, cy, w, h)
                rois.append([x0, y0, x0 + ROI_SIZE, y0 + ROI_SIZE])

        crops = [frame[y1:y2, x1:x2] for x1, y1, x2, y2 in rois]
        t0 = time.time()
        rows_by_crop = detector.detect_batch(crops) if crops else []
        infer_ms = (time.time() - t0) * 1000.0

        detections = []
        for rows, roi in zip(rows_by_crop, rois):
            x0, y0, _, _ = roi
            for row in rows:
                x1 = float(row[0] + x0); y1 = float(row[1] + y0)
                x2 = float(row[2] + x0); y2 = float(row[3] + y0)
                conf = float(row[4])
                if x2 <= x1 or y2 <= y1:
                    continue
                detections.append({'box': [x1, y1, x2, y2], 'confidence': conf})
        detections = global_nms(detections, NMS_IOU)

        assigned = set()
        video_pos = int(cap.get(cv2.CAP_PROP_POS_FRAMES))
        now = video_pos / VIDEO_FPS
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
                tracks[best_id] = {'center': center, 'runtime': _DistanceRuntime(), 'last_seen': frame_count, 'last_out': None}
            tr = tracks[best_id]
            assigned.add(best_id)
            tr['center'] = 0.7 * tr['center'] + 0.3 * center
            tr['last_seen'] = frame_count
            out = tr['runtime'].update(det, now)
            tr['last_out'] = out
            if frame_count % DETAIL_LOG_EVERY == 0:
                bx = det['box']
                bw = bx[2] - bx[0]
                bh = bx[3] - bx[1]
                if out.get('valid'):
                    dist_text = f"distance={out.get('distance', float('nan')):.2f}m,source={out.get('source')}"
                else:
                    dist_text = f"warmup={out.get('warmup_count', 0)}/25,reason={out.get('reason', '')}"
                print(f"[RangingDetail] proc={frame_count},track={best_id},conf={det['confidence']:.2f},box=({bx[0]:.0f},{bx[1]:.0f},{bw:.0f}x{bh:.0f}),{dist_text}", flush=True)

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
            if frame_count - tracks[tid]['last_seen'] > 30:
                del tracks[tid]

        for item in motion_boxes[:30]:
            x1, y1, x2, y2 = [int(v) for v in item['box']]
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 0), 1)
        for roi in rois:
            x1, y1, x2, y2 = roi
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 100, 0), 2)

        if frame_count % DETAIL_LOG_EVERY == 0:
            detail_parts = []
            for det in detections[:6]:
                bx = det['box']
                bw = bx[2] - bx[0]
                bh = bx[3] - bx[1]
                detail_parts.append(f"conf={det['confidence']:.2f},box=({bx[0]:.0f},{bx[1]:.0f},{bw:.0f}x{bh:.0f})")
            print(f"[FrameDiffDetail] proc={frame_count}, video_pos={video_pos}, motion={len(motion_boxes)}, rois={len(rois)}, det={len(detections)}, infer_ms={infer_ms:.1f}, {'; '.join(detail_parts) if detail_parts else 'no_det'}", flush=True)

        cv2.putText(frame, f'FrameDiff + YOLO ROI: motion={len(motion_boxes)}, rois={len(rois)}, det={len(detections)}, infer={infer_ms:.0f}ms, frame={video_pos}',
                    (20, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.78, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, 'cyan=motion, blue=640 ROI, green/orange=YOLO ranging. Press q/Esc to exit.',
                    (20, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.68, (255, 220, 120), 2, cv2.LINE_AA)

        show = cv2.resize(frame, (int(w * DISPLAY_SCALE), int(h * DISPLAY_SCALE))) if DISPLAY_SCALE != 1.0 else frame
        cv2.imshow('gimbal_yolo_framediff_ranging', show)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord('q'), ord('Q')):
            print('[FrameDiff] stopped by key', flush=True)
            break

        if time.time() - last_log_t > 5:
            print(f'[FrameDiff] frame={frame_count}, motion={len(motion_boxes)}, rois={len(rois)}, detections={len(detections)}, infer_ms={infer_ms:.1f}, tracks={len(tracks)}', flush=True)
            last_log_t = time.time()
        if MAX_FRAMES > 0 and frame_count >= MAX_FRAMES:
            print('[FrameDiff] max frame reached', flush=True)
            break
        for _ in range(FRAME_STRIDE - 1):
            if not cap.grab():
                break
finally:
    cap.release()
    cv2.destroyAllWindows()
    print('[FrameDiff] exit', flush=True)
