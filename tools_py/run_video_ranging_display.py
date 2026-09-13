
#!/usr/bin/env python3
import math
import os
import time
from pathlib import Path

import cv2
import numpy as np

from gimbal_vision_ranging import _CpuYoloDetector, _DistanceRuntime, ROI_SIZE

ROOT = Path('/home/linaro/gimbal')
VIDEO_CANDIDATES = [ROOT / '20~50m.mkv', ROOT / '20~250m.mkv', ROOT / '20~250?.mkv']
VIDEO = next((p for p in VIDEO_CANDIDATES if p.is_file()), None)
if VIDEO is None:
    mkvs = sorted(ROOT.glob('*.mkv'))
    VIDEO = mkvs[0] if mkvs else None
if VIDEO is None:
    raise SystemExit('No mkv video found in /home/linaro/gimbal')

CONF = float(os.getenv('GIMBAL_VISION_CONFIDENCE', '0.30'))
MAX_FRAMES = int(os.getenv('VIDEO_RANGING_MAX_FRAMES', '300'))
START_FRAME = int(os.getenv('VIDEO_RANGING_START_FRAME', '0'))
DISPLAY_SCALE = float(os.getenv('VIDEO_RANGING_DISPLAY_SCALE', '0.5'))
ASSOC_PX = float(os.getenv('VIDEO_RANGING_ASSOC_PX', '90'))

print(f'[VideoRanging] video={VIDEO}', flush=True)
print(f'[VideoRanging] max_frames={MAX_FRAMES}, start_frame={START_FRAME}, conf={CONF}', flush=True)

cap = cv2.VideoCapture(str(VIDEO))
if not cap.isOpened():
    raise SystemExit(f'Cannot open video: {VIDEO}')
if START_FRAME > 0:
    cap.set(cv2.CAP_PROP_POS_FRAMES, START_FRAME)

fps = cap.get(cv2.CAP_PROP_FPS)
if not math.isfinite(fps) or fps <= 0:
    fps = 25.0

detector = _CpuYoloDetector(ROOT / 'distance model' / 'best.pt', CONF)
tracks = {}
next_track_id = 1

cv2.namedWindow('gimbal_yolo_ranging_video', cv2.WINDOW_NORMAL)
cv2.resizeWindow('gimbal_yolo_ranging_video', 1280, 720)

frame_count = 0
last_log_t = time.time()
try:
    while True:
        ok, frame = cap.read()
        if not ok:
            print('[VideoRanging] video ended', flush=True)
            break
        frame_count += 1
        h, w = frame.shape[:2]
        x0 = max(0, min(w - ROI_SIZE, w // 2 - ROI_SIZE // 2))
        y0 = max(0, min(h - ROI_SIZE, h // 2 - ROI_SIZE // 2))
        crop = frame[y0:y0 + ROI_SIZE, x0:x0 + ROI_SIZE]

        t0 = time.time()
        rows = detector.detect(crop)
        infer_ms = (time.time() - t0) * 1000.0

        detections = []
        for row in rows:
            x1 = float(row[0] + x0); y1 = float(row[1] + y0)
            x2 = float(row[2] + x0); y2 = float(row[3] + y0)
            conf = float(row[4])
            if x2 <= x1 or y2 <= y1:
                continue
            detections.append({'box': [x1, y1, x2, y2], 'confidence': conf})
        detections.sort(key=lambda d: d['confidence'], reverse=True)

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

        # age out lost tracks
        for tid in list(tracks.keys()):
            if frame_count - tracks[tid]['last_seen'] > 30:
                del tracks[tid]

        cv2.rectangle(frame, (x0, y0), (x0 + ROI_SIZE, y0 + ROI_SIZE), (255, 180, 0), 2)
        cv2.putText(frame, f'video={VIDEO.name} frame={int(cap.get(cv2.CAP_PROP_POS_FRAMES))} det={len(detections)} infer={infer_ms:.0f}ms backend=opencv-cpu',
                    (20, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, 'Center 640x640 ROI shown in blue; press q/Esc to exit',
                    (20, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 220, 120), 2, cv2.LINE_AA)

        if DISPLAY_SCALE != 1.0:
            show = cv2.resize(frame, (int(w * DISPLAY_SCALE), int(h * DISPLAY_SCALE)))
        else:
            show = frame
        cv2.imshow('gimbal_yolo_ranging_video', show)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord('q'), ord('Q')):
            print('[VideoRanging] stopped by key', flush=True)
            break

        if time.time() - last_log_t > 5:
            print(f'[VideoRanging] frame={frame_count}, detections={len(detections)}, infer_ms={infer_ms:.1f}, tracks={len(tracks)}', flush=True)
            last_log_t = time.time()
        if MAX_FRAMES > 0 and frame_count >= MAX_FRAMES:
            print('[VideoRanging] max frame reached', flush=True)
            break
finally:
    cap.release()
    cv2.destroyAllWindows()
    print('[VideoRanging] exit', flush=True)
