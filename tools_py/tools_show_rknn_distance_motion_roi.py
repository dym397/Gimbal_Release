#!/usr/bin/env python3
from __future__ import annotations

import math
import os
import time
from pathlib import Path

import cv2
import numpy as np
from rknnlite.api import RKNNLite

import tools_show_rknn_distance_video as base

ROOT = Path('/home/linaro/gimbal')
DISTANCE_ROOT = ROOT / 'distance model'
RKNN_PATH = DISTANCE_ROOT / 'bestall_640.rknn'
VIDEO_PATH = ROOT / '20~250m.mkv'
FRAME_W = 2560
FRAME_H = 1440
ROI_SIZE = 640
CONF_THRES = float(os.environ.get('CONF_THRES', '0.15'))
IOU_THRES = float(os.environ.get('IOU_THRES', '0.45'))
START_FRAME = int(os.environ.get('START_FRAME', '1600'))
MAX_FRAMES = int(os.environ.get('MAX_FRAMES', '0'))
SHOW = os.environ.get('SHOW', '1') != '0'
WINDOW = 'RKNN bestall_640 motion ROI + distance'

# ????????????????????????
FD_THRESH = int(os.environ.get('FD_THRESH', '8'))
FD_MIN_AREA = float(os.environ.get('FD_MIN_AREA', '4'))
FD_MAX_AREA = float(os.environ.get('FD_MAX_AREA', '20000'))
ROI_LOST_KEEP = int(os.environ.get('ROI_LOST_KEEP', '45'))
ROI_SMOOTH = float(os.environ.get('ROI_SMOOTH', '0.65'))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class MotionRoiTracker:
    def __init__(self, frame_w: int, frame_h: int, roi_size: int):
        self.frame_w = frame_w
        self.frame_h = frame_h
        self.roi_size = roi_size
        self.prev_gray = None
        self.center = None
        self.lost_count = 10**9
        self.last_motion_box = None

    def _preprocess_gray(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        return gray

    def _detect_motion(self, frame):
        gray = self._preprocess_gray(frame)
        if self.prev_gray is None:
            self.prev_gray = gray
            return None, None
        diff = cv2.absdiff(gray, self.prev_gray)
        # ??????????????????????
        self.prev_gray = cv2.addWeighted(self.prev_gray, 0.80, gray, 0.20, 0)
        _, mask = cv2.threshold(diff, FD_THRESH, 255, cv2.THRESH_BINARY)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for c in contours:
            area = float(cv2.contourArea(c))
            if area < FD_MIN_AREA or area > FD_MAX_AREA:
                continue
            x, y, w, h = cv2.boundingRect(c)
            if w <= 1 or h <= 1:
                continue
            aspect = w / max(h, 1)
            if aspect > 12.0 or aspect < 0.08:
                continue
            cx = x + w / 2.0
            cy = y + h / 2.0
            # ???? ROI ???????????????????????????
            if self.center is not None and self.lost_count < ROI_LOST_KEEP:
                dist = math.hypot(cx - self.center[0], cy - self.center[1])
                score = area - 0.12 * dist
            else:
                # ??????????????????????????????????
                center_bias = -0.0008 * abs(cx - self.frame_w / 2.0) - 0.0004 * abs(cy - self.frame_h / 2.5)
                score = area + center_bias
            candidates.append((score, area, (x, y, w, h), (cx, cy)))
        if not candidates:
            return None, mask
        candidates.sort(key=lambda item: item[0], reverse=True)
        _, _, box, center = candidates[0]
        return (box, center), mask

    def update(self, frame):
        motion, mask = self._detect_motion(frame)
        source = 'fallback_center'
        if motion is not None:
            box, new_center = motion
            self.last_motion_box = box
            if self.center is None or self.lost_count >= ROI_LOST_KEEP:
                self.center = new_center
            else:
                self.center = (
                    ROI_SMOOTH * self.center[0] + (1.0 - ROI_SMOOTH) * new_center[0],
                    ROI_SMOOTH * self.center[1] + (1.0 - ROI_SMOOTH) * new_center[1],
                )
            self.lost_count = 0
            source = 'frame_diff'
        elif self.center is not None and self.lost_count < ROI_LOST_KEEP:
            self.lost_count += 1
            source = 'last_roi'
        else:
            self.center = (self.frame_w / 2.0, self.frame_h / 2.0)
            self.lost_count += 1
        cx, cy = self.center
        x0 = int(round(clamp(cx - self.roi_size / 2.0, 0, self.frame_w - self.roi_size)))
        y0 = int(round(clamp(cy - self.roi_size / 2.0, 0, self.frame_h - self.roi_size)))
        return (x0, y0, self.roi_size, self.roi_size), source, self.last_motion_box, mask


def xywh2xyxy(x):
    y = np.empty_like(x)
    y[:, 0] = x[:, 0] - x[:, 2] / 2
    y[:, 1] = x[:, 1] - x[:, 3] / 2
    y[:, 2] = x[:, 0] + x[:, 2] / 2
    y[:, 3] = x[:, 1] + x[:, 3] / 2
    return y


def nms(boxes, scores, iou_thres):
    order = scores.argsort()[::-1]
    keep = []
    while order.size > 0:
        i = int(order[0])
        keep.append(i)
        if order.size == 1:
            break
        b = boxes[i]
        rest = boxes[order[1:]]
        x1 = np.maximum(b[0], rest[:, 0])
        y1 = np.maximum(b[1], rest[:, 1])
        x2 = np.minimum(b[2], rest[:, 2])
        y2 = np.minimum(b[3], rest[:, 3])
        inter = np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)
        a1 = max(0, b[2] - b[0]) * max(0, b[3] - b[1])
        a2 = np.maximum(0, rest[:, 2] - rest[:, 0]) * np.maximum(0, rest[:, 3] - rest[:, 1])
        iou = inter / (a1 + a2 - inter + 1e-9)
        order = order[1:][iou <= iou_thres]
    return keep


def postprocess_roi(pred, roi_x0, roi_y0, frame_shape):
    pred = np.squeeze(np.asarray(pred))
    if pred.ndim != 2:
        pred = pred.reshape(-1, pred.shape[-1])
    if pred.shape[1] < 6:
        return []
    scores = pred[:, 4] * pred[:, 5]
    mask = scores >= CONF_THRES
    if not np.any(mask):
        return []
    p = pred[mask]
    s = scores[mask]
    boxes = xywh2xyxy(p[:, :4].astype(np.float32))
    boxes[:, [0, 2]] += roi_x0
    boxes[:, [1, 3]] += roi_y0
    h, w = frame_shape[:2]
    boxes[:, [0, 2]] = boxes[:, [0, 2]].clip(0, w - 1)
    boxes[:, [1, 3]] = boxes[:, [1, 3]].clip(0, h - 1)
    keep = nms(boxes, s, IOU_THRES)
    return [(boxes[i].astype(np.float32), float(s[i])) for i in keep[:20]]


def draw_text(img, text, xy, color=(0, 255, 0), scale=0.75, thickness=2):
    x, y = xy
    cv2.putText(img, text, (x + 1, y + 1), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thickness + 2, cv2.LINE_AA)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)


def main():
    print('[Model]', RKNN_PATH, flush=True)
    print('[Video]', VIDEO_PATH, 'start_frame=', START_FRAME, 'show=', SHOW, flush=True)
    print('[ROI] frame-diff crop 640x640, thresh=', FD_THRESH, 'min_area=', FD_MIN_AREA, flush=True)
    rknn = RKNNLite(verbose=False)
    ret = rknn.load_rknn(str(RKNN_PATH))
    if ret != 0:
        raise SystemExit(f'load_rknn failed {ret}')
    core = getattr(RKNNLite, 'NPU_CORE_0_1_2', RKNNLite.NPU_CORE_AUTO)
    ret = rknn.init_runtime(core_mask=core)
    if ret != 0:
        raise SystemExit(f'init_runtime failed {ret}')
    dist = base.DistanceRuntime()
    roi_tracker = MotionRoiTracker(FRAME_W, FRAME_H, ROI_SIZE)
    cap = cv2.VideoCapture(str(VIDEO_PATH))
    if not cap.isOpened():
        raise SystemExit(f'cannot open {VIDEO_PATH}')
    video_fps = cap.get(cv2.CAP_PROP_FPS) or 25.0
    if START_FRAME > 0:
        cap.set(cv2.CAP_PROP_POS_FRAMES, START_FRAME)
    if SHOW:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, 1280, 720)
    count = 0
    det_count = 0
    valid_count = 0
    t0 = time.time()
    infer_ema = None
    last_print = 0.0
    while True:
        ok, frame = cap.read()
        if not ok:
            print('[Video] EOF, rewind', flush=True)
            cap.set(cv2.CAP_PROP_POS_FRAMES, START_FRAME)
            dist.reset()
            roi_tracker = MotionRoiTracker(FRAME_W, FRAME_H, ROI_SIZE)
            continue
        frame_id = int(cap.get(cv2.CAP_PROP_POS_FRAMES))
        roi, roi_source, motion_box, _mask = roi_tracker.update(frame)
        rx, ry, rw, rh = roi
        crop = frame[ry:ry + rh, rx:rx + rw]
        if crop.shape[0] != ROI_SIZE or crop.shape[1] != ROI_SIZE:
            crop = cv2.resize(crop, (ROI_SIZE, ROI_SIZE), interpolation=cv2.INTER_LINEAR)
        inp = np.expand_dims(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB), 0).astype(np.uint8)
        ti = time.time()
        outputs = rknn.inference(inputs=[inp])
        infer_ms = (time.time() - ti) * 1000.0
        infer_ema = infer_ms if infer_ema is None else infer_ema * 0.9 + infer_ms * 0.1
        dets = postprocess_roi(outputs[0], rx, ry, frame.shape)
        detection = None
        if dets:
            box, conf = max(dets, key=lambda item: item[1])
            detection = {'box': box, 'confidence': conf, 'source': 'rknn_bestall_640_motion_roi'}
            det_count += 1
        timestamp = frame_id / video_fps
        d = dist.update(detection, timestamp)
        if d.get('valid'):
            valid_count += 1
        count += 1
        fps = count / max(time.time() - t0, 1e-6)

        cv2.rectangle(frame, (rx, ry), (rx + rw, ry + rh), (255, 255, 0), 2)
        draw_text(frame, f'ROI:{roi_source}', (rx + 6, max(24, ry + 24)), (255, 255, 0), 0.65, 2)
        if motion_box is not None:
            mx, my, mw, mh = motion_box
            cv2.rectangle(frame, (mx, my), (mx + mw, my + mh), (255, 0, 255), 1)
        for box, conf in dets:
            x1, y1, x2, y2 = box.tolist()
            is_main = detection is not None and abs(conf - detection['confidence']) < 1e-6
            color = (0, 255, 0) if is_main else (0, 180, 255)
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            draw_text(frame, f'{conf:.2f}', (int(x1), max(25, int(y1) - 8)), color, 0.7, 2)
        dist_val = d.get('distance', math.nan)
        if math.isfinite(float(dist_val)):
            dist_text = f"D={dist_val:.1f}m {d.get('source', '')} warm={d.get('warmup_count', 0)}/25"
        else:
            dist_text = f"D=-- {d.get('reason', '')} warm={d.get('warmup_count', 0)}/25"
        physics = d.get('physics_distance', math.nan)
        mlp = d.get('mlp_distance', math.nan)
        draw_text(frame, f'RKNN motion-ROI frame={frame_id} det={len(dets)} infer={infer_ema:.1f}ms fps={fps:.1f}', (20, 40), (0, 0, 255), 0.8, 2)
        draw_text(frame, dist_text, (20, 78), (0, 255, 0) if d.get('valid') else (0, 255, 255), 0.9, 2)
        if math.isfinite(float(physics)):
            draw_text(frame, f'physics={physics:.1f}m mlp={mlp:.1f}m conf={(detection or {}).get("confidence", 0):.2f} det_rate={det_count}/{count}', (20, 114), (255, 255, 0), 0.75, 2)
        now = time.time()
        if now - last_print > 1.0:
            print(f"[Frame {frame_id}] roi={roi_source}@({rx},{ry}) det={len(dets)} {dist_text} physics={physics if math.isfinite(float(physics)) else None} infer={infer_ema:.1f}ms fps={fps:.1f} det_rate={det_count}/{count} valid={valid_count}", flush=True)
            last_print = now
        if SHOW:
            view = cv2.resize(frame, (1280, 720))
            cv2.imshow(WINDOW, view)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):
                break
        if MAX_FRAMES and count >= MAX_FRAMES:
            print(f'[Summary] frames={count} det_frames={det_count} valid_distance_frames={valid_count} det_rate={det_count / max(count,1):.3f}', flush=True)
            break
    cap.release()
    if SHOW:
        cv2.destroyAllWindows()
    rknn.release()


if __name__ == '__main__':
    main()

