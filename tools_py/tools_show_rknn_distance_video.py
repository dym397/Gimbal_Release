#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import os
import sys
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import yaml
from rknnlite.api import RKNNLite

ROOT = Path('/home/linaro/gimbal')
DISTANCE_ROOT = ROOT / 'distance model'
DISTANCE_SRC = DISTANCE_ROOT / 'src'
RKNN_PATH = DISTANCE_ROOT / 'bestall_640.rknn'
VIDEO_PATH = ROOT / '20~250m.mkv'
FRAME_W = 2560
FRAME_H = 1440
IMG_SIZE = 640
TARGET_WIDTH_M = 0.30
TARGET_HEIGHT_M = 0.15
FX = 11494.25
FY = 11494.25
CONF_THRES = float(os.environ.get('CONF_THRES', '0.15'))
IOU_THRES = float(os.environ.get('IOU_THRES', '0.45'))
START_FRAME = int(os.environ.get('START_FRAME', '1600'))
MAX_FRAMES = int(os.environ.get('MAX_FRAMES', '0'))
SHOW = os.environ.get('SHOW', '1') != '0'
WINDOW = 'RKNN bestall_640 YOLO + distance'

if str(DISTANCE_SRC) not in sys.path:
    sys.path.insert(0, str(DISTANCE_SRC))
from uav_distance_pipeline.runtime_motion_gate import RuntimeMotionGate


def load_mlp(path: Path) -> dict:
    data = np.load(path, allow_pickle=True)
    return {
        'x_mean': data['x_mean'],
        'x_std': np.where(data['x_std'] < 1e-2, 1.0, data['x_std']),
        'y_mean': data['y_mean'],
        'y_std': data['y_std'],
        'w1': data['w1'], 'b1': data['b1'],
        'w2': data['w2'], 'b2': data['b2'],
        'w3': data['w3'], 'b3': data['b3'],
        'feature_columns': [str(v) for v in data['feature_columns']],
    }


def predict_mlp(model: dict, features: dict) -> float:
    raw = np.array([[features[name] for name in model['feature_columns']]], dtype=np.float32)
    normalized = (raw - model['x_mean']) / model['x_std']
    h1 = np.maximum(normalized @ model['w1'] + model['b1'], 0.0)
    h2 = np.maximum(h1 @ model['w2'] + model['b2'], 0.0)
    scaled = h2 @ model['w3'] + model['b3']
    return float((scaled * model['y_std'] + model['y_mean']).reshape(-1)[0])


def sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-x))


class NumpyGruDistanceModel:
    def __init__(self, path: Path):
        w = np.load(path)
        self.weight_ih = w['gru.weight_ih_l0'].astype(np.float32)
        self.weight_hh = w['gru.weight_hh_l0'].astype(np.float32)
        self.bias_ih = w['gru.bias_ih_l0'].astype(np.float32)
        self.bias_hh = w['gru.bias_hh_l0'].astype(np.float32)
        self.fc_weight = w['fc.weight'].astype(np.float32)
        self.fc_bias = w['fc.bias'].astype(np.float32)
        self.hidden_size = int(self.weight_hh.shape[1])

    def predict(self, sequence: np.ndarray) -> float:
        h = np.zeros((self.hidden_size,), dtype=np.float32)
        for x in np.asarray(sequence, dtype=np.float32):
            gi = self.weight_ih @ x + self.bias_ih
            gh = self.weight_hh @ h + self.bias_hh
            i_r, i_z, i_n = np.split(gi, 3)
            h_r, h_z, h_n = np.split(gh, 3)
            r = sigmoid(i_r + h_r)
            z = sigmoid(i_z + h_z)
            n = np.tanh(i_n + r * h_n)
            h = (1.0 - z) * n + z * h
        return float((self.fc_weight @ h + self.fc_bias).reshape(-1)[0])


def robust_physics_distance(width_px: float, height_px: float, disagreement_ratio: float) -> dict:
    result = {'valid': False, 'reason': '', 'width_m': math.nan, 'height_m': math.nan, 'area_m': math.nan, 'fused_m': math.nan}
    if width_px < 3.0 or height_px < 3.0:
        result['reason'] = 'BBOX_TOO_SMALL'; return result
    aspect = width_px / max(height_px, 1e-6)
    if aspect < 1.15 or aspect >= 8.0:
        result['reason'] = 'ASPECT_RATIO_OUT_OF_RANGE'; return result
    d_width = FX * TARGET_WIDTH_M / width_px
    d_height = FY * TARGET_HEIGHT_M / height_px
    d_area = math.sqrt(FX * FY * TARGET_WIDTH_M * TARGET_HEIGHT_M / (width_px * height_px))
    candidates = [d_width, d_height, d_area]
    median = float(np.median(candidates))
    weights = [0.0 if abs(v - median) / max(1.0, median) > disagreement_ratio else 1.0 for v in candidates]
    if sum(weights) < 1.5:
        result.update(reason='PHYSICS_DISAGREEMENT', width_m=d_width, height_m=d_height, area_m=d_area, fused_m=sum(candidates)/3.0)
        return result
    result.update(valid=True, width_m=d_width, height_m=d_height, area_m=d_area, fused_m=sum(w*v for w,v in zip(weights,candidates))/sum(weights))
    return result


class DistanceRuntime:
    def __init__(self):
        self.config = yaml.safe_load((DISTANCE_ROOT / 'configs' / 'runtime_distance_stabilization.yaml').read_text(encoding='utf-8'))
        schema = json.loads((DISTANCE_ROOT / 'models' / 'feature_schema.json').read_text(encoding='utf-8'))
        self.feature_columns = schema['feature_columns']
        self.sequence_length = int(schema.get('sequence_length', 25))
        norm = json.loads((DISTANCE_ROOT / 'models' / 'normalization.json').read_text(encoding='utf-8'))
        self.means = np.array([norm['mean'][n] for n in self.feature_columns], dtype=np.float32)
        self.stds = np.where(np.array([norm['std'][n] for n in self.feature_columns], dtype=np.float32) < 2e-2, 1.0, np.array([norm['std'][n] for n in self.feature_columns], dtype=np.float32))
        self.mlp = load_mlp(DISTANCE_ROOT / 'models' / 'distance_residual_mlp_numpy.npz')
        self.gru = NumpyGruDistanceModel(DISTANCE_ROOT / 'models' / 'distance_gru_static_25f_numpy.npz')
        self.max_reject = int(self.config.get('tracking', {}).get('max_consecutive_reject_frames', 5))
        self.reset()

    def reset(self):
        self.motion_gate = RuntimeMotionGate(self.config)
        self.features = deque(maxlen=self.sequence_length)
        self.last_timestamp = None
        self.reject_count = 0

    def update(self, detection: dict | None, timestamp: float) -> dict:
        if detection is None:
            if self.last_timestamp is not None and timestamp - self.last_timestamp > 1.0:
                self.reset()
            return {'valid': False, 'reason': 'TARGET_MISSING', 'warmup_count': len(self.features), 'distance': math.nan, 'source': 'none'}
        box = detection['box']
        width_px = float(box[2] - box[0]); height_px = float(box[3] - box[1])
        physics = robust_physics_distance(width_px, height_px, float(self.config.get('measurement_quality', {}).get('max_physics_disagreement_ratio', 0.5)))
        if not physics['valid']:
            return {'valid': False, 'reason': physics['reason'], 'warmup_count': len(self.features), 'distance': math.nan, 'source': 'invalid', 'physics_distance': physics['fused_m']}
        gate, kalman_distance, radial_velocity = self.motion_gate.process(physics['fused_m'], timestamp, measurement_valid=True)
        if gate == 'REJECT':
            self.reject_count += 1
            if self.reject_count < self.max_reject:
                return {'valid': False, 'reason': 'MOTION_GATE_REJECT', 'warmup_count': len(self.features), 'distance': kalman_distance, 'source': 'gate', 'physics_distance': physics['fused_m']}
            self.motion_gate.reset(physics['fused_m'])
            kalman_distance = physics['fused_m']; radial_velocity = 0.0; self.reject_count = 0
        else:
            self.reject_count = 0
        center_u = float((box[0] + box[2]) / 2.0); center_v = float((box[1] + box[3]) / 2.0)
        frame_dt = 1.0 / 25.0 if self.last_timestamp is None else max(1e-3, timestamp - self.last_timestamp)
        features = {
            'bbox_width_px': width_px, 'bbox_height_px': height_px, 'bbox_area_px': width_px * height_px,
            'bbox_aspect_ratio': width_px / max(height_px, 1e-6), 'bbox_center_u': center_u, 'bbox_center_v': center_v,
            'bbox_center_u_norm': center_u / FRAME_W, 'bbox_center_v_norm': center_v / FRAME_H,
            'detector_confidence': float(detection['confidence']), 'is_missing': 0.0, 'is_interpolated': 0.0,
            'physics_width_range_m': physics['width_m'], 'physics_height_range_m': physics['height_m'],
            'physics_area_range_m': physics['area_m'], 'fused_range_m': physics['fused_m'], 'frame_delta_t_s': frame_dt,
        }
        residual = predict_mlp(self.mlp, features)
        mlp_distance = physics['fused_m'] + residual
        if not math.isfinite(mlp_distance) or mlp_distance <= 0.0:
            mlp_distance = physics['fused_m']
        features['mlp_predicted_residual_m'] = residual
        features['raw_predicted_range_m'] = mlp_distance
        self.features.append(np.array([features[n] for n in self.feature_columns], dtype=np.float32))
        self.last_timestamp = timestamp
        stable_distance = mlp_distance; source = 'mlp_warmup'
        if len(self.features) == self.sequence_length:
            seq = (np.array(self.features, dtype=np.float32) - self.means) / self.stds
            stable_distance = self.gru.predict(seq); source = 'gru'
        if not math.isfinite(stable_distance) or stable_distance <= 0.0 or abs(stable_distance - kalman_distance) > 30.0:
            stable_distance = float(kalman_distance); source = 'kalman_fallback' if len(self.features) == self.sequence_length else 'mlp_warmup'
        return {'valid': len(self.features) == self.sequence_length, 'reason': '', 'distance': stable_distance, 'source': source, 'warmup_count': len(self.features), 'physics_distance': physics['fused_m'], 'mlp_distance': mlp_distance, 'kalman_distance': kalman_distance, 'radial_velocity': radial_velocity}


def letterbox(im, new_shape=(640, 640), color=(114, 114, 114)):
    shape = im.shape[:2]
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    new_unpad = (int(round(shape[1] * r)), int(round(shape[0] * r)))
    dw = (new_shape[1] - new_unpad[0]) / 2
    dh = (new_shape[0] - new_unpad[1]) / 2
    if shape[::-1] != new_unpad:
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return im, r, (left, top)


def xywh2xyxy(x):
    y = np.empty_like(x)
    y[:, 0] = x[:, 0] - x[:, 2] / 2; y[:, 1] = x[:, 1] - x[:, 3] / 2
    y[:, 2] = x[:, 0] + x[:, 2] / 2; y[:, 3] = x[:, 1] + x[:, 3] / 2
    return y


def nms(boxes, scores, iou_thres):
    order = scores.argsort()[::-1]
    keep = []
    while order.size > 0:
        i = int(order[0]); keep.append(i)
        if order.size == 1: break
        b = boxes[i]; rest = boxes[order[1:]]
        x1 = np.maximum(b[0], rest[:, 0]); y1 = np.maximum(b[1], rest[:, 1])
        x2 = np.minimum(b[2], rest[:, 2]); y2 = np.minimum(b[3], rest[:, 3])
        inter = np.maximum(0, x2-x1) * np.maximum(0, y2-y1)
        a1 = max(0, b[2]-b[0]) * max(0, b[3]-b[1])
        a2 = np.maximum(0, rest[:,2]-rest[:,0]) * np.maximum(0, rest[:,3]-rest[:,1])
        iou = inter / (a1 + a2 - inter + 1e-9)
        order = order[1:][iou <= iou_thres]
    return keep


def postprocess(pred, ratio, pad, orig_shape):
    pred = np.squeeze(np.asarray(pred))
    if pred.ndim != 2: pred = pred.reshape(-1, pred.shape[-1])
    if pred.shape[1] < 6: return []
    scores = pred[:, 4] * pred[:, 5]
    mask = scores >= CONF_THRES
    if not np.any(mask): return []
    p = pred[mask]; s = scores[mask]
    boxes = xywh2xyxy(p[:, :4].astype(np.float32))
    boxes[:, [0,2]] -= pad[0]; boxes[:, [1,3]] -= pad[1]
    boxes[:, :4] /= ratio
    h, w = orig_shape[:2]
    boxes[:, [0,2]] = boxes[:, [0,2]].clip(0, w - 1)
    boxes[:, [1,3]] = boxes[:, [1,3]].clip(0, h - 1)
    return [(boxes[i].astype(np.float32), float(s[i])) for i in nms(boxes, s, IOU_THRES)[:20]]


def draw_text(img, text, xy, color=(0,255,0), scale=0.75, thickness=2):
    x, y = xy
    cv2.putText(img, text, (x+1,y+1), cv2.FONT_HERSHEY_SIMPLEX, scale, (0,0,0), thickness+2, cv2.LINE_AA)
    cv2.putText(img, text, (x,y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)


def main():
    print('[Model]', RKNN_PATH, flush=True)
    print('[Video]', VIDEO_PATH, 'start_frame=', START_FRAME, 'show=', SHOW, flush=True)
    rknn = RKNNLite(verbose=False)
    ret = rknn.load_rknn(str(RKNN_PATH))
    if ret != 0: raise SystemExit(f'load_rknn failed {ret}')
    core = getattr(RKNNLite, 'NPU_CORE_0_1_2', RKNNLite.NPU_CORE_AUTO)
    ret = rknn.init_runtime(core_mask=core)
    if ret != 0: raise SystemExit(f'init_runtime failed {ret}')
    dist = DistanceRuntime()
    cap = cv2.VideoCapture(str(VIDEO_PATH))
    if not cap.isOpened(): raise SystemExit(f'cannot open {VIDEO_PATH}')
    video_fps = cap.get(cv2.CAP_PROP_FPS) or 25.0
    if START_FRAME > 0: cap.set(cv2.CAP_PROP_POS_FRAMES, START_FRAME)
    if SHOW:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, 1280, 720)
    count = 0; t0 = time.time(); infer_ema = None; last_print = 0.0
    while True:
        ok, frame = cap.read()
        if not ok:
            cap.set(cv2.CAP_PROP_POS_FRAMES, START_FRAME)
            dist.reset()
            continue
        frame_id = int(cap.get(cv2.CAP_PROP_POS_FRAMES))
        img, ratio, pad = letterbox(frame, (IMG_SIZE, IMG_SIZE))
        inp = np.expand_dims(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), 0).astype(np.uint8)
        ti = time.time(); outputs = rknn.inference(inputs=[inp]); infer_ms = (time.time()-ti)*1000.0
        infer_ema = infer_ms if infer_ema is None else infer_ema * 0.9 + infer_ms * 0.1
        dets = postprocess(outputs[0], ratio, pad, frame.shape)
        detection = None
        if dets:
            box, conf = max(dets, key=lambda item: item[1])
            detection = {'box': box, 'confidence': conf, 'source': 'rknn_bestall_640'}
        timestamp = frame_id / video_fps
        d = dist.update(detection, timestamp)
        for box, conf in dets:
            x1,y1,x2,y2 = box.tolist()
            color = (0,255,0) if detection is not None and conf == detection['confidence'] else (0,180,255)
            cv2.rectangle(frame, (int(x1),int(y1)), (int(x2),int(y2)), color, 2)
            draw_text(frame, f'{conf:.2f}', (int(x1), max(25,int(y1)-8)), color, 0.7, 2)
        count += 1; fps = count / max(time.time()-t0, 1e-6)
        if math.isfinite(float(d.get('distance', math.nan))):
            dist_text = f"D={d['distance']:.1f}m {d.get('source','')} warm={d.get('warmup_count',0)}/25"
        else:
            dist_text = f"D=-- {d.get('reason','')} warm={d.get('warmup_count',0)}/25"
        physics = d.get('physics_distance', math.nan); mlp = d.get('mlp_distance', math.nan)
        draw_text(frame, f'RKNN bestall_640 frame={frame_id} det={len(dets)} infer={infer_ema:.1f}ms fps={fps:.1f}', (20,40), (0,0,255), 0.8, 2)
        draw_text(frame, dist_text, (20,78), (0,255,0) if d.get('valid') else (0,255,255), 0.9, 2)
        if math.isfinite(float(physics)):
            draw_text(frame, f'physics={physics:.1f}m mlp={mlp:.1f}m conf={(detection or {}).get("confidence",0):.2f}', (20,114), (255,255,0), 0.75, 2)
        now = time.time()
        if now - last_print > 1.0:
            print(f"[Frame {frame_id}] det={len(dets)} {dist_text} physics={physics if math.isfinite(float(physics)) else None} infer={infer_ema:.1f}ms fps={fps:.1f}", flush=True)
            last_print = now
        view = cv2.resize(frame, (1280,720))
        if SHOW:
            cv2.imshow(WINDOW, view)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):
                break
        if MAX_FRAMES and count >= MAX_FRAMES:
            break
    cap.release()
    if SHOW: cv2.destroyAllWindows()
    rknn.release()

if __name__ == '__main__':
    main()

