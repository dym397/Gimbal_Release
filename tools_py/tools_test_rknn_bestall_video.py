import os
import time
from pathlib import Path
import cv2
import numpy as np
from rknnlite.api import RKNNLite

ROOT = Path('/home/linaro/gimbal')
RKNN_PATH = ROOT / 'distance model' / 'bestall_640.rknn'
VIDEO_PATH = ROOT / '20~250m.mkv'
IMG_SIZE = 640
CONF_THRES = float(os.environ.get('CONF_THRES', '0.25'))
IOU_THRES = float(os.environ.get('IOU_THRES', '0.45'))
START_FRAME = int(os.environ.get('START_FRAME', '0'))
MAX_FRAMES = int(os.environ.get('MAX_FRAMES', '0'))
SHOW = os.environ.get('SHOW', '1') != '0'


def letterbox(im, new_shape=(640, 640), color=(114, 114, 114)):
    shape = im.shape[:2]  # h,w
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    new_unpad = (int(round(shape[1] * r)), int(round(shape[0] * r)))
    dw = new_shape[1] - new_unpad[0]
    dh = new_shape[0] - new_unpad[1]
    dw /= 2
    dh /= 2
    if shape[::-1] != new_unpad:
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return im, r, (left, top)


def xywh2xyxy(x):
    y = np.empty_like(x)
    y[:, 0] = x[:, 0] - x[:, 2] / 2
    y[:, 1] = x[:, 1] - x[:, 3] / 2
    y[:, 2] = x[:, 0] + x[:, 2] / 2
    y[:, 3] = x[:, 1] + x[:, 3] / 2
    return y


def box_iou_one(box, boxes):
    x1 = np.maximum(box[0], boxes[:, 0])
    y1 = np.maximum(box[1], boxes[:, 1])
    x2 = np.minimum(box[2], boxes[:, 2])
    y2 = np.minimum(box[3], boxes[:, 3])
    inter = np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)
    a1 = max(0, box[2]-box[0]) * max(0, box[3]-box[1])
    a2 = np.maximum(0, boxes[:, 2]-boxes[:, 0]) * np.maximum(0, boxes[:, 3]-boxes[:, 1])
    return inter / (a1 + a2 - inter + 1e-9)


def nms(boxes, scores, iou_thres=0.45):
    order = scores.argsort()[::-1]
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        if order.size == 1:
            break
        ious = box_iou_one(boxes[i], boxes[order[1:]])
        order = order[1:][ious <= iou_thres]
    return keep


def postprocess(pred, ratio, pad, orig_shape):
    pred = np.asarray(pred)
    pred = np.squeeze(pred)
    if pred.ndim != 2:
        pred = pred.reshape(-1, pred.shape[-1])
    if pred.shape[1] < 6:
        return []
    obj = pred[:, 4]
    cls = pred[:, 5]
    scores = obj * cls
    mask = scores >= CONF_THRES
    if not np.any(mask):
        return []
    p = pred[mask]
    s = scores[mask]
    boxes = xywh2xyxy(p[:, :4].astype(np.float32))
    # map 640 letterbox coords back to original frame coords
    boxes[:, [0, 2]] -= pad[0]
    boxes[:, [1, 3]] -= pad[1]
    boxes[:, :4] /= ratio
    h, w = orig_shape[:2]
    boxes[:, [0, 2]] = boxes[:, [0, 2]].clip(0, w - 1)
    boxes[:, [1, 3]] = boxes[:, [1, 3]].clip(0, h - 1)
    keep = nms(boxes, s, IOU_THRES)
    dets = []
    for i in keep[:50]:
        x1, y1, x2, y2 = boxes[i].tolist()
        dets.append((float(x1), float(y1), float(x2), float(y2), float(s[i])))
    return dets


def main():
    print('[RKNNLite] model:', RKNN_PATH)
    print('[Video]    path :', VIDEO_PATH)
    rknn = RKNNLite(verbose=False)
    ret = rknn.load_rknn(str(RKNN_PATH))
    if ret != 0:
        raise SystemExit(f'load_rknn failed: {ret}')
    core_mask = getattr(RKNNLite, 'NPU_CORE_0_1_2', RKNNLite.NPU_CORE_AUTO)
    ret = rknn.init_runtime(core_mask=core_mask)
    if ret != 0:
        raise SystemExit(f'init_runtime failed: {ret}')
    cap = cv2.VideoCapture(str(VIDEO_PATH))
    if not cap.isOpened():
        raise SystemExit(f'open video failed: {VIDEO_PATH}')
    if START_FRAME > 0:
        cap.set(cv2.CAP_PROP_POS_FRAMES, START_FRAME)
    frame_idx = int(cap.get(cv2.CAP_PROP_POS_FRAMES))
    t0 = time.time(); count = 0; infer_ms_ema = None
    print('[Start] frame=', frame_idx, 'conf=', CONF_THRES, 'show=', SHOW)
    printed_shape = False
    while True:
        ok, frame = cap.read()
        if not ok:
            print('[Video] EOF')
            break
        frame_idx += 1
        img, ratio, pad = letterbox(frame, (IMG_SIZE, IMG_SIZE))
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        inp = np.expand_dims(img_rgb, 0).astype(np.uint8)  # NHWC uint8, RKNN config does /255
        t1 = time.time()
        outputs = rknn.inference(inputs=[inp])
        infer_ms = (time.time() - t1) * 1000
        infer_ms_ema = infer_ms if infer_ms_ema is None else infer_ms_ema * 0.9 + infer_ms * 0.1
        if not printed_shape:
            print('[Output shapes]', [np.asarray(o).shape for o in outputs])
            printed_shape = True
        dets = postprocess(outputs[0], ratio, pad, frame.shape)
        for x1, y1, x2, y2, score in dets:
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(frame, f'{score:.2f}', (int(x1), max(20, int(y1)-5)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        count += 1
        fps = count / max(time.time() - t0, 1e-6)
        cv2.putText(frame, f'RKNN bestall frame={frame_idx} det={len(dets)} infer={infer_ms_ema:.1f}ms fps={fps:.1f}',
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        if frame_idx % 20 == 0 or dets:
            best = max([d[4] for d in dets], default=0.0)
            print(f'[Frame {frame_idx}] det={len(dets)} best={best:.3f} infer_ms={infer_ms:.1f} fps={fps:.2f}', flush=True)
        if SHOW:
            view = cv2.resize(frame, (1280, 720)) if frame.shape[1] > 1280 else frame
            cv2.imshow('RKNN bestall 640 NPU', view)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):
                print('[User] quit')
                break
        if MAX_FRAMES and count >= MAX_FRAMES:
            print('[Done] max frames')
            break
    cap.release()
    if SHOW:
        cv2.destroyAllWindows()
    rknn.release()

if __name__ == '__main__':
    main()
