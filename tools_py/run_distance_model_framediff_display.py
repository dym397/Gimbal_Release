
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
    raise SystemExit('No mkv video found')

CONF = float(os.getenv('GIMBAL_VISION_CONFIDENCE', '0.25'))
MAX_FRAMES = int(os.getenv('VIDEO_RANGING_MAX_FRAMES', '600'))
FRAME_STRIDE = max(1, int(os.getenv('VIDEO_FRAME_STRIDE', '2')))
DISPLAY_SCALE = float(os.getenv('VIDEO_RANGING_DISPLAY_SCALE', '0.5'))
DIFF_THRESHOLD = int(os.getenv('VIDEO_DIFF_THRESHOLD', '16'))
MIN_MOTION_AREA = float(os.getenv('VIDEO_MIN_MOTION_AREA', '8'))
MAX_ROIS = int(os.getenv('VIDEO_MAX_ROIS', '3'))
DETAIL_EVERY = max(1, int(os.getenv('VIDEO_DETAIL_LOG_EVERY', '1')))

print(f'[DMFrameDiff] video={VIDEO}', flush=True)
print(f'[DMFrameDiff] conf={CONF}, frame_stride={FRAME_STRIDE}, max_rois={MAX_ROIS}', flush=True)

cap = cv2.VideoCapture(str(VIDEO))
if not cap.isOpened():
    raise SystemExit(f'cannot open video {VIDEO}')
fps = float(cap.get(cv2.CAP_PROP_FPS))
if not math.isfinite(fps) or fps <= 0:
    fps = 25.0
print(f'[DMFrameDiff] fps={fps:.3f}', flush=True)

detector = _CpuYoloDetector(ROOT / 'distance model' / 'best.pt', CONF)
runtime = _DistanceRuntime()
print(f'[DMFrameDiff] ranging_backend={runtime.gru_backend}', flush=True)

prev_gray = None
last_best_box = None
last_best_center = None

cv2.namedWindow('distance_model_framediff_yolo_ranging', cv2.WINDOW_NORMAL)
cv2.resizeWindow('distance_model_framediff_yolo_ranging', 1280, 720)

def iou(a, b):
    x1=max(a[0],b[0]); y1=max(a[1],b[1]); x2=min(a[2],b[2]); y2=min(a[3],b[3])
    inter=max(0,x2-x1)*max(0,y2-y1)
    area_a=max(0,a[2]-a[0])*max(0,a[3]-a[1])
    area_b=max(0,b[2]-b[0])*max(0,b[3]-b[1])
    return inter/max(1e-6, area_a+area_b-inter)

def clamp_roi(cx, cy, w, h):
    x0=int(round(cx-ROI_SIZE/2)); y0=int(round(cy-ROI_SIZE/2))
    x0=max(0, min(max(0, w-ROI_SIZE), x0)); y0=max(0, min(max(0, h-ROI_SIZE), y0))
    return [x0, y0, x0+ROI_SIZE, y0+ROI_SIZE]

def motion_rois(frame):
    global prev_gray, last_best_center
    h,w=frame.shape[:2]
    gray=cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray=cv2.GaussianBlur(gray, (5,5), 0)
    if prev_gray is None:
        prev_gray=gray
        return [], [], gray
    diff=cv2.absdiff(gray, prev_gray)
    _,mask=cv2.threshold(diff, DIFF_THRESHOLD, 255, cv2.THRESH_BINARY)
    mask=cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3),np.uint8), iterations=1)
    mask=cv2.dilate(mask, np.ones((5,5),np.uint8), iterations=2)
    contours,_=cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boxes=[]
    for c in contours:
        area=float(cv2.contourArea(c))
        if area < MIN_MOTION_AREA:
            continue
        x,y,bw,bh=cv2.boundingRect(c)
        if bw*bh > 0.20*w*h:
            continue
        boxes.append({'box':[x,y,x+bw,y+bh], 'area':area})
    boxes.sort(key=lambda d:d['area'], reverse=True)
    rois=[]
    # Prefer ROI around previous accepted target, if available, to keep GRU continuous.
    if last_best_center is not None:
        rois.append(clamp_roi(last_best_center[0], last_best_center[1], w, h))
    for item in boxes:
        x1,y1,x2,y2=item['box']
        roi=clamp_roi((x1+x2)/2, (y1+y2)/2, w, h)
        if all(iou(roi, old) < 0.65 for old in rois):
            rois.append(roi)
        if len(rois) >= MAX_ROIS:
            break
    prev_gray=gray
    return boxes, rois, gray

def global_nms(dets, thr=0.45):
    dets=sorted(dets, key=lambda d:d['confidence'], reverse=True)
    kept=[]
    for d in dets:
        if all(iou(d['box'], k['box']) < thr for k in kept):
            kept.append(d)
    return kept

def choose_target(dets):
    global last_best_center
    if not dets:
        return None
    valid=[]
    for d in dets:
        x1,y1,x2,y2=d['box']; bw=x2-x1; bh=y2-y1
        ar=bw/max(1e-6,bh)
        # Same basic quality idea as distance_model; avoid square/tall clutter and huge edge blocks.
        if bw < 3 or bh < 3 or ar < 1.15 or ar >= 8.0:
            continue
        if x1 <= 2 and y1 > 0.75*1440 and bw > 300:
            # This is the observed stable lower-left false positive; only for this visual test.
            continue
        valid.append(d)
    if not valid:
        return None
    if last_best_center is not None:
        def score(d):
            x1,y1,x2,y2=d['box']; c=np.array([(x1+x2)/2,(y1+y2)/2])
            dist=np.linalg.norm(c-last_best_center)
            return d['confidence'] - 0.0015*dist
        return max(valid, key=score)
    return max(valid, key=lambda d:d['confidence'])

proc=0
try:
    while True:
        ok,frame=cap.read()
        if not ok:
            print('[DMFrameDiff] ended', flush=True); break
        proc+=1
        video_pos=int(cap.get(cv2.CAP_PROP_POS_FRAMES))
        ts=video_pos/fps
        h,w=frame.shape[:2]
        mboxes,rois,_=motion_rois(frame)
        crops=[frame[y1:y2,x1:x2] for x1,y1,x2,y2 in rois]
        t0=time.time()
        rows_by=detector.detect_batch(crops) if crops else []
        infer_ms=(time.time()-t0)*1000
        dets=[]
        for rows,roi in zip(rows_by, rois):
            x0,y0,_,_=roi
            for r in rows:
                box=[float(r[0]+x0),float(r[1]+y0),float(r[2]+x0),float(r[3]+y0)]
                dets.append({'box':box,'confidence':float(r[4])})
        dets=global_nms(dets)
        best=choose_target(dets)
        out=None
        reason='TARGET_MISSING'
        if best is not None:
            out=runtime.update(best, ts)
            reason=out.get('reason','')
            x1,y1,x2,y2=best['box']
            last_best_center=np.array([(x1+x2)/2,(y1+y2)/2], dtype=np.float32)
            last_best_box=best['box']
        else:
            runtime.mark_missing(ts)

        show=frame.copy()
        for mb in mboxes[:30]:
            x1,y1,x2,y2=[int(v) for v in mb['box']]
            cv2.rectangle(show,(x1,y1),(x2,y2),(255,255,0),1)
        for roi in rois:
            x1,y1,x2,y2=roi
            cv2.rectangle(show,(x1,y1),(x2,y2),(255,100,0),2)
        for d in dets:
            x1,y1,x2,y2=d['box']
            cv2.rectangle(show,(int(x1),int(y1)),(int(x2),int(y2)),(120,120,120),1)
        if best is not None:
            x1,y1,x2,y2=best['box']
            color=(0,255,0) if out and out.get('valid') else (0,255,255)
            cv2.rectangle(show,(int(x1),int(y1)),(int(x2),int(y2)),color,3)
            label=f'conf={best["confidence"]:.2f}'
            if out and out.get('valid'):
                label += f' {out["distance"]:.2f}m {out.get("source")}'
            elif out:
                label += f' warmup={out.get("warmup_count",0)}/25 {out.get("reason","")}'
            cv2.putText(show,label,(int(x1),max(24,int(y1)-8)),cv2.FONT_HERSHEY_SIMPLEX,0.75,color,2,cv2.LINE_AA)
        overlay=show.copy()
        cv2.rectangle(overlay,(15,15),(720,165),(20,20,20),-1)
        cv2.addWeighted(overlay,0.62,show,0.38,0,show)
        state='LOST' if best is None else ('TRACKING' if out and out.get('valid') else 'WARMING_UP')
        final='N/A' if not (out and out.get('valid')) else f'{out["distance"]:.2f}m'
        warm='N/A' if out is None else f'{out.get("warmup_count",0)}/25'
        cv2.putText(show,'FrameDiff ROI + distance_model single-target ranging',(25,42),cv2.FONT_HERSHEY_SIMPLEX,0.72,(255,255,255),2,cv2.LINE_AA)
        cv2.putText(show,f'frame={video_pos} motion={len(mboxes)} rois={len(rois)} det={len(dets)} infer={infer_ms:.0f}ms state={state}',(25,76),cv2.FONT_HERSHEY_SIMPLEX,0.62,(220,220,220),1,cv2.LINE_AA)
        cv2.putText(show,f'Final={final} warmup={warm} reason={reason}',(25,110),cv2.FONT_HERSHEY_SIMPLEX,0.62,(0,255,255),1,cv2.LINE_AA)
        cv2.putText(show,'cyan=motion blue=YOLO ROI gray=all YOLO green/yellow=selected target',(25,144),cv2.FONT_HERSHEY_SIMPLEX,0.58,(255,220,120),1,cv2.LINE_AA)
        if proc % DETAIL_EVERY == 0:
            if best is None:
                print(f'[DMFrameDiffDetail] proc={proc},video_pos={video_pos},motion={len(mboxes)},rois={len(rois)},det={len(dets)},infer_ms={infer_ms:.1f},no_target', flush=True)
            else:
                bx=best['box']; bw=bx[2]-bx[0]; bh=bx[3]-bx[1]
                if out and out.get('valid'):
                    dist=f'distance={out["distance"]:.2f}m,source={out.get("source")}'
                elif out:
                    dist=f'warmup={out.get("warmup_count",0)}/25,reason={out.get("reason","")}'
                else:
                    dist='no_out'
                print(f'[DMFrameDiffDetail] proc={proc},video_pos={video_pos},motion={len(mboxes)},rois={len(rois)},det={len(dets)},conf={best["confidence"]:.2f},box=({bx[0]:.0f},{bx[1]:.0f},{bw:.0f}x{bh:.0f}),infer_ms={infer_ms:.1f},{dist}', flush=True)
        disp=cv2.resize(show,(int(w*DISPLAY_SCALE),int(h*DISPLAY_SCALE))) if DISPLAY_SCALE!=1 else show
        cv2.imshow('distance_model_framediff_yolo_ranging', disp)
        key=cv2.waitKey(1)&0xff
        if key in (27, ord('q'), ord('Q')):
            break
        for _ in range(FRAME_STRIDE-1):
            if not cap.grab(): break
        if MAX_FRAMES>0 and proc>=MAX_FRAMES:
            break
finally:
    cap.release(); cv2.destroyAllWindows(); print('[DMFrameDiff] exit', flush=True)
