from pathlib import Path
from rknn.api import RKNN
root = Path('/home/linaro/gimbal')
onnx_path = root / 'distance model' / 'bestall_2k.onnx'
rknn_path = root / 'distance model' / 'bestall_2k.rknn'
print('[RKNN] onnx:', onnx_path)
print('[RKNN] out :', rknn_path)
rknn = RKNN(verbose=True)
ret = rknn.config(
    target_platform='rk3588',
    mean_values=[[0, 0, 0]],
    std_values=[[255, 255, 255]],
    quantized_dtype='asymmetric_quantized-8',
    optimization_level=3,
)
print('[RKNN] config ret:', ret)
if ret != 0: raise SystemExit(ret)
ret = rknn.load_onnx(model=str(onnx_path))
print('[RKNN] load_onnx ret:', ret)
if ret != 0: raise SystemExit(ret)
ret = rknn.build(do_quantization=False)
print('[RKNN] build ret:', ret)
if ret != 0: raise SystemExit(ret)
ret = rknn.export_rknn(str(rknn_path))
print('[RKNN] export ret:', ret)
if ret != 0: raise SystemExit(ret)
rknn.release()
print('[RKNN] done:', rknn_path)
