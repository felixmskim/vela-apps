"""YOLOv8 ONNX inference without ultralytics.

The guest has no riscv64 PyTorch, so the ultralytics package cannot run there
(defect D-13). This module carries the three things ultralytics would have
provided -- letterbox preprocessing, output decoding with NMS, and the COCO
class names -- against a plain onnxruntime session. It works with any
onnxruntime-compatible build (upstream wheel, spacemit-ort); the model is
exported on the host by scripts/export_model.sh.

NMS is plain numpy rather than cv2.dnn.NMSBoxes so that nothing here depends
on the OpenCV version: the guest's python3-opencv is 4.6, whose dnn module
cannot even load YOLOv8 graphs, and this module must not care. cv2 is used
by the callers for image I/O only.
"""

import time

import numpy as np

COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
    "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon",
    "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
    "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
    "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
    "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush",
]


def letterbox(img, new_size, pad_value=114):
    """Resize keeping aspect ratio, pad to (new_size, new_size).

    Returns the padded image, the scale ratio, and the (left, top) padding --
    everything needed to map detections back to the original pixel grid.
    """
    import cv2

    h, w = img.shape[:2]
    r = min(new_size / h, new_size / w)
    nh, nw = round(h * r), round(w * r)
    if (nh, nw) != (h, w):
        img = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    top = (new_size - nh) // 2
    left = (new_size - nw) // 2
    out = np.full((new_size, new_size, 3), pad_value, dtype=img.dtype)
    out[top:top + nh, left:left + nw] = img
    return out, r, (left, top)


def nms(boxes, scores, iou_thr):
    """Pure-numpy NMS. boxes are (N, 4) xyxy; returns kept indices."""
    x1, y1, x2, y2 = boxes.T
    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort()[::-1]
    keep = []
    while order.size:
        i = order[0]
        keep.append(i)
        if order.size == 1:
            break
        rest = order[1:]
        xx1 = np.maximum(x1[i], x1[rest])
        yy1 = np.maximum(y1[i], y1[rest])
        xx2 = np.minimum(x2[i], x2[rest])
        yy2 = np.minimum(y2[i], y2[rest])
        inter = np.clip(xx2 - xx1, 0, None) * np.clip(yy2 - yy1, 0, None)
        iou = inter / (areas[i] + areas[rest] - inter + 1e-9)
        order = rest[iou <= iou_thr]
    return keep


class Yolov8Onnx:
    """One loaded model, callable on BGR images.

    intra_op threads default to 2 of the guest's 4 vCPUs: the point of this
    workload in TC.10.007 is to run BESIDE Nav2 and slam_toolbox, and a
    4-thread GEMM on 4 TCG vCPUs starves them (see gz-split RUNLOG Step 8 for
    what clock starvation does to bt_navigator).
    """

    def __init__(self, model_path, imgsz=320, conf=0.25, iou=0.45, threads=2):
        import onnxruntime as ort

        self.imgsz = imgsz
        self.conf = conf
        self.iou = iou
        so = ort.SessionOptions()
        if threads:
            so.intra_op_num_threads = threads
        self.session = ort.InferenceSession(
            model_path, sess_options=so, providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.runtime = f"onnxruntime {ort.__version__}"

    def __call__(self, bgr):
        """Run one image. Returns (detections, seconds).

        Each detection: dict with xyxy (original-image pixels), conf,
        cls_id, cls_name.
        """
        padded, r, (px, py) = letterbox(bgr, self.imgsz)
        blob = padded[:, :, ::-1].transpose(2, 0, 1)[None]
        blob = np.ascontiguousarray(blob, dtype=np.float32) / 255.0

        t0 = time.monotonic()
        out = self.session.run(None, {self.input_name: blob})[0]
        dt = time.monotonic() - t0

        # (1, 84, N) -> (N, 84): 4 box coords (cxcywh, letterbox pixels)
        # then 80 class scores. There is no separate objectness in YOLOv8.
        pred = out[0].T
        cls_scores = pred[:, 4:]
        cls_ids = cls_scores.argmax(1)
        confs = cls_scores[np.arange(len(cls_ids)), cls_ids]
        m = confs >= self.conf
        if not m.any():
            return [], dt
        boxes_cxcywh, confs, cls_ids = pred[m, :4], confs[m], cls_ids[m]

        boxes = np.empty_like(boxes_cxcywh)
        boxes[:, 0] = boxes_cxcywh[:, 0] - boxes_cxcywh[:, 2] / 2
        boxes[:, 1] = boxes_cxcywh[:, 1] - boxes_cxcywh[:, 3] / 2
        boxes[:, 2] = boxes_cxcywh[:, 0] + boxes_cxcywh[:, 2] / 2
        boxes[:, 3] = boxes_cxcywh[:, 1] + boxes_cxcywh[:, 3] / 2

        # Class-aware NMS via the coordinate-offset trick: shift each class
        # into its own region so boxes of different classes never overlap.
        offset = cls_ids[:, None].astype(np.float32) * (self.imgsz * 2)
        keep = nms(boxes + offset, confs, self.iou)

        h, w = bgr.shape[:2]
        dets = []
        for i in keep:
            x1 = float(np.clip((boxes[i, 0] - px) / r, 0, w))
            y1 = float(np.clip((boxes[i, 1] - py) / r, 0, h))
            x2 = float(np.clip((boxes[i, 2] - px) / r, 0, w))
            y2 = float(np.clip((boxes[i, 3] - py) / r, 0, h))
            cid = int(cls_ids[i])
            dets.append({
                "xyxy": [x1, y1, x2, y2],
                "conf": float(confs[i]),
                "cls_id": cid,
                "cls_name": COCO_CLASSES[cid],
            })
        dets.sort(key=lambda d: -d["conf"])
        return dets, dt
