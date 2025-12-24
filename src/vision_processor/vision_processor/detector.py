"""
YOLO 目标检测器
提供目标检测和跟踪功能
"""

import time
import logging
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass, field
from enum import Enum

logger = logging.getLogger(__name__)


class DetectionClass(Enum):
    """检测类别"""
    PERSON = "person"
    CAR = "car"
    DOG = "dog"
    CAT = "cat"
    BALL = "ball"
    CHAIR = "chair"
    UNKNOWN = "unknown"


@dataclass
class Detection:
    """检测结果"""
    class_name: str
    class_id: int
    confidence: float
    bbox: Tuple[float, float, float, float]  # x1, y1, x2, y2 (normalized 0-1)
    center: Tuple[float, float] = field(init=False)  # 中心点
    area: float = field(init=False)  # 面积比例
    
    def __post_init__(self):
        x1, y1, x2, y2 = self.bbox
        self.center = ((x1 + x2) / 2, (y1 + y2) / 2)
        self.area = (x2 - x1) * (y2 - y1)


@dataclass
class TrackedObject:
    """跟踪目标"""
    id: int
    detection: Detection
    last_seen: float
    frames_tracked: int = 1
    velocity: Tuple[float, float] = (0.0, 0.0)  # 速度估计


class YOLODetector:
    """
    YOLO 目标检测器
    
    注意: 这是一个框架实现，实际使用需要：
    1. 安装 ultralytics 或 onnxruntime
    2. 下载 YOLO 模型文件
    3. 配置摄像头输入
    """
    
    def __init__(
        self,
        model_path: str = "yolov8n.pt",
        confidence_threshold: float = 0.5,
        device: str = "auto",
    ):
        self.model_path = model_path
        self.confidence_threshold = confidence_threshold
        self.device = device
        self.model = None
        self._is_loaded = False
        
        # 尝试加载模型
        self._try_load_model()
    
    def _try_load_model(self) -> bool:
        """尝试加载 YOLO 模型"""
        try:
            from ultralytics import YOLO
            self.model = YOLO(self.model_path)
            self._is_loaded = True
            logger.info(f"YOLO 模型已加载: {self.model_path}")
            return True
        except ImportError:
            logger.warning("ultralytics 未安装，YOLO 功能不可用")
            logger.info("安装命令: pip install ultralytics")
            return False
        except Exception as e:
            logger.error(f"加载 YOLO 模型失败: {e}")
            return False
    
    @property
    def is_available(self) -> bool:
        """检查 YOLO 是否可用"""
        return self._is_loaded
    
    def detect(self, frame) -> List[Detection]:
        """
        执行目标检测
        
        Args:
            frame: 输入图像帧 (numpy array, BGR 格式)
            
        Returns:
            检测结果列表
        """
        if not self._is_loaded:
            return []
        
        try:
            results = self.model(frame, conf=self.confidence_threshold, verbose=False)
            detections = []
            
            for result in results:
                boxes = result.boxes
                if boxes is None:
                    continue
                
                h, w = frame.shape[:2]
                
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    conf = float(box.conf[0])
                    cls_id = int(box.cls[0])
                    cls_name = result.names[cls_id]
                    
                    # 归一化坐标
                    detection = Detection(
                        class_name=cls_name,
                        class_id=cls_id,
                        confidence=conf,
                        bbox=(x1/w, y1/h, x2/w, y2/h)
                    )
                    detections.append(detection)
            
            return detections
            
        except Exception as e:
            logger.error(f"检测失败: {e}")
            return []
    
    def find_persons(self, frame) -> List[Detection]:
        """检测画面中的人"""
        detections = self.detect(frame)
        return [d for d in detections if d.class_name == "person"]
    
    def get_largest_person(self, frame) -> Optional[Detection]:
        """获取画面中最大的人（最近的人）"""
        persons = self.find_persons(frame)
        if not persons:
            return None
        return max(persons, key=lambda d: d.area)


class ObjectTracker:
    """
    简单的目标跟踪器
    基于 IoU (交并比) 进行跨帧关联
    """
    
    def __init__(self, max_age: float = 1.0, min_iou: float = 0.3):
        self.max_age = max_age  # 目标消失后保留的最大时间（秒）
        self.min_iou = min_iou  # 最小 IoU 阈值
        self.tracked_objects: Dict[int, TrackedObject] = {}
        self._next_id = 1
    
    def _compute_iou(self, bbox1: Tuple, bbox2: Tuple) -> float:
        """计算两个边界框的 IoU"""
        x1 = max(bbox1[0], bbox2[0])
        y1 = max(bbox1[1], bbox2[1])
        x2 = min(bbox1[2], bbox2[2])
        y2 = min(bbox1[3], bbox2[3])
        
        if x2 <= x1 or y2 <= y1:
            return 0.0
        
        intersection = (x2 - x1) * (y2 - y1)
        area1 = (bbox1[2] - bbox1[0]) * (bbox1[3] - bbox1[1])
        area2 = (bbox2[2] - bbox2[0]) * (bbox2[3] - bbox2[1])
        union = area1 + area2 - intersection
        
        return intersection / union if union > 0 else 0.0
    
    def update(self, detections: List[Detection]) -> Dict[int, TrackedObject]:
        """
        更新跟踪状态
        
        Args:
            detections: 当前帧的检测结果
            
        Returns:
            更新后的跟踪目标字典
        """
        current_time = time.time()
        
        # 移除过期的跟踪目标
        expired_ids = [
            tid for tid, obj in self.tracked_objects.items()
            if current_time - obj.last_seen > self.max_age
        ]
        for tid in expired_ids:
            del self.tracked_objects[tid]
        
        # 匹配检测结果与现有跟踪目标
        used_detections = set()
        
        for tid, tracked in list(self.tracked_objects.items()):
            best_iou = 0.0
            best_det_idx = -1
            
            for i, det in enumerate(detections):
                if i in used_detections:
                    continue
                if det.class_name != tracked.detection.class_name:
                    continue
                
                iou = self._compute_iou(tracked.detection.bbox, det.bbox)
                if iou > best_iou and iou >= self.min_iou:
                    best_iou = iou
                    best_det_idx = i
            
            if best_det_idx >= 0:
                # 更新跟踪目标
                old_center = tracked.detection.center
                new_det = detections[best_det_idx]
                new_center = new_det.center
                
                # 计算速度（像素/帧）
                dt = current_time - tracked.last_seen
                if dt > 0:
                    vx = (new_center[0] - old_center[0]) / dt
                    vy = (new_center[1] - old_center[1]) / dt
                    tracked.velocity = (vx, vy)
                
                tracked.detection = new_det
                tracked.last_seen = current_time
                tracked.frames_tracked += 1
                used_detections.add(best_det_idx)
        
        # 创建新的跟踪目标
        for i, det in enumerate(detections):
            if i not in used_detections:
                new_obj = TrackedObject(
                    id=self._next_id,
                    detection=det,
                    last_seen=current_time
                )
                self.tracked_objects[self._next_id] = new_obj
                self._next_id += 1
        
        return self.tracked_objects
    
    def get_primary_target(self, class_name: str = "person") -> Optional[TrackedObject]:
        """
        获取主要跟踪目标（跟踪时间最长的目标）
        
        Args:
            class_name: 目标类别
            
        Returns:
            主要跟踪目标或 None
        """
        candidates = [
            obj for obj in self.tracked_objects.values()
            if obj.detection.class_name == class_name
        ]
        
        if not candidates:
            return None
        
        # 优先选择跟踪时间最长的
        return max(candidates, key=lambda obj: obj.frames_tracked)

