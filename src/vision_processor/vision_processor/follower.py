"""
人体跟随控制器
基于视觉检测实现人体跟随功能
"""

import logging
import time
from dataclasses import dataclass
from typing import Optional, Tuple, Callable
from enum import Enum

from .detector import YOLODetector, ObjectTracker, TrackedObject

logger = logging.getLogger(__name__)


class FollowState(Enum):
    """跟随状态"""
    IDLE = "idle"           # 空闲
    SEARCHING = "searching" # 搜索目标
    FOLLOWING = "following" # 正在跟随
    LOST = "lost"           # 丢失目标
    PAUSED = "paused"       # 暂停


@dataclass
class FollowConfig:
    """跟随配置"""
    # 目标距离控制
    target_area: float = 0.15       # 目标面积比例（占画面）
    area_tolerance: float = 0.05    # 面积容差
    
    # 位置控制
    center_x_tolerance: float = 0.15  # X方向居中容差
    
    # 速度限制
    max_linear_speed: float = 0.2   # 最大前进速度 m/s
    max_angular_speed: float = 1.0  # 最大旋转速度 rad/s
    
    # PID 控制参数
    kp_linear: float = 1.5          # 线速度比例系数
    kp_angular: float = 2.0         # 角速度比例系数
    
    # 丢失处理
    lost_timeout: float = 3.0       # 丢失超时（秒）
    search_angular_speed: float = 0.5  # 搜索时的旋转速度


@dataclass
class FollowCommand:
    """跟随控制命令"""
    linear_x: float = 0.0   # 前进速度
    linear_y: float = 0.0   # 侧移速度（麦轮）
    angular_z: float = 0.0  # 旋转速度
    
    def is_stop(self) -> bool:
        return abs(self.linear_x) < 0.01 and abs(self.angular_z) < 0.01


class PersonFollower:
    """
    人体跟随控制器
    
    工作流程:
    1. 通过摄像头获取图像
    2. 使用 YOLO 检测人体
    3. 跟踪主要目标
    4. 计算控制命令使车辆跟随目标
    """
    
    def __init__(
        self,
        detector: Optional[YOLODetector] = None,
        config: Optional[FollowConfig] = None,
    ):
        self.detector = detector or YOLODetector()
        self.tracker = ObjectTracker(max_age=1.0)
        self.config = config or FollowConfig()
        
        self.state = FollowState.IDLE
        self._target_id: Optional[int] = None
        self._last_seen_time: float = 0
        self._is_running = False
        
        # 回调函数
        self._on_command: Optional[Callable[[FollowCommand], None]] = None
        self._on_state_change: Optional[Callable[[FollowState], None]] = None
    
    @property
    def is_available(self) -> bool:
        """检查跟随功能是否可用"""
        return self.detector.is_available
    
    def set_command_callback(self, callback: Callable[[FollowCommand], None]):
        """设置控制命令回调"""
        self._on_command = callback
    
    def set_state_callback(self, callback: Callable[[FollowState], None]):
        """设置状态变化回调"""
        self._on_state_change = callback
    
    def _set_state(self, new_state: FollowState):
        """更新状态并触发回调"""
        if new_state != self.state:
            old_state = self.state
            self.state = new_state
            logger.info(f"跟随状态: {old_state.value} -> {new_state.value}")
            if self._on_state_change:
                self._on_state_change(new_state)
    
    def start(self):
        """开始跟随"""
        if not self.is_available:
            logger.error("YOLO 检测器不可用，无法启动跟随")
            return
        
        self._is_running = True
        self._set_state(FollowState.SEARCHING)
        logger.info("人体跟随已启动")
    
    def stop(self):
        """停止跟随"""
        self._is_running = False
        self._target_id = None
        self._set_state(FollowState.IDLE)
        
        # 发送停止命令
        if self._on_command:
            self._on_command(FollowCommand())
        
        logger.info("人体跟随已停止")
    
    def pause(self):
        """暂停跟随"""
        if self._is_running:
            self._set_state(FollowState.PAUSED)
            if self._on_command:
                self._on_command(FollowCommand())
    
    def resume(self):
        """恢复跟随"""
        if self._is_running and self.state == FollowState.PAUSED:
            if self._target_id is not None:
                self._set_state(FollowState.FOLLOWING)
            else:
                self._set_state(FollowState.SEARCHING)
    
    def process_frame(self, frame) -> FollowCommand:
        """
        处理一帧图像
        
        Args:
            frame: 输入图像帧 (numpy array, BGR 格式)
            
        Returns:
            控制命令
        """
        if not self._is_running or self.state == FollowState.PAUSED:
            return FollowCommand()
        
        current_time = time.time()
        
        # 检测人体
        detections = self.detector.find_persons(frame)
        
        # 更新跟踪
        tracked_objects = self.tracker.update(
            [d for d in detections if d.class_name == "person"]
        )
        
        # 获取目标
        target = self._get_target(tracked_objects)
        
        # 根据状态计算命令
        if target is None:
            # 没有目标
            if self.state == FollowState.FOLLOWING:
                # 刚丢失目标
                if current_time - self._last_seen_time > self.config.lost_timeout:
                    self._set_state(FollowState.LOST)
                    self._target_id = None
            
            if self.state in (FollowState.SEARCHING, FollowState.LOST):
                # 搜索模式：缓慢旋转
                cmd = FollowCommand(angular_z=self.config.search_angular_speed)
            else:
                cmd = FollowCommand()
        else:
            # 有目标
            self._last_seen_time = current_time
            
            if self.state != FollowState.FOLLOWING:
                self._set_state(FollowState.FOLLOWING)
                self._target_id = target.id
            
            # 计算控制命令
            cmd = self._compute_command(target)
        
        # 发送命令
        if self._on_command:
            self._on_command(cmd)
        
        return cmd
    
    def _get_target(self, tracked_objects: dict) -> Optional[TrackedObject]:
        """获取跟踪目标"""
        if not tracked_objects:
            return None
        
        # 如果有指定目标，优先使用
        if self._target_id is not None and self._target_id in tracked_objects:
            return tracked_objects[self._target_id]
        
        # 否则选择最大/最近的人
        persons = [
            obj for obj in tracked_objects.values()
            if obj.detection.class_name == "person"
        ]
        
        if not persons:
            return None
        
        # 选择面积最大的（最近的）
        return max(persons, key=lambda obj: obj.detection.area)
    
    def _compute_command(self, target: TrackedObject) -> FollowCommand:
        """计算控制命令"""
        detection = target.detection
        cx, cy = detection.center
        area = detection.area
        
        cfg = self.config
        cmd = FollowCommand()
        
        # 计算角速度（保持目标居中）
        # cx 范围 [0, 1]，中心为 0.5
        x_error = 0.5 - cx  # 正值表示目标在左边
        if abs(x_error) > cfg.center_x_tolerance:
            cmd.angular_z = cfg.kp_angular * x_error
            cmd.angular_z = max(-cfg.max_angular_speed, 
                               min(cfg.max_angular_speed, cmd.angular_z))
        
        # 计算线速度（保持目标距离）
        # area 越大表示越近
        area_error = cfg.target_area - area  # 正值表示目标太远
        if abs(area_error) > cfg.area_tolerance:
            cmd.linear_x = cfg.kp_linear * area_error
            cmd.linear_x = max(-cfg.max_linear_speed,
                              min(cfg.max_linear_speed, cmd.linear_x))
        
        return cmd
    
    def get_status(self) -> dict:
        """获取当前状态"""
        return {
            "state": self.state.value,
            "is_running": self._is_running,
            "is_available": self.is_available,
            "target_id": self._target_id,
            "tracked_count": len(self.tracker.tracked_objects),
        }


