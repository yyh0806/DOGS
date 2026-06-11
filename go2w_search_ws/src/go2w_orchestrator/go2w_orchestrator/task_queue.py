"""动态优先级任务队列。

支持实时插入、重排序、取消。任务编排器通过此队列调度
Nav2 导航、区域搜索、目标跟踪等子任务。
"""

import time
import uuid
import threading
from typing import Optional, List, Dict


class TaskItem:
    """单个任务。"""

    def __init__(self, task_type: str, params: dict = None,
                 priority: int = 5, parent_id: str = None):
        self.task_id = uuid.uuid4().hex[:8]
        self.task_type = task_type  # navigate, search_area, follow, observe, return_home, wait
        self.priority = priority    # 1-10, higher = more urgent
        self.params = params or {}
        self.status = "pending"     # pending, active, completed, failed, cancelled
        self.parent_id = parent_id
        self.created_at = time.time()
        self.result: Optional[dict] = None

    def to_dict(self) -> dict:
        return {
            "task_id": self.task_id,
            "task_type": self.task_type,
            "priority": self.priority,
            "params": self.params,
            "status": self.status,
            "parent_id": self.parent_id,
            "created_at": self.created_at,
            "result": self.result,
        }


class TaskQueue:
    """优先级任务队列，支持动态调整。"""

    def __init__(self):
        self._lock = threading.Lock()
        self._tasks: List[TaskItem] = []
        self._active: Optional[TaskItem] = None

    @property
    def active(self) -> Optional[TaskItem]:
        with self._lock:
            return self._active

    def push(self, task: TaskItem):
        """按优先级插入任务。"""
        with self._lock:
            self._tasks.append(task)
            self._tasks.sort(key=lambda t: -t.priority)

    def push_front(self, task: TaskItem):
        """紧急插入到队列最前面。"""
        task.priority = 10
        with self._lock:
            self._tasks.insert(0, task)

    def push_list(self, tasks: List[TaskItem]):
        """批量插入（保持顺序，优先级递减）。"""
        with self._lock:
            for i, task in enumerate(tasks):
                if task.priority == 5:  # 未显式设置优先级
                    task.priority = max(1, 8 - i)
                self._tasks.append(task)
            self._tasks.sort(key=lambda t: (-t.priority, t.created_at))

    def pop_next(self) -> Optional[TaskItem]:
        """取出最高优先级的 pending 任务并标记 active。"""
        with self._lock:
            if self._active is not None:
                return None
            for task in self._tasks:
                if task.status == "pending":
                    task.status = "active"
                    self._active = task
                    return task
        return None

    def complete_active(self, result: dict = None):
        """完成当前活跃任务。"""
        with self._lock:
            if self._active:
                self._active.status = "completed"
                self._active.result = result
                self._tasks.remove(self._active)
                self._active = None

    def fail_active(self, reason: str = ""):
        """标记当前活跃任务失败。"""
        with self._lock:
            if self._active:
                self._active.status = "failed"
                self._active.result = {"reason": reason}
                self._tasks.remove(self._active)
                self._active = None

    def cancel_all(self):
        """取消所有任务。"""
        with self._lock:
            if self._active:
                self._active.status = "cancelled"
            self._active = None
            self._tasks.clear()

    def cancel_by_type(self, task_type: str):
        """取消指定类型的所有任务。"""
        with self._lock:
            self._tasks = [
                t for t in self._tasks
                if t.task_type != task_type or t.status != "pending"
            ]

    def insert_after_current(self, task: TaskItem):
        """在当前任务之后插入（观察子任务）。"""
        task.priority = 9
        with self._lock:
            self._tasks.insert(0, task)

    def size(self) -> int:
        with self._lock:
            return len([t for t in self._tasks if t.status == "pending"])

    def get_state(self) -> dict:
        """获取队列状态（用于发布到话题）。"""
        with self._lock:
            active_dict = self._active.to_dict() if self._active else None
            pending = [t.to_dict() for t in self._tasks if t.status == "pending"]
            return {
                "active": active_dict,
                "pending": pending,
                "pending_count": len(pending),
                "state": "executing" if self._active else "idle",
            }
