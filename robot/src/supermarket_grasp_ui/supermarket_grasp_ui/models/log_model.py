from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime


@dataclass(frozen=True)
class ConsoleLog:
    level: str
    module: str
    message: str
    task_id: str = "-"
    state: str = "-"
    timestamp: str = ""

    def normalized(self) -> "ConsoleLog":
        return ConsoleLog(
            level=(self.level or "INFO").upper(),
            module=(self.module or "SYSTEM").upper(),
            message=self.message,
            task_id=self.task_id or "-",
            state=self.state or "-",
            timestamp=self.timestamp or datetime.now().strftime("%H:%M:%S.%f")[:-3],
        )

    def line(self) -> str:
        item = self.normalized()
        return (
            f"{item.timestamp} [{item.level:<5}] [{item.module:<10}] "
            f"task={item.task_id} state={item.state}  {item.message}"
        )

