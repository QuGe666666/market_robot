from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class PerceptionRequest:
    arm: str
    yolo_label: str
    qwen_prompt: str
    fallback_prompts: tuple[str, ...] = ()


class PerceptionAdapter:
    """Names the YOLO-first/Qwen-fallback contract used by the FSM."""

    def __init__(self, yolo_topic: str, qwen_prompt_topic: str, qwen_result_topic: str):
        self.yolo_topic = yolo_topic
        self.qwen_prompt_topic = qwen_prompt_topic
        self.qwen_result_topic = qwen_result_topic

    def request(self, request: PerceptionRequest) -> PerceptionRequest:
        return request
