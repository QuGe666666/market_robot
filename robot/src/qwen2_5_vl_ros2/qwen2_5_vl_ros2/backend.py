import json
import math
import re
from enum import Enum

import cv2
import numpy as np
from PIL import Image
import torch
from transformers import AutoProcessor, Qwen2_5_VLForConditionalGeneration


class VLMStatus(str, Enum):
    DETECTION_OK = "VLM_DETECTION_OK"
    NO_DETECTION = "VLM_NO_DETECTION"
    JSON_PARSE_ERROR = "VLM_JSON_PARSE_ERROR"
    INVALID_BBOX = "VLM_INVALID_BBOX"
    DEPTH_INVALID = "VLM_DEPTH_INVALID"
    TF_FAILED = "VLM_TF_FAILED"


class QwenVLBackend:
    def __init__(self, model_path, input_width=560, input_height=420, max_new_tokens=192):
        if not torch.cuda.is_available():
            raise RuntimeError("Qwen2.5-VL requires CUDA on this deployment")
        self.device = torch.device("cuda")
        self.input_width = int(input_width)
        self.input_height = int(input_height)
        self.max_new_tokens = int(max_new_tokens)
        pixels = self.input_width * self.input_height
        self.processor = AutoProcessor.from_pretrained(
            model_path,
            local_files_only=True,
            min_pixels=pixels,
            max_pixels=pixels,
            use_fast=False,
        )
        self.model = Qwen2_5_VLForConditionalGeneration.from_pretrained(
            model_path,
            local_files_only=True,
            dtype=torch.bfloat16,
            attn_implementation="eager",
            low_cpu_mem_usage=True,
        ).to(self.device).eval()
        self.model.generation_config.temperature = None

    @torch.inference_mode()
    def infer(self, bgr_image, keyword):
        original_height, original_width = bgr_image.shape[:2]
        resized = cv2.resize(
            bgr_image, (self.input_width, self.input_height), interpolation=cv2.INTER_AREA
        )
        image = Image.fromarray(cv2.cvtColor(resized, cv2.COLOR_BGR2RGB))
        instruction = (
            "在图片中确认描述为" + keyword + "的一个目标。"
            "只能输出一个JSON对象，严格使用 schema："
            '{"found":true,"bbox_2d":[x1,y1,x2,y2]}。'
            '无法确认时必须输出 {"found":false,"bbox_2d":null}。'
            f"坐标必须是当前{self.input_width}x{self.input_height}图片上的绝对像素坐标。"
            "目标过远、模糊、遮挡、反光、视角异常或有多个相似商品无法确认时必须 found=false，禁止猜测。"
        )
        messages = [
            {
                "role": "user",
                "content": [
                    {"type": "image"},
                    {"type": "text", "text": instruction},
                ],
            }
        ]
        text = self.processor.apply_chat_template(
            messages, tokenize=False, add_generation_prompt=True
        )
        inputs = self.processor(
            text=[text], images=[image], padding=True, return_tensors="pt"
        ).to(self.device)
        generated = self.model.generate(
            **inputs,
            max_new_tokens=self.max_new_tokens,
            do_sample=False,
            return_dict_in_generate=True,
            output_scores=True,
        )
        input_length = inputs.input_ids.shape[1]
        output_ids = generated.sequences[:, input_length:]
        raw_text = self.processor.batch_decode(
            output_ids, skip_special_tokens=True, clean_up_tokenization_spaces=False
        )[0]
        confidence = self._generation_confidence(output_ids[0], generated.scores)
        objects, parse_status = self._parse_objects(
            raw_text, self.input_width, self.input_height, return_status=True
        )
        scale_x = original_width / float(self.input_width)
        scale_y = original_height / float(self.input_height)
        normalized = []
        for item in objects:
            box = np.asarray(item["bbox_2d"], dtype=np.float32)
            box[[0, 2]] *= scale_x
            box[[1, 3]] *= scale_y
            box[[0, 2]] = np.clip(box[[0, 2]], 0, original_width - 1)
            box[[1, 3]] = np.clip(box[[1, 3]], 0, original_height - 1)
            if box[2] <= box[0] or box[3] <= box[1]:
                continue
            normalized.append({"label": str(item.get("label", keyword)), "box": box})
        return normalized, raw_text, confidence, parse_status

    @staticmethod
    def _generation_confidence(output_ids, scores):
        selected = []
        for token_id, logits in zip(output_ids.tolist(), scores):
            probability = torch.softmax(logits[0].float(), dim=-1)[token_id]
            selected.append(torch.log(probability.clamp_min(1e-12)))
        if not selected:
            return 0.0
        return float(torch.exp(torch.stack(selected).mean()).cpu())

    @staticmethod
    def _parse_objects(raw_text, image_width=560, image_height=420, return_status=False):
        def done(items, status):
            return (items, status) if return_status else items
        cleaned = re.sub(
            r"^\s*```(?:json)?\s*|\s*```\s*$", "", raw_text.strip(), flags=re.I
        )
        cleaned = cleaned.replace("\u201c", '"').replace("\u201d", '"')
        # Small-model decoding can leave a stray quote in a JSON key.
        cleaned = re.sub(r"[\"']bbox_2d[\"']*\s*:", '"bbox_2d":', cleaned)
        number = r"-?\d+(?:\.\d+)?"
        missing_brackets = re.compile(
            rf'(\"bbox_2d\"\s*:\s*)({number}\s*,\s*{number}\s*,\s*{number}\s*,\s*{number})'
        )
        cleaned = missing_brackets.sub(r"\1[\2]", cleaned)
        cleaned = re.sub(r",\s*([}\]])", r"\1", cleaned)
        # The model may put a newline between the final coordinate and the
        # object brace; recover only the exact four-number bbox form.
        cleaned = re.sub(
            r'("bbox_2d"\s*:\s*\[[^\]]*?)(\s*})',
            lambda match: match.group(1) + "]" + match.group(2),
            cleaned,
        )
        cleaned = cleaned.replace("]}}", "]}")
        candidates = [cleaned]
        array_match = re.search(r"\[[\s\S]*\]", cleaned)
        object_match = re.search(r"\{[\s\S]*\}", cleaned)
        if array_match:
            candidates.append(array_match.group(0))
        if object_match:
            candidates.append(object_match.group(0))
        payload = None
        for candidate in candidates:
            try:
                payload = json.loads(candidate)
                break
            except json.JSONDecodeError:
                continue
        if isinstance(payload, dict) and "found" in payload:
            if not bool(payload.get("found")):
                return done([], VLMStatus.NO_DETECTION.value)
            payload = [payload]
        elif isinstance(payload, dict):
            payload = payload.get("objects", [payload])
        if not isinstance(payload, list):
            return done([], VLMStatus.JSON_PARSE_ERROR.value)
        result = []
        for item in payload:
            if not isinstance(item, dict):
                continue
            box = item.get("bbox_2d", item.get("bbox"))
            if isinstance(box, list) and len(box) == 4 and all(
                isinstance(value, (int, float)) and math.isfinite(value) for value in box
            ):
                x1, y1, x2, y2 = map(float, box)
                area = (x2 - x1) * (y2 - y1)
                if (0 <= x1 < x2 <= image_width and 0 <= y1 < y2 <= image_height
                        and area >= max(16.0, image_width * image_height * 0.0002)):
                    result.append({"label": item.get("label", "object"), "bbox_2d": box})
        if result:
            return done(result, VLMStatus.DETECTION_OK.value)
        if payload == []:
            return done([], VLMStatus.NO_DETECTION.value)
        return done([], VLMStatus.INVALID_BBOX.value)
