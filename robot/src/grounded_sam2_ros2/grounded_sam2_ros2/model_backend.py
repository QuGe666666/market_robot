import os
import json

import numpy as np
from PIL import Image
import torch
from transformers import (
    GroundingDinoForObjectDetection,
    GroundingDinoProcessor,
    Sam2Config,
    Sam2Model,
    Sam2Processor,
)


class GroundedSam2Backend:
    def __init__(self, dino_path, sam2_path, device="cuda", use_fp16=True):
        if device.startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError("CUDA was requested but torch.cuda.is_available() is false")
        for path in (dino_path, sam2_path):
            if not os.path.isdir(path):
                raise FileNotFoundError(f"Model directory does not exist: {path}")
        self.device = torch.device(device)
        self.sam_dtype = torch.float16 if use_fp16 and self.device.type == "cuda" else torch.float32
        self.dino_processor = GroundingDinoProcessor.from_pretrained(dino_path, local_files_only=True)
        self.dino = GroundingDinoForObjectDetection.from_pretrained(
            dino_path, local_files_only=True, dtype=torch.float32
        ).to(self.device).eval()
        self.sam_processor = Sam2Processor.from_pretrained(sam2_path, local_files_only=True)
        with open(os.path.join(sam2_path, "config.json"), encoding="utf-8") as stream:
            sam_config_values = json.load(stream)
        sam_config_values["model_type"] = "sam2"
        sam_config_values["architectures"] = ["Sam2Model"]
        sam_config = Sam2Config.from_dict(sam_config_values)
        self.sam = Sam2Model.from_pretrained(
            sam2_path, local_files_only=True, config=sam_config, dtype=self.sam_dtype
        ).to(self.device).eval()

    @torch.inference_mode()
    def infer(self, bgr_image, prompt, box_threshold, text_threshold, max_detections):
        rgb = bgr_image[:, :, ::-1]
        image = Image.fromarray(np.ascontiguousarray(rgb))
        text = prompt.strip()
        if text and not text.endswith("."):
            text += "."
        inputs = self.dino_processor(images=image, text=text, return_tensors="pt")
        inputs = {key: value.to(self.device) for key, value in inputs.items()}
        if "pixel_values" in inputs:
            inputs["pixel_values"] = inputs["pixel_values"].float()
        outputs = self.dino(**inputs)
        result = self.dino_processor.post_process_grounded_object_detection(
            outputs,
            inputs["input_ids"],
            threshold=float(box_threshold),
            text_threshold=float(text_threshold),
            target_sizes=[image.size[::-1]],
        )[0]
        scores = result["scores"].detach().cpu().numpy()
        boxes = result["boxes"].detach().cpu().numpy()
        labels = result.get("text_labels", result.get("labels", []))
        order = np.argsort(scores)[::-1][: int(max_detections)]
        detections = []
        for index in order:
            box = boxes[index].astype(np.float32)
            sam_inputs = self.sam_processor(
                images=image,
                input_boxes=[[box.tolist()]],
                return_tensors="pt",
            )
            original_sizes = sam_inputs["original_sizes"]
            sam_inputs = {key: value.to(self.device) for key, value in sam_inputs.items()}
            if "pixel_values" in sam_inputs:
                sam_inputs["pixel_values"] = sam_inputs["pixel_values"].to(self.sam_dtype)
            sam_outputs = self.sam(**sam_inputs, multimask_output=True)
            masks = self.sam_processor.post_process_masks(
                sam_outputs.pred_masks.cpu(), original_sizes.cpu()
            )[0][0]
            iou = sam_outputs.iou_scores[0, 0].detach().float().cpu().numpy()
            best = int(np.argmax(iou))
            mask = masks[best].numpy().astype(bool)
            label = labels[index] if len(labels) else text.rstrip(".")
            detections.append(
                {
                    "label": str(label),
                    "score": float(scores[index]),
                    "box": box,
                    "mask": mask,
                    "mask_iou": float(iou[best]),
                }
            )
        return detections
