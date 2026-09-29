import unittest

from qwen2_5_vl_ros2.backend import QwenVLBackend


class BackendParsingTest(unittest.TestCase):
    def test_parses_json_array_and_fence(self):
        objects = QwenVLBackend._parse_objects(
            '```json\n[{"label":"罐子","bbox_2d":[10,20,100,200]}]\n```'
        )
        self.assertEqual(objects[0]["label"], "罐子")
        self.assertEqual(objects[0]["bbox_2d"], [10, 20, 100, 200])

    def test_rejects_malformed_boxes(self):
        self.assertEqual(QwenVLBackend._parse_objects('[{"bbox_2d":[1,2,3]}]'), [])

    def test_accepts_wrapped_objects(self):
        objects = QwenVLBackend._parse_objects(
            '{"objects":[{"label":"box","bbox_2d":[10,20,100,200]}]}'
        )
        self.assertEqual(len(objects), 1)

    def test_repairs_bbox_without_array_brackets(self):
        objects = QwenVLBackend._parse_objects(
            '[{"label":"百事可乐","bbox_2d":380,92,462,363}]'
        )
        self.assertEqual(objects[0]["bbox_2d"], [380, 92, 462, 363])

    def test_repairs_stray_quote_in_bbox_key(self):
        objects = QwenVLBackend._parse_objects(
            '[{"label":"百事可乐", "bbox_2d\'\':[387,93,479,362]}]'
        )
        self.assertEqual(objects[0]["bbox_2d"], [387, 93, 479, 362])

    def test_repairs_bbox_closed_by_brace_and_reports_status(self):
        objects, status = QwenVLBackend._parse_objects(
            '```json\n[{"label":"红色瓶身饮料","bbox_2d":[118,79,190,267}}]\n```',
            return_status=True,
        )
        self.assertEqual(status, "VLM_DETECTION_OK")
        self.assertEqual(objects[0]["bbox_2d"], [118, 79, 190, 267])

    def test_repairs_missing_bbox_bracket_before_object_brace(self):
        objects, status = QwenVLBackend._parse_objects(
            '```json\n{"found":true,"bbox_2d":[83,76,129,247\n}\n```',
            return_status=True,
        )
        self.assertEqual(status, "VLM_DETECTION_OK")
        self.assertEqual(objects[0]["bbox_2d"], [83, 76, 129, 247])

    def test_distinguishes_no_detection_parse_error_and_invalid_bbox(self):
        _, no_detection = QwenVLBackend._parse_objects(
            '{"found":false,"bbox_2d":null}', return_status=True
        )
        _, parse_error = QwenVLBackend._parse_objects("not json", return_status=True)
        _, invalid = QwenVLBackend._parse_objects(
            '{"found":true,"bbox_2d":[10,10,9,100]}', return_status=True
        )
        self.assertEqual(no_detection, "VLM_NO_DETECTION")
        self.assertEqual(parse_error, "VLM_JSON_PARSE_ERROR")
        self.assertEqual(invalid, "VLM_INVALID_BBOX")


if __name__ == "__main__":
    unittest.main()
