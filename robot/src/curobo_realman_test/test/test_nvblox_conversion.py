import unittest

import numpy as np


def convert_esdf(values, unknown_value=10.0, margin=0.02, unknown_collision=True):
    esdf = np.asarray(values, dtype=np.float32)
    observed = np.isfinite(esdf) & (np.abs(esdf - unknown_value) > 1e-5)
    features = margin - esdf
    features[~observed] = margin if unknown_collision else -max(1.0, margin)
    return features, observed


class NvbloxConversionTest(unittest.TestCase):
    def test_sign_and_margin(self):
        features, observed = convert_esdf([-0.01, 0.01, 0.05, 10.0])
        self.assertGreater(features[0], 0.0)
        self.assertGreater(features[1], 0.0)
        self.assertLess(features[2], 0.0)
        self.assertGreater(features[3], 0.0)
        self.assertEqual(observed.tolist(), [True, True, True, False])

    def test_unknown_can_be_free_for_debug_only(self):
        features, _ = convert_esdf([10.0], unknown_collision=False)
        self.assertLess(features[0], 0.0)


if __name__ == "__main__":
    unittest.main()
