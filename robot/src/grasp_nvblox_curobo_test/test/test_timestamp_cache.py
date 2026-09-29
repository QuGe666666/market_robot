import unittest

from grasp_nvblox_curobo_test.timestamped_graspnet_node import ExactStampCache, stamp_key


class Stamp:
    def __init__(self, sec, nanosec):
        self.sec = sec
        self.nanosec = nanosec


class TimestampCacheTest(unittest.TestCase):
    def test_stamp_key_preserves_both_fields(self):
        self.assertEqual(stamp_key(Stamp(17, 42)), (17, 42))

    def test_cache_only_returns_exact_timestamp(self):
        cache = ExactStampCache[str](2)
        cache.put((1, 10), "first")
        self.assertEqual(cache.get((1, 10)), "first")
        self.assertIsNone(cache.get((1, 11)))

    def test_cache_evicts_oldest_entry(self):
        cache = ExactStampCache[str](2)
        cache.put((1, 0), "first")
        cache.put((2, 0), "second")
        cache.put((3, 0), "third")
        self.assertIsNone(cache.get((1, 0)))
        self.assertEqual(cache.span(), ((2, 0), (3, 0)))

    def test_cache_rejects_zero_stamp(self):
        cache = ExactStampCache[str](1)
        with self.assertRaises(ValueError):
            cache.put((0, 0), "invalid")


if __name__ == "__main__":
    unittest.main()
