import unittest

from geometry_msgs.msg import PoseStamped, TransformStamped

from grasp_nvblox_curobo_test.grasp_test_node import transform_pose_stamped_exact


class GraspTransformTest(unittest.TestCase):
    def test_humble_pose_api_preserves_exact_stamp(self):
        source = PoseStamped()
        source.header.frame_id = "camera"
        source.header.stamp.sec = 17
        source.header.stamp.nanosec = 42
        source.pose.position.x = 0.25
        source.pose.orientation.w = 1.0

        transform = TransformStamped()
        transform.header.frame_id = "base"
        transform.child_frame_id = "camera"
        transform.transform.translation.x = 1.0
        transform.transform.rotation.w = 1.0

        result = transform_pose_stamped_exact(source, transform, "base")

        self.assertEqual(result.header.frame_id, "base")
        self.assertEqual(result.header.stamp.sec, 17)
        self.assertEqual(result.header.stamp.nanosec, 42)
        self.assertAlmostEqual(result.pose.position.x, 1.25)
        self.assertAlmostEqual(result.pose.orientation.w, 1.0)


if __name__ == "__main__":
    unittest.main()
