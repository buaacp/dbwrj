import math
import unittest
import numpy as np
from uam_ocp_gazebo_bridge.frame_converter import FrameConverter


class TestFrames(unittest.TestCase):
    def test_resolved_mavros_enu_identity(self):
        converter=FrameConverter()
        for vector in ([0,0,0],[1,0,0],[0,1,0],[0,0,1]):
            np.testing.assert_array_equal(converter.world_to_setpoint_position(vector),vector)
            np.testing.assert_array_equal(converter.world_to_setpoint_velocity(vector),vector)
            np.testing.assert_array_equal(converter.world_to_setpoint_acceleration(vector),vector)
        for yaw in (0,math.pi/2,-math.pi/2):self.assertAlmostEqual(converter.world_to_setpoint_yaw(yaw),yaw)


if __name__=="__main__":unittest.main()
