import unittest
from mock import patch, MagicMock
from rfid_controller.scripts.lib.sources.robot import Robot


class TestRobot(unittest.TestCase):
    @patch('tf.TransformListener')
    @patch('rospy.get_param')
    @patch('rospy.Duration')
    @patch('actionlib.SimpleActionClient', return_value=MagicMock())
    def setUp(self, mock1, mock2, mock3, mock4):
        self.robot = Robot()

    @patch('tf.TransformListener')
    @patch('rospy.get_param')
    @patch('rospy.Duration')
    @patch('actionlib.SimpleActionClient', return_value=MagicMock())
    def test_singleton(self, mock1, mock2, mock3, mock4):
        #Prepare
        #Execute
        robot2 = Robot()
        #Assert
        self.assertEquals(self.robot, robot2)

    def test_decide_twist_twist(self):
        # Prepare
        self.robot.lower_threshold_twist = 3
        self.robot.upper_threshold_twist = 6
        # Execute
        res = self.robot._decide_twist(7)
        # Assert
        self.assertEquals(res, "TWIST")

    def test_decide_twist_continue(self):
        # Prepare
        self.robot.lower_threshold_twist = 3
        self.robot.upper_threshold_twist = 6
        # Execute
        res = self.robot._decide_twist(5)
        # Assert
        self.assertEquals(res, "CONTINUE")

    def test_decide_twist_navigate(self):
        # Prepare
        self.robot.lower_threshold_twist = 3
        self.robot.upper_threshold_twist = 6
        # Execute
        res = self.robot._decide_twist(2)
        # Assert
        self.assertEquals(res, "NAVIGATE")

    @patch('rfid_controller.scripts.lib.sources.robot.Robot._decide_twist', return_value="TWIST")
    @patch('rfid_controller.scripts.lib.sources.robot.Robot._pause_navigation')
    @patch('rfid_controller.scripts.lib.sources.robot.Robot._twist_robot')
    def test_control_navigation_twist_change(self, mock1, mock2, mock3):
        #Prepare
        self.robot.is_navigating = True
        self.robot.twisting = False
        #Execute
        self.robot.control_navigation_mode(0)
        #Assert
        mock1.assert_called_once()

    @patch('rfid_controller.scripts.lib.sources.robot.Robot._decide_twist', return_value="TWIST")
    @patch('rfid_controller.scripts.lib.sources.robot.Robot._pause_navigation')
    @patch('rfid_controller.scripts.lib.sources.robot.Robot._twist_robot')
    def test_control_navigation_twist_continue(self, mock1, mock2, mock3):
        #Prepare
        self.robot.is_navigating = True
        self.robot.twisting = True
        #Execute
        self.robot.control_navigation_mode(0)
        #Assert
        mock1.assert_not_called()

    @patch('rfid_controller.scripts.lib.sources.robot.Robot._decide_twist', return_value="CONTINUE")
    @patch('rfid_controller.scripts.lib.sources.robot.Robot._pause_navigation')
    @patch('rfid_controller.scripts.lib.sources.robot.Robot._twist_robot')
    def test_control_navigation_continue_twist(self, mock1, mock2, mock3):
        #Prepare
        self.robot.is_navigating = True
        self.robot.twisting = True
        #Execute
        self.robot.control_navigation_mode(0)
        #Assert
        mock1.assert_not_called()

    @patch('rfid_controller.scripts.lib.sources.robot.Robot._decide_twist', return_value="CONTINUE")
    @patch('rfid_controller.scripts.lib.sources.robot.Robot._restart_navigation')
    def test_control_navigation_continue_navigate(self, mock1, mock2):
        #Prepare
        self.robot.is_navigating = True
        self.robot.twisting = False
        #Execute
        self.robot.control_navigation_mode(0)
        #Assert
        mock1.assert_not_called()

    @patch('rfid_controller.scripts.lib.sources.robot.Robot._decide_twist', return_value="NAVIGATE")
    @patch('rfid_controller.scripts.lib.sources.robot.Robot._restart_navigation')
    def test_control_navigation_navigate_change(self, mock1, mock2):
        #Prepare
        self.robot.is_navigating = True
        self.robot.twisting = True
        #Execute
        self.robot.control_navigation_mode(0)
        #Assert
        mock1.assert_called_once()

    @patch('rfid_controller.scripts.lib.sources.robot.Robot._decide_twist', return_value="NAVIGATE")
    @patch('rfid_controller.scripts.lib.sources.robot.Robot._restart_navigation')
    def test_control_navigation_navigate_continue(self, mock1, mock2):
        #Prepare
        self.robot.is_navigating = True
        self.robot.twisting = False
        #Execute
        self.robot.control_navigation_mode(0)
        #Assert
        mock1.assert_not_called()

    @patch('rfid_controller.scripts.lib.sources.robot.Robot._decide_twist', return_value="TWIST")
    @patch('rfid_controller.scripts.lib.sources.robot.Robot._pause_navigation')
    @patch('rfid_controller.scripts.lib.sources.robot.Robot._twist_robot')
    def test_control_navigation_change_twist_true(self, mock1, mock2, mock3):
        #Prepare
        self.robot.is_navigating = True
        self.robot.twisting = False
        #Execute
        self.robot.control_navigation_mode(0)
        #Assert
        self.assertTrue(self.robot.twisting)

    @patch('rfid_controller.scripts.lib.sources.robot.Robot._decide_twist', return_value="NAVIGATE")
    @patch('rfid_controller.scripts.lib.sources.robot.Robot._restart_navigation')
    def test_control_navigation_change_twist_true(self, mock1, mock2):
        #Prepare
        self.robot.is_navigating = True
        self.robot.twisting = True
        #Execute
        self.robot.control_navigation_mode(0)
        #Assert
        self.assertFalse(self.robot.twisting)
