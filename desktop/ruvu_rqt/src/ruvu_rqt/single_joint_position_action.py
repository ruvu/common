import rospy

from qt_gui.plugin import Plugin
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QPushButton, QMessageBox, QInputDialog, QLineEdit
import control_msgs.msg
import actionlib

from dialogs import get_double_from_input, get_string_from_input


class SingleJointPositionActionPlugin(Plugin):
    def __init__(self, context):
        """
        Send a single joint position goal
        """
        super(SingleJointPositionActionPlugin, self).__init__(context)

        # Create QWidget
        self._widget = QWidget()
        self._layout = QVBoxLayout()
        self._widget.setLayout(self._layout)
        self._widget.setObjectName('SingleJointPositionAction')

        # Add widget to the user interface
        context.add_widget(self._widget)
        self._publish_pose_button = QPushButton('')
        self._publish_pose_button.clicked.connect(self._send_button_pressed)

        self._layout.addWidget(self._publish_pose_button)

        self._setup()

    def _setup(self, position=0, action_name="single_joint_position_action", timeout=2.0):
        self._position = position
        self._action_name = action_name
        self._timeout = timeout
        self._client = actionlib.SimpleActionClient(self._action_name, control_msgs.msg.SingleJointPositionAction)
        self._publish_pose_button.setText("Send position {}".format(self._position))
        rospy.loginfo("SingleJointPositionActionPlugin initialized on action={}, timeout={}, position={}"
                      .format(action_name, timeout, position))

    def _send_button_pressed(self):
        goal = control_msgs.msg.SingleJointPositionGoal(position=self._position)

        rospy.loginfo("Sending SingleJointPositionGoal position = {}".format(self._position))
        self._client.send_goal(goal)
        if not self._client.wait_for_result(rospy.Duration(self._timeout)):
            QMessageBox.warning(self._widget, "Goal failed", "Goal did not succeed in {} seconds".format(self._timeout))
        else:
            rospy.loginfo("Done")

    def save_settings(self, plugin_settings, instance_settings):
        """
        Saves the settings the perspective cfg 
        """
        instance_settings.set_value("position", self._position)
        instance_settings.set_value("action_name", self._action_name)
        instance_settings.set_value("timeout", self._timeout)

    def restore_settings(self, plugin_settings, instance_settings):
        """
        Restores the settings the perspective cfg 
        """
        action_name = instance_settings.value("action_name")
        position = instance_settings.value("position")
        timeout = instance_settings.value("timeout")
        if None not in [action_name, position, timeout]:
            self._setup(float(position), action_name, float(timeout))

    def trigger_configuration(self):
        """
        Triggered when the gear wheel configuration is triggered
        """
        self._setup(float(get_double_from_input(self._widget, "Position", self._position, -1e4, 1e4, 3)),
                    get_string_from_input(self._widget, "Action name", self._action_name),
                    float(get_double_from_input(self._widget, "Timeout", self._timeout, 0, 1e3)))
