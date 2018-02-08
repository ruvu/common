import rospy

from qt_gui.plugin import Plugin
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QPushButton
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class PublishPoseButton(Plugin):
    def __init__(self, context):
        super(PublishPoseButton, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('PublishPoseButton')

        # Create QWidget
        self._widget = QWidget()
        self._layout = QVBoxLayout()
        self._widget.setLayout(self._layout)
        self._widget.setObjectName('Navigation Graph Tool')
        # Show title on left-top
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self._robot_pose = PoseStamped()
        self._pub = rospy.Publisher("/graph_navigation/add_node", PoseStamped, queue_size=10)
        self._pose_sub = rospy.Subscriber("pose", PoseWithCovarianceStamped, self._pose_cb)

        store_pose_button = QPushButton('Send pose')
        store_pose_button.clicked.connect(self._send_button_pressed)

        self._layout.addWidget(store_pose_button)

    def _pose_cb(self, msg):
        self._robot_pose = PoseStamped(header=msg.header,
                                       pose=msg.pose.pose)

    def _send_button_pressed(self):
        self._pub.publish(self._robot_pose)

    def shutdown_plugin(self):
        self._pub.unregister()
        self._pose_sub.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass

