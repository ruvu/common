import rospy
import tf2_ros

from qt_gui.plugin import Plugin
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QPushButton, QMessageBox, QInputDialog, QLineEdit
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QPushButton, QMessageBox
from geometry_msgs.msg import PoseStamped, Pose, Point

from dialogs import get_double_from_input, get_string_from_input


class PublishPosePlugin(Plugin):
    def __init__(self, context):
        """
        Publishes pose from a specified frame to another
        """
        super(PublishPosePlugin, self).__init__(context)

        # Create QWidget
        self._widget = QWidget()
        self._layout = QVBoxLayout()
        self._widget.setLayout(self._layout)
        self._widget.setObjectName('PublishPose')

        # Add widget to the user interface
        context.add_widget(self._widget)
        publish_pose_button = QPushButton('Publish pose')
        publish_pose_button.clicked.connect(self._send_button_pressed)

        self._layout.addWidget(publish_pose_button)

        self._setup()

    def _setup(self, source_frame="base_link", target_frame="map", timeout=2, output_topic="pose"):
        """
        Sets up the parameters with defaults
        :param source_frame: Source TF frame (defaults to base_link)
        :param target_frame: Target TF frame (defaults to map)
        :param timeout: Timeout of tf buffer (defaults to 2.0)
        :param output_topic: Output topic to publish the pose to (defaults to pose)
        """
        self._source_frame = source_frame
        self._target_frame = target_frame
        self._timeout = timeout
        self._pub = rospy.Publisher(output_topic, PoseStamped, queue_size=10)

    def _send_button_pressed(self):
        # Request transform between source and target frame
        """
        Triggered when button pressed, requests transform and publishes to the output topic
        """
        tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(tf_buffer)

        try:
            trans = tf_buffer.lookup_transform(self._target_frame, self._source_frame,
                                               rospy.Time(0), rospy.Duration(self._timeout))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            QMessageBox.warning(self._widget, "Transform lookup failed", str(e))
        else:
            self._pub.publish(PoseStamped(header=trans.header,
                                          pose=Pose(position=Point(
                                              x=trans.transform.translation.x,
                                              y=trans.transform.translation.y,
                                              z=trans.transform.translation.z
                                          ), orientation=trans.transform.rotation)))

    def save_settings(self, plugin_settings, instance_settings):
        """
        Saves the settings the perspective cfg
        """
        instance_settings.set_value("source_frame", self._source_frame)
        instance_settings.set_value("target_frame", self._target_frame)
        instance_settings.set_value("timeout", self._timeout)
        instance_settings.set_value("output_topic", self._pub.name)

    def restore_settings(self, plugin_settings, instance_settings):
        """
        Restores the settings the perspective cfg
        """
        source_frame = instance_settings.value("source_frame")
        target_frame = instance_settings.value("target_frame")
        timeout = instance_settings.value("timeout")
        output_topic = instance_settings.value("output_topic")
        if None not in [source_frame, target_frame, timeout, output_topic]:
            self._setup(source_frame, target_frame, float(timeout), output_topic)

    def trigger_configuration(self):
        """
        Triggered when the gear wheel configuration is triggered
        """
        self._setup(get_string_from_input(self._widget, "TF source frame", self._source_frame),
                    get_string_from_input(self._widget, "TF target frame", self._target_frame),
                    get_double_from_input(self._widget, "TF timeout", self._timeout, 0, 1e3),
                    get_string_from_input(self._widget, "Output topic", self._pub.name))
