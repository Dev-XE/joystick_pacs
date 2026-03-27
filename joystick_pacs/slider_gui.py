#!/usr/bin/env python3
import sys
import signal

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QPushButton,
    QVBoxLayout,
    QLabel,
    QSlider,
    QCheckBox,
)
from PyQt5.QtCore import QTimer, Qt


class ArmGuiNode(Node):
    def __init__(self):
        super().__init__('arm_gui_node')

        # Arm publisher
        self.arm_pub = self.create_publisher(Bool, '/nemo_auv/arm', 10)

        # Cmd_vel publisher and raw subscriber
        self.cmd_vel_pub = self.create_publisher(Twist, '/nemo_auv/cmd_vel', 10)
        self.cmd_vel_raw_sub = self.create_subscription(
            Twist,
            '/nemo_auv/cmd_vel_raw',
            self.raw_cmd_vel_callback,
            10
        )

        # GUI-controlled state
        self.heave_override_enabled = False
        self.heave_value = 0.0

        self.get_logger().info('Arm/Disarm GUI node started')

    def publish_arm(self, state: bool):
        msg = Bool()
        msg.data = state
        self.arm_pub.publish(msg)
        self.get_logger().info(f'Arm state published: {state}')

    def set_heave_override_enabled(self, enabled: bool):
        self.heave_override_enabled = enabled
        self.get_logger().info(f'Heave override enabled: {enabled}')

    def set_heave_value(self, value: float):
        self.heave_value = value
        self.get_logger().info(f'Heave slider value set to: {value:.2f}')

    def raw_cmd_vel_callback(self, msg: Twist):
        out_msg = Twist()

        # Pass through everything by default
        out_msg.linear.x = msg.linear.x
        out_msg.linear.y = msg.linear.y
        out_msg.linear.z = msg.linear.z

        out_msg.angular.x = msg.angular.x
        out_msg.angular.y = msg.angular.y
        out_msg.angular.z = msg.angular.z

        # Override only linear.z if checkbox is enabled
        if self.heave_override_enabled:
            out_msg.linear.z = self.heave_value

        self.cmd_vel_pub.publish(out_msg)


class ArmWindow(QWidget):
    def __init__(self, ros_node: ArmGuiNode):
        super().__init__()
        self.node = ros_node
        self.armed = False

        self.setWindowTitle("AUV Arm / Disarm + Heave Control")
        self.setMinimumSize(400, 300)

        # Arm button
        self.button = QPushButton("DISARMED")
        self.button.setStyleSheet(
            "font-size: 28px; padding: 30px; background-color: red; color: white;"
        )
        self.button.clicked.connect(self.toggle_arm)

        # Heave label
        self.heave_label = QLabel("Heave: 0.00")
        self.heave_label.setStyleSheet("font-size: 20px;")

        # Heave slider: map -100..100 to -1.0..1.0
        self.heave_slider = QSlider(Qt.Horizontal)
        self.heave_slider.setMinimum(-100)
        self.heave_slider.setMaximum(100)
        self.heave_slider.setValue(0)
        self.heave_slider.setTickInterval(10)
        self.heave_slider.setTickPosition(QSlider.TicksBelow)
        self.heave_slider.valueChanged.connect(self.on_slider_changed)

        # Override checkbox
        self.override_checkbox = QCheckBox("Override linear.z with slider")
        self.override_checkbox.setStyleSheet("font-size: 16px;")
        self.override_checkbox.stateChanged.connect(self.on_override_changed)

        # Layout
        layout = QVBoxLayout()
        layout.addWidget(self.button)
        layout.addWidget(self.override_checkbox)
        layout.addWidget(self.heave_label)
        layout.addWidget(self.heave_slider)
        self.setLayout(layout)

    def toggle_arm(self):
        self.armed = not self.armed
        self.node.publish_arm(self.armed)

        if self.armed:
            self.button.setText("ARMED")
            self.button.setStyleSheet(
                "font-size: 28px; padding: 30px; background-color: green; color: white;"
            )
        else:
            self.button.setText("DISARMED")
            self.button.setStyleSheet(
                "font-size: 28px; padding: 30px; background-color: red; color: white;"
            )

    def on_slider_changed(self, value: int):
        heave = value / 100.0
        self.heave_label.setText(f"Heave: {heave:.2f}")
        self.node.set_heave_value(heave)

    def on_override_changed(self, state: int):
        enabled = state == Qt.Checked
        self.node.set_heave_override_enabled(enabled)


def main(args=None):
    rclpy.init(args=args)
    node = ArmGuiNode()

    app = QApplication(sys.argv)
    window = ArmWindow(node)
    window.show()

    # ROS spin inside Qt loop
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    ros_timer.start(10)

    # Handle Ctrl+C -> close Qt cleanly
    def handle_sigint(signum, frame):
        app.quit()

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        exit_code = app.exec_()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()