import rclpy
from rclpy.node import Node
from nemo_interfaces.msg import RovCommands
from std_msgs.msg import Bool
import pygame
from joystick_pacs.calibrationNode import JoystickCalibrationNode
from geometry_msgs.msg import Twist

import pygame


class joystickReadWrapper:
    def __init__(self,controller_id):
        pygame.init()
        pygame.joystick.init()

        self.last_button_states = {}
        self.toggle_states = {}
        
        self.controller = pygame.joystick.Joystick(controller_id)
        if not self.controller.get_init():
            self.controller.init()

        self.pause_read = False

    def readAxis(self,axis_id):
        rawAxisval = None
        try:
            pygame.event.pump()
            rawAxisval = self.controller.get_axis(axis_id)
            return rawAxisval
        except self.pause_read:
            pass

    def buttonPressedOnce(self, button_id):
        """
        Returns True ONLY on a 0 → 1 transition.
        """
        pygame.event.pump()

        current_state = self.controller.get_button(button_id)
        prev_state = self.last_button_states.get(button_id, 0)

        pressed_once = (current_state == 1 and prev_state == 0)

        self.last_button_states[button_id] = current_state
        return pressed_once

#ros2 master Node for Joystick
class joyMasterNode(Node):
    def __init__(self):
        super().__init__("joystickPublisher")

        #manual loading of calibrated parameters using 
        self.declare_parameter('surgeInput', [1.0, 0.019, -1.0])
        self.declare_parameter('yawInput', [0.99, -0.01, -1.00])
        self.declare_parameter('heaveInput', [0.99, 0.0, -0.99])
        self.declare_parameter('swayInput', [-1.00, 0.00, 0.99])

        #provision to read output parameters from a yaml file
        self.declare_parameter('surgeVelVector', [-1.00,0.00,1.00]) 
        self.declare_parameter('yawVelVector', [-1.00,0.00,1.00]) #in degrees/sec
        self.declare_parameter('heaveVelVector', [-1.00,0.00,1.00]) #dive / surface velocity
        self.declare_parameter('swayVelVector', [-1.00,0.00,1.00])

        self.surgeInVector = self.get_parameter('surgeInput').value
        self.yawInVector = self.get_parameter('yawInput').value
        self.heaveInVector = self.get_parameter('heaveInput').value
        self.swayInVector = self.get_parameter('swayInput').value

        #parametrizing for further Updates
        self.inputAxisVectorMap = {
            'SURGE':self.surgeInVector,
            'YAW':self.yawInVector,
            'HEAVE':self.heaveInVector,
            'SWAY':self.swayInVector
        }

        self.surgeOutVector = self.get_parameter('surgeVelVector').value
        self.yawOutVector = self.get_parameter('yawVelVector').value
        self.heaveOutVector = self.get_parameter('heaveVelVector').value
        self.swayOutVector = self.get_parameter('swayVelVector').value

        self.outputAxisVectorMap = {
            'SURGE':self.surgeOutVector,
            'YAW':self.yawOutVector,
            'HEAVE':self.heaveOutVector,
            'SWAY':self.swayOutVector
        }

        #change axis assignements pre deployements - primitive as of now
        self.invertStatus = {'SURGE':False,'HEAVE':False,'SWAY':False,"YAW":False}
        self.axisIdMap = {0:'YAW',1:'SURGE',3:'SWAY',4:'HEAVE'}
        self.startValuePublisher = False
        self.commandPublisher = self.create_publisher(Twist,'nemo_auv/cmd_vel_raw',10)
        self.nemo_command_publisher = self.create_publisher(RovCommands,'/nemo_auv/input_cmd',10)
        self.depth_lock_publisher = self.create_publisher(Bool,'/nemo_auv/depth_lock',10)
        self.commandTimer = self.create_timer(0.1,self.publish_cmd)
        self.buttonCommandTimer = self.create_timer(0.1, self.button_publisher_timer)

        self.depth_lock_status = False
        self.button_hold_start = None
        self.hold_threshold = 3.0   # seconds
        self.hold_triggered = False

        self.joyWrapper = joystickReadWrapper(0)

        self.userInput = input("Enter Y if you want to start Calibration and N if you don't")
        if self.userInput == "Y":
            try:
                calibrationNode = JoystickCalibrationNode()
                self.startValuePublisher = True
            except:
                return
        else:
            self.startValuePublisher = True

    #publish function
    def publish_cmd(self):
        if not self.startValuePublisher:
            # Skip publishing
            return
        
        msg = Twist()
        msg_command = RovCommands()
        msg.linear.x = float(self.normalizeJoystickInput(1))
        msg.angular.z = float(self.normalizeJoystickInput(0))/2.00
        msg.linear.z = float(self.normalizeJoystickInput(4))
        msg.angular.x = float(0)
        msg.angular.y = float(0)
        msg.linear.y = float(self.normalizeJoystickInput(3))

        msg_command.surge = float(self.normalizeJoystickInput(1))
        msg_command.sway = float(self.normalizeJoystickInput(0))
        msg_command.yaw = float(self.normalizeJoystickInput(3))
        msg_command.heave = float(self.normalizeJoystickInput(4))
        self.commandPublisher.publish(msg)
        self.nemo_command_publisher.publish(msg_command)
        self.get_logger().info(
            f"Publishing: S={msg.linear.x:.2f}, SW={msg.linear.y:.2f}, H={msg.linear.z:2f}, Y={msg.angular.z:.2f}"
        )
        

    def mapFunc(self,mappingVar,axis_id):
        axis_name = self.axisIdMap.get(axis_id)
        axis_inversion_status = self.invertStatus.get(axis_name)
        input_calibrated_vector = self.inputAxisVectorMap.get(axis_name)
        output_velocity_vector = self.outputAxisVectorMap.get(axis_name)

        if axis_inversion_status is True:
            input_calibrated_vector = input_calibrated_vector[::-1]
            mappingVar = mappingVar * -1.00
        
        mean_in = input_calibrated_vector[1]
        mean_out = output_velocity_vector[1]

        max_in = input_calibrated_vector[2]
        max_out = output_velocity_vector[2]

        min_in = input_calibrated_vector[0]
        min_out = output_velocity_vector[0]
        
        if mappingVar < mean_in:
           mappedVal = ((mappingVar - mean_in) / (min_in - mean_in)) * (min_out - mean_out)
        else:
           mappedVal = ((mappingVar - mean_in) / (max_in - mean_in)) * (max_out - mean_out)

        return mappedVal

    def normalizeJoystickInput(self,axis_id):
        rawAxisVal = self.joyWrapper.readAxis(axis_id)
        if abs(rawAxisVal) < 0.1:
            rawAxisVal = 0
            
        normalizedAxisVal = self.mapFunc(rawAxisVal,axis_id)
        if abs(normalizedAxisVal) < 0.1:
            normalizedAxisVal = 0
        

        return normalizedAxisVal

    def button_publisher_timer(self):
        if self.joyWrapper.buttonPressedOnce(3):
            self.depth_lock_status = not self.depth_lock_status
            msg = Bool()
            print(f"Depth Lock Status is f{self.depth_lock_status}")
            msg.data = self.depth_lock_status
            self.depth_lock_publisher.publish(msg)

            
        
           


def main(args=None):
    rclpy.init(args=args)

    node = joyMasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()