import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from time import sleep

class NaoWaveNode(Node):
    def __init__(self):
        super().__init__('nao_wave_node')
        # Publishers for left arm joints (adjust for right if needed)
        self.pub_shoulder_pitch = self.create_publisher(Float64, '/LShoulderPitch/position', 10)
        self.pub_shoulder_roll = self.create_publisher(Float64, '/LShoulderRoll/position', 10)
        self.pub_elbow_yaw = self.create_publisher(Float64, '/LElbowYaw/position', 10)
        self.pub_elbow_roll = self.create_publisher(Float64, '/LElbowRoll/position', 10)
        self.pub_wrist_yaw = self.create_publisher(Float64, '/LWristYaw/position', 10)

    def publish_joint(self, joint_pub, angle):
        msg = Float64()
        msg.data = angle
        joint_pub.publish(msg)
        self.get_logger().info(f'Published {angle} to {joint_pub.topic}')

    def perform_gesture(self, gesture):
        if gesture == 'wave':
            # Wave: Raise arm, oscillate elbow roll
            self.publish_joint(self.pub_shoulder_pitch, -1.4)  # Raise (within -2.08 to 2.08)
            self.publish_joint(self.pub_shoulder_roll, 1.0)    # Out ( -0.31 to 1.32)
            self.publish_joint(self.pub_elbow_yaw, 0.25)      # Slight out ( -2.09 to 1.16)
            for _ in range(3):  # Oscillate 3 times
                self.publish_joint(self.pub_elbow_roll, -0.5)  # Bend ( -1.54 to -0.03)
                sleep(0.5)
                self.publish_joint(self.pub_elbow_roll, -1.0)
                sleep(0.5)
            self.reset_arm()
        elif gesture == 'point':
            # Point: Extend arm forward
            self.publish_joint(self.pub_shoulder_pitch, 0.0)
            self.publish_joint(self.pub_shoulder_roll, 0.0)
            self.publish_joint(self.pub_elbow_yaw, -1.0)      # Rotate out
            self.publish_joint(self.pub_elbow_roll, -0.03)    # Almost straight
            self.publish_joint(self.pub_wrist_yaw, 0.0)
            sleep(2.0)
            self.reset_arm()
        elif gesture == 'grasp':
            # Grasp motion: Bend elbow, rotate wrist
            self.publish_joint(self.pub_shoulder_pitch, -0.5)
            self.publish_joint(self.pub_shoulder_roll, 0.3)
            self.publish_joint(self.pub_elbow_yaw, 0.0)
            self.publish_joint(self.pub_elbow_roll, -1.2)     # Bend to grasp
            self.publish_joint(self.pub_wrist_yaw, 1.0)       # Rotate for grasp
            sleep(2.0)
            self.reset_arm()

    def reset_arm(self):
        # Reset to zero/relax
        self.publish_joint(self.pub_shoulder_pitch, 0.0)
        self.publish_joint(self.pub_shoulder_roll, 0.0)
        self.publish_joint(self.pub_elbow_yaw, 0.0)
        self.publish_joint(self.pub_elbow_roll, 0.0)
        self.publish_joint(self.pub_wrist_yaw, 0.0)
        sleep(1.0)

def main(args=None):
    rclpy.init(args=args)
    node = NaoWaveNode()
    gestures = ['wave', 'point', 'grasp']
    for g in gestures:
        node.perform_gesture(g)
        sleep(2.0)  # Pause between
    rclpy.shutdown()

if __name__ == '__main__':
    main()