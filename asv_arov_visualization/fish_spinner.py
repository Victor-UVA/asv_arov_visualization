import rclpy
import math
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Quaternion

def quaternion_from_euler(rpy) :
    quat = Rotation.from_euler("xyz", rpy, degrees=False).as_quat()
    out = Quaternion()
    out.x = float(quat[0])
    out.y = float(quat[1])
    out.z = float(quat[2])
    out.w = float(quat[3])
    return out

class FishSpinner(Node) :
    def __init__(self) :
        super.__init__('fish_spinner')

        self.declare_parameter('pose_topic', '/amcl_pose')
        self.declare_parameter('start_yaw', 0)

        # Simulation Params
        self.fish_pose_topic: str = self.get_parameter('pose_topic').get_parameter_value().string_value # topic to publish fish poses to, implemented as ROS param in case you need to run multiple of this node
        self.fish_pose = Pose() # fish start pose, don't touch orientation I'm treating it weirdly
        self.euler_roll = 0
        self.euler_pitch = 0
        self.start_euler_yaw: float = self.get_parameter('start_yaw').get_parameter_value().double_value # start yaw in radians, implemented as ROS param in case you want different start yaws between fish nets
        self.dt = 0.1 # time between ticks
        self.spin_rate = 1 * self.dt # spin rate in radians

        self.euler_yaw = self.start_euler_yaw

        self.publisher = self.create_publisher(PoseWithCovarianceStamped, self.fish_pose_topic, 10)

        self.create_timer(self.dt, self.timer_callback)

    def timer_callback(self) :
        self.euler_yaw += self.spin_rate
        self.euler_yaw = (self.euler_yaw + math.pi) % (2 * math.pi) - math.pi
        new_fish_pose = Pose()
        new_fish_pose.position = self.fish_pose.position
        new_fish_pose.orientation = quaternion_from_euler([self.euler_roll, self.euler_pitch, self.euler_yaw])
        out_pwcs = PoseWithCovarianceStamped()
        out_pwcs.pose.pose = new_fish_pose
        out_pwcs.header.frame_id = "map"
        self.publisher.publish(out_pwcs)

def main(args=None) :
    rclpy.init()
    node = FishSpinner()
    rclpy.spin(node)

if __name__ == "__main__" :
    main()