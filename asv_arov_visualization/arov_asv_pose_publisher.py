import rclpy
import math
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Quaternion

def quaternion_from_euler(rpy) :
    quat = Rotation.from_euler("xyz", rpy, degrees=False).as_quat()
    out = Quaternion()
    out.x = float(quat[0])
    out.y = float(quat[1])
    out.z = float(quat[2])
    out.w = float(quat[3])
    return out

def euler_from_quaternion(quat) :
    return Rotation.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler("xyz", degrees=False)

def clamp(x, max_velocity) :
    return max(min(x, abs(max_velocity)), -abs(max_velocity))

def distance_xy(pose1, pose2) :
    return math.sqrt((pose1.position.y - pose2.position.y)**2 +  (pose1.position.x - pose2.position.x)**2)

def normalize(theta) :
    return (theta + math.pi) % (2 * math.pi) - math.pi

def at_targets(poses, targets, tolerance) :
    for i in range(len(poses)) :
        if abs(poses[i] - targets[i]) > tolerance :
            return False
    return True

class AROVASVPosePublisher(Node) :
    def __init__(self) :
        super().__init__('arov_asv_pose_publisher')
        self.arov_publisher = self.create_publisher(PoseWithCovarianceStamped, 'arov/amcl_pose', 10)
        self.asv_publisher = self.create_publisher(PoseWithCovarianceStamped, 'asv/amcl_pose', 10)

        # Simulation Params
        self.dt = 0.1 # Time between ticks, used for speeds and for clock rate
        self.top_of_net_radius = 10 # Radius of top net rim
        self.bottom_of_net_radius = 8 # Radius of bottom net rim
        self.net_height = 0 # z coordinate of top of cleaning surface
        self.net_depth = 10 # z coordinate of bottom of cleaning surface
        self.net_poses = [Pose(), Pose()] # Poses of centers of nets
        self.net_poses[1].position.x = -50
        self.arov_clearance = 2 # How far from the net the AROV tries to be while cleaning
        self.asv_clearance = 7 # How far from the net the ASV tries to be while cleaning
        self.arov_start_pose = Pose() # Where in the map the AROV starts
        self.arov_start_pose.position.x = 15
        self.arov_start_pose.position.y = 15
        self.asv_start_pose = Pose() # Where in the map the ASV starts
        self.asv_start_pose.position.x = 13
        self.asv_start_pose.position.y = 13
        self.desired_linear_speed = 1 * self.dt # The maximum magnitude of motion in the xy plane
        self.desired_dive_speed = 1 * self.dt # The maximum magnitude of motion in the z axis
        self.desired_angular_speed = 3 * self.dt # The maximum magnitude of change in yaw
        self.cleaning_fidelity = 5 # Width of AROV's horiztonal cleaning movements, probably want this to be equal to the width of the AROV for final visualization
        self.cycles = 10 # Number of pairs of vertical strips the AROV will clean on each net
        self.linear_tolerance = 0.2 # How close to desired position in xyz coordinates movement commands will stop at
        self.angular_tolerance = 0.1 # How close to desired yaw movement commands will stop at

        # OpenCV Visualization Params
        self.img_x = 1000
        self.img_y = 1000
        self.arrow_length = 3
        self.coord_scale = 5
        self.do_cv_visualization = False

        # Instance Variables
        self.arov_poses = [self.arov_start_pose]
        self.asv_poses = [self.asv_start_pose]
        self.pose_id = 0
        self.dive_rise_switch = -1

        for n in self.net_poses :
            desired_theta = normalize(math.atan2(n.position.y - self.asv_poses[-1].position.y, n.position.x - self.asv_poses[-1].position.x))
            self.asv_turn_to_yaw(normalize(desired_theta))
            self.asv_drive_distance(distance_xy(self.asv_poses[-1], n) - self.top_of_net_radius - self.asv_clearance, True)
            self.asv_turn_to_yaw(normalize(desired_theta + math.pi / 2))
            for i in range(self.cycles) :
                dx = n.position.x - self.arov_poses[-1].position.x
                dy = n.position.y - self.arov_poses[-1].position.y
                r1 = (self.top_of_net_radius if self.dive_rise_switch == -1 else self.bottom_of_net_radius) + self.arov_clearance
                r2 = (self.top_of_net_radius if self.dive_rise_switch == 1 else self.bottom_of_net_radius) + self.arov_clearance
                theta = math.atan2(dy, dx)
                psi = normalize((theta + math.pi / 2) - math.asin(self.cleaning_fidelity / 2 / r1))
                arov_distance1 = distance_xy(self.arov_poses[-1], n) - r1
                arov_distance2 = distance_xy(self.arov_poses[-1], n) - r2
                new_arov_pose = Pose()
                new_arov_pose.position.x = self.arov_poses[-1].position.x + arov_distance1 * math.cos(theta)
                new_arov_pose.position.y = self.arov_poses[-1].position.y + arov_distance1 * math.sin(theta)
                new_arov_pose.position.z = self.arov_poses[-1].position.z
                current_orientation = euler_from_quaternion(self.arov_poses[-1].orientation)
                new_arov_pose.orientation = quaternion_from_euler([current_orientation[0], current_orientation[1], psi - math.pi / 2])
                self.arov_drive_to_pose(new_arov_pose)
                new_arov_pose2 = Pose()
                new_arov_pose2.position.x = self.arov_poses[-1].position.x + arov_distance2 * math.cos(theta)
                new_arov_pose2.position.y = self.arov_poses[-1].position.y + arov_distance2 * math.sin(theta)
                new_arov_pose2.position.z = self.net_depth if self.dive_rise_switch == -1 else self.net_height
                new_arov_pose2.orientation.x = new_arov_pose.orientation.x
                new_arov_pose2.orientation.y = new_arov_pose.orientation.y
                new_arov_pose2.orientation.z = new_arov_pose.orientation.z
                new_arov_pose2.orientation.w = new_arov_pose.orientation.w
                self.dive_rise_switch = -1 if self.dive_rise_switch == 1 else 1
                self.arov_drive_to_pose(new_arov_pose2)
                new_arov_pose3 = Pose()
                new_arov_pose3.position.x = new_arov_pose.position.x + self.cleaning_fidelity * math.cos(psi)
                new_arov_pose3.position.y = new_arov_pose.position.y + self.cleaning_fidelity * math.sin(psi)
                new_arov_pose3.position.z = new_arov_pose.position.z
                new_arov_pose3.orientation = quaternion_from_euler([current_orientation[0], current_orientation[1], psi - math.pi / 2])
                self.arov_drive_to_pose(new_arov_pose3)
                self.asv_turn_to_yaw(psi)
                self.asv_drive_distance((self.net_radius + self.asv_clearance) / (r1 if self.dive_rise_switch == -1 else r2) * self.cleaning_fidelity, False)
            final_rise_pose = Pose()
            final_rise_pose.position.x = self.arov_poses[-1].position.x
            final_rise_pose.position.y = self.arov_poses[-1].position.y
            final_rise_pose.position.z = self.net_height
            final_rise_pose.orientation.x = self.arov_poses[-1].orientation.x
            final_rise_pose.orientation.y = self.arov_poses[-1].orientation.y
            final_rise_pose.orientation.z = self.arov_poses[-1].orientation.z
            final_rise_pose.orientation.w = self.arov_poses[-1].orientation.w
            self.arov_drive_to_pose(final_rise_pose)
        if self.do_cv_visualization :
            for i in range(len(self.asv_poses)) :
                frame = np.zeros(shape=(self.img_y, self.img_x, 3), dtype=np.uint8)
                asv_yaw = euler_from_quaternion(self.asv_poses[i].orientation)[2]
                arov_yaw = euler_from_quaternion(self.arov_poses[i].orientation)[2]
                asv_p1 = (int(self.coord_scale * self.asv_poses[i].position.y + self.img_y / 2), int(self.coord_scale * self.asv_poses[i].position.x + self.img_x / 2))
                arov_p1 = (int(self.coord_scale * self.arov_poses[i].position.y + self.img_y / 2), int(self.coord_scale * self.arov_poses[i].position.x + self.img_x / 2))
                asv_p2 = (int(self.coord_scale * (self.asv_poses[i].position.y + self.arrow_length * math.sin(asv_yaw)) + self.img_y / 2), int(self.coord_scale * (self.asv_poses[i].position.x + self.arrow_length * math.cos(asv_yaw)) + self.img_x / 2))
                arov_p2 = (int(self.coord_scale * (self.arov_poses[i].position.y + self.arrow_length * math.sin(arov_yaw)) + self.img_y / 2), int(self.coord_scale * (self.arov_poses[i].position.x + self.arrow_length * math.cos(arov_yaw)) + self.img_x / 2))
                cv2.arrowedLine(frame, asv_p1, asv_p2, (0, 0, 255), 2)
                cv2.arrowedLine(frame, arov_p1, arov_p2, (0, 255, 0), 2)
                for n in self.net_poses :
                    cv2.circle(frame, (int(self.coord_scale * n.position.y + self.img_y / 2), int(self.coord_scale * n.position.x + self.img_x / 2)), int(self.coord_scale * self.net_radius), (255, 255, 255), 3)
                cv2.imshow('Cleaning Sim', frame)
                if cv2.waitKey(10) & 0xFF == ord('b') :
                    break
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def arov_drive_to_pose(self, pose) :
        movement_direction = math.atan2(pose.position.y - self.arov_poses[-1].position.y, pose.position.x - self.arov_poses[-1].position.x)
        current_orientation = euler_from_quaternion(self.arov_poses[-1].orientation)
        target_orientation = euler_from_quaternion(pose.orientation)
        while not at_targets([self.arov_poses[-1].position.x, self.arov_poses[-1].position.y, self.arov_poses[-1].position.z], [pose.position.x, pose.position.y, pose.position.z], self.linear_tolerance) or not at_targets([current_orientation[2]], [target_orientation[2]], self.angular_tolerance) :
            dx = clamp(pose.position.x - self.arov_poses[-1].position.x, self.desired_linear_speed * math.cos(movement_direction))
            dy = clamp(pose.position.y - self.arov_poses[-1].position.y, self.desired_linear_speed * math.sin(movement_direction))
            dz = clamp(pose.position.z - self.arov_poses[-1].position.z, self.desired_dive_speed)
            current_orientation = euler_from_quaternion(self.arov_poses[-1].orientation)
            target_orientation = euler_from_quaternion(pose.orientation)
            dtheta = clamp(normalize(target_orientation[2] - current_orientation[2]), self.desired_angular_speed)
            new_pose = Pose()
            new_pose.position.x = self.arov_poses[-1].position.x + dx
            new_pose.position.y = self.arov_poses[-1].position.y + dy
            new_pose.position.z = self.arov_poses[-1].position.z + dz
            new_pose.orientation = quaternion_from_euler([current_orientation[0], current_orientation[1], current_orientation[2] + dtheta])
            self.arov_poses.append(new_pose)
            self.asv_poses.append(self.asv_poses[-1])

    def asv_drive_distance(self, distance, follow) :
        current_orientation = euler_from_quaternion(self.asv_poses[-1].orientation)
        desired_pose = Pose()
        desired_pose.position.x = self.asv_poses[-1].position.x + distance * math.cos(current_orientation[2])
        desired_pose.position.y = self.asv_poses[-1].position.y + distance * math.sin(current_orientation[2])
        while not at_targets([self.asv_poses[-1].position.x, self.asv_poses[-1].position.y], [desired_pose.position.x, desired_pose.position.y], self.linear_tolerance) :
            dx = clamp(desired_pose.position.x - self.asv_poses[-1].position.x, self.desired_linear_speed * math.cos(current_orientation[2]))
            dy = clamp(desired_pose.position.y - self.asv_poses[-1].position.y, self.desired_linear_speed * math.sin(current_orientation[2]))
            new_asv_pose = Pose()
            new_asv_pose.position.x = self.asv_poses[-1].position.x + dx
            new_asv_pose.position.y = self.asv_poses[-1].position.y + dy
            new_asv_pose.position.z = self.asv_poses[-1].position.z
            new_asv_pose.orientation = self.asv_poses[-1].orientation
            if follow :
                new_arov_pose = Pose()
                new_arov_pose.position.x = self.arov_poses[-1].position.x + dx
                new_arov_pose.position.y = self.arov_poses[-1].position.y + dy
                new_arov_pose.position.z = self.arov_poses[-1].position.z
                new_arov_pose.orientation = self.arov_poses[-1].orientation
            else :
                new_arov_pose = self.arov_poses[-1]
            self.asv_poses.append(new_asv_pose)
            self.arov_poses.append(new_arov_pose)

    def asv_turn_to_yaw(self, desired_theta) :
        current_orientation = euler_from_quaternion(self.asv_poses[-1].orientation)
        current_theta = current_orientation[2]
        while not at_targets([current_theta], [desired_theta], self.angular_tolerance) :
            dtheta = clamp(normalize(desired_theta - current_theta), self.desired_angular_speed)
            new_pose = Pose()
            new_pose.position.x = self.asv_poses[-1].position.x
            new_pose.position.y = self.asv_poses[-1].position.y
            new_pose.position.z = self.asv_poses[-1].position.z
            new_pose.orientation = quaternion_from_euler([current_orientation[0], current_orientation[1], current_orientation[2] + dtheta])
            self.arov_poses.append(self.arov_poses[-1])
            self.asv_poses.append(new_pose)
            current_orientation = euler_from_quaternion(self.asv_poses[-1].orientation)
            current_theta = current_orientation[2]
            
    def timer_callback(self) :
        asv_pwcs = PoseWithCovarianceStamped()
        asv_pwcs.pose.pose = self.asv_poses[self.pose_id]
        arov_pwcs = PoseWithCovarianceStamped()
        arov_pwcs.pose.pose = self.arov_poses[self.pose_id]
        self.asv_publisher.publish(asv_pwcs)
        self.arov_publisher.publish(arov_pwcs)
        if self.do_cv_visualization :
            frame = np.zeros(shape=(self.img_y, self.img_x, 3), dtype=np.uint8)
            asv_yaw = euler_from_quaternion(self.asv_poses[self.pose_id].orientation)[2]
            arov_yaw = euler_from_quaternion(self.arov_poses[self.pose_id].orientation)[2]
            asv_p1 = (int(self.coord_scale * self.asv_poses[self.pose_id].position.y + self.img_y / 2), int(self.coord_scale * self.asv_poses[self.pose_id].position.x + self.img_x / 2))
            arov_p1 = (int(self.coord_scale * self.arov_poses[self.pose_id].position.y + self.img_y / 2), int(self.coord_scale * self.arov_poses[self.pose_id].position.x + self.img_x / 2))
            asv_p2 = (int(self.coord_scale * (self.asv_poses[self.pose_id].position.y + self.arrow_length * math.sin(asv_yaw)) + self.img_y / 2), int(self.coord_scale * (self.asv_poses[self.pose_id].position.x + self.arrow_length * math.cos(asv_yaw)) + self.img_x / 2))
            arov_p2 = (int(self.coord_scale * (self.arov_poses[self.pose_id].position.y + self.arrow_length * math.sin(arov_yaw)) + self.img_y / 2), int(self.coord_scale * (self.arov_poses[self.pose_id].position.x + self.arrow_length * math.cos(arov_yaw)) + self.img_x / 2))
            cv2.arrowedLine(frame, asv_p1, asv_p2, (0, 0, 255), 2)
            cv2.arrowedLine(frame, arov_p1, arov_p2, (0, 255, 0), 2)
            for n in self.net_poses :
                cv2.circle(frame, (int(self.coord_scale * n.position.y + self.img_y / 2), int(self.coord_scale * n.position.x + self.img_x / 2)), int(self.coord_scale * self.net_radius), (255, 255, 255), 3)
            cv2.imshow('Cleaning Sim', frame)
        self.pose_id += 1
        if len(self.asv_poses) <= self.asv_pose_id or (self.do_cv_visualization and cv2.waitKey(10) & 0xFF == ord('b')) :
           self.timer.destroy()

def main(args=None) :
    rclpy.init()
    node = AROVASVPosePublisher()
    rclpy.spin(node)

if __name__ == '__main__' :
    main()