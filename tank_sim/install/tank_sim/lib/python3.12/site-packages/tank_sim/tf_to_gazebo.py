import rclpy
from rclpy.node import Node
import tf2_ros
import gz.transport
import gz.msgs

class TFToGazebo(Node):
    def __init__(self):
        super().__init__('tf_to_gazebo')

        self.link_name = 'arov_link'
        self.model_name = 'arov'
        self.world_name = 'tank_world'
        self.parent_frame = 'world'

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.gz_node = gz.transport.Node()
        self.publisher = self.gz_node.advertise(
            f"/world/{self.world_name}/model/{self.model_name}/link/{self.link_name}/pose",
            "gz.msgs.Pose"
        )

        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.parent_frame,
                self.link_name,
                rclpy.time.Time()
            )

            pose_msg = gz.msgs.Pose()
            pose_msg.position.x = tf.transform.translation.x
            pose_msg.position.y = tf.transform.translation.y
            pose_msg.position.z = tf.transform.translation.z
            pose_msg.orientation.x = tf.transform.rotation.x
            pose_msg.orientation.y = tf.transform.rotation.y
            pose_msg.orientation.z = tf.transform.rotation.z
            pose_msg.orientation.w = tf.transform.rotation.w

            self.publisher.publish(pose_msg)
        except Exception as e:
            self.get_logger().warn(f"TF not available: {e}")

def main():
    rclpy.init()
    node = TFToGazebo()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
