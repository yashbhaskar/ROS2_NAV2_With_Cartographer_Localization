import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatusArray

class CombinedPathPublisher(Node):
    def __init__(self):
        super().__init__('combined_path_publisher')
        self.sub_path = self.create_subscription(Path, '/plan', self.path_callback, 10)
        self.sub_status = self.create_subscription(GoalStatusArray, '/waypoint_follower/_action/status', self.status_callback, 10)
        self.pub = self.create_publisher(Path, '/combined_path', 10)

        self.all_waypoints = []
        self.last_goal = None
        self.active_mission = False

    def path_callback(self, msg):
        if not msg.poses:
            return

        # Get goal pose (last point in current plan)
        current_goal = msg.poses[-1].pose

        # Add new goal only if it's unique (ignore replans)
        if self.last_goal is None or self._distance(self.last_goal, current_goal) > 0.1:
            self.get_logger().info('New unique goal added to path')
            self.all_waypoints.append(msg.poses[-1])
            self.last_goal = current_goal
            self.active_mission = True  # A new mission started

        # Publish combined path
        combined = Path()
        combined.header.frame_id = msg.header.frame_id
        combined.header.stamp = self.get_clock().now().to_msg()
        combined.poses = self.all_waypoints
        self.pub.publish(combined)

    def status_callback(self, msg):
        # Check if all goals are done
        if len(msg.status_list) == 0 and self.active_mission:
            self.get_logger().info('✅ All waypoints reached. Clearing path...')
            self.all_waypoints = []
            self.last_goal = None
            self.active_mission = False

            # Publish empty path to clear RViz display
            empty_path = Path()
            empty_path.header.frame_id = 'map'
            empty_path.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(empty_path)

    def _distance(self, p1, p2):
        dx = p1.position.x - p2.position.x
        dy = p1.position.y - p2.position.y
        return (dx**2 + dy**2) ** 0.5

def main(args=None):
    rclpy.init(args=args)
    node = CombinedPathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
