import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from cube_interfaces.srv import GetClosestCube


class CubeTrackerNode(Node):
    def __init__(self):
        super().__init__('cube_tracker_node')

        # Declare and retrieve the NUM_CUBES parameter (defaulting to 5 if not passed)
        self.declare_parameter('NUM_CUBES', 5)
        self.num_cubes = self.get_parameter('NUM_CUBES').get_parameter_value().integer_value

        # Skladište za pozicije kocki
        self.cube_positions = {}
        self.threshold = 0.04  # threshold in meters
        
        # Skup za praćenje kocki čije smo koordinate u svetu već ispisali (da se ispišu samo jednom)
        self.logged_cubes = set()

        self.get_logger().info(f'Starting tracker for {self.num_cubes} cubes with single-print world coordinate logging.')

        # Dynamically build subscribers based on the parameter count
        for i in range(self.num_cubes):
            cube_id = f'cube_{i}'
            topic_name = f'/model/{cube_id}/pose'

            self.create_subscription(
                TransformStamped,
                topic_name,
                lambda msg, cid=cube_id: self.cube_pose_callback(msg, cid),
                10
            )

        # Initialize the service server
        self.srv = self.create_service(
            GetClosestCube,
            'get_closest_cube',
            self.get_closest_cube_callback
        )
        self.get_logger().info('Service "/get_closest_cube" is online and listening.')

    def cube_pose_callback(self, msg, cube_id):
        """Stores the latest translation coordinates for the specific cube and logs them once valid."""
        world_x = msg.transform.translation.x
        world_y = msg.transform.translation.y
        world_z = msg.transform.translation.z

        # Ignorišemo nulte pozicije koje Gazebo šalje pre inicijalizacije fizike
        if world_x == 0.0 and world_y == 0.0 and world_z == 0.0:
            return

        # Ispis čistih svetskih koordinata iz Gazeba samo jednom za svaku kocku
        if cube_id not in self.logged_cubes:
            self.get_logger().info(f"[SVETSKI SISTEM - PRAVI] {cube_id} -> X: {world_x:.3f}, Y: {world_y:.3f}, Z: {world_z:.3f}")
            self.logged_cubes.add(cube_id)

        # Čuvamo direktno poziciju (već je u svetskim koordinatama iz Gazeba)
        self.cube_positions[cube_id] = msg.transform.translation

    def get_closest_cube_callback(self, request, response):
        """Processes incoming service queries to find the nearest tracked cube without any rotation."""
        target = request.target_pose.position
        closest_id = ""
        min_dist = float('inf')

        # Ensure we actually have received data before computing
        if not self.cube_positions:
            response.cube_id = ""
            response.found = False
            self.get_logger().warn("Service requested, but no cube pose data has been received yet!")
            return response

        # Compute Euclidean distance to all tracked cubes directly
        for cube_id, pos in self.cube_positions.items():
            dist = math.sqrt(
                (target.x - pos.x)**2 +
                (target.y - pos.y)**2 +
                (target.z - pos.z)**2
            )
            if dist < min_dist:
                min_dist = dist
                closest_id = cube_id

        # Evaluate findings against threshold constraint
        if min_dist <= self.threshold:
            response.cube_id = closest_id
            response.found = True
            matched_pos = self.cube_positions[closest_id]
            self.get_logger().info(f"Target found! Closest: {closest_id} (Distance: {min_dist:.3f}m) na poziciji: X={matched_pos.x:.3f}, Y={matched_pos.y:.3f}")
        else:
            response.cube_id = ""
            response.found = False
            self.get_logger().info(f"No cube within threshold. Nearest was {closest_id} at {min_dist:.3f}m")

        return response


def main(args=None):
    rclpy.init(args=args)
    node = CubeTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
