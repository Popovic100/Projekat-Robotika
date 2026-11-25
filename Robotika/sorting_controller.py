#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PointStamped, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Empty
from cube_interfaces.srv import GetClosestCube


class StateMachineSortingController(Node):

    def __init__(self):
        super().__init__('state_machine_sorting_controller')

        self.cb_group = ReentrantCallbackGroup()

        self.detected_cubes = {"red": [], "green": [], "blue": []}
        self.surface_positions = {"red": None, "green": None, "blue": None}
        self.cube_counts_on_surface = {"red": 0, "green": 0, "blue": 0}

        self.input_collection_locked = False

        # Ofset baze robota iz projektnog zadatka
        self.ROBOT_BASE_X = -0.2
        self.ROBOT_BASE_Y = -0.5
        self.ROBOT_BASE_Z = 1.021

        self.create_subscription(PointStamped, '/red_position', lambda msg: self.cube_callback(msg, "red"), 10, callback_group=self.cb_group)
        self.create_subscription(PointStamped, '/green_position', lambda msg: self.cube_callback(msg, "green"), 10, callback_group=self.cb_group)
        self.create_subscription(PointStamped, '/blue_position', lambda msg: self.cube_callback(msg, "blue"), 10, callback_group=self.cb_group)

        self.create_subscription(PointStamped, '/red_surface', lambda msg: self.surface_callback(msg, "red"), 10, callback_group=self.cb_group)
        self.create_subscription(PointStamped, '/green_surface', lambda msg: self.surface_callback(msg, "green"), 10, callback_group=self.cb_group)
        self.create_subscription(PointStamped, '/blue_surface', lambda msg: self.surface_callback(msg, "blue"), 10, callback_group=self.cb_group)

        self.move_group_client = ActionClient(self, MoveGroup, 'move_action', callback_group=self.cb_group)
        while not self.move_group_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info('Čekam da MoveIt MoveGroup akcioni server postane dostupan...')

        self.closest_cube_client = self.create_client(GetClosestCube, 'get_closest_cube', callback_group=self.cb_group)
        while not self.closest_cube_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Čekam da /get_closest_cube servis postane dostupan...')

        self.sorting_timer = self.create_timer(5.0, self.start_sorting_sequence, callback_group=self.cb_group)
        self.get_logger().info("Inicijalizacija sistema: Sakupljanje prostornih podataka iz vizionog podsistema (5s)...")


    def cube_callback(self, msg, color):
        if self.input_collection_locked:
            return

        cube_info = {"x": msg.point.x, "y": msg.point.y, "z": msg.point.z}
        
        exists = False
        for item in self.detected_cubes[color]:
            if abs(item["x"] - cube_info["x"]) < 0.02 and abs(item["y"] - cube_info["y"]) < 0.02:
                exists = True
                break
        if not exists:
            self.detected_cubes[color].append(cube_info)
            self.get_logger().info(
                f"[DETEKTOVANO] {color.capitalize()} objekat -> "
                f"X={cube_info['x']:.3f}, Y={cube_info['y']:.3f}, Z={cube_info['z']:.3f}"
            )


    def surface_callback(self, msg, color):
        if self.input_collection_locked:
            return

        if self.surface_positions[color] is None:
            self.surface_positions[color] = {"x": msg.point.x, "y": msg.point.y, "z": msg.point.z}
            
            self.get_logger().info(
                f"[DEPO] Referentna površ {color} -> "
                f"X={msg.point.x:.3f}, Y={msg.point.y:.3f}, Z={msg.point.z:.3f}"
            )


    def move_to_pose(self, target_pose: PoseStamped, velocity_scale: float = 1.0):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "xarm7"
        goal_msg.request.num_planning_attempts = 40
        goal_msg.request.allowed_planning_time = 20.0

        # Postavljanje brzine i ubrzanja (usporavanje ako je faktor manji, npr. 0.2)
        goal_msg.request.max_velocity_scaling_factor = velocity_scale
        goal_msg.request.max_acceleration_scaling_factor = velocity_scale

        c = Constraints()
        pos_con = PositionConstraint()
        pos_con.header = target_pose.header
        pos_con.link_name = "link_eef"
        
        box_primitive = SolidPrimitive()
        box_primitive.type = SolidPrimitive.BOX
        box_primitive.dimensions = [0.015, 0.015, 0.015]
        
        pos_con.constraint_region.primitives.append(box_primitive)
        pos_con.constraint_region.primitive_poses.append(target_pose.pose)
        pos_con.weight = 1.0
        c.position_constraints.append(pos_con)

        ori_con = OrientationConstraint()
        ori_con.header = target_pose.header
        ori_con.link_name = "link_eef"
        ori_con.orientation = target_pose.pose.orientation
        ori_con.absolute_x_axis_tolerance = 0.1
        ori_con.absolute_y_axis_tolerance = 0.1
        ori_con.absolute_z_axis_tolerance = 0.1
        ori_con.weight = 1.0
        c.orientation_constraints.append(ori_con)

        goal_msg.request.goal_constraints.append(c)
        goal_msg.planning_options.plan_only = False

        send_goal_future = self.move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        return result.error_code.val == 1


    def find_cube_id_via_service(self, target_x, target_y, target_z):
        req = GetClosestCube.Request()
        req.target_pose.position.x = target_x
        req.target_pose.position.y = target_y
        req.target_pose.position.z = target_z
        req.target_pose.orientation.w = 1.0

        for attempt in range(3):
            future = self.closest_cube_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            if response and response.found:
                self.get_logger().info(f"[SERVIS] Pronađena kocka ID: {response.cube_id}")
                return response.cube_id
            time.sleep(0.1)
        return None


    def trigger_gazebo_attach(self, cube_id: str):
        topic_name = f"/gz/{cube_id}/attach"
        pub = self.create_publisher(Empty, topic_name, 10)
        time.sleep(0.2)
        pub.publish(Empty())
        self.get_logger().info(f"[GAZEBO ATTACH] Uspešno zakačena kocka: {cube_id}")


    def trigger_gazebo_detach(self, cube_id: str):
        topic_name = f"/gz/{cube_id}/detach"
        pub = self.create_publisher(Empty, topic_name, 10)
        time.sleep(0.2)
        pub.publish(Empty())
        self.get_logger().info(f"[GAZEBO DETACH] Uspešno otpuštena kocka: {cube_id}")


    def get_grid_offset(self, index):
        row = index // 3
        col = index % 3
        dx = (col - 1) * 0.045
        dy = (row - 1) * 0.045
        return dx, dy
 

    def start_sorting_sequence(self):
        self.sorting_timer.cancel()
        self.input_collection_locked = True
        self.get_logger().info("=== PROSTORNI PODACI USPEŠNO ZAKLJUČANI. Pokretanje sekvencijalne obrade. ===")

        color_priority = ["red", "green", "blue"]

        for color in color_priority:
            cubes = self.detected_cubes[color]
            if not cubes:
                self.get_logger().info(f"Nema evidentiranih objekata za kategoriju: {color.upper()}. Preskačem.")
                continue

            if self.surface_positions[color] is None:
                self.get_logger().warn(f"Upozorenje: Ciljna površ za kategoriju {color.upper()} nije definisana. Preskačem.")
                continue

            self.get_logger().info(f"--- Započinjem procesiranje grupe: {color.upper()} (Ukupno jedinica: {len(cubes)}) ---")

            i = 0
            while i < len(cubes):
                if self.cube_counts_on_surface[color] >= 9:
                    self.get_logger().warn(f"Kapacitet ciljne površi {color.upper()} je popunjen (maksimalno 9 jedinica).")
                    break

                cube = cubes[i]
                target_x = cube["x"]
                target_y = cube["y"]
                target_z = cube["z"]

                ordinal_name = f"{i+1}. {color}"
                self.get_logger().info(f">> Obrada objekta [{ordinal_name} kocka]: Planiram prilaz iznad pozicije...")

                approach_pose = PoseStamped()
                approach_pose.header.frame_id = "link_base"
                approach_pose.pose.position.x = target_x
                approach_pose.pose.position.y = target_y
                approach_pose.pose.position.z = target_z + 0.35
                approach_pose.pose.orientation.x = 1.0
                approach_pose.pose.orientation.y = 0.0
                approach_pose.pose.orientation.z = 0.0
                approach_pose.pose.orientation.w = 0.0

                if not self.move_to_pose(approach_pose):
                    self.get_logger().error(f"Neuspešno planiranje kretanja iznad [{ordinal_name} kocka].")
                    i += 1
                    continue

                self.get_logger().info(f">> Spuštam se do [{ordinal_name} kocka] (izvršavam zahvat)...")
                pick_pose = PoseStamped()
                pick_pose.header.frame_id = "link_base"
                pick_pose.pose.position.x = target_x
                pick_pose.pose.position.y = target_y
                pick_pose.pose.position.z = target_z + 0.18
                pick_pose.pose.orientation = approach_pose.pose.orientation

                if not self.move_to_pose(pick_pose, velocity_scale=0.4):
                    self.get_logger().error(f"Neuspešno spuštanje na [{ordinal_name} kocka].")
                    i += 1
                    continue

                # PRILAGOĐAVANJE ZA ROTACIJU I TRANSLACIJU BAZE ROBOTA KA SVETSKIM KOORDINATAMA
                world_target_x = self.ROBOT_BASE_X - target_y
                world_target_y = self.ROBOT_BASE_Y + target_x
                world_target_z = self.ROBOT_BASE_Z + target_z

                active_cube_id = self.find_cube_id_via_service(world_target_x, world_target_y, world_target_z)
                if active_cube_id:
                    self.trigger_gazebo_attach(active_cube_id)
                else:
                    self.get_logger().warn(f"Upozorenje: Servis nije pronašao ID kocke, nastavljam bez fizičkog attach-a za [{ordinal_name}].")

                time.sleep(0.3)

                self.get_logger().info(f">> Podižem se sa pozicije [{ordinal_name} kocka]...")
                if not self.move_to_pose(approach_pose):
                    self.get_logger().error(f"Neuspešno podizanje sa [{ordinal_name} kocka].")
                    if active_cube_id:
                        self.trigger_gazebo_detach(active_cube_id)
                    i += 1
                    continue

                surf = self.surface_positions[color]
                idx = self.cube_counts_on_surface[color]
                dx, dy = self.get_grid_offset(idx)

                place_x = surf["x"] + dx
                place_y = surf["y"] + dy
                place_z = surf["z"]

                self.get_logger().info(f">> Nosim do {color.upper()} podloge (slot #{idx}). Planiram prilaz...")

                place_approach_pose = PoseStamped()
                place_approach_pose.header.frame_id = "link_base"
                place_approach_pose.pose.position.x = place_x
                place_approach_pose.pose.position.y = place_y
                place_approach_pose.pose.position.z = place_z + 0.35
                place_approach_pose.pose.orientation = approach_pose.pose.orientation

                if not self.move_to_pose(place_approach_pose):
                    self.get_logger().error(f"Neuspešno planiranje prilaza za {color.upper()} podlogu.")
                    if active_cube_id:
                        self.trigger_gazebo_detach(active_cube_id)
                    i += 1
                    continue

                self.get_logger().info(f">> Spuštam teret na {color.upper()} podlogu (izvršavam otpust)...")
                place_down_pose = PoseStamped()
                place_down_pose.header.frame_id = "link_base"
                place_down_pose.pose.position.x = place_x
                place_down_pose.pose.position.y = place_y
                place_down_pose.pose.position.z = place_z + 0.20
                place_down_pose.pose.orientation = approach_pose.pose.orientation

                if not self.move_to_pose(place_down_pose, velocity_scale=0.4):
                    self.get_logger().error(f"Neuspešno spuštanje na {color.upper()} podlogu.")
                    if active_cube_id:
                        self.trigger_gazebo_detach(active_cube_id)
                    i += 1
                    continue

                if active_cube_id:
                    self.trigger_gazebo_detach(active_cube_id)

                time.sleep(0.8)
                self.cube_counts_on_surface[color] += 1
                self.get_logger().info(f"[USPEH] Objektom [{ordinal_name}] uspešno popunjen slot #{idx} na {color.upper()} podlozi.")

                self.move_to_pose(place_approach_pose)

                i += 1

        self.get_logger().info("=== Ciklus sortiranja je u potpunosti i uspešno okončan. ===")


def main(args=None):
    rclpy.init(args=args)
    controller = StateMachineSortingController()
    
    executor = MultiThreadedExecutor()
    executor.add_node(controller)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
