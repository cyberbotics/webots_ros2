import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PointStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path
import math
import re
import csv
import json
class LLMResponseParser(Node):
    def __init__(self):
        super().__init__('llm_response_parser')

        # Subscriber untuk data GPS
        self.gps_subscription = self.create_subscription(
            PointStamped,
            '/Mavic_2_PRO/gps',
            self.gps_callback,
            10
        )
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

        # Subscriber untuk menerima respons dari LLM
        self.llm_response_subscription = self.create_subscription(
            String,
            '/llm_to_drone_command',
            self.process_llm_response,
            10
        )
        # Publisher untuk marker waypoint
        self.marker_publisher = self.create_publisher(MarkerArray, '/waypoints_marker', 10)


        # Publisher untuk mengirim perintah navigasi ke drone
        self.command_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher untuk path
        self.path_publisher = self.create_publisher(Path, '/drone_path', 10)

        # Inisialisasi variabel
        self.current_position = None
        self.target_waypoint = None  # Waypoint dari LLM
        self.reached_target = False
        self.orientation = None
        self.waypoints = []  # Daftar waypoint
        self.current_waypoint_index = 0 
        self.markers = MarkerArray()
        self.csv_file = 'gps_point_data.csv'
        self.initialize_csv()

        # Path untuk RViz
        self.path = Path()
        self.path.header.frame_id = "map"  # Ubah sesuai dengan frame RViz Anda

    def initialize_csv(self):
        with open(self.csv_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Timestamp', 'X', 'Y', 'Z'])  # Header file CSV
            
    def process_llm_response(self, msg):
        response = msg.data.strip()
        self.get_logger().info(f"Parsing LLM Response: {response}")
        #memisahkan json waypoints dengan text lainnya
        match = re.search(r'\[\s*{.*?}\s*\]', response, re.S)
        if match:
            response = match.group(0)  # Ambil string JSON yang cocok
            response = response.strip()  # Hapus spasi atau karakter kosong di awal/akhir
        # Parsing respons dari LLM
        try:
            self.waypoints = self.parse_response(response)
            if self.waypoints:
                self.get_logger().info(f"Parsed Waypoints: {self.waypoints}")
                self.current_waypoint_index = 0  # Reset ke waypoint pertama
                # Buat marker untuk setiap waypoint
                self.markers.markers = []  # Reset marker sebelumnya
                for i, waypoint in enumerate(self.waypoints):
                    marker = self.create_marker(waypoint, i)
                    self.markers.markers.append(marker)

                # Publikasikan marker ke RViz
                self.marker_publisher.publish(self.markers)
            else:
                self.get_logger().error("Parsed response is empty or invalid.")
        except Exception as e:
            self.get_logger().error(f"Error during parsing: {e}")

    def gps_callback(self, msg):
        with open(self.csv_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            timestamp = msg.header.stamp
            writer.writerow([
                f"{timestamp.sec}.{timestamp.nanosec}",  # Timestamp dari header
                msg.point.x,  # Koordinat X
                msg.point.y,  # Koordinat Y
                msg.point.z   # Koordinat Z
            ])
        # Update posisi GPS saat ini
        self.current_position = msg.point
        # Buat PoseStamped untuk posisi saat ini
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"  # Ubah sesuai frame RViz
        pose.pose.position.x = self.current_position.x
        pose.pose.position.y = self.current_position.y
        pose.pose.position.z = self.current_position.z

        # Tambahkan pose ke path
        self.path.header.stamp = pose.header.stamp
        self.path.poses.append(pose)

        # Batasi jumlah pose dalam path (opsional, untuk efisiensi)
        max_path_length = 100
        if len(self.path.poses) > max_path_length:
            self.path.poses.pop(0)  # Hapus pose tertua

        # Publikasikan path ke RViz
        self.path_publisher.publish(self.path)

        self.navigate_to_waypoint()
        
    def imu_callback(self, imu_msg):
        # Update orientasi dari data IMU
        orientation_q = imu_msg.orientation
        roll, pitch, yaw = self.quaternion_to_euler(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        
        #self.get_logger().info(f'Orientation (roll, pitch, yaw): ({roll:.2f}, {pitch:.2f}, {yaw:.2f})')
        
        # Gunakan data IMU untuk menjaga kestabilan
        self.orientation = (roll, pitch, yaw)

    def quaternion_to_euler(self, x, y, z, w):
        # Mengonversi quaternion ke Euler angles (roll, pitch, yaw)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        
        return roll, pitch, yaw
        
    def create_marker(self, waypoint, marker_id):
        marker = Marker()
        marker.header.frame_id = "map"  # Sesuaikan frame RViz
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "waypoints"
        marker.id = marker_id
        marker.type = Marker.SPHERE  # Marker berbentuk bola
        marker.action = Marker.ADD

        # Koordinat waypoint
        marker.pose.position.x = waypoint['x']
        marker.pose.position.y = waypoint['y']
        marker.pose.position.z = waypoint['z']
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Ukuran marker
        marker.scale.x = 0.2  # Diameter bola
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        # Warna marker
        marker.color.r = 1.0  # Merah
        marker.color.g = 0.0  # Hijau
        marker.color.b = 0.0  # Biru
        marker.color.a = 1.0  # Transparansi (1 = solid)

        return marker
    
    def parse_response(self, response):
        try:
            # Parsing respons sebagai JSON jika berbentuk list
            waypoints = json.loads(response)
            # Validasi bahwa semua waypoint memiliki x, y, z
            if not isinstance(waypoints, list):
                self.get_logger().error("Parsed response is not a list of waypoints.")
                return None
            # Validasi bahwa semua waypoint memiliki x, y, z
            for waypoint in waypoints:
                if not all(k in waypoint for k in ('x', 'y', 'z')):
                    self.get_logger().error(f"Invalid waypoint format: {waypoint}")
                    return None

            return waypoints
        except Exception as e:
            self.get_logger().error(f"Error parsing response: {e}")
            return None

    def navigate_to_waypoint(self):
        # Pastikan ada daftar waypoint
        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("No more waypoints to navigate.")
            # Hentikan drone sepenuhnya setelah waypoint terakhir
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            stop_cmd.linear.y = 0.0
            stop_cmd.linear.z = 0.0
            self.command_publisher.publish(stop_cmd)
            return

        # Ambil waypoint saat ini
        target_waypoint = self.waypoints[self.current_waypoint_index]
        
        # Hitung perbedaan posisi dari target
        delta_x = -(target_waypoint['x'] - self.current_position.x) 
        delta_y = -(target_waypoint['y'] - self.current_position.y)
        delta_z = target_waypoint['z'] - self.current_position.z
        distance = math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)
        
        # Jarak ke target
        distance = math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)
        
        # Buat perintah kecepatan berdasarkan jarak ke target
        move_cmd = Twist()
        max_speed = 0.5  # Kecepatan maksimum dalam m/s
        speed_factor = min(max_speed, max(0.05, distance / 5))
        if distance > 0.05:  # Jika masih jauh dari target
            # Normalisasi delta agar drone bergerak ke arah target
            move_cmd.linear.x = speed_factor * (delta_x / distance)
            move_cmd.linear.y = speed_factor * (delta_y / distance)
            move_cmd.linear.z = speed_factor * (delta_z / distance)
        else:
            move_cmd.linear.x = 0.0
            move_cmd.linear.y = 0.0
            move_cmd.linear.z = 0.0
      
        # Koreksi stabilitas dengan data IMU
        imu_correction_factor = 0.005  # Faktor koreksi IMU
        if self.orientation is not None:
            roll, pitch, _ = self.orientation
            move_cmd.linear.x -= imu_correction_factor * pitch
            move_cmd.linear.y -= imu_correction_factor * roll

        # Publikasikan perintah gerak jika belum mencapai target
        if distance > 0.05 or abs(delta_z) > 0.05:
            self.command_publisher.publish(move_cmd)
            self.get_logger().info(f'Published command to move towards target, horizontal distance: {distance:.5f}, vertical delta: {delta_z:.5f}')
        else:
            self.get_logger().info(f"Waypoint {self.current_waypoint_index + 1} Reached!")
            self.current_waypoint_index += 1  # Pindah ke waypoint berikutnya
            # Jika ini adalah waypoint terakhir, hentikan drone sepenuhnya
            if self.current_waypoint_index >= len(self.waypoints):
                self.get_logger().info("All waypoints reached. Drone is now holding position.")
                stop_cmd = Twist()
                stop_cmd.linear.x = 0.0
                stop_cmd.linear.y = 0.0
                stop_cmd.linear.z = 0.0
                self.command_publisher.publish(stop_cmd)
        
def main(args=None):
    rclpy.init(args=args)
    node = LLMResponseParser()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
