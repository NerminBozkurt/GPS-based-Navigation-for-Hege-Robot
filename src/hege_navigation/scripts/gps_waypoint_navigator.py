#!/usr/bin/env python3
"""GPS Waypoint Navigator for Hege Robot.

Dynamically reads the robot's current GPS position, generates a square
patrol pattern around it, converts each point to map-frame coordinates
via the NavSat /fromLL service, and sends them to Nav2's FollowWaypoints
action server.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import FollowWaypoints
from robot_localization.srv import FromLL
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from builtin_interfaces.msg import Duration
from gazebo_msgs.srv import SpawnEntity, DeleteEntity


# ===========================================================================
#  ROZMIAR KWADRATU (metry od punktu startowego)
#  Zmień tę wartość, żeby robot objechał większy lub mniejszy obszar.
# ===========================================================================
SQUARE_HALF_SIZE_M = 10.0  # robot objędzie kwadrat 20m x 20m


def gps_offset(lat, lon, delta_north_m, delta_east_m):
    """Przesuwa punkt GPS o zadaną liczbę metrów na północ i wschód."""
    lat_per_m = 1.0 / 111320.0
    lon_per_m = 1.0 / (111320.0 * math.cos(math.radians(lat)))
    new_lat = lat + delta_north_m * lat_per_m
    new_lon = lon + delta_east_m * lon_per_m
    return (new_lat, new_lon)


class GpsWaypointNavigator(Node):

    def __init__(self):
        super().__init__('gps_waypoint_navigator')
        
        # Wymuś używanie czasu z symulatora Gazebo (Sim Time)
        from rclpy.parameter import Parameter
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        
        self._fromll_client = self.create_client(FromLL, '/fromLL')
        self._follow_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self._spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self._delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self._start_gps = None

        # Publisher wizualizacji w RViz
        self._marker_pub = self.create_publisher(
            MarkerArray, '/waypoint_markers', 10)

        # Subskrybuj GPS, żeby pobrać pozycję startową
        self._gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix_fixed', self._gps_callback, 10)

        self.get_logger().info('GPS Waypoint Navigator uruchomiony.')
        self.get_logger().info(
            'Dodaj w RViz display: MarkerArray, topic: /waypoint_markers')

    def _gps_callback(self, msg):
        if self._start_gps is None and msg.status.status >= 0:
            self._start_gps = (msg.latitude, msg.longitude)
            self.get_logger().info(
                f'Pozycja startowa GPS: ({msg.latitude:.7f}, {msg.longitude:.7f})')

    def _wait_for_gps(self, timeout=15.0):
        """Czeka aż zostanie odebrana pierwsza poprawka GPS."""
        self.get_logger().info('Czekam na pierwszy fix GPS...')
        import time
        start = time.time()
        while self._start_gps is None:
            rclpy.spin_once(self, timeout_sec=0.2)
            if (time.time() - start) > timeout:
                self.get_logger().error('Timeout – brak sygnału GPS!')
                return False
        return True

    def _wait_for_services(self):
        self.get_logger().info('Czekam na usługę /fromLL ...')
        self._fromll_client.wait_for_service()
        self.get_logger().info('Czekam na serwer /follow_waypoints ...')
        self._follow_client.wait_for_server()
        self.get_logger().info('Wszystkie serwisy gotowe!')

    def _gps_to_map(self, lat, lon):
        req = FromLL.Request()
        req.ll_point = GeoPoint(latitude=lat, longitude=lon, altitude=0.0)
        future = self._fromll_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            self.get_logger().error(f'Usługa /fromLL nie odpowiedziała!')
            return None

        pt = future.result().map_point
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = pt.x
        pose.pose.position.y = pt.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f'  GPS ({lat:.6f}, {lon:.6f}) → Mapa ({pt.x:.2f}, {pt.y:.2f})')
        return pose

    def _build_square_waypoints(self):
        lat, lon = self._start_gps
        # Zmieniono na ścieżkę rolniczą (zygzak) z 6 punktów, szerokość ok. 30m
        w = 1.0e-4  # ok. 11m
        h = 0.5e-4  # ok. 3.5m
        
        return [
            (lat + w, lon + h),
            (lat - w, lon + h),
            (lat - w, lon),
            (lat + w, lon),
            (lat + w, lon - h),
            (lat - w, lon - h)
        ]
        return [gps_offset(lat0, lon0, dn, de) for dn, de in offsets]

    def _euler_to_quaternion(self, yaw):
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return (qx, qy, qz, qw)

    def run(self):
        self._wait_for_services()

        if not self._wait_for_gps():
            return

        waypoints_gps = self._build_square_waypoints()
        self.get_logger().info(
            f'\nPlan misji – {len(waypoints_gps)} punktów wokół '
            f'({self._start_gps[0]:.6f}, {self._start_gps[1]:.6f}):')

        waypoints = []
        for i, (lat, lon) in enumerate(waypoints_gps):
            self.get_logger().info(f'  Punkt {i+1}/{len(waypoints_gps)}:')
            pose = self._gps_to_map(lat, lon)
            if pose is None:
                self.get_logger().error('Przerwano – błąd konwersji GPS!')
                return
            waypoints.append(pose)

        # -------------------------------------------------------------
        # TUTAJ USTALASZ ORIENTACJĘ KOŃCOWĄ NA PUNKCIE (dla podnoszenia bali)
        # Obecnie wymuszamy 0 stopni (w=1.0) - czyli celowanie na Wschód mapy.
        # -------------------------------------------------------------
        for i in range(len(waypoints)):
            waypoints[i].pose.orientation.x = 0.0
            waypoints[i].pose.orientation.y = 0.0
            waypoints[i].pose.orientation.z = 0.0
            waypoints[i].pose.orientation.w = 1.0

        # Sprawdź zanim wyślesz – czy punkty mają sens?
        self.get_logger().info('\nSzczegóły punktów na mapie:')
        for i, wp in enumerate(waypoints):
            self.get_logger().info(
                f'  Punkt {i+1}: x={wp.pose.position.x:.2f}, y={wp.pose.position.y:.2f}')

        # Opublikuj markery w RViz i Gazebo
        self._publish_markers(waypoints)
        self._spawn_gazebo_markers(waypoints)
        # Odczekaj chwilę, żeby RViz i Gazebo zdążyły odebrać obiekty
        import time; time.sleep(1.0)

        self.get_logger().info(f'\nWysyłam misję do Nav2...')
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints

        send_future = self._follow_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Misja odrzucona przez Nav2!')
            return

        self.get_logger().info('✅ Misja przyjęta! Traktor jedzie...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.missed_waypoints:
            missed = [i + 1 for i in result.missed_waypoints]
            self.get_logger().warn(f'⚠️  Pominięte punkty: {missed}')
        else:
            self.get_logger().info('🏁 Misja zakończona! Wszystkie punkty odwiedzone.')

    def _publish_markers(self, waypoints: list):
        """Publikuje kolorowe markery waypointów w RViz."""
        markers = MarkerArray()
        lifetime = Duration(sec=3600)  # markery widoczne przez godzinę

        # Kolory dla każdego punktu
        colors = [
            ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0),  # czerwony
            ColorRGBA(r=0.2, g=1.0, b=0.2, a=1.0),  # zielony
            ColorRGBA(r=0.2, g=0.5, b=1.0, a=1.0),  # niebieski
            ColorRGBA(r=1.0, g=0.8, b=0.0, a=1.0),  # żółty
        ]

        for i, wp in enumerate(waypoints):
            color = colors[i % len(colors)]

            # Kula w miejscu waypointa
            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = 'waypoints'
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = wp.pose
            sphere.pose.position.z = 0.5  # unieś nad ziemię
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 1.0
            sphere.color = color
            sphere.lifetime = lifetime
            markers.markers.append(sphere)

            # Etykieta z numerem punktu
            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = 'waypoint_labels'
            text.id = i + 100
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose = wp.pose
            text.pose.position.z = 1.8
            text.scale.z = 1.2
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = f'Pkt {i + 1}'
            text.lifetime = lifetime
            markers.markers.append(text)

        # Linia łącząca punkty trasy
        line = Marker()
        line.header.frame_id = 'map'
        line.header.stamp = self.get_clock().now().to_msg()
        line.ns = 'waypoint_path'
        line.id = 200
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.15
        line.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)
        line.lifetime = lifetime
        for wp in waypoints + [waypoints[0]]:  # zamknij pętlę
            pt = Point()
            pt.x = wp.pose.position.x
            pt.y = wp.pose.position.y
            pt.z = 0.1
            line.points.append(pt)
        markers.markers.append(line)

        self._marker_pub.publish(markers)
        self.get_logger().info('📍 Markery opublikowane w RViz!')

    def _spawn_gazebo_markers(self, waypoints: list):
        """Umieszcza kolorowe kule w środowisku Gazebo."""
        self.get_logger().info('Czekam na usługę /spawn_entity (Gazebo)...')
        if not self._spawn_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('Brak usługi /spawn_entity - pomijam markery w Gazebo.')
            return

        # Najpierw usuń stare kule z Gazebo (żeby się zaktualizowały, jeśli odpalamy 2 raz)
        if self._delete_client.wait_for_service(timeout_sec=1.0):
            for i in range(len(waypoints)):
                req = DeleteEntity.Request()
                req.name = f'waypoint_marker_{i+1}'
                self._delete_client.call_async(req)
            import time; time.sleep(0.2)

        colors = [
            '1 0 0 1',  # czerwony
            '0 1 0 1',  # zielony
            '0 0 1 1',  # niebieski
            '1 1 0 1',  # żółty
        ]

        for i, wp in enumerate(waypoints):
            color = colors[i % len(colors)]
            sdf = f"""<?xml version='1.0'?>
            <sdf version='1.6'>
              <model name='waypoint_marker_{i+1}'>
                <static>true</static>
                <link name='link'>
                  <visual name='visual'>
                    <geometry><sphere><radius>0.4</radius></sphere></geometry>
                    <material>
                      <ambient>{color}</ambient>
                      <diffuse>{color}</diffuse>
                    </material>
                  </visual>
                </link>
              </model>
            </sdf>"""

            req = SpawnEntity.Request()
            req.name = f'waypoint_marker_{i+1}'
            req.xml = sdf
            req.robot_namespace = ''
            req.initial_pose = wp.pose
            req.initial_pose.position.z = 2.0  # unieś nad trawę
            
            self._spawn_client.call_async(req)
            import time; time.sleep(0.5)
            
        self.get_logger().info('📍 Markery (kule) wstawione do Gazebo!')

    def _feedback_callback(self, feedback_msg):
        current = feedback_msg.feedback.current_waypoint + 1
        total = 4
        self.get_logger().info(f'>>> Jadę do punktu {current}/{total}...')


def main():
    rclpy.init()
    navigator = GpsWaypointNavigator()
    try:
        navigator.run()
    except KeyboardInterrupt:
        navigator.get_logger().info('Przerwano przez użytkownika.')
    finally:
        navigator.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
