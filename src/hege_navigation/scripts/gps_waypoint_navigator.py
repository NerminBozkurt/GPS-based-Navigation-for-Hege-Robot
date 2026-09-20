#!/usr/bin/env python3
"""GPS Waypoint Navigator for Hege Robot - Hay Bale Collection Mission.

Układ rzędowy (Row harvesting):
- Rząd 1: traktor jedzie po linii prostej na wschód przez Belę 1 i Belę 2.
- Nawrót (Headland): gładki łuk na poprzeczniaku o promieniu dopasowanym do traktora.
- Rząd 2: traktor jedzie po linii prostej na zachód przez Belę 3 i Belę 4.

Każda bela to leżący poziomo walec ze zintegrowaną zieloną strzałką najazdu
widoczną bezpośrednio w Gazebo oraz RViz.
"""

import math
import time
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
from gazebo_msgs.srv import SpawnEntity, DeleteEntity, SetEntityState
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


# Parametry beli siana (standardowy wymiar rolniczy)
BALE_RADIUS = 0.6   # promień walca (średnica 1.2m)
BALE_LENGTH = 1.2   # długość walca (1.2m)
GROUND_Z = -0.52    # poziom podłoża w Gazebo


def euler_to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    """Konwersja roll, pitch, yaw do kwaternionu."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return (qx, qy, qz, qw)


class BaleDetection:
    """Reprezentacja leżącej beli siana wykrytej przez kamerę.
    
    Kamera dostarcza:
    - (center_x, center_y): współrzędne środka leżącej beli
    - (normal_x, normal_y): wektor normalny podstawy (płaskiej części) leżącej beli
    
    Na tej podstawie wyznaczany jest wektor i kąt najazdu na płaską część beli.
    """

    def __init__(self, name: str, center_x: float, center_y: float, normal_x: float, normal_y: float):
        self.name = name
        self.x = float(center_x)
        self.y = float(center_y)
        norm = math.hypot(normal_x, normal_y)
        if norm > 1e-6:
            self.normal_x = float(normal_x) / norm
            self.normal_y = float(normal_y) / norm
        else:
            self.normal_x = 1.0
            self.normal_y = 0.0
        # Kąt najazdu wzdłuż wektora podstawy płaskiej części beli:
        self.yaw = math.atan2(self.normal_y, self.normal_x)

    def to_dict(self):
        return {
            'name': self.name,
            'x': self.x,
            'y': self.y,
            'yaw': self.yaw,
            'normal_x': self.normal_x,
            'normal_y': self.normal_y
        }


class GpsWaypointNavigator(Node):

    def __init__(self):
        super().__init__('gps_waypoint_navigator')
        
        from rclpy.parameter import Parameter
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        
        self._fromll_client = self.create_client(FromLL, '/fromLL')
        self._follow_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self._spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self._delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self._set_state_client = self.create_client(SetEntityState, '/set_entity_state')
        self._start_gps = None
        self._start_map_pose = None
        self._total_waypoints = 0
        self._cached_markers = None

        # QoS Transient Local - każdy nowy subscriber (np. RViz) od razu otrzymuje markery!
        marker_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self._marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', marker_qos)
        self._waypoints_pub = self.create_publisher(MarkerArray, '/waypoints', marker_qos)
        
        # Ciągłe odświeżanie markerów co 1 sekundę (gwarancja widoczności w RViz w dowolnym momencie)
        self._marker_timer = self.create_timer(1.0, self._timer_marker_publish)

        self._gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix_fixed', self._gps_callback, 10)

        self.get_logger().info('GPS Waypoint Navigator (Zbiór Beli Siana) zainicjalizowany.')

    def _timer_marker_publish(self):
        """Cykliczna publikacja markerów do RViz."""
        if self._cached_markers is not None:
            self._marker_pub.publish(self._cached_markers)
            self._waypoints_pub.publish(self._cached_markers)

    def _gps_callback(self, msg):
        if self._start_gps is None and msg.status.status >= 0:
            self._start_gps = (msg.latitude, msg.longitude)
            self.get_logger().info(
                f'Pozycja startowa GPS: ({msg.latitude:.7f}, {msg.longitude:.7f})')

    def _wait_for_gps(self, timeout=15.0):
        self.get_logger().info('Czekam na sygnał GPS...')
        start = time.time()
        while self._start_gps is None:
            rclpy.spin_once(self, timeout_sec=0.2)
            if (time.time() - start) > timeout:
                self.get_logger().error('Brak sygnału GPS!')
                return False
        return True

    def _wait_for_services(self):
        self.get_logger().info('Czekam na usługi /fromLL i /follow_waypoints...')
        self._fromll_client.wait_for_service()
        self._follow_client.wait_for_server()
        self.get_logger().info('Wszystkie serwisy aktywne!')

    def _gps_to_map(self, lat, lon):
        req = FromLL.Request()
        req.ll_point = GeoPoint(latitude=lat, longitude=lon, altitude=0.0)
        future = self._fromll_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            self.get_logger().error('Błąd konwersji GPS /fromLL!')
            return None

        pt = future.result().map_point
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = pt.x
        pose.pose.position.y = pt.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        return pose

    def create_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def run(self):
        self._wait_for_services()
        if not self._wait_for_gps():
            return

        # =====================================================================
        # MISJA ROLNICZA: NATURALNY ZBIÓR 4 ROZPROSZONYCH BELI SIANA
        # =====================================================================
        # Traktor startuje z pozycji (0, 0) z kursem na Wschód (Yaw = 0.0).
        # Dla każdej beli wyznaczany jest 3-punktowy korytarz najazdu:
        # 1. Wjazd na początek zielonej strzałki (-3.2m) - pełna rotacja przed wjazdem
        # 2. Środek beli (0.0m) - zbiór na wprost
        # 3. Czysty wyjazd (+1.6m) - opuszczenie beli na wprost, brak skręcania w obrysie beli

        # Dane wejściowe w formacie detekcji z kamery:
        # Kamera dostarcza współrzędne środka beli oraz wektor normalny podstawy leżącej beli (płaskiej części).
        detected_bales = [
            BaleDetection('BELA 1', center_x=16.0, center_y=-4.0, normal_x=math.cos(-0.15), normal_y=math.sin(-0.15)),
            BaleDetection('BELA 2', center_x=28.0, center_y=8.0,  normal_x=math.cos(0.75),  normal_y=math.sin(0.75)),
            BaleDetection('BELA 3', center_x=14.0, center_y=22.0, normal_x=math.cos(2.35),  normal_y=math.sin(2.35)),
            BaleDetection('BELA 4', center_x=-2.0, center_y=10.0, normal_x=math.cos(-2.55), normal_y=math.sin(-2.55)),
        ]
        bales = [b.to_dict() for b in detected_bales]

        entry_d = 3.5   # Odległość punktu wjazdowego (początek zielonej strzałki najazdu)
        waypoints = []
        wp_labels = []

        wp_num = 1
        for idx, b in enumerate(bales):
            yaw = b['yaw']
            cos_y = math.cos(yaw)
            sin_y = math.sin(yaw)

            # 1. Punkt wjazdu na zieloną strzałkę (traktor wjeżdża z wyprostowanymi kołami w jej osi)
            app_x = b['x'] - entry_d * cos_y
            app_y = b['y'] - entry_d * sin_y
            waypoints.append(self.create_pose(app_x, app_y, yaw))
            wp_labels.append(f"Pkt {wp_num}: {b['name']} (Wjazd na strzałkę)")
            wp_num += 1

            # 2. Punkt bezpośredniego podjazdu pod belę (zbiór w 100% po linii prostej strzałki)
            waypoints.append(self.create_pose(b['x'], b['y'], yaw))
            wp_labels.append(f"Pkt {wp_num}: {b['name']} (Zbiór)")
            wp_num += 1

        # 3. Meta / Baza
        waypoints.append(self.create_pose(0.0, 0.0, -1.0))
        wp_labels.append(f"Pkt {wp_num}: BAZA / META")

        self._total_waypoints = len(waypoints)

        self.get_logger().info(f'\nZdefiniowano {len(bales)} leżące bele oraz {len(waypoints)} punktów nawigacji (w tym prostoliniowe korytarze najazdu).')
        for i, wp in enumerate(waypoints):
            self.get_logger().info(f'  {wp_labels[i]}: x={wp.pose.position.x:.2f}, y={wp.pose.position.y:.2f}')

        # Opublikuj wizualizacje
        self._publish_markers(bales, waypoints, wp_labels)
        self._spawn_gazebo_markers(bales)
        time.sleep(0.5)

        # Wyślij misję do Nav2
        self.get_logger().info('\nWysyłam misję do Nav2 FollowWaypoints...')
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints

        send_future = self._follow_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Misja została odrzucona przez Nav2!')
            return

        self.get_logger().info('✅ Misja przyjęta! Traktor zbiera bele w rzędach...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.missed_waypoints:
            missed = [i + 1 for i in result.missed_waypoints]
            self.get_logger().warn(f'⚠️  Pominięte punkty: {missed}')
        else:
            self.get_logger().info('🏁 Misja zakończona sukcesem! Wszystkie bele zebrane.')

    def _publish_markers(self, bales: list, waypoints: list, wp_labels: list = None):
        """Wizualizacja w RViz: leżące walce, zielone strzałki, pinezki i linia trasy."""
        markers = MarkerArray()
        lifetime = Duration(sec=3600)
        stamp = self.get_clock().now().to_msg()

        for i, b in enumerate(bales):
            bx = b['x']
            by = b['y']
            yaw = b['yaw']
            cos_y = math.cos(yaw)
            sin_y = math.sin(yaw)

            # 1. Walec leżący na ziemi w osi rzędu
            qx, qy, qz, qw = euler_to_quaternion(0.0, 1.570796, yaw)
            cylinder = Marker()
            cylinder.header.frame_id = 'map'
            cylinder.header.stamp = stamp
            cylinder.ns = 'hay_bales'
            cylinder.id = i
            cylinder.type = Marker.CYLINDER
            cylinder.action = Marker.ADD
            cylinder.pose.position.x = bx
            cylinder.pose.position.y = by
            cylinder.pose.position.z = BALE_RADIUS
            cylinder.pose.orientation.x = qx
            cylinder.pose.orientation.y = qy
            cylinder.pose.orientation.z = qz
            cylinder.pose.orientation.w = qw
            cylinder.scale.x = BALE_RADIUS * 2.0
            cylinder.scale.y = BALE_RADIUS * 2.0
            cylinder.scale.z = BALE_LENGTH
            cylinder.color = ColorRGBA(r=0.9, g=0.8, b=0.3, a=0.95)  # słomiano-złoty
            cylinder.lifetime = lifetime
            markers.markers.append(cylinder)

            # 2. Zielona strzałka najazdu na ziemi (wskazuje kierunek wprost do beli)
            arrow = Marker()
            arrow.header.frame_id = 'map'
            arrow.header.stamp = stamp
            arrow.ns = 'approach_arrows'
            arrow.id = i + 100
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.points = [
                Point(x=bx - 3.5 * cos_y, y=by - 3.5 * sin_y, z=0.15),
                Point(x=bx - 0.6 * cos_y, y=by - 0.6 * sin_y, z=0.15)
            ]
            arrow.scale.x = 0.45  # grubość trzonu
            arrow.scale.y = 0.9   # szerokość grotu
            arrow.scale.z = 0.8   # długość grotu
            arrow.color = ColorRGBA(r=0.0, g=1.0, b=0.2, a=0.98)  # jaskrawa zieleń
            arrow.lifetime = lifetime
            markers.markers.append(arrow)

            # 3. Etykieta beli
            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp = stamp
            text.ns = 'bale_labels'
            text.id = i + 200
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = bx
            text.pose.position.y = by
            text.pose.position.z = 2.2
            text.scale.z = 1.0
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = b['name']
            text.lifetime = lifetime
            markers.markers.append(text)

        # 4. Linia całej ścieżki
        line = Marker()
        line.header.frame_id = 'map'
        line.header.stamp = stamp
        line.ns = 'mission_path'
        line.id = 300
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.2
        line.color = ColorRGBA(r=1.0, g=0.85, b=0.0, a=0.9)
        line.lifetime = lifetime
        for wp in waypoints:
            line.points.append(Point(x=wp.pose.position.x, y=wp.pose.position.y, z=0.15))
        markers.markers.append(line)

        # 5. Wyraźne pinezki i numerowane etykiety dla każdego punktu
        for i, wp in enumerate(waypoints):
            wx = wp.pose.position.x
            wy = wp.pose.position.y
            label_text = wp_labels[i] if (wp_labels and i < len(wp_labels)) else f"Pkt {i+1}"

            # Dobór kolorystyki w zależności od roli punktu:
            if "Wjazd" in label_text:
                pin_color = ColorRGBA(r=0.0, g=0.85, b=0.3, a=0.85)     # zieleń (wjazd na strzałkę)
                sphere_color = ColorRGBA(r=0.0, g=1.0, b=0.2, a=0.95)   # jaskrawa zieleń
            elif "Zbiór" in label_text:
                pin_color = ColorRGBA(r=1.0, g=0.75, b=0.1, a=0.85)     # złoty (bela)
                sphere_color = ColorRGBA(r=1.0, g=0.45, b=0.0, a=0.95)  # pomarańczowy
            else:
                pin_color = ColorRGBA(r=0.2, g=0.8, b=1.0, a=0.85)      # błękitny (meta)
                sphere_color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.95)   # niebieski

            # Słupek waypointa
            pin = Marker()
            pin.header.frame_id = 'map'
            pin.header.stamp = stamp
            pin.ns = 'waypoint_pins'
            pin.id = i + 400
            pin.type = Marker.CYLINDER
            pin.action = Marker.ADD
            pin.pose.position.x = wx
            pin.pose.position.y = wy
            pin.pose.position.z = 0.5
            pin.scale.x = 0.15
            pin.scale.y = 0.15
            pin.scale.z = 1.0
            pin.color = pin_color
            pin.lifetime = lifetime
            markers.markers.append(pin)

            # Świecąca kula na szczycie słupka
            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp = stamp
            sphere.ns = 'waypoint_spheres'
            sphere.id = i + 500
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = wx
            sphere.pose.position.y = wy
            sphere.pose.position.z = 1.1
            sphere.scale.x = 0.4
            sphere.scale.y = 0.4
            sphere.scale.z = 0.4
            sphere.color = sphere_color
            sphere.lifetime = lifetime
            markers.markers.append(sphere)

            # Pływająca etykieta tekstowa
            lbl = Marker()
            lbl.header.frame_id = 'map'
            lbl.header.stamp = stamp
            lbl.ns = 'waypoint_text'
            lbl.id = i + 600
            lbl.type = Marker.TEXT_VIEW_FACING
            lbl.action = Marker.ADD
            lbl.pose.position.x = wx
            lbl.pose.position.y = wy
            lbl.pose.position.z = 1.6
            lbl.scale.z = 0.65
            lbl.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            lbl.text = label_text
            lbl.lifetime = lifetime
            markers.markers.append(lbl)

        self._cached_markers = markers
        self._marker_pub.publish(markers)
        self._waypoints_pub.publish(markers)
        self.get_logger().info('📍 Bele, zielone strzałki, pinezki i ścieżka opublikowane w RViz!')

    def _spawn_gazebo_markers(self, bales: list):
        """Upewnia się, że leżące bele i zielone strzałki są w Gazebo na zadanych pozycjach."""
        for i, b in enumerate(bales):
            name = f'hay_bale_{i+1}'
            yaw = b['yaw']
            qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, yaw)

            # 1. Próba aktualizacji pozycji istniejącego modelu za pomocą /set_entity_state
            if self._set_state_client.service_is_ready():
                set_req = SetEntityState.Request()
                set_req.state.name = name
                set_req.state.pose.position.x = float(b['x'])
                set_req.state.pose.position.y = float(b['y'])
                set_req.state.pose.position.z = GROUND_Z
                set_req.state.pose.orientation.x = qx
                set_req.state.pose.orientation.y = qy
                set_req.state.pose.orientation.z = qz
                set_req.state.pose.orientation.w = qw
                set_req.state.reference_frame = 'world'
                fut = self._set_state_client.call_async(set_req)
                rclpy.spin_until_future_complete(self, fut, timeout_sec=0.5)
                if fut.result() and fut.result().success:
                    continue

            # 2. Jeśli model nie istnieje, spawnujemy go przez /spawn_entity
            if not self._spawn_client.wait_for_service(timeout_sec=0.5):
                continue

            sdf = f"""<?xml version='1.0'?>
            <sdf version='1.6'>
              <model name='hay_bale_{i+1}'>
                <static>true</static>
                <link name='bale_link'>
                  <pose>0 0 {BALE_RADIUS} 0 1.570796 0</pose>
                  <visual name='bale_visual'>
                    <geometry>
                      <cylinder>
                        <radius>{BALE_RADIUS}</radius>
                        <length>{BALE_LENGTH}</length>
                      </cylinder>
                    </geometry>
                    <material>
                      <ambient>0.92 0.78 0.25 1</ambient>
                      <diffuse>0.92 0.78 0.25 1</diffuse>
                      <emissive>0.25 0.18 0.02 1</emissive>
                    </material>
                  </visual>
                </link>
                <link name='arrow_link'>
                  <visual name='arrow_shaft'>
                    <pose>-1.9 0 0.125 0 0 0</pose>
                    <geometry><box><size>2.6 0.45 0.25</size></box></geometry>
                    <material>
                      <ambient>0.0 1.0 0.2 1</ambient>
                      <diffuse>0.0 1.0 0.2 1</diffuse>
                      <emissive>0.0 0.9 0.2 1</emissive>
                    </material>
                  </visual>
                  <visual name='arrow_head'>
                    <pose>-0.45 0 0.125 0 0 0</pose>
                    <geometry><box><size>0.6 1.0 0.25</size></box></geometry>
                    <material>
                      <ambient>0.0 1.0 0.2 1</ambient>
                      <diffuse>0.0 1.0 0.2 1</diffuse>
                      <emissive>0.0 0.9 0.2 1</emissive>
                    </material>
                  </visual>
                </link>
              </model>
            </sdf>"""

            req = SpawnEntity.Request()
            req.name = f'hay_bale_{i+1}'
            req.xml = sdf
            req.robot_namespace = ''
            req.initial_pose.position.x = b['x']
            req.initial_pose.position.y = b['y']
            req.initial_pose.position.z = GROUND_Z
            req.initial_pose.orientation.x = qx
            req.initial_pose.orientation.y = qy
            req.initial_pose.orientation.z = qz
            req.initial_pose.orientation.w = qw
            
            future = self._spawn_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            
        self.get_logger().info('🌾 Leżące bele i zielone strzałki najazdu zsynchronizowane w Gazebo!')

    def _feedback_callback(self, feedback_msg):
        current = feedback_msg.feedback.current_waypoint + 1
        total = self._total_waypoints
        self.get_logger().info(f'>>> Postęp: punkt {current}/{total}...')


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
