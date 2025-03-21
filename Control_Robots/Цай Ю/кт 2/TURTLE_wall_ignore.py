import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, pi, sqrt
import time

class TurtleController(Node):
    def __init__(self):
        super().__init__('cherepavel')
        
        # Инициализация основного функционала
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.timer = self.create_timer(0.1, self.move_turtle)
        
        # Параметры движения
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.target_points = [
            (2.0, 9.0),
            (8.0, 2.0),
            (6.0, 8.0),
            (2.0, 5.0)
        ]
        self.current_target_index = 0
        self.distance_threshold = 0.1
        
        # Параметры стены (сегменты)
        self.wall_segments = [
            ((2.0, 5.0), (2.5, 7.5)),  # Первый сегмент стены
            ((2.5, 7.5), (4.0, 8.0))   # Второй сегмент стены
        ]
        self.safe_distance = 0.7
        self.avoidance_mode = False
        self.last_obstacle_time = 0.0
        self.escape_counter = 0

    @property
    def current_target(self):
        return self.target_points[self.current_target_index]

    def distance_to_segment(self, seg):
        """Вычисляет минимальное расстояние до сегмента стены"""
        (x1, y1), (x2, y2) = seg
        dx = x2 - x1
        dy = y2 - y1
        
        # Параметр t определяет ближайшую точку на сегменте
        t = ((self.x - x1)*dx + (self.y - y1)*dy) / (dx**2 + dy**2 + 1e-9)
        t = max(0, min(1, t))  # Ограничиваем t в пределах [0, 1]
        
        # Ближайшая точка на сегменте
        closest_x = x1 + t*dx
        closest_y = y1 + t*dy
        
        # Расстояние до ближайшей точки
        return sqrt((self.x - closest_x)**2 + (self.y - closest_y)**2)

    def check_wall_collision(self):
        """Проверяет, находится ли черепаха слишком близко к любому сегменту стены"""
        return any(self.distance_to_segment(seg) < self.safe_distance for seg in self.wall_segments)

    def calculate_escape_angle(self):
        """Рассчитывает угол для объезда стены"""
        # Находим ближайший сегмент стены
        closest_seg = min(self.wall_segments,
                        key=lambda seg: self.distance_to_segment(seg))
        
        (x1, y1), (x2, y2) = closest_seg
        dx_wall = x2 - x1
        dy_wall = y2 - y1
        
        # Перпендикулярное направление от стены
        return atan2(dy_wall, dx_wall) + pi/2

    def move_to_next_target(self):
        if self.current_target_index < len(self.target_points) - 1:
            self.current_target_index += 1
            self.get_logger().info(f"New target: {self.current_target}")
        else:
            self.get_logger().info("All targets reached!")

    def move_turtle(self):
        msg = Twist()
        target_x, target_y = self.current_target
        dx = target_x - self.x
        dy = target_y - self.y
        distance = sqrt(dx**2 + dy**2)

        if self.check_wall_collision():
            if not self.avoidance_mode:
                self.avoidance_mode = True
                self.last_obstacle_time = time.time()
                self.escape_counter = 0
                
            # Улучшенный алгоритм объезда
            escape_angle = self.calculate_escape_angle()
            angle_diff = (escape_angle - self.theta + pi) % (2 * pi) - pi
            
            msg.linear.x = 0.5
            msg.angular.z = angle_diff * 3.0
            
            # Проверка на застревание
            if time.time() - self.last_obstacle_time > 3.0:
                self.escape_counter += 1
                msg.linear.x = 0.8
                msg.angular.z = 2.0 if self.escape_counter % 2 else -2.0
                self.last_obstacle_time = time.time()
        else:
            if self.avoidance_mode:
                self.avoidance_mode = False
                self.escape_counter = 0

            if distance > self.distance_threshold:
                angle_to_target = atan2(dy, dx)
                angle_diff = (angle_to_target - self.theta + pi) % (2 * pi) - pi
                
                msg.linear.x = min(1.5, distance)
                msg.angular.z = angle_diff * 2.5
            else:
                self.move_to_next_target()
                msg.linear.x = 0.0
                msg.angular.z = 0.0

        self.publisher_.publish(msg)

    def pose_callback(self, pose):
        self.x = pose.x
        self.y = pose.y
        self.theta = pose.theta
        print(f"Position: X={self.x:.2f}, Y={self.y:.2f}, Theta={self.theta:.2f}")

def main():
    rclpy.init()
    controller = TurtleController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()