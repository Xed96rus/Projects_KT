import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute, SetPen

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')

        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Сервис teleport_absolute недоступен, ждем...')
        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Сервис set_pen недоступен, ждем...')

    def teleport_to(self, x, y, theta=0.0):
        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = theta
        future = self.teleport_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Черепаха перемещена в ({x}, {y})')
        else:
            self.get_logger().error('Не удалось переместить черепаху')

    def set_pen(self, r=255, g=255, b=255, width=2, off=False):
        request = SetPen.Request()
        request.r = r  # Цвет (по умолчанию белый)
        request.g = g
        request.b = b
        request.width = width  # Толщина линии
        request.off = off  # Если True, перо поднято
        future = self.pen_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Перо настроено')
        else:
            self.get_logger().error('Не удалось настроить перо')

    def draw_obstacles(self, points):
        # Поднимаем перо (чтобы не рисовать при перемещении в первую точку)
        self.set_pen(off=True)

        # Перемещаем черепаху в первую точку (без рисования)
        self.teleport_to(points[0][0], points[0][1])

        # Рисуем линии между точками
        for i in range(len(points) - 1):
            start_x, start_y = points[i]
            end_x, end_y = points[i + 1]

            # Опускаем перо (начинаем рисовать)
            self.set_pen(r=255, g=0, b=0, width=2, off=False)  # Красное перо для препятствий

            # Перемещаем черепаху в конечную точку (рисуем линию)
            self.teleport_to(end_x, end_y)

            # Поднимаем перо (заканчиваем рисовать)
            self.set_pen(off=True)

    def reset_turtle(self):
        # Возвращаем черепаху в начальное положение с поднятым пером
        self.set_pen(off=True)  # Поднимаем перо
        self.teleport_to(5.5, 5.5, 0.0)  # Возвращаем в начальную точку
        self.set_pen(r=255, g=255, b=255, width=2, off=False)  # Восстанавливаем перо по умолчанию
        self.get_logger().info('Черепаха возвращена в начальное положение с пером по умолчанию')

def main(args=None):
    rclpy.init(args=args)
    controller = TurtleController()

    # Координаты точек-препятствий
    points = [(2.0, 5.0), (2.5, 7.5), (4.0, 8.0)]

    # Рисуем линии между точками-препятствиями
    controller.draw_obstacles(points)

    # Возвращаем черепаху в начальное положение
    controller.reset_turtle()

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()