import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Float32
import csv
from datetime import datetime


class AS5600(Node):
    def __init__(self):
        super().__init__('as5600')

        # Suscripcion
        self.sub_raw_angle = self.create_subscription(Int64, 'as5600/raw_angle', self.raw_angle, 10)
        self.sub_angle = self.create_subscription(Float32, 'as5600/angle', self.angle, 10)

        # Valor por defecto
        self.current_raw_angle = 0
        self.current_angle = 0.0

        # Crea el archivo CSV 
        filename = f"historial_angulos_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.csv_file = open(filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Ángulo Crudo', 'Ángulo Convertido'])

        #Crea un timer de que cada 0,1s captura las variables
        self.timer = self.create_timer(0.5, self.log_data)  # 10 Hz

        self.get_logger().info(f'Nodo iniciado. Guardando datos en {filename}')

    def raw_angle(self, msg):
        self.current_raw_angle = msg.data

    def angle(self, msg):
        self.current_angle = msg.data

    def log_data(self):
        stamp = self.get_clock().now().to_msg()
        time_sec = stamp.sec + (stamp.nanosec * 1e-9)

        self.csv_writer.writerow([time_sec, self.current_raw_angle])
        self.csv_file.flush()

        # Muestra en la terminal
        self.get_logger().info(f'Ángulo crudo: {self.current_raw_angle:.2f} | Ángulo: {self.current_angle:.2f}')

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AS5600()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()