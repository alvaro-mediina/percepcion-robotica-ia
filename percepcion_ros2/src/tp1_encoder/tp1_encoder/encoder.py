import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Float32
import csv
from datetime import datetime


class EncoderLogger(Node):
    def __init__(self):
        super().__init__('encoder_logger')

        # Suscripcion
        self.sub_pos = self.create_subscription(Int64, 'encoder/position', self.pos_callback, 10)
        self.sub_vel = self.create_subscription(Float32, 'encoder/velocity', self.vel_callback, 10)

        # Valor por defecto
        self.current_pos = 0
        self.current_vel = 0.0

        # Crea el archivo CSV 
        filename = f"historial_encoder_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.csv_file = open(filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Timestamp_sec', 'Posicion_ticks', 'Velocidad_RPM'])

        #Crea un timer de que cada 0,1s captura las variables
        self.timer = self.create_timer(0.5, self.log_data)  # 10 Hz

        self.get_logger().info(f'Nodo iniciado. Guardando datos en {filename}')

    def pos_callback(self, msg):
        self.current_pos = msg.data

    def vel_callback(self, msg):
        self.current_vel = msg.data

    def log_data(self):
        stamp = self.get_clock().now().to_msg() # Toma el timepo del ROS2
        time_sec = stamp.sec + (stamp.nanosec * 1e-9)

        self.csv_writer.writerow([time_sec, self.current_pos, self.current_vel])    # Inserta una fila y guarda los datos
        self.csv_file.flush()

        # Muestra en la terminal
        self.get_logger().info(f'Pos: {self.current_pos} ticks | Vel: {self.current_vel:.2f} RPM')

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EncoderLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()