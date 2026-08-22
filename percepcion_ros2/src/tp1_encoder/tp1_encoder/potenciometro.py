import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Float32
import csv
from datetime import datetime


class Potenciometro(Node):
    def __init__(self):
        super().__init__('potenciometro')

        # Suscripcion
        self.sub_raw_signal = self.create_subscription(Float32, 'potenciometro/voltaje_crudo', self.raw_signal, 10)
        self.sub_raw_position = self.create_subscription(Float32, 'potenciometro/posicion_cruda', self.raw_position, 10)
        self.sub_conditioned_signal = self.create_subscription(Float32, 'potenciometro/voltaje_acondicionado', self.conditioned_signal, 10)
        self.sub_conditioned_position = self.create_subscription(Float32, 'potenciometro/posicion_acondicionada', self.conditioned_position, 10)

        # Valor por defecto
        self.current_raw_signal = 0.0
        self.current_conditioned_signal = 0.0
        self.current_raw_position = 0.0
        self.current_conditioned_position = 0.0

        # Crea el archivo CSV 
        filename = f"historial_potenciometro_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.csv_file = open(filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Señal Cruda', 'Posición Cruda', 'Señal Acondicionada', 'Posición Acondicionada'])

        #Crea un timer de que cada 0,1s captura las variables
        self.timer = self.create_timer(0.5, self.log_data)  # 10 Hz

        self.get_logger().info(f'Nodo iniciado. Guardando datos en {filename}')

    def raw_signal(self, msg):
        self.current_raw_signal = msg.data

    def conditioned_signal(self, msg):
        self.current_conditioned_signal = msg.data

    def conditioned_position(self, msg):
        self.current_conditioned_position = msg.data

    def raw_position(self, msg):
        self.current_raw_position = msg.data


    def log_data(self):
        stamp = self.get_clock().now().to_msg() # Toma el timepo del ROS2
        time_sec = stamp.sec + (stamp.nanosec * 1e-9)

        self.csv_writer.writerow([time_sec, self.current_raw_signal, self.current_raw_position, self.current_conditioned_signal, self.current_conditioned_position])
        self.csv_file.flush()

        # Muestra en la terminal
        self.get_logger().info(f'Señal Cruda: {self.current_raw_signal:.2f} V  | Pos. Cruda: {self.current_raw_position:.2f}')
        self.get_logger().info(f'Señal Acond.:{self.current_conditioned_signal:.2f} V | Pos. Acond: {self.current_conditioned_position:.2f}')

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Potenciometro()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()