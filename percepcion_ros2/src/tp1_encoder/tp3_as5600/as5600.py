import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int64, Float32
import csv
from datetime import datetime


class Potenciometro(Node):
    def __init__(self):
        super().__init__('potenciometro')
        # Suscripciones
        self.sub_raw_signal = self.create_subscription(
            Float32, 'potenciometro/voltaje_crudo', self.raw_signal, 10)
        self.sub_raw_position = self.create_subscription(
            Float32, 'potenciometro/posicion_cruda', self.raw_position, 10)
        self.sub_conditioned_signal = self.create_subscription(
            Float32, 'potenciometro/voltaje_acondicionado', self.conditioned_signal, 10)
        self.sub_conditioned_position = self.create_subscription(
            Float32, 'potenciometro/posicion_acondicionada', self.conditioned_position, 10)
        # Valores por defecto
        self.current_raw_signal = 0.0
        self.current_conditioned_signal = 0.0
        self.current_raw_position = 0.0
        self.current_conditioned_position = 0.0
        # Archivo CSV
        filename = f"historial_potenciometro_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.csv_file = open(filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            ['Tiempo', 'Señal Cruda', 'Posición Cruda',
             'Señal Acondicionada', 'Posición Acondicionada'])
        # Timer de captura
        self.timer = self.create_timer(1.0, self.log_data)  # 2 Hz
        self.get_logger().info(f'Nodo potenciometro iniciado. Guardando en {filename}')

    def raw_signal(self, msg):
        self.current_raw_signal = msg.data

    def conditioned_signal(self, msg):
        self.current_conditioned_signal = msg.data

    def conditioned_position(self, msg):
        self.current_conditioned_position = msg.data

    def raw_position(self, msg):
        self.current_raw_position = msg.data

    def log_data(self):
        stamp = self.get_clock().now().to_msg()
        time_sec = stamp.sec + (stamp.nanosec * 1e-9)
        self.csv_writer.writerow([
            time_sec,
            self.current_raw_signal,
            self.current_raw_position,
            self.current_conditioned_signal,
            self.current_conditioned_position,
        ])
        self.csv_file.flush()
        
        self.get_logger().info(
            f'Señal Acond.: {self.current_conditioned_signal:.2f} | '
            f'Pos. Acond: {self.current_conditioned_position:.2f}')

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


class AS5600(Node):
    def __init__(self):
        super().__init__('as5600')
        # Suscripciones
        self.sub_raw_angle = self.create_subscription(
            Int64, 'as5600/raw_angle', self.raw_angle, 10)
        self.sub_angle = self.create_subscription(
            Float32, 'as5600/angle', self.angle, 10)
        # Valores por defecto
        self.current_raw_angle = 0
        self.current_angle = 0.0
        # Archivo CSV
        filename = f"historial_angulos_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.csv_file = open(filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Tiempo', 'Ángulo Crudo', 'Ángulo Convertido'])
        # Timer de captura
        self.timer = self.create_timer(0.5, self.log_data)  # 2 Hz
        self.get_logger().info(f'Nodo as5600 iniciado. Guardando en {filename}')

    def raw_angle(self, msg):
        self.current_raw_angle = msg.data

    def angle(self, msg):
        self.current_angle = msg.data

    def log_data(self):
        stamp = self.get_clock().now().to_msg()
        time_sec = stamp.sec + (stamp.nanosec * 1e-9)
        self.csv_writer.writerow([
            time_sec,
            self.current_raw_angle,
            self.current_angle,
        ])
        self.csv_file.flush()
        self.get_logger().info(
            f'Ángulo crudo: {self.current_raw_angle} | '
            f'Ángulo: {self.current_angle:.2f}')

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    nodo_pot = Potenciometro()
    nodo_as = AS5600()

    executor = MultiThreadedExecutor()
    executor.add_node(nodo_pot)
    executor.add_node(nodo_as)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        nodo_pot.destroy_node()
        nodo_as.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()