#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import struct
import math
import time
import threading

class LD19LidarNode(Node):
    def __init__(self):
        super().__init__('ld19_lidar_node')
        
        # Parámetros configurables
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 230400)
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('scan_frequency', 10.0)
        
        # Obtener parámetros
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.scan_frequency = self.get_parameter('scan_frequency').get_parameter_value().double_value
        
        # Publisher para los datos del LaserScan
        self.laser_pub = self.create_publisher(LaserScan, 'scan', 10)
        
        # Variables para el procesamiento del LD19
        self.serial_port = None
        self.ranges = [0.0] * 360  # 360 grados
        self.intensities = [0.0] * 360
        self.packet_buffer = bytearray()
        self.last_scan_time = time.time()
        self.scan_complete = False
        
        # Lock para thread safety
        self.data_lock = threading.Lock()
        
        # Inicializar conexión serie
        if self.init_serial():
            # Thread para leer datos continuamente
            self.read_thread = threading.Thread(target=self.read_lidar_loop, daemon=True)
            self.read_thread.start()
            
            # Timer para publicar los scans
            self.timer = self.create_timer(1.0/self.scan_frequency, self.publish_scan)
        
        self.get_logger().info(f'LD19 LiDAR node iniciado en puerto: {self.port}')

    def init_serial(self):
        """Inicializar la conexión serie con el LiDAR"""
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            self.get_logger().info(f'Conexión serie establecida: {self.port} @ {self.baudrate}')
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'Error al abrir puerto serie: {e}')
            return False

    def read_lidar_loop(self):
        """Loop continuo para leer datos del LiDAR en thread separado"""
        while rclpy.ok() and self.serial_port and self.serial_port.is_open:
            try:
                # Leer datos disponibles
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    self.packet_buffer.extend(data)
                    self.process_buffer()
                else:
                    time.sleep(0.001)  # Pequeña pausa para no saturar CPU
            except Exception as e:
                self.get_logger().warn(f'Error leyendo datos: {e}')
                time.sleep(0.1)

    def process_buffer(self):
        """Procesar el buffer de datos del LD19"""
        while len(self.packet_buffer) >= 47:  # Tamaño mínimo del paquete LD19
            # Buscar el header del paquete (0x54)
            start_idx = -1
            for i in range(len(self.packet_buffer) - 46):
                if self.packet_buffer[i] == 0x54:  # Header byte
                    start_idx = i
                    break
            
            if start_idx == -1:
                # No se encontró header, limpiar buffer
                self.packet_buffer.clear()
                break
            
            # Remover datos antes del header
            if start_idx > 0:
                del self.packet_buffer[:start_idx]
            
            # Verificar que tenemos suficientes datos para un paquete completo
            if len(self.packet_buffer) < 47:
                break
            
            # Extraer y procesar el paquete
            packet = self.packet_buffer[:47]
            del self.packet_buffer[:47]
            
            self.parse_ld19_packet(packet)

    def parse_ld19_packet(self, packet):
        """Parsear un paquete de datos del LD19"""
        try:
            if len(packet) < 47:
                return
            
            # Verificar header
            if packet[0] != 0x54:
                return
            
            # Extraer información del paquete
            ver_len = packet[1]
            speed = struct.unpack('<H', packet[2:4])[0]  # RPM
            start_angle = struct.unpack('<H', packet[4:6])[0]  # Ángulo inicial * 100
            
            # Procesar 12 puntos de datos (cada punto: 3 bytes)
            for i in range(12):
                base_idx = 6 + i * 3
                if base_idx + 2 >= len(packet):
                    continue
                
                # Extraer distancia e intensidad
                distance_raw = struct.unpack('<H', packet[base_idx:base_idx+2])[0]
                intensity = packet[base_idx+2]
                
                # Calcular ángulo para este punto
                angle_step = i * 0.167  # Aproximadamente 0.167 grados por punto
                angle = (start_angle / 100.0 + angle_step) % 360.0
                
                # Corregir orientación (invertir para arreglar efecto espejo)
                angle = (360.0 - angle) % 360.0
                
                # Convertir distancia (el LD19 reporta en mm, convertir a metros)
                distance = distance_raw / 1000.0 if distance_raw > 0 else 0.0
                
                # Validar rango
                if distance > 12.0:  # Máximo del LD19
                    distance = 0.0
                if distance < 0.12:  # Mínimo del LD19
                    distance = 0.0
                
                # Almacenar en el array (thread safe)
                angle_idx = int(angle) % 360
                with self.data_lock:
                    self.ranges[angle_idx] = distance
                    self.intensities[angle_idx] = float(intensity)
            
            # Debug: mostrar información ocasionalmente
            current_time = time.time()
            if current_time - self.last_scan_time > 2.0:  # Cada 2 segundos
                self.get_logger().info(f'LiDAR RPM: {speed/100.0:.1f}, Start angle: {start_angle/100.0:.1f}°')
                self.last_scan_time = current_time
                
        except Exception as e:
            self.get_logger().warn(f'Error parseando paquete: {e}')

    def publish_scan(self):
        """Publicar mensaje LaserScan"""
        scan_msg = LaserScan()
        
        # Header
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = self.frame_id
        
        # Parámetros del LiDAR LD19
        scan_msg.angle_min = 0.0
        scan_msg.angle_max = 2 * math.pi
        scan_msg.angle_increment = math.radians(1.0)  # 1 grado por medición
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 1.0 / self.scan_frequency
        
        # Rangos del LD19
        scan_msg.range_min = 0.12  # 12 cm mínimo
        scan_msg.range_max = 12.0  # 12 m máximo
        
        # Copiar datos de forma thread-safe
        with self.data_lock:
            scan_msg.ranges = self.ranges.copy()
            scan_msg.intensities = self.intensities.copy()
        
        # Publicar
        self.laser_pub.publish(scan_msg)

    def destroy_node(self):
        """Limpiar recursos al cerrar"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        lidar_node = LD19LidarNode()
        rclpy.spin(lidar_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'lidar_node' in locals():
            lidar_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()