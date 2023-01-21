import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import KeyValue
from diagnostic_msgs.msg import DiagnosticStatus

import serial


class Alchemy(Node):
    """
    Alchemy Node publishes the parsed data from serial port to relevant topics of its own.
    """

    def __init__(self):
        super().__init__('alchemy_node')

        # Create 8 separate publishers with topic names Ammonia, Methane, Subsurface_temp, Atmosphere_temp, Humidity, Atmospheric_pressure, Moisture and CO2
        self.ammonia_pub = self.create_publisher(KeyValue, 'Ammonia', 10)
        self.methane_pub = self.create_publisher(KeyValue, 'Methane', 10)
        self.subsurface_temp_pub = self.create_publisher(
            KeyValue, 'Subsurface_temp', 10)
        self.atmosphere_temp_pub = self.create_publisher(
            KeyValue, 'Atmosphere_temp', 10)
        self.humidity_pub = self.create_publisher(KeyValue, 'Humidity', 10)
        self.atmospheric_pressure_pub = self.create_publisher(
            KeyValue, 'Atmospheric_pressure', 10)
        self.moisture_pub = self.create_publisher(KeyValue, 'Moisture', 10)
        self.co2_pub = self.create_publisher(KeyValue, 'CO2', 10)
        
        #Create publisher for publishing data group wise
        self.gas_pub = self.create_publisher(DiagnosticStatus, 'Gases', 10)
        self.temp_pub = self.create_publisher(DiagnosticStatus, 'Temperatures', 10)
        self.misc_pub = self.create_publisher(DiagnosticStatus, 'Miscellaneous', 10)

        # Prepare serial port for reading Bio information
        self.prepare_serial("/dev/ttyUSB0", baudrate=115200)

        # Set default values for Bio information
        self.BIO_INFO = [0, 0, 0, 0, 0, 0, 0, 0]

        # Create timer to publish Bio information every 1 second
        self.timer = self.create_timer(1, self.publish_bio_info)

        # Create timer to get Bio information from serial port every 1 second
        self.timer = self.create_timer(1, self.get_bio_info)


    def prepare_serial(self, port, baudrate=115200):
        """
        Prepare serial port for reading Bio information
        """
        try:
            self.ser = serial.Serial(port, baudrate)
            self.get_logger().info("Serial port opened successfully")
        except Exception as e:
            self.get_logger().info("Error opening serial port: {}".format(e))

    def get_bio_info(self):
        """
        Parse the message received from serial port and updates the BIO_INFO
        """
        self.BIO_INFO = self.ser.readline().decode().split(",")

    def publish_bio_info(self):
        """
        Publish Bio information to all the topics
        """
        self.publish_ammonia()
        self.publish_methane()
        self.publish_subsurface_temp()
        self.publish_atmosphere_temp()
        self.publish_humidity()
        self.publish_atmospheric_pressure()
        self.publish_moisture()
        self.publish_co2()
        self.publish_gas_data()
        self.publish_temp_data()
        self.publish_misc_data()

    def publish_ammonia(self):
        """
        Publish Ammonia data to Ammonia topic
        """
        msg = KeyValue()
        msg.key = "Ammonia"
        msg.value = str(self.BIO_INFO[0])
        self.ammonia_pub.publish(msg)

    def publish_methane(self):
        """
        Publish Methane data to Methane topic
        """
        msg = KeyValue()
        msg.key = "Methane"
        msg.value = str(self.BIO_INFO[1])
        self.methane_pub.publish(msg)

    def publish_subsurface_temp(self):
        """
        Publish Subsurface_temp data to Subsurface_temp topic
        """
        msg = KeyValue()
        msg.key = "Subsurface_temp"
        msg.value = str(self.BIO_INFO[2])
        self.subsurface_temp_pub.publish(msg)

    def publish_atmosphere_temp(self):
        """
        Publish Atmosphere_temp data to Atmosphere_temp topic
        """
        msg = KeyValue()
        msg.key = "Atmosphere_temp"
        msg.value = str(self.BIO_INFO[3])
        self.atmosphere_temp_pub.publish(msg)

    def publish_humidity(self):
        """
        Publish Humidity data to Humidity topic
        """
        msg = KeyValue()
        msg.key = "Humidity"
        msg.value = str(self.BIO_INFO[4])
        self.humidity_pub.publish(msg)

    def publish_atmospheric_pressure(self):
        """
        Publish Atmospheric_pressure data to Atmospheric_pressure topic
        """
        msg = KeyValue()
        msg.key = "Atmospheric_pressure"
        msg.value = str(self.BIO_INFO[5])
        self.atmospheric_pressure_pub.publish(msg)

    def publish_moisture(self):
        """
        Publish Moisture data to Moisture topic
        """
        msg = KeyValue()
        msg.key = "Moisture"
        msg.value = str(self.BIO_INFO[6])
        self.moisture_pub.publish(msg)
    
    def publish_co2(self):
        """
        Publish CO2 data to CO2 topic
        """
        msg = KeyValue()
        msg.key = "CO2"
        msg.value = str(self.BIO_INFO[7])
        self.co2_pub.publish(msg)
    
    def publish_gas_data(self):
        """
        Publish all the gas data to Gases topic
        """
        msg = DiagnosticStatus()
        fields = [("Ammonia", str(self.BIO_INFO[0]) + " ppm"),
                     ("Methane", str(self.BIO_INFO[1]) + " ppm"),
                     ("CO2", str(self.BIO_INFO[7]) + " ppm")]            

        
        msg.values = [KeyValue()]*len(fields)
        for kv, f in zip(msg.values, fields):
            kv.key = f[0]
            kv.value = f[1]
        
        self.gas_pub.publish(msg)

    def publish_temp_data(self):
        """
        Publish all the temperature data to Temperatures topic
        """
        msg = DiagnosticStatus()
        fields = [("Subsurface_temp", str(self.BIO_INFO[2]) + " C"),
                     ("Atmosphere_temp", str(self.BIO_INFO[3]) + " C")]
        
        msg.values = [KeyValue()]*len(fields)
        for kv, f in zip(msg.values, fields):
            kv.key = f[0]
            kv.value = f[1]

        self.temp_pub.publish(msg)

    def publish_misc_data(self):
        """
        Publish all the miscellaneous data to Miscellaneous topic
        """
        msg = DiagnosticStatus()
        fields = [("Humidity", str(self.BIO_INFO[4]) + " %"),
                     ("Atmospheric_pressure", str(self.BIO_INFO[5]) + " Pa"),
                     ("Moisture", str(self.BIO_INFO[6]) + " %")]
        
        msg.values = [KeyValue()]*len(fields)
        for kv, f in zip(msg.values, fields):
            kv.key = f[0]
            kv.value = f[1]

        self.misc_pub.publish(msg)

def main(args=None):

    rclpy.init(args=args)

    alchemy_node = Alchemy()
    
    rclpy.spin(alchemy_node)

    alchemy_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
