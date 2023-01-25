import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import KeyValue
from diagnostic_msgs.msg import DiagnosticStatus

import serial
import numpy
from itertools import starmap


class PseudoAlchemy(Node):
    """
    Alchemy Node publishes the default ranged data to relevant topics of its own.
    """

    def __init__(self):
        super().__init__('pseudo_node')

        # Create 8 separate publishers with topic names ammonia, methane, subsurface_temp, atmosphere_temp, humidity, atmospheric_pressure, moisture and co2
        self.ammonia_pub = self.create_publisher(KeyValue, 'ammonia', 10)
        self.methane_pub = self.create_publisher(KeyValue, 'methane', 10)
        self.subsurface_temp_pub = self.create_publisher(
            KeyValue, 'subsurface_temp', 10)
        self.atmosphere_temp_pub = self.create_publisher(
            KeyValue, 'atmosphere_temp', 10)
        self.humidity_pub = self.create_publisher(KeyValue, 'humidity', 10)
        self.atmospheric_pressure_pub = self.create_publisher(
            KeyValue, 'atmospheric_pressure', 10)
        self.moisture_pub = self.create_publisher(KeyValue, 'moisture', 10)
        self.co2_pub = self.create_publisher(KeyValue, 'co2', 10)

        # Set default values for Bio information
        self.BIO_INFO = [0, 0, 0, 0, 0, 0, 0, 0]

        #Create publisher for publishing data group wise
        self.gas_pub = self.create_publisher(DiagnosticStatus, 'gases', 10)
        self.temp_pub = self.create_publisher(DiagnosticStatus, 'temperatures', 10)
        self.misc_pub = self.create_publisher(DiagnosticStatus, 'miscellaneous', 10)

        # Create timer to publish Bio information every 1 second
        self.timer = self.create_timer(1, self.publish_bio_info)

        # Create timer to get Bio information from serial port every 1 second
        self.timer = self.create_timer(1, self.make_bio_info)

    def make_bio_info(self):
        """
        Generate  Bio information within default range
        """
        # Generate Bio information within default range
        # default values and deviation for ammonia = (2.5,2.5), methane = (1,1),
        #  subsurface_temp = (30,10), atmosphere_temp = (30,10), humidity = (50,10), 
        # atmospheric_pressure = (1000,100), moisture = (50,10), co2 = (400,100)
        list_mean_deviation = [(2.5, 2.5), (1, 1), (30, 10),
                               (27, 1.5), (65, 7.8), (1000, 100), (18.45, 5.2), (400, 100)]
        
        self.BIO_INFO = list(starmap(self.__make_noisy_data, list_mean_deviation))

    def __make_noisy_data(self, mean, std_deviation):
        """
        Generate noisy data around mean value  according to normal gaussian distribution
        """
        return numpy.random.normal(mean, std_deviation)

    def __make_noisy_data_over_mean(self, mean, std_deviation):
        """
        Generate noisy data over mean value and take mean of the list of data
        according to normal gaussian distribution
        """
        list_data = numpy.random.normal(mean, std_deviation,10000)
        return numpy.mean(list_data)

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

        self.data_atmos_temp = self.__make_noisy_data_over_mean(27, 1.5)

        msg.value = str(self.data_atmos_temp)
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

        self.data_atmos_pressure = self.__make_noisy_data_over_mean(1000, 100)

        msg.value = str(self.data_atmos_pressure)
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

        msg.values = []
        for i in range(len(fields)):
            msg.values.append(KeyValue())
            msg.values[i].key = fields[i][0]
            msg.values[i].value = fields[i][1]

        self.gas_pub.publish(msg)

    def publish_temp_data(self):
        """
        Publish all the temperature data to Temperatures topic
        """
        msg = DiagnosticStatus()

        fields = [("Subsurface_temp", str(self.BIO_INFO[2]) + " C"),
                  ("Atmosphere_temp", str(self.data_atmos_temp) + " C")]

        msg.values = []
        for i in range(len(fields)):
            msg.values.append(KeyValue())
            msg.values[i].key = fields[i][0]
            msg.values[i].value = fields[i][1]

        self.temp_pub.publish(msg)

    def publish_misc_data(self):
        """
        Publish all the miscellaneous data to Miscellaneous topic
        """
        msg = DiagnosticStatus()
        fields = [("Humidity", str(self.BIO_INFO[4]) + " %"),
                  ("Atmospheric_pressure", str(self.data_atmos_pressure) + " Pa"),
                  ("Moisture", str(self.BIO_INFO[6]) + " %")]

        msg.values = []
        for i in range(len(fields)):
            msg.values.append(KeyValue())
            msg.values[i].key = fields[i][0]
            msg.values[i].value = fields[i][1]

        self.misc_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    alchemy_node = PseudoAlchemy()

    rclpy.spin(alchemy_node)

    alchemy_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
