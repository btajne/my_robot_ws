#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
import smbus

# MPU6050 Registers
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
DEVICE_ADDRESS = 0x68

class MPU6050_Driver(Node):

    def __init__(self):
        super().__init__("mpu6050_driver")

        # I2C Interface
        self.bus_ = None          # Ensure attribute exists
        self.is_connected_ = False
        self.init_i2c()

        # ROS 2 Interface
        self.imu_pub_ = self.create_publisher(Imu, "/imu/out", qos_profile=qos_profile_sensor_data)
        self.imu_msg_ = Imu()
        self.imu_msg_.header.frame_id = "base_footprint"

        # Timer setup (0.05s = 20Hz)
        self.frequency_ = 0.05
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)

    def timerCallback(self):
        try:
            if not self.is_connected_:
                self.init_i2c()

            if not self.is_connected_:
                self.get_logger().warn("MPU6050 not connected!")
                return

            # Read Accelerometer raw values
            acc_x = self.read_raw_data(ACCEL_XOUT_H)
            acc_y = self.read_raw_data(ACCEL_YOUT_H)
            acc_z = self.read_raw_data(ACCEL_ZOUT_H)

            # Read Gyroscope raw values
            gyro_x = self.read_raw_data(GYRO_XOUT_H)
            gyro_y = self.read_raw_data(GYRO_YOUT_H)
            gyro_z = self.read_raw_data(GYRO_ZOUT_H)

            # Scale values to proper units
            self.imu_msg_.linear_acceleration.x = acc_x / 1670.13
            self.imu_msg_.linear_acceleration.y = acc_y / 1670.13
            self.imu_msg_.linear_acceleration.z = acc_z / 1670.13
            self.imu_msg_.angular_velocity.x = gyro_x / 7509.55
            self.imu_msg_.angular_velocity.y = gyro_y / 7509.55
            self.imu_msg_.angular_velocity.z = gyro_z / 7509.55

            # Publish IMU message
            self.imu_msg_.header.stamp = self.get_clock().now().to_msg()
            self.imu_pub_.publish(self.imu_msg_)

        except OSError as e:
            self.is_connected_ = False
            self.get_logger().warn(f"I2C communication failed: {e}")

    def init_i2c(self):
        try:
            self.bus_ = smbus.SMBus(1)
            self.bus_.write_byte_data(DEVICE_ADDRESS, SMPLRT_DIV, 7)
            self.bus_.write_byte_data(DEVICE_ADDRESS, PWR_MGMT_1, 1)
            self.bus_.write_byte_data(DEVICE_ADDRESS, CONFIG, 0)
            self.bus_.write_byte_data(DEVICE_ADDRESS, GYRO_CONFIG, 24)
            self.bus_.write_byte_data(DEVICE_ADDRESS, INT_ENABLE, 1)
            self.is_connected_ = True
            self.get_logger().info("MPU6050 initialized successfully")
        except OSError as e:
            self.is_connected_ = False
            self.get_logger().error(f"Failed to initialize MPU6050: {e}")

    def read_raw_data(self, addr):
        if self.bus_ is None:
            raise OSError("I2C bus not initialized")

        high = self.bus_.read_byte_data(DEVICE_ADDRESS, addr)
        low = self.bus_.read_byte_data(DEVICE_ADDRESS, addr+1)
        value = (high << 8) | low

        # Convert to signed 16-bit
        if value >= 0x8000:
            value -= 0x10000
        return value


def main():
    rclpy.init()
    mpu6050_driver = MPU6050_Driver()
    rclpy.spin(mpu6050_driver)
    mpu6050_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
