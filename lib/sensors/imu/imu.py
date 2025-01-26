import time

from adafruit_mpu6050 import MPU6050
import board
import numpy as np

from lib.sensors.imu.madgwickahrs import MadgwickAHRS, Quaternion

class FilteredIMU():
    """
    Class for filtering IMU data using the MadgwickAHRS algorithm.
    """

    def __init__(self):
        """
        Initializes the FilteredIMU class with default values and instances.
        """
        self.imu = IMU()
        self.ahrs = MadgwickAHRS(beta=0.008, zeta=0.)
        self.accel = np.array([0, 0, 0])
        self.gyro = np.array([0, 0, 0])
        self.t = time.time()
        self.quat = Quaternion(1, 0, 0, 0)
        self.grav = np.array([0, 0, 0])
        self.ahrs.quaternion = self.quat
        self.alpha = 1  # LPF alpha: x[t] := a*x[t] + (1-a)*x[t-1]

    def calibrate(self):
        """
        Calibrates the IMU and sets the initial quaternion and gravity vector.
        """
        accel, gyro, t = self.imu.calibrate()
        self.accel, self.gyro = self.remap_accel_gyro(accel, gyro)
        self.t = t
        self.quat = self._calculate_initial_q(self.accel)
        self.grav = self.quat_rotate(self.quat.conj(), [0, 0, 1])
        self.ahrs.quaternion = self.quat

    def get_orientation(self):
        """
        Calculates the pitch, roll, and yaw angles of the robot.

        Returns
        -------
            tuple: The pitch, roll, and yaw angles in degrees.
        """
        gx, gy, gz = self.grav
        qw, qx, qy, qz = self.quat

        # Map gravity vector components to new axes
        gX_new = gy
        gY_new = -gx
        gZ_new = gz

        # Calculate pitch (technically roll about x-axis)
        pitch = np.degrees(np.arctan2(-gX_new, np.sqrt(gY_new**2 + gZ_new**2)))

        # Calculate roll (about y-axis)
        roll = np.degrees(np.arctan2(gY_new, gZ_new))

        # Calculate yaw (about z-axis)
        yaw = np.degrees(np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2)))

        return pitch, roll, yaw

    def _calculate_initial_q(self, accel):
        """
        Calculates the initial quaternion based on accelerometer data.
        Args:
            accel (np.array): The accelerometer data.

        Returns
        -------
            Quaternion: The initial quaternion.
        """
        acc_norm = accel / np.linalg.norm(accel)

        # Estimate initial roll and pitch from accelerometer
        initial_roll = np.arctan2(acc_norm[1], acc_norm[2])
        initial_pitch = np.arctan2(-acc_norm[0], np.sqrt(acc_norm[1]**2 + acc_norm[2]**2))
        initial_yaw = 0

        # Initialize quaternion using the from_angle_axis function
        initial_q = Quaternion.from_angle_axis(initial_roll, 1, 0, 0)
        initial_q = initial_q * Quaternion.from_angle_axis(initial_pitch, 0, 1, 0)
        initial_q = initial_q * Quaternion.from_angle_axis(initial_yaw, 0, 0, 1)
        return initial_q

    def update(self):
        """Update the IMU data and filters it using the MadgwickAHRS algorithm."""
        accel, gyro, gyro_raw, t = self.imu.update()
        self.accel, self.gyro = self.remap_accel_gyro(accel, gyro)

        # Store raw IMU data
        self.accel_RAW = self.accel
        self.gyro_RAW = self.gyro
        self.quat_RAW = self._calculate_initial_q(self.accel_RAW)
        self.grav_RAW = self.quat_rotate(self.quat_RAW.conj(), [0, 0, 1])

        # Filtering
        self.ahrs.samplePeriod = t - self.t
        self.ahrs.update_imu(self.gyro, self.accel)
        self.t = t

        # Setting vars
        quat = self.ahrs.quaternion
        self.quat = quat.q
        self.grav = self.quat_rotate(quat.conj(), [0, 0, 1])

    def remap_accel_gyro(self, accel, gyro):
        """
        Remaps the accelerometer and gyroscope data to a new coordinate system.
        Args:
            accel (np.array): The accelerometer data.
            gyro (np.array): The gyroscope data.

        Returns
        -------
            tuple: The remapped accelerometer and gyroscope data.
        """
        accel = np.array([-accel[1], accel[0], accel[2]])
        gyro = np.array([-gyro[1], gyro[0], gyro[2]])
        return accel, gyro

    def quat_rotate(self, q, v):
        """
        Rotates a vector by a quaternion.
        Args:
            q (Quaternion): The quaternion.
            v (list): The vector to rotate.

        Returns
        -------
            np.array: The rotated vector.
        """
        qv = np.concatenate(([0], v))
        return (q * Quaternion(qv) * q.conj()).q[1:]
    
class IMU():
    def __init__(self):
        """
        Initializes the IMU sensor and sets up initial parameters.
        """
        self.sensor = MPU6050(board.I2C())  # Initialize the MPU6050 sensor
        # self.sensor.gyro_range = adafruit_mpu6050.GyroRange.RANGE_1000_DPS # Set gyroscope range to ±1000 dps
        self.gyro_bias = np.array([0., 0., 0.])  # Initialize gyroscope bias

    def calibrate(self):
        """
        Calibrates the gyroscope by averaging 50 readings to determine the bias.
        
        Returns
        -------
        tuple
            A tuple containing the accelerometer data, gyroscope data with bias correction, 
            and the timestamp of the calibration.
        """
        self.gyro_bias = np.array([0., 0., 0.])  # Reset gyro bias
        for _ in range(50):
            _, gyro = self.read_sensor()  # Read sensor data
            self.gyro_bias += gyro / 50  # Accumulate gyro data for bias calculation
            time.sleep(0.01)  # Wait for 10ms between readings
        print('Calculated gyro bias:', self.gyro_bias)

        self.accel, gyro_raw = self.read_sensor()  # Get initial sensor readings
        self.gyro = gyro_raw - self.gyro_bias  # Apply bias correction
        self.t = time.monotonic()  # Record the current time

        return self.accel, self.gyro, self.t

    def read_sensor(self):
        """
        Reads the accelerometer and gyroscope data from the sensor.

        Returns
        -------
        tuple
            A tuple containing the accelerometer data and gyroscope data.
        """
        accel = np.array(self.sensor.acceleration)  # Get accelerometer data
        gyro = np.array(self.sensor.gyro)  # Get gyroscope data
        return accel, gyro

    def update(self):
        """
        Updates the IMU data by reading the sensor and applying bias correction.

        Returns
        -------
        tuple
            A tuple containing the accelerometer data, gyroscope data with bias correction,
            raw gyroscope data, and the timestamp of the sensor reading.
        """
        accel, gyro = self.read_sensor()  # Read sensor data
        self.accel = accel  # Update accelerometer data
        self.gyro = gyro - self.gyro_bias  # Apply bias correction to gyroscope data
        self.gyro_raw = gyro  # Store raw gyroscope data
        self.t = time.monotonic()  # Record the current time

        return self.accel, self.gyro, self.gyro_raw, self.t

    def get_accel(self):
        """
        Returns the accelerometer data.

        Returns
        -------
        numpy.ndarray
            The accelerometer data.
        """
        return self.accel

    def get_gyro(self):
        """
        Returns the gyroscope data with bias correction.

        Returns
        -------
        numpy.ndarray
            The gyroscope data with bias correction.
        """
        return self.gyro
    
    def get_gyro_raw(self):
        """
        Returns the raw gyroscope data without bias correction.

        Returns
        -------
        numpy.ndarray
            The raw gyroscope data.
        """
        return self.gyro_raw
    
    def get_time(self):
        """
        Returns the timestamp of the sensor reading since epoch.

        Returns
        -------
        float
            The timestamp of the sensor reading.
        """
        return self.t

if __name__ == "__main__":
    imu = FilteredIMU()
    imu.calibrate()

    while True:
        imu.update()
        pitch, roll, yaw = imu.get_orientation()
        print(f"roll: {roll}")
        time.sleep(1/200)

    # imu = IMU()  # Create an instance of the IMU class
    # imu.calibrate()  # Calibrate the IMU
    # while True:
    #     accel, gyro, gyro_raw, t = imu.update()  # Update and get sensor data
    #     print("Accelerometer: ", accel)
    #     print("Gyroscope: ", gyro)
    #     print("Gyroscope Raw: ", gyro_raw)
    #     print("Timestamp: ", t)
    #     time.sleep(0.1)  # Wait for 100ms before the next update
