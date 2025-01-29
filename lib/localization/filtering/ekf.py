import numpy as np
import scipy.constants
import torch
from navlie.filters import ExtendedKalmanFilter
from navlie.lib import IMU, IMUKinematics, IMUState
from navlie.lib.imu import get_unbiased_imu
from navlie.types import Measurement, StateWithCovariance
from pymlg.numpy import SE23
from pymlg.torch import SO3 as SO3t

import lib.constants as CFG
from lib.localization.modelling.wheel_encoder_mm import WheelEncoderGravityAligned


class WheelEncoderEKF():
    """EKF for a simple wheel encoder model"""

    def __init__(self):
        """Initialize the WheelEncoderEKF with the localization configuration."""

        # create the wheel encoder measurement model
        R_enc_meas = CFG.LOCALIZATION_UNCERTAINTY_R_ENC
        R_enc_pseudo = CFG.LOCALIZATION_UNCERTAINTY_R_ENC_PSEUDO

        # form measurement uncertainty matrix
        R_enc = np.eye(3)
        R_enc[0, :] *= R_enc_pseudo
        R_enc[1, :] *= R_enc_meas
        R_enc[2, :] *= R_enc_pseudo
        self.encoder = WheelEncoderGravityAligned(R_enc)

        sigma_gyro_ct = CFG.LOCALIZATION_IMU_SIGMA_GYRO_CT
        sigma_accel_ct = CFG.LOCALIZATION_IMU_SIGMA_ACCEL_CT
        sigma_gyro_bias_ct = CFG.LOCALIZATION_IMU_SIGMA_GYRO_BIAS_CT
        sigma_accel_bias_ct = CFG.LOCALIZATION_IMU_SIGMA_ACCEL_BIAS_CT

        self.delta_t = 1 / CFG.LOCALIZATION_IMU_UPDATE_RATE

        Q_c = np.eye(12)
        Q_c[0:3, 0:3] *= sigma_gyro_ct**2
        Q_c[3:6, 3:6] *= sigma_accel_ct**2
        Q_c[6:9, 6:9] *= sigma_gyro_bias_ct**2
        Q_c[9:12, 9:12] *= sigma_accel_bias_ct**2

        # define gravity-up vector
        if CFG.LOCALIZATION_PHYSICAL_GRAVITY_UP:
            self.g_a = np.array([0, 0, scipy.constants.g]).reshape(3, 1)
        else:
            self.g_a = np.array([0, 0, -scipy.constants.g]).reshape(3, 1)

        self.Q_imu = Q_c / self.delta_t
        self.ekf = ExtendedKalmanFilter(IMUKinematics(self.Q_imu, self.g_a))

        self.x_k = None

        self.t_k = None

        self.delta_t = None

    def get_unbiased_imu(self, u : IMU):
        """
        Helper to retrieve unbiased IMU measurements

        Parameters
        ----------
            u : IMU
                The IMU measurement
        
        Returns
        -------
            omega_b_ub : np.ndarray with shape (3, 1)
                The unbiased angular velocity measurement
            a_b_ub : np.ndarray with shape (3, 1)
                The unbiased acceleration measurement
        """
        u_ub = get_unbiased_imu(self.x_k, u)

        # retrieve individual components
        omega_b_ub = u_ub.gyro.reshape(3, 1)
        a_b_ub = u_ub.accel.reshape(3, 1)

        return omega_b_ub, a_b_ub

    def zero_initialize(self):
        """

        Initialize the internal state of the EKF to zero. Mimicking a SLAM initialization procedure minus the static IMU initialization.

        """
        r_a_0 = np.zeros((3, 1))
        v_a_0 = np.zeros((3, 1))
        phi_0 = np.zeros((3, 1))
        C_0 = SO3t.Exp(torch.Tensor(phi_0).reshape(-1, 3, 1)).numpy().reshape(3, 3)
        x_0 = SE23.from_components(C_0, v_a_0, r_a_0)

        x_k_pose = IMUState(x_0, np.zeros((3, 1)), np.zeros((3, 1)), direction="left")

        self.x_k = StateWithCovariance(x_k_pose, np.ones((15, 15)) * 1e-2)

    def static_initialize(self, b_g : np.ndarray, b_a : np.ndarray, C_ab_0 : np.ndarray):
        """
        Initializes EKF with the results of an inertial-only static initialization procedure

        Parameters
        ----------
            b_g : np.ndarray with shape (3, 1)
                The gyro bias
            b_a : np.ndarray with shape (3, 1)
                The accel bias
            C_ab_0 : np.ndarray with shape (3, 3)
                The initial rotation estimate
        """
        r_a_0 = np.zeros((3, 1))
        v_a_0 = np.zeros((3, 1))
        x_0 = SE23.from_components(C_ab_0, v_a_0, r_a_0)

        x_k_pose = IMUState(x_0, b_g, b_a, direction="left")

        P_0 = np.eye(15)

        # allocate specific uncertainties based on configuration
        P_0[0:3, 0:3] = np.eye(3) * np.array(CFG.LOCALIZATION_UNCERTAINTY_P_PHI, dtype=np.float64)
        P_0[3:6, 3:6] = np.eye(3) * np.array(CFG.LOCALIZATION_UNCERTAINTY_P_VEL, dtype=np.float64)
        P_0[6:9, 6:9] = np.eye(3) * np.array(CFG.LOCALIZATION_UNCERTAINTY_P_POS, dtype=np.float64)
        P_0[9:12, 9:12] = np.eye(3) * np.array(CFG.LOCALIZATION_UNCERTAINTY_P_GYRO_BIAS, dtype=np.float64)
        P_0[12:15, 12:15] = np.eye(3) * np.array(CFG.LOCALIZATION_UNCERTAINTY_P_ACCEL_BIAS, dtype=np.float64)

        self.x_k = StateWithCovariance(x_k_pose, P_0)

    def initialize_gyro_bias(self, b_g : np.ndarray):
        """
        Initialize the gyro bias of the EKF

        Parameters
        ----------
            b_g : np.ndarray with shape (3, 1)
                The gyro bias
        """
        self.x_k.state.value[1].value = b_g.ravel()

        # reset internal covariance associated with bias to low uncertainty
        # self.x_k.covariance[9:12, 9:12] = np.eye(3) * 1e-6

        print("Initialized gyro bias: ", b_g)

    def initialize_accel_bias(self, b_a : np.ndarray):
        """
        Initialize the accel bias of the EKF

        Parameters
        ----------
            b_a : np.ndarray with shape (3, 1)
                The accel bias
        """
        self.x_k.state.value[2].value = b_a.ravel()

        print("Initialized accel bias: ", b_a)

    def predict(self, u : IMU, t_k : float):
        """
        Propagate the state estimate forward in time

        Parameters
        ----------
            x_k : StateWithCovariance
                the current IMUState (extended pose + bias) and associated covariance
            u : IMU
                the IMU measurement
            dt : float
                the time step

        Returns
        -------
            x_k_1 : StateWithCovariance
                the IMUState and associated covariance at the next time step
        """
        if self.t_k is None:
            self.t_k = t_k
        else:
            self.delta_t = t_k - self.t_k
            self.x_k = self.ekf.predict(self.x_k, u, self.delta_t)

            # print("Applied delta_t: ", self.delta_t)

            self.t_k = t_k

    def correct(self, v_l : float, v_r : float):
        """
        Correct the state estimate with a wheel encoder measurement
        """
        # convert using simple encoder model
        v_b_enc = (v_l + v_r) / 2
        v_b_enc = np.array([0, v_b_enc, 0]).reshape(3, 1)

        enc_meas = Measurement(value = v_b_enc, model = self.encoder)

        self.x_k = self.ekf.correct(x=self.x_k, y=enc_meas, u=None)
