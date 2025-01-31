import numpy as np
from navlie import MeasurementModel
from navlie.lib import IMUState
from pymlg.numpy import SO3

from lib.localization.modelling.spline_smoothing import EncoderVelocitySmoother

class ZuPTDetector:
    def __init__(self, loc_config : dict):
        """
        Initialize the ZuPTDetector with the localization configuration.

        Parameters
        ----------
        loc_config (dict): The localization configuration dictionary.
        """
        # retrieve ZuPT threshold
        self.zupt_threshold = loc_config['zupt_params']['zupt_threshold']

        # retrieve spline sample #
        self.spline_samples = loc_config['zupt_params']['spline_samples']

        # initialize spline smoother for velocity
        self.encoder_smoother = EncoderVelocitySmoother(self.spline_samples, self.zupt_threshold)

    def detect(self, encoder_data : np.array, t_k : float) -> bool:
        
        """
        Perform zero-velocity detection by adding to the encoder velocity smoother and checking if the smoothed velocity is below the threshold.

        Parameters
        ----------
        encoder_data (np.array): 
            The encoder velocity data, a np.array of shape (1, 2), with the left and right encoder velocities.

        t_k (float):
            The timestamp of the encoder data.
        """

        # add encoder data to the smoother
        self.encoder_smoother.add_encoder_velocity_sample(encoder_velocity=encoder_data, t_k=t_k)

        # fit spline
        self.encoder_smoother.fit_encoder_velocity_spline()

        # check if the smoothed velocity is below the threshold
        return self.encoder_smoother.get_zero_velocity()

class ZuPT(MeasurementModel):
    """
    Measurement model for the Zero-velocity update (ZuPT) algorithm. We use an internally smoothened version of SplineSmoother to retrieve the zero linear velocity and angular velocity constraints separately, then correct.
    """

    def __init__(self, perturbation : str, R_ang : float, R_vel : float, g_a : np.array):
        
        # Define the perturbation direction
        self.perturbation = perturbation

        # define noises
        self.R_ang = R_ang
        self.R_vel = R_vel

        # define covariance
        self.R = np.eye(6)

        self.R[:3, :3] *= R_ang
        self.R[3:, 3:] *= R_vel

        # define gravity
        self.g_a = g_a

    def evaluate(self, x: IMUState) -> np.ndarray:

        """
        Evaluate measurement value given an IMUState object.

        Parameters
        ----------
        x (IMUState): The state containing attitude and velocity.
        """

        # retrieve relevant state values
        C_ab = x.attitude

        b_a = x.bias_accel
        b_g = x.bias_gyro

        # zero-declare measurement
        z = np.zeros((6, 1))

        z[:3] = b_g.reshape(3, 1)

        z[3:] = b_a.reshape(3, 1) + C_ab.T @ self.g_a

        return z
    
    def jacobian(self, x : IMUState) -> np.ndarray:
        
        """
        Compute the Jacobian of the measurement model.

        Parameters
        ----------
        x (IMUState): The state containing attitude and bias.
        """

        if self.perturbation == "left":

            # retrieve relevant state values
            C_ab = x.attitude

            # zero-declare Jacobian
            H = np.zeros((6, 15))

            H[:3, 9:12] = np.eye(3)
            H[3:, :3] = C_ab.T @ SO3.wedge(self.g_a)
            H[3:, 12:15] = np.eye(3)

            return H
        
        elif self.perturbation == "right":

            # retrieve relevant state values
            C_ab = x.attitude

            # zero-declare Jacobian
            H = np.zeros((6, 15))

            H[:3, 9:12] = np.eye(3)
            H[3:, :3] = SO3.wedge(C_ab.T @ self.g_a)
            H[3:, 12:15] = np.eye(3)

            return H
        
        else:
            raise ValueError("Invalid perturbation direction in ZuPT model!")
        
    def covariance(self, x : IMUState) -> np.ndarray:
        
        """
        Return the covariance matrix for the measurement model.

        Parameters
        ----------
        x (IMUState): The state (not used in this function).
        """

        return self.R