from enum import Enum

class PID_type(Enum):
    """Enumeration for different types of PID controllers."""
    P = 0
    PD = 1
    PI = 2
    PID = 3

class PID_ctrl:
    """A class representing a PID controller."""

    def __init__(self, type_: PID_type, kp=1.2, kv=0.8, ki=0.2, history_length=3):
        """
        Initialize the PID controller with given parameters.

        :param type_: The type of PID controller (P, PD, PI, PID).
        :param kp: Proportional gain.
        :param kv: Derivative gain.
        :param ki: Integral gain.
        :param history_length: Number of past errors to keep for calculation.
        :param filename_: Filename to store errors (not used in current implementation).
        """
        self.history_length = history_length
        self.history = []
        self.type: PID_type = type_

        self.kp = kp
        self.kv = kv
        self.ki = ki

    def update(self, error, stamp, status):
        """
        Update the PID controller with a new error and timestamp.

        :param error: The current error value.
        :param stamp: The current timestamp.
        :param status: Boolean indicating if the controller should update.
        :return: Tuple of control values (0, 0) if status is False, otherwise the result of __update.
        """
        if status == False:
            self.__update(error, stamp)
            return 0, 0
        else:
            return self.__update(error, stamp)
        
    def __update(self, error, stamp):
        """
        Internal method to update the PID controller's state.

        :param error: The current error value.
        :param stamp: The current timestamp.
        :return: The control output based on the PID type.
        """
        latest_error = error
        
        # Append the current error and timestamp to the history
        self.history.append((error, stamp))        
        
        # Maintain the history length
        if len(self.history) > self.history_length:
            self.history.pop(0)

        # If history is not full, return proportional control
        if len(self.history) != self.history_length:
            return self.kp * latest_error
        
        dt_avg = 0
        error_dot = 0

        # Calculate average time difference and error derivative
        for i in range(1, len(self.history)):
            t0 = self.history[i-1][1]
            t1 = self.history[i][1]
            
            dt = (t1 - t0)

            dt_avg += dt
            error_dot += (self.history[i][0] - self.history[i-1][0]) / dt
            
        error_dot /= len(self.history)
        dt_avg /= len(self.history)

        # Calculate integral of error
        sum_ = 0
        for hist in self.history:
            sum_ += hist[0]
        
        error_int = sum_ * dt_avg

        # Return control output based on the type of PID controller
        if self.type == PID_type.P:
            return self.kp * latest_error
        
        elif self.type == PID_type.PD:
            return self.kp * latest_error + self.kv * error_dot
        
        elif self.type == PID_type.PI:
            return self.kp * latest_error + self.ki * error_int
        
        elif self.type == PID_type.PID:
            return self.kp * latest_error + self.kv * error_dot + self.ki * error_int
