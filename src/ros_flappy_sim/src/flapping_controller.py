import numpy as np

class FlappingController:
    def __init__(self, freq, amp_J5, amp_J6, phase_J5=0.0, phase_J6=0.0, offset_J5=0.0, offset_J6=0.0):
        """
        Initialize the sinusoidal flapping controller.
        
        Args:
            freq (float): Flapping frequency in Hz
            amp_J5 (float): Amplitude for Joint 5 (rad)
            amp_J6 (float): Amplitude for Joint 6 (rad)
            phase_J5 (float): Phase shift for Joint 5 (rad)
            phase_J6 (float): Phase shift for Joint 6 (rad)
            offset_J5 (float): Center offset for Joint 5 (rad)
            offset_J6 (float): Center offset for Joint 6 (rad)
        """
        self.freq = freq
        self.omega = 2 * np.pi * freq
        
        self.amp_J5 = amp_J5
        self.amp_J6 = amp_J6
        
        self.phase_J5 = phase_J5
        self.phase_J6 = phase_J6
        
        self.offset_J5 = offset_J5
        self.offset_J6 = offset_J6

    def update(self, time):
        """
        Calculate joint angles and velocities for a given time.
        
        Args:
            time (float): Current simulation time
            
        Returns:
            tuple: (J5_pos, J6_pos, J5_vel, J6_vel)
        """
        # Calculate angles: A * sin(omega * t + phi) + offset
        J5_pos = self.amp_J5 * np.sin(self.omega * time + self.phase_J5) + self.offset_J5
        J6_pos = self.amp_J6 * np.sin(self.omega * time + self.phase_J6) + self.offset_J6
        
        # Calculate velocities: A * omega * cos(omega * t + phi)
        J5_vel = self.amp_J5 * self.omega * np.cos(self.omega * time + self.phase_J5)
        J6_vel = self.amp_J6 * self.omega * np.cos(self.omega * time + self.phase_J6)
        
        return J5_pos, J6_pos, J5_vel, J6_vel
