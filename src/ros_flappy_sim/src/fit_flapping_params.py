import numpy as np
import pandas as pd
import os
import rospkg

def fit_sinusoid(t, y, freq):
    """
    Fit y = A * sin(2*pi*freq*t + phase) + offset
    Returns A, phase, offset
    """
    omega = 2 * np.pi * freq
    
    # Estimate offset and amplitude
    offset = (np.max(y) + np.min(y)) / 2
    amp = (np.max(y) - np.min(y)) / 2
    
    # Estimate phase
    # Find time of maximum value
    idx_max = np.argmax(y)
    t_max = t[idx_max]
    
    # sin(omega*t_max + phase) = 1  =>  omega*t_max + phase = pi/2
    phase = np.pi/2 - omega * t_max
    
    # Normalize phase to [-pi, pi]
    phase = (phase + np.pi) % (2 * np.pi) - np.pi
    
    return amp, phase, offset

def main():
    # Get path to CSV file
    try:
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('ros_flappy_sim')
        csv_path = os.path.join(package_path, 'src', 'JointAngleData.csv')
    except:
        # Fallback if rospkg is not available or fails
        current_dir = os.path.dirname(os.path.abspath(__file__))
        csv_path = os.path.join(current_dir, 'JointAngleData.csv')
    
    print(f"Reading data from: {csv_path}")
    
    if not os.path.exists(csv_path):
        print(f"Error: File not found at {csv_path}")
        return

    # Load data
    flap_freq = 6.0
    Angle_data = pd.read_csv(csv_path, header=None)
    
    # Extract joint angles (columns 0 and 1)
    J5_m = Angle_data.loc[:, 0].values
    J6_m = Angle_data.loc[:, 1].values
    
    # Create time vector
    t_m = np.linspace(0, 1.0/flap_freq, num=len(J5_m))
    
    # Fit parameters
    amp_J5, phase_J5, offset_J5 = fit_sinusoid(t_m, J5_m, flap_freq)
    amp_J6, phase_J6, offset_J6 = fit_sinusoid(t_m, J6_m, flap_freq)
    
    print("\n" + "="*50)
    print("       FLAPPING CONTROLLER PARAMETERS")
    print("="*50)
    print(f"Frequency: {flap_freq} Hz")
    print("-" * 50)
    print(f"Joint 5 (Proximal):")
    print(f"  Amplitude: {amp_J5:.6f} rad")
    print(f"  Phase:     {phase_J5:.6f} rad")
    print(f"  Offset:    {offset_J5:.6f} rad")
    print("-" * 50)
    print(f"Joint 6 (Distal):")
    print(f"  Amplitude: {amp_J6:.6f} rad")
    print(f"  Phase:     {phase_J6:.6f} rad")
    print(f"  Offset:    {offset_J6:.6f} rad")
    print("="*50)
    
    print("\nCopy and paste this into your dual_render_test.py:")
    print("-" * 50)
    print(f"""flapping_ctrl = FlappingController(
    freq={flap_freq},
    amp_J5={amp_J5:.4f},
    amp_J6={amp_J6:.4f},
    phase_J5={phase_J5:.4f},
    phase_J6={phase_J6:.4f},
    offset_J5={offset_J5:.4f},
    offset_J6={offset_J6:.4f}
)""")
    print("-" * 50)

if __name__ == "__main__":
    main()
