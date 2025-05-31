def format_imu_data(data):
    # Convert IMU data to a format suitable for ROS messages
    return {
        'orientation': data['orientation'],
        'angular_velocity': data['angular_velocity'],
        'linear_acceleration': data['linear_acceleration']
    }

def format_camera_frame(image):
    # Convert image data to a format suitable for ROS messages
    return {
        'header': {
            'stamp': image['timestamp'],
            'frame_id': image['frame_id']
        },
        'data': image['data']
    }

def format_axes_data(axes):
    # Convert axes data to a format suitable for ROS messages
    return {
        'x_axis': axes['x'],
        'y_axis': axes['y'],
        'z_axis': axes['z']
    }