from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='iotbot_shield',
            executable='iotbot_shield_node',
            parameters=
            [
                {"portName": "/dev/ttyS1"},     # serial port file of the linux system [e.g. /dev/ttyS1]
                {"baudrate": 115200},           # speed of transmittion in bits/s [9600, 19200, 38400, 57600, 115200]
                {"dataBits": 8},                # amount of data bits [5, 6, 7, 8]
                {"stopBits": 1},                # amount of data bits [1, 2]
                {"parity": 0},                  # operational parity mode [NONE: 0, EVEN: 1, ODD: 2] 
                {"xonXoff": 0},                 # software flow control [enable: 1, disable: 0]
                {"rtsCts": 0},                  # hardware flow control [enable: 1, disable: 0]
                {"gearRatio": 70.00},           # Economic Gearbox
                {"encoderTicksPerRev": 64.00},  # Economic Encoder
                {"kp": 0.50},                   # Regulator P
                {"ki": 5.00},                   # Regulator I
                {"kd": 0.00},                   # Regulator D
                {"controlFrequency": 16000},    # Control Frequency
                {"lowPassInputFilter": 0.20},   # Input Low Pass Filter
                {"lowPassEncoderTicks": 0.30},  # Low Pass Encoder
                {"rawIMUData": 0},              # IMU raw data [enable: 1, disable: 0]
                {"driftWeight": 0.02}           # IMU drift weight [0..1]
            ]
        ),
        Node(
            package='iotbot_motion_control',
            executable='iotbot_motion_differential_node',
            parameters=
            [
                {"gearRatio": 70.00},           # Economic Gearbox
                {"encoderTicksPerRev": 64.00},  # Economic Encoder
                {"maxRPM": 80.00},              # Economic max. Speed
                {"track": 0.35},                # IOTBot's Track-Dimension
                {"wheelBase": 0.24},            # IOTBot's Wheelbase
                {"wheelDiameter": 0.17}         # 17cm RC-Wheels
            ]
        ),
        Node(
            package='joy',
            executable='joy_node'
        ),
    ])
