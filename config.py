

# COM port

com = "COM7"


# Sonar Servo motor config

servo1_pin = 2              # pin number in arduino board
max_angle = 180             # maximum angle to reach (in degrees)
min_angle = 0               # minimum angle to reach (in degrees)
time = 4                    # time of the half cycle (from min_angle to max_angle and vice versa) (in seconds)
servo1_offset_angle = 0    # the angle between the zero position of the motor and the actual zero 0 (in degrees)

# sonar config

echo_pin = 4                  # echo pin number in arduino board
trigger_pin = 5               # trigger pin number in arduino board
sonar_offset_distance = 1.6   # the distance between the center of the servo motor axis to the center of the ultrasonic sensor (in cm)

# kalman filter config (sonar kalman filter)

Q = 1e-2            # process noise covariance
R = 2.92e-4         # measurement noise covariance

# visuals config (radar sweep)

window_height = 600    # height of the window (in pixels)
window_width = 1200    # width of the window (in pixels)
range = 30            # maximum range or radius of the sonar mesurements (in cm)

# Laser Servo motor config

servo2_pin = 6               # pin number in arduino board
relative_x_distance = 7.5    # distance between the center of the servo motor axis to laser servo motor axis (in the x axis) (in cm)
relative_y_distance = 0      # distance between the center of the servo motor axis to laser servo motor axis (in the y axis) (in cm) 
servo2_offsit_angle = 10    # the angle between the zero position of the motor and the actual zero 0 (in degrees)

# laser config

laser_pin = 8           # pin number in arduino board