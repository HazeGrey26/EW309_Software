from machine import Pin, PWM
from time import sleep, ticks_ms
#from bno055 import *  # IMU Library
import ttyacm
tty = ttyacm.open(1)

# Define all functions

def generateDutyCycles(elev_error, azi_error, time_delta, sum_of_error_x, sum_of_error_y):
    # This uses proportional control
    P = 0.003  # Proportional gain
    I = 0.0001  # Integral gain
    
    # Elevation Motor
    elev_gain = P * elev_error + I * sum_of_error_y
    if elev_gain > 1: elev_gain = 1.0
    elif elev_gain < -1: elev_gain = -1.0
    
    # Azimuth Motor
    azi_gain = P * azi_error + I * sum_of_error_x
    if azi_gain > 1: azi_gain = 1.0
    elif azi_gain < -1: azi_gain = -1.0

    return elev_gain, azi_gain


def centerTurret(tty):
    centering_speed = 0.2
    print("START MATLAB NOW\n")
    input("Press a button when MATLAB is started.\n")
    print("\nCENTERING TURRET VIA MATLAB\n")
    flag = 0
    while flag != 1:
        # Tell laptop that the pico is ready for a serial command
        tty.print('1')
        # Receive command from laptop:
        command_from_laptop = tty.readline()
        command_from_laptop = command_from_laptop.split(",")
        elev_input = int(command_from_laptop[0])
        azi_input = int(command_from_laptop[1])
        flag = int(command_from_laptop[2])
        # Generate PWM float values
        if elev_input == 1:
            elev_float = centering_speed
        elif elev_input == -1:
            elev_float = -centering_speed
        else:
            elev_float = 0
        if azi_input == 1:
            azi_float = centering_speed
        elif azi_input == -1:
            azi_float = -centering_speed
        else:
            azi_float = 0
        # Actuate the motors with the floats
        actuateMotors(elev_float, azi_float)
    return

def actuateMotors(elev_float, azi_float):
    elev_deadzone = 7500
    azi_deadzone = 7300
    
    elev_duty = int(abs(elev_float*65535) + elev_deadzone)
    azi_duty = int(abs(azi_float*65535) + azi_deadzone)
    if elev_duty > 65535: elev_duty = 65535
    if azi_duty > 65535: azi_duty = 65535
    
    if elev_float > 0:
        pitch_motor_down.duty_u16(0)
        pitch_motor_up.duty_u16(elev_duty)
        print(f"Elev. Output: {int(elev_duty/65535*100)}%\n")
    else:
        pitch_motor_up.duty_u16(0)
        pitch_motor_down.duty_u16(elev_duty)
        print(f"Elev. Output: {-int(elev_duty/65535*100)}%\n")
    if azi_float > 0:
        yaw_motor_ccw.duty_u16(0)
        yaw_motor_cw.duty_u16(azi_duty)
        print(f"Azi. Output: {int(azi_duty/65535*100)}%\n")
    else:
        yaw_motor_cw.duty_u16(0)
        yaw_motor_ccw.duty_u16(azi_duty)
        print(f"Azi. Output: {-int(azi_duty/65535*100)}%\n")
    return

def pulsed_proportional(elev_error, azi_error):
    pulse_magnitude_x = 0.65
    pulse_magnitude_y = 0.5
    pulse_duration = 0.008
    pulse_duty_x = int(pulse_magnitude_x*65535)
    pulse_duty_y = int(pulse_magnitude_y*65535)
    if elev_error == 0:
        pitch_motor_down.duty_u16(65535)
        pitch_motor_up.duty_u16(65535)
    elif elev_error > 0:
        pitch_motor_down.duty_u16(0)
        pitch_motor_up.duty_u16(pulse_duty_y)
        print("Pulsing Y")
    else:
        pitch_motor_up.duty_u16(0)
        pitch_motor_down.duty_u16(pulse_duty_y)
        print("Pulsing Y")
    if azi_error == 0:
        yaw_motor_ccw.duty_u16(65535)
        yaw_motor_cw.duty_u16(65535)
    elif azi_error > 0:
        yaw_motor_ccw.duty_u16(0)
        yaw_motor_cw.duty_u16(pulse_duty_x)
        print("Pulsing X")
    else:
        yaw_motor_cw.duty_u16(0)
        yaw_motor_ccw.duty_u16(pulse_duty_x)
        print("Pulsing X")
    sleep(pulse_duration)
    # Brake the motors
    yaw_motor_cw.duty_u16(65535)
    yaw_motor_ccw.duty_u16(65535)
    pitch_motor_up.duty_u16(65535)
    pitch_motor_down.duty_u16(65535)
    sleep(pulse_duration*6)
    return

def SHOOT():
    # Brake the motors
    yaw_motor_cw.duty_u16(65535)
    yaw_motor_ccw.duty_u16(65535)
    pitch_motor_up.duty_u16(65535)
    pitch_motor_down.duty_u16(65535)
    # Fire the gun
    roller_motors.duty_u16(65535)
    sleep(3)  # Let motors spool up
    belt_motor.duty_u16(65535)
    sleep(5)  # FIRE!!!
    roller_motors.duty_u16(0)
    belt_motor.duty_u16(0)
    sleep(100)
    return

# ---------- Main Program ----------

# Initialize motors
pitch_motor_down = PWM(Pin(12))
pitch_motor_down.freq(500)
pitch_motor_down.duty_u16(0)
pitch_motor_up = PWM(Pin(13))
pitch_motor_up.freq(500)
pitch_motor_up.duty_u16(0)
yaw_motor_ccw = PWM(Pin(9))
yaw_motor_ccw.freq(500)
yaw_motor_ccw.duty_u16(0)           
yaw_motor_cw = PWM(Pin(10))
yaw_motor_cw.freq(500)
yaw_motor_cw.duty_u16(0)

roller_motors = PWM(Pin(15))
roller_motors.freq(1500)
roller_motors.duty_u16(0)
belt_motor = PWM(Pin(14))
belt_motor.freq(1500)
belt_motor.duty_u16(0)

# Center the turret
centerTurret(tty)

time_delta = 0
sum_of_error_x = 0
sum_of_error_y = 0
pause_movement = False
while True:
    start_time = ticks_ms() # Start collecting time
    
    pause_movement = False  # Reset pause variable
    
    # Tell laptop that the pico is ready for a serial command
    tty.print('1')
    
    drop_compensation = 3
    windage_compensation = 5
    # Receive command from laptop:
    command_from_laptop = tty.readline()
    command_from_laptop = command_from_laptop.split(",")
    print(command_from_laptop)
    elevation_error = int(command_from_laptop[0]) + drop_compensation
    azimuth_error = int(command_from_laptop[1]) + windage_compensation
    
    if elevation_error == azimuth_error == 0:
        pause_movement = True  # Stop the turret if no targets are seen, but keep sum of errors
        yaw_motor_cw.duty_u16(0)
        yaw_motor_ccw.duty_u16(0)
        pitch_motor_up.duty_u16(0)
        pitch_motor_down.duty_u16(0)
    
    # Add to total error for integral
    sum_of_error_x = sum_of_error_x + azimuth_error
    sum_of_error_y = sum_of_error_y + elevation_error

    # Convert error to duty cycles
    elev_float, azi_float = generateDutyCycles(elevation_error, azimuth_error, time_delta, sum_of_error_x, sum_of_error_y)
    # Actuate the motors
    margin_of_error = 15  # Pixels
    margin_lower_bound = 2 # Pixels
    if not pause_movement:
        if abs(azimuth_error) > margin_of_error or abs(elevation_error) > margin_of_error:
            actuateMotors(elev_float, azi_float)
        elif margin_lower_bound <= abs(azimuth_error) < margin_of_error:
            pulsed_proportional(0, azimuth_error)
        elif margin_lower_bound <= abs(elevation_error) < margin_of_error:
            pulsed_proportional(elevation_error, 0)
    # Shoot?
    if abs(elevation_error) < margin_lower_bound and abs(azimuth_error) < margin_lower_bound:
        # Brake the motors
        yaw_motor_cw.duty_u16(65535)
        yaw_motor_ccw.duty_u16(65535)
        pitch_motor_up.duty_u16(65535)
        pitch_motor_down.duty_u16(65535)
        sleep(0.4)
        # Refresh errors
        tty.print('1')
        print("Waiting for laptop")
        command_from_laptop = tty.readline()
        print("Done")
        command_from_laptop = command_from_laptop.split(",")
        elevation_error = int(command_from_laptop[0]) + drop_compensation
        azimuth_error = int(command_from_laptop[1]) + windage_compensation
        print(command_from_laptop)
        
        if abs(elevation_error) < margin_lower_bound and abs(azimuth_error) < margin_lower_bound:
            SHOOT()
        
    # Print report
    time_delta = ticks_ms() - start_time # End collecting time
    print(f"Frames/Sec: {int(1000/time_delta)}\n")