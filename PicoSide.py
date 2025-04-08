from machine import Pin, PWM, I2C
from time import sleep, ticks_ms
#from bno055 import *  # IMU Library
import ttyacm
tty = ttyacm.open(1)

## Credits
print("\nRP2040 EW309 Turret Base Program\nVersion: WIP 7\n(͡°͜ʖ°)\n")

# Define all functions

def generateDutyCycles(elev_error, azi_error, time_delta, sum_of_error_x, sum_of_error_y):
    # This uses proportional control
    #P = 0.003  # Proportional gain
    #I = 0.0001  # Integral gain
    P = 0.002;
    I = 0.00005;
    
    # Elevation Motor
    elev_gain = P * elev_error + I * sum_of_error_y
    if elev_gain > 1: elev_gain = 1.0
    elif elev_gain < -1: elev_gain = -1.0
    
    # Azimuth Motor
    azi_gain = P * azi_error + I * sum_of_error_x
    if azi_gain > 1: azi_gain = 1.0
    elif azi_gain < -1: azi_gain = -1.0

    return elev_gain, azi_gain


def centerTurret(tty, color, min_target_area, max_target_area):
    centering_speed = 0.2
    print("\nCENTERING TURRET VIA MATLAB\n")
    flag = 0
    while flag != 1:
        # Tell laptop that the pico is ready for a serial command
        tty.print(f"{color},{min_target_area},{max_target_area}")
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

def specifyTargets():
    color = input("Color? (r/y/o): ")
    distance_to_board = int(input("What POSITION is the turret in? (1/2/3): ")) 
    return color, distance_to_board

def actuateMotors(elev_float, azi_float):
    elev_deadzone = 7500  # Deadzone with a 500 duty cycle margin
    azi_deadzone = 7500  # Deadzone with a 500 duty cycle margin
    
    elev_duty = int(abs(elev_float*65535) + elev_deadzone)
    azi_duty = int(abs(azi_float*65535) + azi_deadzone)
    if elev_duty > 65535: elev_duty = 65535
    if azi_duty > 65535: azi_duty = 65535
    
    if elev_float < 0:
        pitch_motor_down.duty_u16(0)
        pitch_motor_up.duty_u16(elev_duty)
        #print(f"Elev. Output: {int(elev_duty/65535*100)}%\n")
    else:
        pitch_motor_up.duty_u16(0)
        pitch_motor_down.duty_u16(elev_duty)
        #print(f"Elev. Output: {-int(elev_duty/65535*100)}%\n")
    if azi_float > 0:
        yaw_motor_ccw.duty_u16(0)
        yaw_motor_cw.duty_u16(azi_duty)
        #print(f"Azi. Output: {int(azi_duty/65535*100)}%\n")
    else:
        yaw_motor_cw.duty_u16(0)
        yaw_motor_ccw.duty_u16(azi_duty)
        #print(f"Azi. Output: {-int(azi_duty/65535*100)}%\n")
    return

def pulsed_proportional(elev_error, azi_error):
    pulse_magnitude_x = 0.5
    pulse_magnitude_y = 0.5
    pulse_duration = 0.02
    pulse_duty_x = int(pulse_magnitude_x*65535)
    pulse_duty_y = int(pulse_magnitude_y*65535)
    if elev_error == 0:
        pitch_motor_down.duty_u16(65535)
        pitch_motor_up.duty_u16(65535)
    elif elev_error < 0:
        pitch_motor_down.duty_u16(0)
        pitch_motor_up.duty_u16(pulse_duty_y)
        #print("Pulsing Y")
    else:
        pitch_motor_up.duty_u16(0)
        pitch_motor_down.duty_u16(pulse_duty_y)
        #print("Pulsing Y")
    if azi_error == 0:
        yaw_motor_ccw.duty_u16(65535)
        yaw_motor_cw.duty_u16(65535)
    elif azi_error > 0:
        yaw_motor_ccw.duty_u16(0)
        yaw_motor_cw.duty_u16(pulse_duty_x)
        #print("Pulsing X")
    else:
        yaw_motor_cw.duty_u16(0)
        yaw_motor_ccw.duty_u16(pulse_duty_x)
        #print("Pulsing X")
    sleep(pulse_duration)
    # Brake the motors
    yaw_motor_cw.duty_u16(65535)
    yaw_motor_ccw.duty_u16(65535)
    pitch_motor_up.duty_u16(65535)
    pitch_motor_down.duty_u16(65535)
    sleep(pulse_duration*4)
    return

def shoot(shots_desired):
    # Initial Variables
    rounds_fired = 0
    sensor_reset = True
    base_current = 2500
    # Current Sensor Definitions
    INA260_ADDRESS = 0x40
    INA260_CURRENT_REGISTER = 0x01
    ina260_i2c = I2C(0, scl=Pin(1), sda=Pin(0))

    # Brake the motors
    yaw_motor_cw.duty_u16(65535)
    yaw_motor_ccw.duty_u16(65535)
    pitch_motor_up.duty_u16(65535)
    pitch_motor_down.duty_u16(65535)

    while rounds_fired < shots_desired:
        belt_motor.duty_u16(65535)
        data = ina260_i2c.readfrom_mem(INA260_ADDRESS, INA260_CURRENT_REGISTER, 2)
        raw_current = int.from_bytes(data, 'big')
        if raw_current & 0x8000:
            raw_current -= 1 << 16
        current_ma = raw_current * 1.25  # Convert to mA  
        if current_ma > base_current and sensor_reset:
            rounds_fired = rounds_fired + 1
            sensor_reset = False
        if current_ma < base_current:
            sensor_reset = True
        #print(current_ma)
        sleep(0.025)
    
    # Disable the Motors
    belt_motor.duty_u16(0)
    return

# ---------- Main Program ----------

# Initialize motors
pitch_motor_down = PWM(Pin(13))
pitch_motor_down.freq(500)
pitch_motor_down.duty_u16(0)
pitch_motor_up = PWM(Pin(12))
pitch_motor_up.freq(500)
pitch_motor_up.duty_u16(0)

yaw_motor_ccw = PWM(Pin(9))
yaw_motor_ccw.freq(500)
yaw_motor_ccw.duty_u16(0)           
yaw_motor_cw = PWM(Pin(10))
yaw_motor_cw.freq(500)
yaw_motor_cw.duty_u16(0)

roller_motors = PWM(Pin(15))
roller_motors.freq(500)
roller_motors.duty_u16(0)
belt_motor = PWM(Pin(14))
belt_motor.freq(500)
belt_motor.duty_u16(0)

# Ask the user for target details
color, distance_to_board = specifyTargets()

# Change the holdover and number of shots based on distance to target
if distance_to_board == 1:
    drop_compensation = 19
    windage_compensation = 11
    min_target_area = 80
    max_target_area = 200
    shots_desired = 2
if distance_to_board == 2:
    drop_compensation = 33
    windage_compensation = 12
    min_target_area = 180
    max_target_area = 450
    shots_desired = 2
if distance_to_board == 3:
    drop_compensation = 50
    windage_compensation = 16
    min_target_area = 320
    max_target_area = 800
    shots_desired = 3

# Data to Collect Before Starting MATLAB
print("Start MATLAB GUI and UVC Batch File Now\n")
input("Enter a character when MATLAB and UVC services are started: ")

# Center the turret
centerTurret(tty, color, min_target_area, max_target_area)

time_delta = 0
sum_of_error_x = 0
sum_of_error_y = 0
pause_movement = False
targets_engaged = 0
shot = 0

# Let motors spool up
roller_motors.duty_u16(65535)
sleep(3)

program_time_start = ticks_ms() # Start collecting time

while targets_engaged < 2:
    start_time = ticks_ms() # Start collecting time
    
    pause_movement = False  # Reset pause variable
    
    # Tell laptop that the pico is ready for a serial command
    if not shot:
        tty.print('1')
    else:
        tty.print('2')

    # Receive command from laptop:
    command_from_laptop = tty.readline()
    command_from_laptop = command_from_laptop.split(",")
    elevation_error = int(command_from_laptop[0]) + drop_compensation
    azimuth_error = int(command_from_laptop[1]) + windage_compensation
    #print(elevation_error, azimuth_error)
    
    if (elevation_error - drop_compensation) == (azimuth_error - windage_compensation) == 0:
        pause_movement = True  # Stop the turret if no targets are seen, but keep sum of errors
        yaw_motor_cw.duty_u16(0)
        yaw_motor_ccw.duty_u16(0)
        pitch_motor_up.duty_u16(0)
        pitch_motor_down.duty_u16(0)
        print("PAUSED")
    
    # Add to total error for integral
    sum_of_error_x = sum_of_error_x + azimuth_error
    sum_of_error_y = sum_of_error_y + elevation_error

    # Convert error to duty cycles
    elev_float, azi_float = generateDutyCycles(elevation_error, azimuth_error, time_delta, sum_of_error_x, sum_of_error_y)
    # Actuate the motors
    margin_of_error = 15  # Pixels
    margin_lower_bound = 5 # Pixels
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
        
        if abs(elevation_error) < margin_lower_bound and abs(azimuth_error) < margin_lower_bound:
            shoot(shots_desired)
            targets_engaged = targets_engaged + 1
            sleep(0.5)
            shot = 1
            if targets_engaged == 1:
                shot_1_x_error = azimuth_error
                shot_1_y_error = elevation_error  # Saves error for shot
            else:
                shot_2_x_error = azimuth_error
                shot_2_y_error = elevation_error  # Saves error for shot
        else:
            shot = 0
        
    # Print report
    time_delta = ticks_ms() - start_time # End collecting time
    #print(f"Frames/Sec: {int(1000/time_delta)}\n")

# Disable the Motors
roller_motors.duty_u16(0)

program_time_end = ticks_ms() # End collecting time
total_time = (program_time_end - program_time_start)/1000

degrees_per_pixel = 69 / (1920 * 0.15);
shot_1_x = shot_1_x_error * degrees_per_pixel;
shot_1_y = shot_1_y_error * degrees_per_pixel;
shot_2_x = shot_2_x_error * degrees_per_pixel;
shot_2_y = shot_2_y_error * degrees_per_pixel;

print("\n\n------------- Stats ------------\n\n")
print(f"Time of Engagement = {total_time} Seconds")
print(f"Shot 1 Error = {shot_1_x, shot_1_y} Degrees")
print(f"Shot 2 Error = {shot_2_x, shot_2_y} Degrees")
print("\n\n------ Program Terminated ------")
