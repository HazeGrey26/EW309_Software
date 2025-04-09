from machine import Pin, PWM, I2C
from time import sleep, ticks_ms
import ttyacm
tty = ttyacm.open(1)

## Credits
print("\nRP2040 EW309 Turret Base Program\nVersion 9 Revision 1 (Revised Apr. 8, 2025)\nSubmission Group: Aaron Goff and Jay Barreto\n")

# Define all functions

# Generates the gains (0-1 floats) for each of the motors using a PI controller
def generateGains(elev_error, azi_error, time_delta, sum_of_error_x, sum_of_error_y):
    # This uses proportional-integral (PI) control
    P = 0.002  # Proportional gain
    I = 0.00005  # Integral gain
    
    # Create Gain for Elevation Motor
    elev_gain = P * elev_error + I * sum_of_error_y
    if elev_gain > 1: elev_gain = 1.0
    elif elev_gain < -1: elev_gain = -1.0
    
    # Create Gain for Azimuth Motor
    azi_gain = P * azi_error + I * sum_of_error_x
    if azi_gain > 1: azi_gain = 1.0
    elif azi_gain < -1: azi_gain = -1.0

    return elev_gain, azi_gain

# Recieves user input from MATLAB to actuate the turret with the keyboard
def centerTurret(tty, color, min_target_area, max_target_area):
    centering_speed = 0.2  # Moves turret at 20% max power
    print("\nCENTERING TURRET VIA MATLAB\n")
    flag = 0  # Becomes 1 when MATLAB senses that 'q' is pressed
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

# Gather data about the turret position and desired target color from the user
def specifyTargets():
    color = input("Color? (r/y/o): ")
    distance_to_board = int(input("How many feet is the turret from the target plane? (Integer from 10-20) ")) 
    return color, distance_to_board

# Apply deadzone compensation and send PWM values to motors
def actuateMotors(elev_float, azi_float):
    elev_deadzone = 7500  # Deadzone with a 500 duty cycle margin
    azi_deadzone = 7500  # Deadzone with a 500 duty cycle margin
    
    elev_duty = int(abs(elev_float*65535) + elev_deadzone)
    azi_duty = int(abs(azi_float*65535) + azi_deadzone)
    # Caps PWM value to 65535 (the max 16 bit integer)
    if elev_duty > 65535: elev_duty = 65535
    if azi_duty > 65535: azi_duty = 65535
    
    # To aim the turret up:
    if elev_float < 0:
        pitch_motor_down.duty_u16(0)
        pitch_motor_up.duty_u16(elev_duty)
    else:  # To aim the turret down:
        pitch_motor_up.duty_u16(0)
        pitch_motor_down.duty_u16(elev_duty)
    # To aim the turret right:
    if azi_float > 0:
        yaw_motor_ccw.duty_u16(0)
        yaw_motor_cw.duty_u16(azi_duty)
    else:  # To aim the turret left:
        yaw_motor_cw.duty_u16(0)
        yaw_motor_ccw.duty_u16(azi_duty)
    return

# Once the PI controller gets the turret close to the target, this higher-precision controller takes over.
def pulsed_directional(elev_error, azi_error):
    # Sends a 50% duty cycle to the motors in 0.02 second pulses
    pulse_magnitude_x = 0.5
    pulse_magnitude_y = 0.5
    pulse_duration = 0.02
    pulse_duty_x = int(pulse_magnitude_x*65535)
    pulse_duty_y = int(pulse_magnitude_y*65535)
    # Assign the 50% PWM values to the motors
    if elev_error == 0:
        pitch_motor_down.duty_u16(65535)
        pitch_motor_up.duty_u16(65535)
    elif elev_error < 0:
        pitch_motor_down.duty_u16(0)
        pitch_motor_up.duty_u16(pulse_duty_y)
    else:
        pitch_motor_up.duty_u16(0)
        pitch_motor_down.duty_u16(pulse_duty_y)
    if azi_error == 0:
        yaw_motor_ccw.duty_u16(65535)
        yaw_motor_cw.duty_u16(65535)
    elif azi_error > 0:
        yaw_motor_ccw.duty_u16(0)
        yaw_motor_cw.duty_u16(pulse_duty_x)
    else:
        yaw_motor_cw.duty_u16(0)
        yaw_motor_ccw.duty_u16(pulse_duty_x)
    sleep(pulse_duration)
    # Brake the motors and pause to allow the motors to come to a stop
    yaw_motor_cw.duty_u16(65535)
    yaw_motor_ccw.duty_u16(65535)
    pitch_motor_up.duty_u16(65535)
    pitch_motor_down.duty_u16(65535)
    sleep(pulse_duration*4)
    return

# Fire the gun for the number of shots desired
def shoot(shots_desired):
    # Initial Variables
    rounds_fired = 0
    sensor_reset = True
    high_current = 2300
    low_current = 1900
    # Current Sensor Definitions
    INA260_ADDRESS = 0x40
    INA260_CURRENT_REGISTER = 0x01
    ina260_i2c = I2C(0, scl=Pin(1), sda=Pin(0))

    # Brake the motors
    yaw_motor_cw.duty_u16(65535)
    yaw_motor_ccw.duty_u16(65535)
    pitch_motor_up.duty_u16(65535)
    pitch_motor_down.duty_u16(65535)

    # Loops until the turret has fired the desired number of shots
    while rounds_fired < shots_desired:
        belt_motor.duty_u16(65535)  # Actuates the ball feed motor
        # Collects data from the INA260 current sensor
        data = ina260_i2c.readfrom_mem(INA260_ADDRESS, INA260_CURRENT_REGISTER, 2)
        raw_current = int.from_bytes(data, 'big')
        if raw_current & 0x8000:
            raw_current -= 1 << 16
        current_ma = raw_current * 1.25  # Convert the INA260 data to mA  
        # Counts a shot if current sensor is above 2.3 amps
        if current_ma > high_current and sensor_reset == True:
            rounds_fired = rounds_fired + 1
            sensor_reset = False
        # Confirms the motor current has settled to below 1.9 amps before counting another shot
        if current_ma < low_current and sensor_reset == False:
            sensor_reset = True
        sleep(0.025)
    
    # Disable the belt feed motor
    belt_motor.duty_u16(0)
    return

# ---------- Main Program ----------

# Initialize motors
# Pitch motors
pitch_motor_down = PWM(Pin(13))
pitch_motor_down.freq(500)
pitch_motor_down.duty_u16(0)
pitch_motor_up = PWM(Pin(12))
pitch_motor_up.freq(500)
pitch_motor_up.duty_u16(0)

# Yaw motors
yaw_motor_ccw = PWM(Pin(9))  # Counterclockwise
yaw_motor_ccw.freq(500)
yaw_motor_ccw.duty_u16(0)           
yaw_motor_cw = PWM(Pin(10))  # Colockwise
yaw_motor_cw.freq(500)
yaw_motor_cw.duty_u16(0)

# Ball feed motor and roller motors
roller_motors = PWM(Pin(15))
roller_motors.freq(500)
roller_motors.duty_u16(0)
belt_motor = PWM(Pin(14))
belt_motor.freq(500)
belt_motor.duty_u16(0)

# Ask the user for the color of the targets and the distance from the turret to the target plane
color, distance_to_board = specifyTargets()

# Determines the holdover and number of shots based on distance to target
drop_compensation = 1.9 * distance_to_board  # Pixels to aim above target
windage_compensation = 1.1 * distance_to_board  # Pixels to aim right of target
min_target_area = 3.14159 * (0.5*distance_to_board)**2  # Minimum target area in pixels (for filtering)
max_target_area = 3.14159 * (0.8*distance_to_board)**2  # Maximum target area in pixels (for filtering)
if distance_to_board <= 10:
    shots_desired = 2
elif distance_to_board <= 15:
    shots_desired = 3
else:
    shots_desired = 4

# Data to Collect Before Starting MATLAB
print("Start MATLAB GUI and UVC Batch File Now\n")
input("Enter a character when MATLAB and UVC services are started: ")

# Center the turret
centerTurret(tty, color, min_target_area, max_target_area)

# Initial variables
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

# Loops until a total of 2 targets are shot. Afterwards, the program is terminated.
while targets_engaged < 2:
    start_time = ticks_ms() # Start collecting time
    
    pause_movement = False  # Reset pause variable
    
    # Tell laptop that the pico is ready for a serial command
    if not shot:
        tty.print('1')
    else:
        tty.print('2')

    # Receive serial command from laptop:
    command_from_laptop = tty.readline()
    command_from_laptop = command_from_laptop.split(",")
    elevation_error = int(command_from_laptop[0]) + drop_compensation
    azimuth_error = int(command_from_laptop[1]) + windage_compensation
    
    # If the laptop says that all errors are zero, stop the turret.
    # This is either the result of the MATLAB program attempting to recover from a loss of tracking
    # or the result of the turret being perfectly on target.
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
    elev_float, azi_float = generateGains(elevation_error, azimuth_error, time_delta, sum_of_error_x, sum_of_error_y)
    # Actuate the motors
    margin_of_error = 15  # Will switch from PI to pulsed controller once the max error is less than 15 pixels
    margin_lower_bound = 5  # Will stop motors and fire once the max error is less than 5 pixels
    # If a tracking error is not causing the turret to be paused, actuate the turret motors
    if not pause_movement:
        if abs(azimuth_error) > margin_of_error or abs(elevation_error) > margin_of_error:
            actuateMotors(elev_float, azi_float)
        elif margin_lower_bound <= abs(azimuth_error) < margin_of_error:
            pulsed_directional(0, azimuth_error)
        elif margin_lower_bound <= abs(elevation_error) < margin_of_error:
            pulsed_directional(elevation_error, 0)
    # If the max error is less than 5 pixels, pause the gun and test errors one more time before firing
    if abs(elevation_error) < margin_lower_bound and abs(azimuth_error) < margin_lower_bound:
        # Brake the motors
        yaw_motor_cw.duty_u16(65535)
        yaw_motor_ccw.duty_u16(65535)
        pitch_motor_up.duty_u16(65535)
        pitch_motor_down.duty_u16(65535)
        sleep(0.4)
        # Refresh errors from MATLAB
        tty.print('1')
        print("Waiting for laptop")
        command_from_laptop = tty.readline()
        print("Done")
        command_from_laptop = command_from_laptop.split(",")
        elevation_error = int(command_from_laptop[0]) + drop_compensation
        azimuth_error = int(command_from_laptop[1]) + windage_compensation
        # If the refreshed errors are still in the acceptable range, fire the gun
        if abs(elevation_error) < margin_lower_bound and abs(azimuth_error) < margin_lower_bound:
            shoot(shots_desired)  # Fire the gun on one target
            targets_engaged = targets_engaged + 1
            sleep(0.5)
            shot = 1  # Flag to say the turret has finished engaging the current target
            if targets_engaged == 1:
                shot_1_x_error = azimuth_error
                shot_1_y_error = elevation_error  # Saves error for shot
            else:
                shot_2_x_error = azimuth_error
                shot_2_y_error = elevation_error  # Saves error for shot
        else:
            shot = 0  # Reset the shot flag to 0 and move on to the next target
        
    # Print report
    time_delta = ticks_ms() - start_time # End collecting time
    #print(f"Frames/Sec: {int(1000/time_delta)}\n")  # Uncomment to see frames per second of tracking system

# Disable the Motors
roller_motors.duty_u16(0)

program_time_end = ticks_ms() # End collecting time
total_time = (program_time_end - program_time_start)/1000  # Find the total time to shoot both targets

# Calculates the degrees per pixel to convert pixel error to degree error
degrees_per_pixel = 69 / (1920 * 0.15);
shot_1_x = shot_1_x_error * degrees_per_pixel;
shot_1_y = shot_1_y_error * degrees_per_pixel;
shot_2_x = shot_2_x_error * degrees_per_pixel;
shot_2_y = shot_2_y_error * degrees_per_pixel;

# Print total time and errors to the terminal
print("\n\n------------- Stats ------------\n\n")
print(f"Time of Engagement = {total_time} Seconds")
print(f"Target 1 Error = {shot_1_x, shot_1_y} Degrees")
print(f"Target 2 Error = {shot_2_x, shot_2_y} Degrees")
print("\n\n------ Program Terminated ------")
