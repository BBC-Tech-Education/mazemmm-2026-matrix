from hub import port, light_matrix, motion_sensor
import motor, color_sensor, color, time


################# STATES (Strict Constants) #################
 
MAPPING_STATE    = 0
FORWARD_STATE    = 1
START_STATE      = 2
BLACK_STATE      = 3
GREEN_STATE      = 4
RED_STATE        = 5
TURN_L_90_STATE  = 6
TURN_R_90_STATE  = 7
TURN_L_180_STATE = 8
END_STATE        = 9
FINAL_STATE      = 10

state = START_STATE

################# LRFs #################

LRF_FL = 0
LRF_F  = 1
LRF_FR = 2
LRF_BR = 3
LRF_B  = 4
LRF_BL = 5
LRF_L  = 6
LRF_R  = 7
LRF_FB = 8

lrf_value = [0, 0, 0, 0, 0, 0, 0, 0, 0]


################# Motors #################

MOTOR_L_PORT = port.E
MOTOR_R_PORT = port.F

################# Colour Sensor #################

COLOUR_PORT = port.A

colour = 0

################# Movement #################

TURN_TIME = 750
MIN_FORWARD_DIST = 130
TILE_DIST = 290

BASE_SPEED = -300
TURN_DIST = 250
FORWARD_DIST = 170
LRF_BUFFER = 600

ANGLE_THRESH = 2.5
TURN_P = 15
MAX_TURN = 300
CORRECTION_LRF_DIST = 190
CORRECTION_ANGLE_P = -0.2
CORRECTION_MOTOR_P = 7

target_bearing = 0.0
target_forward_dist = 0
target_backward_dist = 0
forward_target = True

################# Victims #################
red_count = 0
green_count = 0
victim_found = False


################# yaw #################
#
CURRENT_YAW = 0


motion_sensor.reset_yaw(0)
yaw_value = -1 * motion_sensor.tilt_angles()[0]/10


################ Loop management #################
last_loop = 0
LOOP_PERIOD = 20

################ Debug ################
last_print = 0





################################## FUNCTIONS ##################################


# ----------------------------- Reading Sensors ----------------------------- #
def read_lrfs():
    global lrf_value

    lrf_value[LRF_FL] = color_sensor.rgbi(port.D)[0]
    lrf_value[LRF_F] = color_sensor.rgbi(port.D)[1]
    lrf_value[LRF_FR] = color_sensor.rgbi(port.D)[2]
    lrf_value[LRF_BR] = color_sensor.rgbi(port.C)[0]
    lrf_value[LRF_B] = color_sensor.rgbi(port.C)[1]
    lrf_value[LRF_BL] = color_sensor.rgbi(port.C)[2]
    lrf_value[LRF_L] = int((lrf_value[LRF_FL] + lrf_value[LRF_BL]) / 2)
    lrf_value[LRF_R] = int((lrf_value[LRF_FR] + lrf_value[LRF_BR]) / 2)
    lrf_value[LRF_FB] = int((lrf_value[LRF_B]+ lrf_value[LRF_F])/ 2)


def read_colour():
    global reflected_value, colour

    colour = color_sensor.color(COLOUR_PORT)
    reflected_value = color_sensor.reflection(port.A)


def read_imu():
    global yaw_value
    yaw_value = -1 * motion_sensor.tilt_angles()[0] / 10








# ------------------------------- Movement ------------------------------- 
def mapping():
    global lrf_value, state, target_forward_dist, forward_target, victim_found, target_bearing, target_backward_dist
    # imports gloabal varible
    # mapping decides where the robot goes next
    motor.stop(MOTOR_L_PORT)
    motor.stop(MOTOR_R_PORT)

    time.sleep_ms(50)
    read_lrfs()
    # if the lrf is greater than 300 turn left left adds -90 to target bearing so target bearing now = -90
    if lrf_value[LRF_L] > TURN_DIST:
        target_bearing -= 90.0
        if target_bearing < -90.0:
            target_bearing += 360.0
        state = TURN_L_90_STATE
    # if the lrf is greater than 140 move forward
    elif lrf_value[LRF_F] > FORWARD_DIST:
        set_target_distance()
        victim_found = False
        state = FORWARD_STATE

    # same a left turn but opistites
    elif lrf_value[LRF_R] > TURN_DIST:
        target_bearing += 90.0
        if target_bearing > 180.0:
            target_bearing -= 360.0
        state = TURN_R_90_STATE
    else:
        target_bearing -= 180.0
        if target_bearing < -180.0:
            target_bearing += 360.0
        state = TURN_L_180_STATE




def set_target_distance():
    global target_forward_dist, target_backward_dist, forward_target, lrf_value

    if ((lrf_value[LRF_B] + LRF_BUFFER) < lrf_value[LRF_F]):
        forward_target = False
        light_matrix.write("B")
        tiles_away_B= lrf_value[LRF_B] / 300
        target_backward_dist = (tiles_away_B + 1) * 300 ;

        # while 
        # target_backward_dist = lrf_value[LRF_B] + TILE_DIST
        # if target_backward_dist < (TILE_DIST + MIN_FORWARD_DIST):
        #     target_backward_dist = TILE_DIST + MIN_FORWARD_DIST
    else:
        forward_target = True
        light_matrix.write("F")
        tiles_away_F= lrf_value[LRF_F] / 300
        target_forward_dist = (tiles_away_F - 1) * 300 + 40;

        #target_forward_dist = lrf_value[LRF_F] - TILE_DIST
        if target_forward_dist < MIN_FORWARD_DIST:
            target_forward_dist = MIN_FORWARD_DIST


def forward():
    global colour, reflected_value, target_forward_dist, state, victim_found, lrf_value, target_bearing, yaw_value, target_dist, close_lrf

    read_colour()
    if not victim_found and colour is color.GREEN:
        victim_found = True
        state = GREEN_STATE
    elif not victim_found and colour is color.RED:
        state = RED_STATE;
        victim_found = True
    elif reflected_value > 95:
        state = END_STATE
    elif colour is color.BLACK:
        state = BLACK_STATE
    else:
        read_lrfs()

        if forward_target:
            if lrf_value[LRF_F] < target_forward_dist:
                motor.stop(MOTOR_L_PORT)
                motor.stop(MOTOR_R_PORT)
                state = MAPPING_STATE
                return
        elif lrf_value[LRF_B] > target_backward_dist:
            motor.stop(MOTOR_L_PORT)
            motor.stop(MOTOR_R_PORT)
            state = MAPPING_STATE
            return        
        
        if lrf_value[LRF_BL] < CORRECTION_LRF_DIST and lrf_value[LRF_FL] < CORRECTION_LRF_DIST:
            left_value = (lrf_value[LRF_BL] + lrf_value[LRF_FL]) / 2
        elif lrf_value[LRF_BL] < CORRECTION_LRF_DIST:
            left_value = lrf_value[LRF_BL]
        elif lrf_value[LRF_FL] < CORRECTION_LRF_DIST:
            left_value = lrf_value[LRF_FL]
        else:
            left_value = 125

        if lrf_value[LRF_BR] < CORRECTION_LRF_DIST and lrf_value[LRF_FR] < CORRECTION_LRF_DIST:
            right_value = (lrf_value[LRF_BR] + lrf_value[LRF_FR]) / 2
        elif lrf_value[LRF_BR] < CORRECTION_LRF_DIST:
            right_value = lrf_value[LRF_BR]
        elif lrf_value[LRF_FR] < CORRECTION_LRF_DIST:
            right_value = lrf_value[LRF_FR]
        else:
            right_value = 125

        error = left_value - right_value

        angle_adjustment = error * CORRECTION_ANGLE_P

        read_imu()

        setpoint = target_bearing + angle_adjustment

        if setpoint > 180:
            setpoint -= 360
        elif setpoint < -180:
            setpoint += 360

        imu_error = yaw_value - setpoint

        if imu_error > 180:
            imu_error -= 360
        elif imu_error < -180:
            imu_error += 360

        correction = imu_error * CORRECTION_MOTOR_P

        motor.run(MOTOR_L_PORT, BASE_SPEED + int(correction))
        motor.run(MOTOR_R_PORT, BASE_SPEED - int(correction))




def turn_left_90():
    global yaw_value, lrf_value, target_bearing, state, victim_found, target_forward_dist
    read_imu()
    error = yaw_value - target_bearing

    if error < -180.0:
        error += 360.0

    error *= TURN_P

    if abs(error) > MAX_TURN:
        error *= MAX_TURN / abs(error)

    if abs(error) < ANGLE_THRESH:
        motor.stop(MOTOR_L_PORT)
        motor.stop(MOTOR_R_PORT)
        read_lrfs()
        set_target_distance()
        victim_found = False
        state = FORWARD_STATE
    else:
        motor.run(MOTOR_R_PORT, int(-error))
        motor.run(MOTOR_L_PORT, int(error))
    



def turn_right_90():
    global yaw_value, lrf_value, target_bearing, state, victim_found, target_forward_dist
    read_imu()
    error = yaw_value - target_bearing

    if error > 180.0:
        error -= 360.0

    error *= TURN_P

    if abs(error) > MAX_TURN:
        error *= MAX_TURN / abs(error)

    
    if abs(error) < ANGLE_THRESH:
        motor.stop(MOTOR_L_PORT)
        motor.stop(MOTOR_R_PORT)
        read_lrfs()
         
        set_target_distance()
        victim_found = False
        state = FORWARD_STATE
    else:
        motor.run(MOTOR_R_PORT, int(-error))
        motor.run(MOTOR_L_PORT, int(error))




def turn_left_180():
    global state, yaw_value, lrf_value, target_bearing, victim_found, target_forward_dist
    read_imu()
    error = yaw_value - target_bearing

    if error < -180.0:
        error += 360.0

    error *= TURN_P

    if abs(error) > MAX_TURN:
        error *= MAX_TURN / abs(error)

    if abs(error) < ANGLE_THRESH:
        motor.stop(MOTOR_L_PORT)
        motor.stop(MOTOR_R_PORT)
        read_lrfs()

        set_target_distance()
        victim_found = False
        state = FORWARD_STATE
    else:
        motor.run(MOTOR_R_PORT, int(-error))
        motor.run(MOTOR_L_PORT, int(error))



# ------------------------------- Victims ------------------------------- #


def red_victim():
    global state, red_count
    motor.stop(MOTOR_L_PORT)
    motor.stop(MOTOR_R_PORT)
    light_matrix.write("R")
    time.sleep_ms(3000)    
    light_matrix.clear()
    red_count += 1
    state = FORWARD_STATE




def green_victim():
    global state, green_count
    motor.stop(MOTOR_L_PORT)
    motor.stop(MOTOR_R_PORT)
    light_matrix.write("G")
    time.sleep_ms(3000)
    light_matrix.clear()
    green_count += 1
    state = FORWARD_STATE




# ------------------------------- Special Tiles ------------------------------- #

def black_tile():
    global state, lrf_value, target_forward_dist, target_bearing

    read_lrfs()

    if lrf_value[LRF_F] > (target_forward_dist + 250):

        if lrf_value[LRF_FR] > TURN_DIST:
            target_bearing += 90.0
            if target_bearing > 180.0:
                target_bearing -= 360.0
            state = TURN_R_90_STATE
        else:
            target_bearing -= 180.0
            if target_bearing < -90.0:
                target_bearing += 360.0
            state = TURN_L_180_STATE
    else:
        motor.run(MOTOR_L_PORT, -BASE_SPEED)
        motor.run(MOTOR_R_PORT, -BASE_SPEED)



def start_tile():
    global colour, target_forward_dist, state, victim_found, lrf_value, target_bearing, yaw_value

    read_colour()

    if not victim_found and colour is color.GREEN:
        victim_found = True
        state = GREEN_STATE
    elif not victim_found and colour is color.RED:
        state = RED_STATE;
        victim_found = True
    else:
        read_lrfs()

        if lrf_value[LRF_F] < target_forward_dist:
            state = MAPPING_STATE

        if lrf_value[LRF_BL] < CORRECTION_LRF_DIST and lrf_value[LRF_FL] < CORRECTION_LRF_DIST:
            left_value = (lrf_value[LRF_BL] + lrf_value[LRF_FL]) / 2
        elif lrf_value[LRF_BL] < CORRECTION_LRF_DIST:
            left_value = lrf_value[LRF_BL]
        elif lrf_value[LRF_FL] < CORRECTION_LRF_DIST:
            left_value = lrf_value[LRF_FL]
        else:
            left_value = 125

        if lrf_value[LRF_BR] < CORRECTION_LRF_DIST and lrf_value[LRF_FR] < CORRECTION_LRF_DIST:
            right_value = (lrf_value[LRF_BR] + lrf_value[LRF_FR]) / 2
        elif lrf_value[LRF_BR] < CORRECTION_LRF_DIST:
            right_value = lrf_value[LRF_BR]
        elif lrf_value[LRF_FR] < CORRECTION_LRF_DIST:
            right_value = lrf_value[LRF_FR]
        else:
            right_value = 125

        error = left_value - right_value

        angle_adjustment = error * CORRECTION_ANGLE_P

        read_imu()

        setpoint = target_bearing + angle_adjustment

        imu_error = yaw_value - setpoint

        correction = imu_error * CORRECTION_MOTOR_P

        motor.run(MOTOR_L_PORT, BASE_SPEED + int(correction))
        motor.run(MOTOR_R_PORT, BASE_SPEED - int(correction))




def end_tile():
    global state, target_forward_dist, lrf_value, target_bearing, yaw_value, red_count, green_count

    read_lrfs()

    if lrf_value[LRF_F] < target_forward_dist:
        motor.stop(MOTOR_L_PORT)
        motor.stop(MOTOR_R_PORT)
        state = FINAL_STATE
    else:
        if lrf_value[LRF_BL] < CORRECTION_LRF_DIST and lrf_value[LRF_FL] < CORRECTION_LRF_DIST:
            left_value = (lrf_value[LRF_BL] + lrf_value[LRF_FL]) / 2
        elif lrf_value[LRF_BL] < CORRECTION_LRF_DIST:
            left_value = lrf_value[LRF_BL]
        elif lrf_value[LRF_FL] < CORRECTION_LRF_DIST:
            left_value = lrf_value[LRF_FL]
        else:
            left_value = 125

        if lrf_value[LRF_BR] < CORRECTION_LRF_DIST and lrf_value[LRF_FR] < CORRECTION_LRF_DIST:
            right_value = (lrf_value[LRF_BR] + lrf_value[LRF_FR]) / 2
        elif lrf_value[LRF_BR] < CORRECTION_LRF_DIST:
            right_value = lrf_value[LRF_BR]
        elif lrf_value[LRF_FR] < CORRECTION_LRF_DIST:
            right_value = lrf_value[LRF_FR]
        else:
            right_value = 125

        error = left_value - right_value

        angle_adjustment = error * CORRECTION_ANGLE_P

        read_imu()

        setpoint = target_bearing + angle_adjustment

        if setpoint > 180:
            setpoint -= 360
        elif setpoint < -180:
            setpoint += 360

        imu_error = yaw_value - setpoint

        if imu_error > 180:
            imu_error -= 360
        elif imu_error < -180:
            imu_error += 360

        correction = imu_error * CORRECTION_MOTOR_P

        motor.run(MOTOR_L_PORT, BASE_SPEED + int(correction))
        motor.run(MOTOR_R_PORT, BASE_SPEED - int(correction))





################# Main Loop #################


def wait_for_sensor(p, timeout_ms=8000):
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
        try:
            return color_sensor.rgbi(p)
        except OSError:
            time.sleep_ms(100)

    raise RuntimeError("LRF sensor never enumerated")

wait_for_sensor(port.C)
wait_for_sensor(port.D)


read_lrfs()
target_forward_dist = lrf_value[LRF_F] - 270
if target_forward_dist < 300:  
    target_forward_dist = MIN_FORWARD_DIST
victim_found = False
state = START_STATE


while True:

    if time.ticks_diff(time.ticks_ms(), last_print) > 5:
        print("State: ", state, "Target bearing: ", target_bearing, sep="\t", end="\n")
        
        last_print = time.ticks_ms()

    if time.ticks_diff(time.ticks_ms(), last_loop) > LOOP_PERIOD:
        last_loop = time.ticks_ms()

        if state == MAPPING_STATE:
            mapping()
        elif state == FORWARD_STATE:
            forward()
        elif state == END_STATE:
            end_tile()
        elif state == BLACK_STATE:
            black_tile()
        elif state == GREEN_STATE:
            green_victim()
        elif state == RED_STATE:
            red_victim()
        elif state == TURN_L_90_STATE:
            turn_left_90()
        elif state == TURN_R_90_STATE:
            turn_right_90()
        elif state == TURN_L_180_STATE:
            turn_left_180()
        elif state == START_STATE:
            start_tile()
        elif state == FINAL_STATE:
            break
        else:
            state = MAPPING_STATE


light_matrix.write("R")
time.sleep_ms(2000)
light_matrix.write(str(red_count))
time.sleep_ms(2000)
light_matrix.write("G")
time.sleep_ms(2000)
light_matrix.write(str(green_count))
time.sleep_ms(2000)
