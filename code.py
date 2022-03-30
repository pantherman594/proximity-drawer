from adafruit_motor import stepper
import adafruit_vl53l1x
import board
import digitalio
import math
import neopixel
import time

# CONFIGURATION

# measurements in mm
ARM_A_LENGTH = 159.25 # length of the entire arm
ARM_B_LENGTH = 177.0

RANGE = 20
DISTANCE = 5

# Limit the drawable size to the size of a US Letter-sized page
MAX_X = 215.9
MIN_X = 0
MAX_Y = 139.7
MIN_Y = -139.7

# Connect these to the distance sensors' xshut pin
SENSOR_A_PIN = board.D2 # left
SENSOR_B_PIN = board.D3 # right

STEPPER_A = (
    digitalio.DigitalInOut(board.D9),
    digitalio.DigitalInOut(board.D10),
    digitalio.DigitalInOut(board.D11),
    digitalio.DigitalInOut(board.D12),
)
STEPPER_B = (
    digitalio.DigitalInOut(board.D5),
    digitalio.DigitalInOut(board.D6),
    digitalio.DigitalInOut(board.D7),
    digitalio.DigitalInOut(board.D8),
)
STYLE = stepper.INTERLEAVE
STEPS_PER_REVOLUTION = 4096
# STYLE = stepper.DOUBLE
# STEPS_PER_REVOLUTION = 2048


# CONSTANTS
# These are used to calculate the actual lengths of arms, excluding the motors.

STEPPER_SHAFT_DIAMETER = 5.0
STEPPER_RING_DIAMETER = 9.0
STEPPER_A_TO_ARM_A_START = 2.5 # distance from the end of the arm to the start of the stepper shaft
STEPPER_A_TO_ARM_A = STEPPER_A_TO_ARM_A_START + STEPPER_SHAFT_DIAMETER / 2
STEPPER_B_TO_ARM_A_END = 1.9
STEPPER_B_TO_ARM_A = STEPPER_B_TO_ARM_A_END + STEPPER_RING_DIAMETER / 2

ARM_A = ARM_A_LENGTH - STEPPER_A_TO_ARM_A - STEPPER_B_TO_ARM_A

PEN_DIAMETER = 11.0
PEN_TO_ARM_B_END = 2.3
PEN_TO_ARM_B = PEN_TO_ARM_B_END + PEN_DIAMETER / 2

ARM_B = ARM_B_LENGTH - STEPPER_A_TO_ARM_A - PEN_TO_ARM_B


# FUNCTIONS

# calculates the inner angle opposite c, given side lengths a, b, and c.
def calc_inner_angle(a, b, c):
    ang = a**2 + b**2 - c**2
    ang = ang / (2 * a * b)
    ang = math.acos(ang)
    return ang


# calc_angles calculates the angle_a and angle_b angles (in radians) given an x and y coordinate.
# [0, 0] is the starting position of the pen.
def xy_to_angles(x, y):
    x = x + (ARM_B - ARM_A) # change the origin to the shaft of stepper a.
    d = math.sqrt(x**2 + y**2)
    inner_1 = calc_inner_angle(ARM_A, d, ARM_B)
    inner_2 = calc_inner_angle(ARM_A, ARM_B, d)

    angle_b = inner_2 - math.pi

    theta = math.atan2(y, x)
    angle_a = theta + inner_1

    return angle_a, angle_b


def angles_to_xy(angle_a, angle_b):
    y = math.sin(angle_a) * ARM_A
    y += math.sin(angle_a + angle_b) * ARM_B
    x = math.cos(angle_a) * ARM_A
    x += math.cos(angle_a + angle_b) * ARM_B
    x -= (ARM_B - ARM_A)
    return x, y


def steps_to_angle(steps):
    return steps * 2 * math.pi / STEPS_PER_REVOLUTION


def angle_to_steps(angle):
    return round(angle * STEPS_PER_REVOLUTION / (2 * math.pi))


def clean_angle(angle):
    return angle % (2 * math.pi)


def steps_between(angle_1, angle_2):
    angle_1 = clean_angle(angle_1)
    angle_2 = clean_angle(angle_2)
    diff = clean_angle(angle_2 - angle_1)
    if diff > math.pi:
        diff = -2 * math.pi + diff
    return angle_to_steps(diff)


# Move both motors at the same time, such that they reach the target position at the same time.
def move_steps(motor_a, motor_b, steps_a, steps_b, forward_a=True, forward_b=True):
    if steps_a > steps_b:
        steps_1, steps_2 = steps_a, steps_b
        motor_1, motor_2 = motor_a, motor_b
        forward_1, forward_2 = forward_a, forward_b
    else:
        steps_1, steps_2 = steps_b, steps_a
        motor_1, motor_2 = motor_b, motor_a
        forward_1, forward_2 = forward_b, forward_a

    steps_2_per_1 = steps_2 / steps_1
    steps_2_cnt = 0

    for i in range(steps_1):
        if forward_1:
            motor_1.onestep(style=STYLE)
        else:
            motor_1.onestep(style=STYLE, direction=stepper.BACKWARD)

        if int(i * steps_2_per_1) != steps_2_cnt:
            steps_2_cnt = int(i * steps_2_per_1)
            if forward_2:
                motor_2.onestep(style=STYLE)
            else:
                motor_2.onestep(style=STYLE, direction=stepper.BACKWARD)
        time.sleep(0.005)


# given the step offsets, move to the x and y position.
# returns the new step offsets.
def small_move_to(steps_a, steps_b, x, y):
    angle_a = steps_to_angle(steps_a)
    angle_b = steps_to_angle(steps_b)
    angle_a1, angle_b1 = xy_to_angles(x, y)
    steps_a1 = steps_between(angle_a, angle_a1)
    steps_b1 = steps_between(angle_b, angle_b1)

    print("STEPS", steps_a1, steps_b1)

    forward_a = steps_a1 > 0
    forward_b = steps_b1 > 0

    move_steps(motor_a, motor_b, abs(steps_a1), abs(steps_b1), forward_a, forward_b)

    steps_a2 = (steps_a + steps_a1) % STEPS_PER_REVOLUTION
    steps_b2 = (steps_b + steps_b1) % STEPS_PER_REVOLUTION

    return steps_a2, steps_b2


# given the step offsets, move to the x and y position, moving delta mm per step.
# returns the new step offsets.
def move_to(steps_a, steps_b, x, y, delta=1):
    cx, cy = angles_to_xy(steps_to_angle(steps_a), steps_to_angle(steps_b))
    print(steps_a, steps_b, steps_to_angle(steps_a), steps_to_angle(steps_b), cx, cy)
    dx = x - cx
    dy = y - cy
    d = math.sqrt(dx**2 + dy**2)
    s = d / delta # number of iterations
    sx = dx / s
    sy = dy / s

    print(cx, cy, x, y, sx, sy)

    for i in range(1, s):
        print(cx + sx * i, cy + sy * i)
        steps_a, steps_b = small_move_to(steps_a, steps_b, cx + sx * i, cy + sy * i)

    return small_move_to(steps_a, steps_b, x, y)


# STEPPERS

for coil in STEPPER_A + STEPPER_B:
    coil.direction = digitalio.Direction.OUTPUT

motor_a = stepper.StepperMotor(STEPPER_A[0], STEPPER_A[2], STEPPER_A[1], STEPPER_A[3], microsteps=None)
motor_b = stepper.StepperMotor(STEPPER_B[0], STEPPER_B[2], STEPPER_B[1], STEPPER_B[3], microsteps=None)


# I2C

# Use the pins to select a single sensor to send messages to.
sensor_a = digitalio.DigitalInOut(SENSOR_A_PIN)
sensor_b = digitalio.DigitalInOut(SENSOR_B_PIN)
sensor_a.switch_to_output(value=False)
sensor_b.switch_to_output(value=False)

i2c = board.I2C()

# PROXIMITY SENSOR

while not i2c.try_lock():
    pass

# Select only sensor b, because we want to change its address to something different from
# sensor a's
sensor_b.value = True

# Change the address for sensor b to 0x2a (default is 0x29). This part was difficult to figure out. See:
# Message will need device address, register address, and new device address: https://github.com/sparkfun/SparkFun_VL53L1X_Arduino_Library/blob/master/src/st_src/vl53l1x_class.cpp#L156=
# Register address is 0x0001: https://github.com/sparkfun/SparkFun_VL53L1X_Arduino_Library/blob/master/src/st_src/vl53l1x_class.h#L59=
# Register address takes two bytes: https://github.com/sparkfun/SparkFun_VL53L1X_Arduino_Library/blob/master/src/st_src/vl53l1x_class.cpp#L1008=
# i2c.writeto: https://docs.circuitpython.org/en/7.2.x/shared-bindings/busio/index.html#busio.I2C.writeto
i2c.writeto(0x29, bytes([0, 0x01, 0x2a]))

# Enable sensor a
sensor_a.value = True

i2c.unlock()

d_sensor_a = adafruit_vl53l1x.VL53L1X(i2c)
d_sensor_a.start_ranging()

d_sensor_b = adafruit_vl53l1x.VL53L1X(i2c, 0x2a)
d_sensor_b.start_ranging()


# MAIN PROGRAM

steps_a = angle_to_steps(math.pi)
steps_b = angle_to_steps(math.pi)
x = 0
y = 0

steps_a, steps_b = small_move_to(steps_a, steps_b, (MIN_X + MAX_X) / 2, (MIN_Y + MAX_Y) / 2)
x, y = angles_to_xy(steps_to_angle(steps_a), steps_to_angle(steps_b))

while True:
    dx = 0
    dy = 0

    if d_sensor_a.distance < RANGE * 2 and d_sensor_a.distance > RANGE:
        dx = DISTANCE
    elif d_sensor_a.distance <= RANGE:
        dx = -DISTANCE
    if d_sensor_b.distance < RANGE * 2 and d_sensor_b.distance > RANGE:
        dy = DISTANCE
    elif d_sensor_b.distance <= RANGE:
        dy = -DISTANCE

    if dx == 0 and dy == 0:
        continue
    try:
        x = max(x + dx, MIN_X)
        x = min(x, MAX_X)
        y = max(y + dy, MIN_Y)
        y = min(y, MAX_Y)
        print(x, y)
        steps_a, steps_b = small_move_to(steps_a, steps_b, x, y)
        x, y = angles_to_xy(steps_to_angle(steps_a), steps_to_angle(steps_b))
    except:
        print("ERROR")
    finally:
        pass

motor_a.release()
motor_b.release()
