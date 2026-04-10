from machine import UART
from time import ticks_ms, ticks_diff, sleep_ms
from math import cos, sin, radians, pi, atan2
from board import motor, servo, rgbled_board
import bluepad32
import switch

uart = UART(2, baudrate=115200, tx=13, rx=26)
rx_buf = bytearray(8)

STATE_WAIT_HEADER = 0
STATE_READ_DATA = 1

parser_state = STATE_WAIT_HEADER
rx_index = 0
current_yaw = 0.0
yaw_offset = 0.0

def get_imu():
    global current_yaw, parser_state, rx_index
    
    while uart.any():
        b = uart.read(1)
        if not b: continue
        b = b[0]
        
        if parser_state == STATE_WAIT_HEADER:
            if b != 0xAA: continue
            rx_buf[0] = b
            rx_index = 1
            parser_state = STATE_READ_DATA
                
        elif parser_state == STATE_READ_DATA:
            rx_buf[rx_index] = b
            rx_index += 1
            
            if rx_index != 8: continue
            parser_state = STATE_WAIT_HEADER
            if rx_buf[7] != 0x55: continue

            yaw = (rx_buf[1] << 8) | rx_buf[2]
            if yaw >= 0x8000: yaw -= 0x10000
            
            current_yaw = (yaw / 100.0) - yaw_offset
            return True
    return False

def init_imu():
    global yaw_offset, current_yaw
    while uart.any(): uart.read()

    start = ticks_ms()
    samples = 0
    total = 0.0
    indx = 0

    rgbled_board.set_brightness(10)
    while ticks_diff(ticks_ms(), start) < 1000:
        if samples % 20 == 0:
            rgbled_board.set_color(indx, "#67c0ff")
            rgbled_board.show()
            indx += 1

        if get_imu():
            total += current_yaw
            samples += 1
        sleep_ms(5)

    rgbled_board.clear()
    if samples > 0:
        yaw_offset = total / samples
        current_yaw = 0.0
        rgbled_board.set_color(1, "#6aff67")
    else: rgbled_board.set_color(1, "#ff6767")
    rgbled_board.show()


def clamp(val, min_val=-1.0, max_val=1.0):
    return max(min(val, max_val), min_val)

def wrap_rads(angle):
    while angle > pi: angle -= 2 * pi
    while angle < -pi: angle += 2 * pi
    return angle

class PIDController:

    def __init__(self, kp, ki, kd, output_limit=1.0, integral_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = ticks_ms()

    def calculate(self, setpoint, measurement):
        now = ticks_ms()
        dt = ticks_diff(now, self.prev_time) / 1000.0
        self.prev_time = now

        error = wrap_rads(setpoint - measurement)

        self.integral += error * dt
        self.integral = clamp(self.integral, -self.integral_limit, self.integral_limit)

        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return clamp(output, -self.output_limit, self.output_limit)


controller = PIDController(kp=2, ki=0.02, kd=0.1)
setpoint = 0.0

def speed_control(value):
    abs_val = abs(value)
    if bluepad32.r2(): return value * 0.15
    elif abs_val < 0.1: return 0
    elif abs_val <= 0.3: return value * 0.35
    else: return value

def atan2_heading(rx, ry, yaw):
    global setpoint
    if (rx * rx + ry * ry > 0.5625): setpoint = atan2(rx, ry)
    r = controller.calculate(setpoint, yaw)
    if bluepad32.r2(): r = clamp(r, -0.2, 0.2)
    return r

last_rx = 0
def linear_heading(rx, yaw):
    global last_rx, setpoint
    if bluepad32.r2(): rx = clamp(rx, -0.2, 0.2)
    else: rx = clamp(rx, -0.5, 0.5)
    r = controller.calculate(setpoint, yaw)
    if abs(wrap_rads(setpoint - yaw)) < 0.01: r = 0
    if abs(rx) > 0.1 or ticks_diff(ticks_ms(), last_rx) < 450:
        r = rx
        setpoint = yaw
    if abs(rx) > 0.1: last_rx = ticks_ms()
    return r

def dpad_control(yaw):
    global setpoint
    x = 0; y = 0
    if bluepad32.up(): y += 1
    if bluepad32.down(): y -= 1
    if bluepad32.right(): x -= 1
    if bluepad32.left(): x += 1
    if x != 0 or y != 0:
        setpoint = atan2(x, y)
        x = clamp(speed_control(sin(setpoint)) * 1.5, -1, 1)
        y = speed_control(cos(setpoint))
    return controller.calculate(setpoint, yaw), x, y

heading_toggle = False

def movement():
    global setpoint

    rawx = -bluepad32.axisX() / 512
    rawy = -bluepad32.axisY() / 512
    lx = clamp(speed_control(rawx) * 1.5, -1, 1)
    ly = speed_control(rawy)
    rx = -bluepad32.axisRX() / 512.0
    ry = -bluepad32.axisRY() / 512.0
    yaw = radians(current_yaw)

    if bluepad32.up() or bluepad32.down() or bluepad32.right() or bluepad32.left(): r, lx, ly = dpad_control(yaw)
    elif heading_toggle: r = linear_heading(rx, yaw)
    else:
        if bluepad32.l2(): r = atan2_heading(rawx, rawy, yaw)
        else: r = atan2_heading(rx, ry, yaw)
    if abs(lx) < 0.1 and abs(ly) < 0.1 and abs(r) < 0.01: r = 0

    # movement
    x = (cos(yaw) * lx) + (sin(yaw) * -ly)
    y = (sin(yaw) * lx) - (cos(yaw) * -ly)

    d = max(abs(x) + abs(y) + abs(r), 1)
    fl = (((y - x - r) / d))
    fr = (((y + x + r) / d))
    rl = (((y - x + r) / d))
    rr = (((y + x - r) / d))

    scale = max(abs(fl), abs(fr), abs(rl), abs(rr), 1)

    fl = int((fl / scale) * 100)
    fr = int((fr / scale) * 100)
    rl = int((rl / scale) * 100)
    rr = int((rr / scale) * 100)

    motor.wheel(fl, rr, fr, rl)

button_states = {}
def pressed_once(name, current):
    global button_states
    
    previous_state = button_states.get(name, False)
    clicked = current and not previous_state
    button_states[name] = current
    
    return clicked

LIFT_UP = 35 # TO BE SYNCED
LIFT_STEAL = 45
LIFT_DOWN = 90
LG_OPEN = 0
LG_CLOSE = 60
RG_OPEN = 60
RG_CLOSE = 0

lift_up = True
lg_open = True
rg_open = True
steal_state = True
macro_active = 0

def arm():
    global lift_up, lg_open, rg_open, macro_active, steal_state

    if pressed_once("cross", bluepad32.cross()):
        macro_active = 0
        if lift_up: servo.angle(servo.SV2, LIFT_DOWN)
        else: servo.angle(servo.SV2, LIFT_UP)
        lift_up = not lift_up
    
    if pressed_once("l1", bluepad32.l1()):
        if lg_open: servo.angle(servo.SV3, LG_CLOSE)
        else: servo.angle(servo.SV3, LG_OPEN)
        lg_open = not lg_open

    if pressed_once("r1", bluepad32.r1()):
        if rg_open: servo.angle(servo.SV4, RG_CLOSE)
        else: servo.angle(servo.SV4, RG_OPEN)
        rg_open = not rg_open

    if pressed_once("triangle", bluepad32.triangle()) and macro_active == 0:
        if steal_state:
            servo.angle(servo.SV2, LIFT_STEAL)
            servo.angle(servo.SV3, LG_OPEN)
            servo.angle(servo.SV4, RG_OPEN)
            lg_open, rg_open = True, True
        else:
            servo.angle(servo.SV3, LG_CLOSE)
            servo.angle(servo.SV4, RG_CLOSE)
            lg_open, rg_open = False, False
            macro_active = ticks_ms()
        steal_state = not steal_state

    if pressed_once("circle", bluepad32.circle()) and macro_active == 0:
        servo.angle(servo.SV3, LG_CLOSE)
        servo.angle(servo.SV4, RG_CLOSE)
        lg_open, rg_open = False, False
        macro_active = ticks_ms()

    if macro_active != 0 and ticks_diff(ticks_ms(), macro_active) > 150:
        servo.angle(servo.SV2, LIFT_UP)
        lift_up, steal_state = True, True
        macro_active = 0

def yaw_control():
    global setpoint, yaw_offset
    if pressed_once("square", bluepad32.square()):
        yaw_offset += current_yaw
        setpoint = 0

gamepad_state = True
heading_state = True
team_switch = 0
started = False

def configuration():
    global gamepad_state, heading_toggle, heading_state, team_switch, started

    if pressed_once("circle", bluepad32.circle()):
        if team_switch == 0 or team_switch == 2:
            rgbled_board.fill("#67cfff")
            team_switch = 1
        else:
            rgbled_board.fill("#ff6767")
            team_switch = 2
        rgbled_board.show()

    if team_switch != 0:
        if pressed_once("square", bluepad32.square()):
            servo.angle(servo.SV2, LIFT_UP)
            started = True
        return

    if pressed_once("triangle", bluepad32.triangle()):
        heading_toggle = not heading_toggle

    connection = bluepad32.is_connected()
    if connection != gamepad_state:
        gamepad_state = connection
        rgbled_board.set_color(2, "#6aff67" if gamepad_state else "#ff6767")
        rgbled_board.show()

    if heading_toggle != heading_state:
        heading_state = heading_toggle
        rgbled_board.set_color(3, "#faff67" if heading_state else "#67cfff")
        rgbled_board.show()

def main():
    init_imu()
    while not started: configuration()
    while 1:
        get_imu()
        yaw_control()
        movement()
        arm()

# -------------------- Start --------------------

print("initializing")

servo.angle(servo.SV2, 0)
servo.angle(servo.SV3, LG_CLOSE)
servo.angle(servo.SV4, RG_CLOSE)

main()
