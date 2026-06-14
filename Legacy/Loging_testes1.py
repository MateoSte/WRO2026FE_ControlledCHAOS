from pybricks.pupdevices import Motor, UltrasonicSensor
from pybricks.parameters import Port, Direction, Stop
from pybricks.tools import wait, StopWatch
from pybricks.hubs import PrimeHub
from math import pi, tan


# ------------------------------------------------
# -----------------  LOGGER  ---------------------
# ------------------------------------------------

_sw = StopWatch()

def log(event, **kwargs):
    """Ispiši timestampani log redak za ključne odluke."""
    parts = " | ".join("{}={}".format(k, v) for k, v in kwargs.items())
    print("[{:6}ms] {:22} {}".format(_sw.time(), event, parts))


# ------------------------------------------------
# -----------------  BETTER MOTOR  ---------------
# ------------------------------------------------

class BetterMotor(Motor):
    def __init__(self, port, default_speed,
                 positive_direction=Direction.CLOCKWISE,
                 gears=None, reset_angle=True, profile=None):
        self.default_speed = default_speed
        super().__init__(port, positive_direction, gears, reset_angle, profile)

    def run_target(self, speed, angle=None,
                   then=Stop.HOLD, wait=True,
                   duration=None, fallbackWhenNotStalled=None,
                   fallbackWhenStalled=None, diff=5):
        def default_callback():
            pass

        if fallbackWhenNotStalled is None:
            fallbackWhenNotStalled = default_callback
        if fallbackWhenStalled is None:
            fallbackWhenStalled = default_callback

        if angle is None:
            super().run_target(self.default_speed, speed, then, wait=False)
            if duration is None:
                if wait:
                    while not self.done():
                        pass
                return
            sw = StopWatch()
            while sw.time() < duration:
                if self.done():
                    fallbackWhenNotStalled()
                    return
            if abs(speed - self.angle()) > diff:
                fallbackWhenStalled()
            else:
                fallbackWhenNotStalled()
        else:
            super().run_target(speed, angle, then, wait=False)
            if duration is None:
                if wait:
                    while not self.done():
                        pass
                return
            sw = StopWatch()
            while sw.time() < duration:
                if self.done():
                    fallbackWhenNotStalled()
                    return
            if abs(angle - self.angle()) > diff:
                fallbackWhenStalled()
            else:
                fallbackWhenNotStalled()
        self.brake()

    def run_angle(self, speed, angle=None, then=Stop.HOLD, wait=True,
                  duration=None, fallbackWhenNotStalled=None,
                  fallbackWhenStalled=None, diff=5):
        def default_callback():
            pass

        if fallbackWhenNotStalled is None:
            fallbackWhenNotStalled = default_callback
        if fallbackWhenStalled is None:
            fallbackWhenStalled = default_callback

        oldAngle = self.angle()
        if angle is None:
            super().run_angle(self.default_speed, speed, then, wait=False)
            if duration is None:
                if wait:
                    while not self.done():
                        pass
                return
            sw = StopWatch()
            while sw.time() < duration:
                if self.done():
                    fallbackWhenNotStalled()
                    return
            if abs(oldAngle + speed - self.angle()) > diff:
                fallbackWhenStalled()
            else:
                fallbackWhenNotStalled()
        else:
            super().run_angle(speed, angle, then, wait=False)
            if duration is None:
                if wait:
                    while not self.done():
                        pass
                return
            sw = StopWatch()
            while sw.time() < duration:
                if self.done():
                    fallbackWhenNotStalled()
                    return
            if abs(oldAngle + angle - self.angle()) > diff:
                fallbackWhenStalled()
            else:
                fallbackWhenNotStalled()
        self.brake()

    def set_default_speed(self, new_speed):
        self.default_speed = new_speed


# ------------------------------------------------
# -----------------  CAR DRIVE BASE  -------------
# ------------------------------------------------

class CarDriveBase:
    def __init__(self, drive_motor, steer_motor, wheel_diameter,
                 axle_track, default_speed, default_steer_speed):
        self.drive_motor = drive_motor
        self.steer_motor = steer_motor
        self.wheel_diameter = wheel_diameter
        self.axle_track = axle_track
        self.default_speed = default_speed
        self.default_steer_speed = default_steer_speed

    def drive(self, speed=None):
        if speed is None:
            speed = self.default_speed
        self.drive_motor.drive(speed)

    def stop(self):
        self.drive_motor.stop()

    def brake(self):
        self.drive_motor.brake()

    def straight(self, dist, speed=None):
        if speed is None:
            speed = self.default_speed
        o = self.wheel_diameter * pi
        self.steer_motor.run_target(speed, 0)
        self.drive_motor.run_angle(speed, dist / o * 360)
        while not self.drive_motor.done():
            wait(5)

    def turn(self, target_deg, step_deg, speed_steer=None, speed_drive=None):
        if speed_steer is None:
            speed_steer = self.default_steer_speed
        if speed_drive is None:
            speed_drive = self.default_speed
        dist = (target_deg * self.axle_track * pi) / (tan(step_deg) * 180)
        angle = self.steer_motor.angle()
        self.steer_motor.run_target(speed_steer, step_deg)
        self.straight(dist * 1000, speed_drive)
        self.steer_motor.run_target(speed_steer, angle)


# ------------------------------------------------
# -----------------  INITIAL SETTINGS  -----------
# ------------------------------------------------

steer_motor = Motor(Port.D)
drive_motor = Motor(Port.F, Direction.COUNTERCLOCKWISE)

left_sensor  = UltrasonicSensor(Port.E)
right_sensor = UltrasonicSensor(Port.C)
front_sensor = UltrasonicSensor(Port.A)
hub = PrimeHub()

FRONT_CRITICAL = 120
GAIN = 1.1

STATE = "FORWARD"


# ------------------------------------------------
# -----------------  FUNCTIONS  ------------------
# ------------------------------------------------

def center():
    l = left_sensor.distance()
    r = right_sensor.distance()
    error = l - r
    steering = max(-30, min(30, error * GAIN))
    log("CENTER", L=l, R=r, err=error, steer=steering)
    steer_motor.run_target(200, -steering, wait=False)


def choose_escape():
    l = left_sensor.distance()
    r = right_sensor.distance()
    direction = -40 if l > r else 40
    log("CHOOSE_ESCAPE", L=l, R=r, chosen=direction)
    return direction


def recover():
    global STATE
    log("RECOVER_START", steer=steer_motor.angle())

    drive_motor.dc(-40)
    wait(700)

    drive_motor.dc(0)
    steer_motor.run_target(200, 0)
    wait(200)

    drive_motor.dc(-40)
    wait(700)
    drive_motor.dc(0)
    wait(100)

    prepoznajZavoj()

    drive_motor.dc(70)
    wait(900)

    log("RECOVER_END")
    STATE = "FORWARD"


def prepoznajZavoj():
    l = left_sensor.distance()
    r = right_sensor.distance()
    f = front_sensor.distance()
    steer_motor.run_target(200, 0, wait=True)

    if f < FRONT_CRITICAL and l > r:
        steer_angle = -40
    else:
        steer_angle = 40

    log("PREPOZNAJ_ZAVOJ", F=f, L=l, R=r, steer=steer_angle)
    steer_motor.run_target(200, steer_angle)
    wait(200)


def zavoj():
    kut = -1 * steer_motor.angle()
    if front_sensor.distance() >= 1000:
        l = left_sensor.distance()
        r = right_sensor.distance()
        log("ZAVOJ_START", kut=kut, L=l, R=r)
        flag = 0
        while flag == 0:
            if (kut < 1 and kut > -1) or ((l < 400) and (r < 400)):
                flag = 1
            drive_motor.dc(30)
            steer_motor.run_target(200, -kut, wait=False)
            kut = kut * 0.93
            wait(50)
        drive_motor.dc(0)
        wait(100)
        log("ZAVOJ_END", kut_final=kut)


def turn(decision):
    global STATE
    side = -1 if decision == "L" else 1

    log("TURN_START", decision=decision, side=side,
        F=front_sensor.distance(),
        L=left_sensor.distance(),
        R=right_sensor.distance(),
        steer_now=steer_motor.angle())

    drive_motor.dc(0)
    steer_motor.run_target(200, 0, wait=True)
    wait(100)

    angle = float(side)
    drive_motor.dc(30)
    step = 0

    while front_sensor.distance() < 1000:
        steer_motor.run_target(200, angle, wait=False)
        angle = angle * 1.3
        if angle >  30 and side ==  1: angle = 30.0
        if angle < -30 and side == -1: angle = -30.0

        # Log svaki korak unutar petlje skretanja
        log("TURN_LOOP",
            step=step,
            decision=decision,
            angle=round(angle, 1),
            F=front_sensor.distance(),
            L=left_sensor.distance(),
            R=right_sensor.distance(),
            spd=drive_motor.speed(),
            stalled=drive_motor.stalled())
        step += 1
        wait(50)

        if(left_sensor.distance()>=2000 and decision=="L"):
            drive_motor.dc(0)
            steer_motor.run_target(200,30)
            drive_motor.dc(30)
            wait(2000)
            drive_motor.dc(0)
        elif(right_sensor.distance()>=2000 and decision=="R"):
            drive_motor.dc(0)
            steer_motor.run_target(200,-30)
            drive_motor.dc(30)
            wait(2000)
            drive_motor.dc(0)

    log("TURN_EXIT_LOOP",
        decision=decision,
        F=front_sensor.distance(),
        steps=step)

    wait(4000)
    drive_motor.dc(0)
    steer_motor.run_target(200, 0)

    log("TURN_END", decision=decision)
    STATE = "FORWARD"
    return STATE


# ------------------------------------------------
# -----------------  MAIN LOOP  ------------------
# ------------------------------------------------

log("BOOT", battery=hub.battery.voltage())

_prev_state = None

while True:
    front_dist = front_sensor.distance()
    left_dist  = left_sensor.distance()
    right_dist = right_sensor.distance()

    # --- Odluka o stanju ---
    if front_dist < FRONT_CRITICAL:
        STATE = "RECOVER"
    elif left_dist > right_dist and left_dist > 500:
        STATE = "LEFT_TURN"
    elif left_dist < right_dist and right_dist > 500:
        STATE = "RIGHT_TURN"
    else:
        STATE = "FORWARD"

    # Logiraj samo kad se stanje promijeni (ne svaku iteraciju)
    if STATE != _prev_state:
        log("STATE_CHANGE",
            old=_prev_state,
            new=STATE,
            F=front_dist,
            L=left_dist,
            R=right_dist)
        _prev_state = STATE

    # --- State machine ---
    if STATE == "RECOVER":
        recover()
    elif STATE == "LEFT_TURN":
        turn("L")
    elif STATE == "RIGHT_TURN":
        turn("R")
    else:
        center()
        drive_motor.dc(30)

    wait(40)