from pybricks.pupdevices import Motor, UltrasonicSensor
from pybricks.parameters import Port, Direction, Stop
from pybricks.tools import wait
from pybricks.hubs import PrimeHub
from pybricks.tools import StopWatch, wait
from math import pi, tan


def warn(message):
    """Print a warning message"""
    print("Warning:", message)


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


class CarDriveBase:
    def __init__(self, drive_motor, steer_motor, wheel_diameter,
                 axle_track, default_speed, default_steer_speed):
        # type: (Motor, Motor, float, float, int, int) -> None
        self.drive_motor = drive_motor  # type: Motor
        self.steer_motor = steer_motor  # type: Motor
        self.wheel_diameter = wheel_diameter  # type: float
        self.axle_track = axle_track  # type: float
        self.default_speed = default_speed  # type: int
        self.default_steer_speed = default_steer_speed  # type: int

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
        print("gotovo")            

    def turn(self, target_deg, step_deg, speed_steer=None, speed_drive=None):
        if speed_steer is None:
            speed_steer = self.default_steer_speed
        if speed_drive is None:
            speed_drive = self.default_speed
        dist = (target_deg * self.axle_track * pi) / (tan(step_deg) * 180)
        angle = self.steer_motor.angle()
        self.steer_motor.run_target(speed_steer, step_deg)
        self.straight(dist * 1000, speed_drive)
        print(dist)
        self.steer_motor.run_target(speed_steer, angle)

#------------------------------------------------
#-----------------INITIAL SETTINGS --------------
#------------------------------------------------

# MOTORS
steer_motor = Motor(Port.D)
drive_motor = Motor(Port.F, Direction.COUNTERCLOCKWISE)

# SENSORS
left_sensor = UltrasonicSensor(Port.E)
right_sensor = UltrasonicSensor(Port.C)
front_sensor = UltrasonicSensor(Port.A)
hub = PrimeHub()

# SETTINGS
FRONT_CRITICAL = 120   # glavni prag za hitni recovery
GAIN = 1.1

STATE = "FORWARD"

#------------------------------------------------
#----------------- FUNCTIONS ---------------------
#------------------------------------------------

def center():
    print("CENTER")
    l = left_sensor.distance()
    r = right_sensor.distance()

    error = l - r
    steering = error * GAIN

    # limit steering
    if steering > 30:
        steering = 30
    elif steering < -30:
        steering = -30

    # invert ako ti hardware traži suprotno
    steer_motor.run_target(200, -steering, wait=False)


def choose_escape():
    print("ESCAPE")
    l = left_sensor.distance()
    r = right_sensor.distance()

    if l > r:
        return -40   # lijevo
    else:
        return 40    # desno


def recover():
    global STATE
    print("RECOVERY")
    drive_motor.dc(-40)
    wait(700)
    # STOP
    drive_motor.dc(0)
    steer_motor.run_target(200, 0)
    wait(200)

   # decide direction
    #turn = choose_escape()
    #wait(300)

    # BACK UP
    drive_motor.dc(-40)
    wait(700)
    drive_motor.dc(0)
    wait(100)
    prepoznajZavoj()

    # # decide direction
    # turn = choose_escape()
    # steer_motor.run_target(200, turn)
    # wait(300)

    # exit from wall zone
    drive_motor.dc(70)
    wait(900)

    STATE = "FORWARD"

def prepoznajZavoj():
    l = left_sensor.distance()
    r = right_sensor.distance()
    f = front_sensor.distance()
    steer_motor.run_target(200, 0, wait=True)
    if((f < FRONT_CRITICAL) and (l>r)):
        steer_motor.run_target(200, -40)
        wait(200)
    else:
        steer_motor.run_target(200, 40)
        wait(200)
    
def zavoj():
    print("ZAVOJ")
    kut=-1*steer_motor.angle()
    if front_sensor.distance()>=1000:
        l = left_sensor.distance()
        r = right_sensor.distance()
        flag = 0
        while flag==0:
            if((kut<1 and kut>-1 ) or ((l < 400) and (r < 400))):
                flag = 1
            drive_motor.dc(30)
            steer_motor.run_target(200, -kut, wait=False)
            kut=kut*0.93
            wait(50)
            print(flag, kut, l, r)
        drive_motor.dc(0)
        wait(100)
        

def turn(decision):
    global STATE
    print("TURN")
    side=1
    if decision=="L":
        side=-1
    else: side=1
    drive_motor.dc(0)
    steer_motor.run_target(200, 0, wait=True)
    wait(100)
    angle=side
    drive_motor.dc(30)
    while front_sensor.distance()<1000:
        steer_motor.run_target(200, angle, wait=False)
        angle=angle*1.3
        if(angle > 30 and side==1): angle = 30
        elif(angle < -30 and side==-1): angle = -30
        print(f"decision: {decision}")
        print("TURN:",left_sensor.distance(),right_sensor.distance(),
        front_sensor.distance(), angle)
        wait(50)
        print(drive_motor.speed(), drive_motor.stalled())
    print("UUUUUUU")
    wait(5000)
    zavoj()
    STATE="FORWARD"
    return STATE


#------------------------------------------------
#----------------- MAIN LOOP ---------------------
#------------------------------------------------
print(hub.battery.voltage())

while True:

    front_dist = front_sensor.distance()
    left_dist = left_sensor.distance()
    right_dist = right_sensor.distance()

    print("F:", front_dist, "L:", left_dist, "R:", right_dist, "STATE:", STATE)

    # ---------------- EMERGENCY ----------------
    if front_dist < FRONT_CRITICAL:
        STATE = "RECOVER"
        print("---------------RECOVERING -----------------------")
    elif left_dist>right_dist and left_dist>500:
        STATE="LEFT_TURN"
        print("---------------TURNING LEFT -----------------------")

    elif left_dist<right_dist and right_dist>500:
        STATE="RIGHT_TURN"
        print("---------------TURNING RIGHT -----------------------")
    else: STATE="FORWARD"

    # ---------------- STATE MACHINE ------------
    if STATE == "RECOVER":
        recover()
    elif STATE=="LEFT_TURN":
        turn("L")
    elif STATE=="RIGHT_TURN":
        turn("R")
    else:
        # normal driving
        center()
        drive_motor.dc(30)

    wait(40)

