from pybricks.pupdevices import Motor, UltrasonicSensor, ColorSensor
from pybricks.parameters import Port, Direction, Stop
from pybricks.tools import wait
from pybricks.hubs import PrimeHub
from pybricks.tools import StopWatch, wait
from math import ceil, pi, tan, radians, sin, atan, degrees



#------------------------------------------------
#-----------------INITIAL SETTINGS --------------
#------------------------------------------------

# MOTORS
steer = Motor(Port.D)
drive = Motor(Port.F, Direction.COUNTERCLOCKWISE)

# SENSORS
left_sensor = UltrasonicSensor(Port.E)
right_sensor = UltrasonicSensor(Port.C)
color_sensor = ColorSensor(Port.B)
front_sensor = UltrasonicSensor(Port.A)
hub = PrimeHub()

# SETTINGS
FRONT_CRITICAL = 200   # glavni prag za hitni recovery
WALL_DETECTED = 600
GAIN = 1.05

STATE = "FORWARD"


class CarDriveBase:
    def __init__(self, drive_motor, steer_motor, wheel_diameter,
                 axle_track, default_speed, default_steer_speed, hub):
        self.drive_motor = drive_motor  # type: Motor
        self.steer_motor = steer_motor  # type: Motor
        self.wheel_diameter = wheel_diameter  # type: float
        self.axle_track = axle_track  # type: float
        self.default_speed = default_speed  # type: int
        self.default_steer_speed = default_steer_speed  # type: int
        self.hub = hub  # type: PrimeHub
        self.gyro = False
        self.running = False

    def use_gyro(self, use):
        self.hub.imu.reset_heading(0)
        self.gyro = use
        
    def reset_gyro(self):
        if self.gyro:
            self.hub.imu.reset_heading(0)
        
    def correct(self, angle=0, step=30, pr=False):
        if self.gyro:
            #if pr:
                #print(max(-abs(step), min(abs(step), 15 * ceil((angle - self.hub.imu.heading()) / 15))))
            self.steer_motor.run_target(self.default_steer_speed, max(-abs(step), min(abs(step), 15 * ceil((angle - self.hub.imu.heading()) / 15))), wait=False)

    def drive(self, speed=None):
        if speed is None:
            speed = self.default_speed
        self.drive_motor.run(speed)
        self.running = True

    def stop(self):
        self.drive_motor.stop()
        self.running = False

    def brake(self):
        self.drive_motor.brake()
        self.running = False

    def _straight(self, dist, turn_rate=0, speed=None, _gyro=True):
        self.running = True
        if speed is None:
            speed = self.default_speed
        o = self.wheel_diameter * pi
        degrees = dist / o * 360
        speed = abs(speed) * (1 if degrees > 0 else -1)
        start = self.drive_motor.angle()
        self.drive_motor.run(speed)
        while abs(self.drive_motor.angle() - start) < abs(degrees):
            if self.gyro and _gyro:
                self.correct(turn_rate)
            wait(5)
        self.drive_motor.brake()
        self.running = False

    def straight(self, dist, turn_rate=0, speed=None):
        self.steer_motor.run_target(self.default_steer_speed, 0)
        self._straight(dist, turn_rate, speed)

    def turn(self, target_deg, step_deg, tolerance=1.5, speed_steer=None, speed_drive=None):
        if target_deg == 0:
            return
        if step_deg == 0:
            self.steer_motor.run_target(self.default_steer_speed, 0)
            return
        ta = target_deg / abs(target_deg)
        sa = step_deg / abs(step_deg)
        if speed_steer is None:
            speed_steer = self.default_steer_speed
        if speed_drive is None:
            speed_drive = self.default_speed

        axle_track_mm = self.axle_track * 10  # cm to mm
        dist = (ta * (abs(target_deg) + abs(step_deg)) * axle_track_mm * pi) / (tan(radians(abs(step_deg))) * 180)
        #print(dist, target_deg, step_deg, self.steer_motor.angle())

        self.steer_motor.run_target(speed_steer, step_deg)
        self._straight(dist, speed_drive, _gyro=False)
        if self.gyro:
            started = False
            #print(target_deg, tolerance)
            i = 0
            while abs(self.hub.imu.heading()) - abs(target_deg) > tolerance:
                #if i == 0:
                    #print(self.hub.imu.heading())
                if not started:
                    self.drive(ta * speed_drive)
                started = True
                self.correct(sa * target_deg, step_deg, pr=i == 0)
                wait(5)
                i += 1
                i %= 50
            # wait(50)
            self.brake()
            #print("-------------------------------------------")
            #print(self.hub.imu.heading())
            self.hub.imu.reset_heading(self.hub.imu.heading() - sa * target_deg)
            #print(self.hub.imu.heading())
            #print("-------------------------------------------")



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

def pratiZid(strana):
    print("PRATIM ZID: ", strana)
    l = left_sensor.distance()
    r = right_sensor.distance()

    distWall=400
    if strana=='L':
        error = l - 400
    else:
        error= 400 - r
    steering = error * GAIN

    # limit steering
    if steering > 30:
        steering = 30
    elif steering < -30:
        steering = -30

    # invert ako ti hardware traži suprotno
    steer_motor.run_target(200, -steering, wait=False)





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

def zavoj2(strana):
    print("ZAVOJ2")
    if strana == "R":
        #skrece lijevo
        kut = -50
    else:
        kut = 50
    if front_sensor.distance()<=WALL_DETECTED:
        l = left_sensor.distance()
        r = right_sensor.distance()
        while abs(kut)>2:
            drive_motor.dc(50)
            steer_motor.run_target(200, kut, wait=False)
            kut=kut*0.93
            print(kut)
            wait(120)
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
print("Hub battery: ", hub.battery.voltage())
wait(1000)


car = CarDriveBase(drive, steer, 62.4, 13, 450, 300, hub)
# car.turn(90, 30)
# car.straight(300, use_gyro=True)
steer.run_target(300, 0)
steer.reset_angle(0)
car.use_gyro(True)

l0 = 0
r0 = 0
f0 = 0

skretanje = "NEUTRAL"
skretanje_pomak = 400
korekcija_staze = 300



def pocetak():
    #boja = Color.WHITE
    alpha = 90
    global skretanje 
    l0 = left_sensor.distance()
    r0 = right_sensor.distance()
    f0 = front_sensor.distance()
    print(f"R: {r0} L: {l0} F: {f0}")
    car.drive()
    while(front_sensor.distance()> 800):
        car.correct()
    #     boja = color_sensor.color()
    #     if(skretanje == "NEUTRAL" and boja != Color.WHITE):
    #         hub.speaker.beep()
    #         wait(200)
    #         hub.speaker.beep()
    #         wait(200)
    #         if(boja == Color.BLUE):
    #             skretanje = "LIJEVO"
    #         else:
    #             skretanje = "DESNO"
    # print("skretanje:",skretanje)
    if(skretanje == "NEUTRAL"): skretanje = "DESNO"
    if(skretanje == "DESNO"): alpha = 30
    else: alpha = -30
    for i in range(3):
        car.turn(90, alpha)
        car.straight(3000-l0-korekcija_staze-skretanje_pomak)
    car.turn(90, alpha)
    while(front_sensor.distance() > f0):
        car.straight(10)
        wait(5)


pocetak()


# for i in range(4):
#     car.turn(90, 30)
#     hub.speaker.beep()
#     car.straight(1000)
#     hub.speaker.beep()