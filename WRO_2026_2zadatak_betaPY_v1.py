from pybricks.pupdevices import Motor, UltrasonicSensor, ColorSensor
from pybricks.parameters import Port, Direction, Stop, Color
from pybricks.tools import wait
from pybricks.hubs import PrimeHub
from pybricks.tools import StopWatch, wait
from math import ceil, pi, tan, radians, sin, atan, degrees
from pybricks.tools import AppData
from ustruct import unpack


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
        self.hub.imu.reset_heading(0)

    def use_gyro(self, use):
        self.gyro = use
        
    def reset_gyro(self):
        if self.gyro:
            self.hub.imu.reset_heading(0)
        
    def correct(self, angle=0, step=30, pr=False):
        if self.gyro:
            # if pr:
            # print(max(-abs(step), min(abs(step), 15 * ceil((angle - self.hub.imu.heading()) / 15))))
            self.steer_motor.run_target(self.default_steer_speed, max(-abs(step), min(abs(step), 15 * ceil((angle - self.hub.imu.heading()) / 15))), wait=False)
            if pr:
                print("-c-", front_sensor.distance())

    def distance(self):
        return self.drive_motor.angle() * self.wheel_diameter * pi / 360

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

    def turn(self, target_deg, step_deg, side=1, tolerance=1.5, speed_steer=None, speed_drive=None):
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
        dist = (ta * (abs(target_deg)) * axle_track_mm * pi) / (tan(radians(abs(step_deg))) * 180)
        # print(dist, target_deg, step_deg, self.steer_motor.angle())

        self.steer_motor.run_target(speed_steer, side * step_deg)
        self._straight(dist, speed_drive, _gyro=False)
        if self.gyro:
            started = False
            # print(target_deg, tolerance)
            i = 0
            while abs(abs(self.hub.imu.heading()) - abs(target_deg)) > tolerance:
                # if i == 0:
                #     print(self.hub.imu.heading())
                if not started:
                    self.drive(ta * speed_drive)
                started = True
                self.correct(sa * target_deg, step_deg, pr=i == 0)
                wait(5)
                i += 1
                i %= 50
            # wait(50)
            self.brake()
            # print("-------------------------------------------")
            # print(self.hub.imu.heading())
            self.hub.imu.reset_heading(self.hub.imu.heading() - sa * target_deg)
            # print(self.hub.imu.heading())
            # print("-------------------------------------------")
    
    def turn_radius(self, target_deg, radius, side=1, tolerance=1.5, speed_steer=None, speed_drive=None):
        axle_track_mm = self.axle_track * 10  # cm to mm

        step_deg = degrees(2 * atan(axle_track_mm / (2 * radius)))
        

        # Preserve the sign of target_deg
        if target_deg < 0:
            step_deg = -step_deg
        print("step_deg",step_deg)
        self.turn(target_deg, step_deg, side, tolerance, speed_steer, speed_drive)



hub = PrimeHub()

#init('A')
#add_command('blocks', 2,0)  #pocinjne komunikaciju sa LMS-ESP32




drive = Motor(Port.F, Direction.CLOCKWISE)
steer = Motor(Port.D)

car = CarDriveBase(drive, steer, 62.4, 13, 750, 300, hub)

left_sensor = UltrasonicSensor(Port.E)
right_sensor = UltrasonicSensor(Port.C)
color_sensor = ColorSensor(Port.B)
front_sensor = UltrasonicSensor(Port.C)

def gumb():
    pressed = []
    while not any(pressed):
        pressed = hub.buttons.pressed()
        wait(10)

wall = 1100
turn_r = 330

# st = 0  # 1
# st = 233  # 2
st = 500  # 3

# car.turn(90, 30)
# car.straight(300, use_gyro=True)


NARANCASTA = Color(h=25, s=100, v=100)
PLAVA = Color(h=228, s=100, v=100)
BIJELA = Color(h=60, s=0, v=100)
color_sensor.detectable_colors([NARANCASTA, PLAVA, BIJELA])


IMENA_BOJA = {
    NARANCASTA: "NARANCASTA",
    PLAVA:      "PLAVA",
    BIJELA:      "BIJELA"
}

strana = "LEFT"

app = AppData([(0, 4)])

RED_HUE   = bytes([Color.RED.h // 2 if Color.RED.h > 0 else 1])
GREEN_HUE = bytes([Color.GREEN.h // 2])

red_data   = (0, 0, 0, 0) # x, y, width, height
green_data = (0, 0, 0, 0) # x, y, width, height

def Ocitaj_boje():
    global red_data
    global green_data
    red_data   = (0, 0, 0, 0) # x, y, width, height
    green_data = (0, 0, 0, 0) # x, y, width, height
    # --- Read red ---
    app.configure(0, 0, RED_HUE)
    wait(500)  # give the phone time to switch and send a fresh frame
    red_data = unpack("BBBB", app.get_bytes(0))
    xr,yr,wr,hr = red_data
    if(wr<10 and hr<10):
        red_data = (0,0,0,0)

    # --- Read green ---
    app.configure(0, 0, GREEN_HUE)
    wait(500)
    green_data = unpack("BBBB", app.get_bytes(0))
    xg,yg,wg,hg = green_data
    if(wg<7 and hg<7):
        green_data = (0,0,0,0)

    print("Red:",   red_data)
    print("Green:", green_data)

def obilaziCrvena():
    global red_data
    xr,yr,wr,hr = red_data
    car.turn(45, 20)
    car.drive()
    while(xr > 10):
        wait(10)
    car.brake()
    car.turn(40,20,-1)



obilaziCrvena()

def pocetak():
    global strana
    steer.run_target(300, 0)
    steer.reset_angle(0)
    car.use_gyro(True)
    flag = 1
    car.drive()
    while flag == 1:
        car.correct()
        bojaRaw = color_sensor.color()
        print("bojaRAW:", bojaRaw)
        boja = IMENA_BOJA.get(bojaRaw, str(bojaRaw))
        print("boja:", boja)
        if(boja == "NARANCASTA" or boja == "PLAVA"):
            # hub.speaker.beep()
            car.brake()
            wait(500)
            flag = 0
            if(boja == "NARANCASTA"):
                strana = "LEFT"
            elif(boja == "PLAVA"):
                strana = "RIGHT"
        wait(10)
    print(strana)
    
        

def okrenutLijevo():
    car.drive()
    while front_sensor.distance() > wall:
        car.correct()
        wait(5)
    car.brake()
    udalj = []
    for i in range(15):
        udalj.append(front_sensor.distance())
        wait(20)
    for i in range(5):
        udalj.remove(max(udalj))
    for i in range(5):
        udalj.remove(min(udalj))
    dist = 0
    for i in udalj:
        dist += i
    dist = dist/5
    car.straight(-(wall - st - dist))
    dist = car.distance()
    car.turn_radius(90, turn_r)

    for i in range(11):
        check_target=0
        car.drive()
        boja = "BIJELA"
        while (boja != "NARANCASTA"):
            car.correct()
            bojaRaw = color_sensor.color()
            boja = IMENA_BOJA.get(bojaRaw, str(bojaRaw))
            wait(10)
        car.brake()

        steer.run_target(300, 0)
        wait(10)

        hub.speaker.beep()
        print("---", front_sensor.distance(), i, hub.imu.heading())
        # gumb()
        car.straight(-(wall - st - front_sensor.distance()))
        car.turn_radius(90, turn_r)


    car.drive()
    while front_sensor.distance() > wall + dist:
        car.correct()
        wait(5)
    car.brake()



def okrenutDesno():
    car.drive()
    while front_sensor.distance() > wall:
        car.correct()
        wait(5)
    car.brake()
    car.straight(-(wall - st - front_sensor.distance()))
    dist = car.distance()
    car.turn_radius(90, -turn_r-170)

    for i in range(11):
        check_target=0
        car.drive()
        boja = "BIJELA"
        while (boja != "PLAVA"):
            car.correct()
            bojaRaw = color_sensor.color()
            boja = IMENA_BOJA.get(bojaRaw, str(bojaRaw))
            wait(10)
        car.brake()
        car.drive()
        while front_sensor.distance() > wall:
            car.correct()
            wait(5)
        car.brake()

        steer.run_target(300, 0)
        wait(10)

        hub.speaker.beep()
        print("---", front_sensor.distance(), i, hub.imu.heading())
        # gumb()
        steer.run_target(300, 0)
        wait(10)
        #car.use_gyro(False)
        car.straight(-(wall - st - front_sensor.distance()))
        #car.use_gyro(True)
        car.turn_radius(90 - i / 10, -turn_r-170)


    car.straight(200)


def zaobilazak_lijevo(broj):
    if(broj > 50):
        car.turn(90, 30, -1)
        car.turn(180, 30)
    else:
        car.turn(90, 20, -1)
        car.turn(180, 20)
    car.straight(200)
    #car.straight(50)
    car.turn(10, 30, -1)

def zaobilazak_desno(broj):
    if(broj > 50):
        car.turn(60, 25) 
        car.straight(100)
        car.turn(180, 30, -1)
    else:
        car.turn(60, 15)
        car.straight(100)
        car.turn(180, 20, -1)
    #car.straight(50)
    car.turn(10, 30)




#detekcija()

# try:
#     pocetak()
#     if(strana == "LEFT"):
#         okrenutLijevo()
#     elif(strana == "RIGHT"):
#         okrenutDesno()
# finally:
#     print("battery: ",hub.battery.voltage())



# while True:
#     boja = color_sensor.color()
#     print(IMENA_BOJA.get(boja, str(boja)))

# for i in range(4):
#     car.turn(90, 30)
#     hub.speaker.beep()
#     car.straight(1000)
#     hub.speaker.beep()

