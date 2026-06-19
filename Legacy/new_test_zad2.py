from pybricks.pupdevices import Motor, UltrasonicSensor, ColorSensor
from pybricks.parameters import Port, Direction, Stop, Color, Axis
from pybricks.tools import wait
from pybricks.hubs import PrimeHub
from pybricks.tools import StopWatch, wait
from math import ceil, pi, tan, radians, sin, atan, degrees, floor, cos, log
from ustruct import unpack
from pybricks.tools import AppData



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
        
    def correct(self, angle=0, step=30, pr=False): #step = 30
        #if self.gyro:
            # if pr:
            # print(max(-abs(step), min(abs(step), 15 * ceil((angle - self.hub.imu.heading()) / 15))))
         #   self.steer_motor.run_target(self.default_steer_speed, max(-abs(step), min(abs(step), 15 * ceil((angle - self.hub.imu.heading()) / 15))), wait=False)
            #if pr:
                # print("-c-", front_sensor.distance())
                # print(angle, self.hub.imu.heading())

        gyro = self.hub.imu.heading()
        error = angle - gyro
        if error<0:
            ceil_value = floor(error / 15)
        else:
            ceil_value = ceil(error / 15)
        correction = 15 * ceil_value
        limited = max(-abs(step), min(abs(step), correction))

        if self.gyro:
            self.steer_motor.run_target(self.default_steer_speed, limited, wait=False)
        
        print(
            f"Angle={angle:>4} | ",
            f"Gyro={gyro:>4} | ",
            f"Error={error:>4} | ",
            f"Ceil={ceil_value:>3} | ",
            f"Correction={correction:>4} | ",
            f"Limited={limited:>4}"
        )


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
        # self.drive_motor.run(speed)
        # while abs(self.drive_motor.angle() - start) < abs(degrees):
        #     if self.gyro and _gyro:
        #         self.correct(turn_rate)
        #     wait(5)
        # self.drive_motor.brake()
        self.drive_motor.run_angle(speed, degrees)
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
        dist = (ta * (abs(target_deg)) * axle_track_mm * pi) / (tan(radians(abs(step_deg))) * 180)
        # print(dist, target_deg, step_deg, self.steer_motor.angle())

        self.steer_motor.run_target(speed_steer, step_deg)
        o = self.wheel_diameter * pi
        deg = dist / o * 360
        speed = abs(speed_drive) * (1 if deg > 0 else -1)
        start = self.drive_motor.angle()
        self.drive_motor.run(speed)
        while abs(self.drive_motor.angle() - start) < abs(deg) and abs(abs(self.hub.imu.heading()) - abs(target_deg)) > 3 * tolerance:
            wait(5)
        self.drive_motor.brake()
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
                i %= 1
            # wait(50)
            self.brake()
            # print("-------------------------------------------")
            # print(self.hub.imu.heading())
            self.hub.imu.reset_heading(self.hub.imu.heading() - sa * target_deg)
            # print(self.hub.imu.heading())
            # print("-------------------------------------------")
    
    def turn_radius(self, target_deg, radius, tolerance=1.5, speed_steer=None, speed_drive=None):
        axle_track_mm = self.axle_track * 10  # cm to mm

        step_deg = degrees(2 * atan(axle_track_mm / (2 * radius)))
        

        # Preserve the sign of target_deg
        if target_deg < 0:
            step_deg = -step_deg
        print("step_deg",step_deg)
        self.turn(target_deg, step_deg, tolerance, speed_steer, speed_drive)
        print("turn radius stao")
        wait(100)
    
    def bypass_obstacle(self, lateral_clearance, sensor_distance, direction=1,
                     safety_margin=50, speed_steer=None, speed_drive=None):
        """
        lateral_clearance: potreban bocni odmak u mm (konstanta tvoje staze/prepreke)
        sensor_distance:   front_sensor.distance() u trenutku detekcije, mm
        direction:         +1 ili -1, ovisno na koju stranu zaobilazis (npr. RED vs GREEN)
        safety_margin:     rezerva do same prepreke, mm
        """
        forward_budget = sensor_distance - safety_margin
        if forward_budget <= 0:
            raise ValueError("Nedovoljno prostora - treba se prije odvesti unatrag i preracunati.")

        theta = 2 * atan(lateral_clearance / forward_budget)
        theta_deg = degrees(theta) * direction
        radius = forward_budget / (2 * sin(theta))

        print(f"theta={theta_deg:.1f} deg, radius={radius:.1f} mm, budget={forward_budget:.1f} mm")

        self.turn_radius(theta_deg, radius, speed_steer=speed_steer, speed_drive=speed_drive)
        self.turn_radius(-theta_deg, radius, speed_steer=speed_steer, speed_drive=speed_drive)

hub = PrimeHub(front_side=Axis.Y)
print(hub.battery.voltage())

drive = Motor(Port.F, Direction.COUNTERCLOCKWISE)
steer = Motor(Port.D)

car = CarDriveBase(drive, steer, 62.4, 11, 500, 300, hub) #(drive, steer, 62.4, 10, 450, 300, hub)

#left_sensor = UltrasonicSensor(Port.E)
right_sensor = UltrasonicSensor(Port.A)
color_sensor = ColorSensor(Port.B)
#right_color_sensor = ColorSensor(Port.E)
front_sensor = UltrasonicSensor(Port.C)

def gumb():
    pressed = []
    while not any(pressed):
        pressed = hub.buttons.pressed()
        wait(10)

wall = 1000
turn_r = 290 # 330

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

strana = ""

car.use_gyro(True)


def bypass_c_continuous(self, S_arc, delta_max_deg=30, direction=1, speed=None):
    """
    S_arc: ukupna duljina krivulje (820.2mm iz simulacije), mjerena preko enkodera
    direction: +1 ili -1, ovisno o strani zaobilaska
    """
    if speed is None:
        speed = self.default_speed
    o = self.wheel_diameter * pi
    target_deg = S_arc / o * 360
    start = self.drive_motor.angle()
    self.drive_motor.run(speed)
    while abs(self.drive_motor.angle() - start) < target_deg:
        frac = abs(self.drive_motor.angle() - start) / target_deg  # 0 -> 1
        delta = delta_max_deg * (1 - 2*frac) * direction
        self.steer_motor.run_target(self.default_steer_speed, delta, wait=False)
        wait(5)
    self.drive_motor.brake()
    self.steer_motor.run_target(self.default_steer_speed, 0)






def bypass_c_continuous2(self, S_arc, delta_max_deg=30, direction=1, speed=None, kp=2.0):
    if speed is None:
        speed = self.default_speed
    L = 140  # stvarni razmak izmedju prednje i zadnje osi na kojima su kotači
    delta_max = radians(delta_max_deg)
    o = self.wheel_diameter * pi
    target_wheel_deg = S_arc / o * 360

    start_wheel = self.drive_motor.angle()
    start_heading = self.hub.imu.heading()  # lokalna referenca, ne resetira nista globalno
    self.drive_motor.run(speed)

    while abs(self.drive_motor.angle() - start_wheel) < target_wheel_deg:
        s = abs(self.drive_motor.angle() - start_wheel) / 360 * o
        frac = min(s / S_arc, 1.0)
        delta_s = delta_max * (1 - 2*frac)  # feedforward dio, bez direction

        cos_ratio = cos(delta_s) / cos(delta_max)
        psi_target = degrees(S_arc / (2*delta_max*L) * log(cos_ratio)) * direction
        target_heading = start_heading + psi_target

        err = target_heading - self.hub.imu.heading()
        steer_cmd = degrees(delta_s) * direction + kp * err
        steer_cmd = max(-delta_max_deg, min(delta_max_deg, steer_cmd))

        self.steer_motor.run_target(self.default_steer_speed, steer_cmd, wait=False)
        wait(5)

    self.drive_motor.brake()
    self.steer_motor.run_target(self.default_steer_speed, 0)
    wait(500)

def pocetak():
    car.use_gyro(True)
    global strana
    steer.run_target(100, 0)
    wait(500)
    #steer.reset_angle(0)
    flag = 1
    wait(200)
    car.drive()
    while flag == 1:
        car.correct()
        bojaRaw = color_sensor.color()
        #print("bojaRAW:", bojaRaw)
        bojaL = IMENA_BOJA.get(bojaRaw, str(bojaRaw))
        #print("boja:", boja)
        if(boja == "NARANCASTA" or boja == "PLAVA" ):
            # hub.speaker.beep()
            car.brake()
            wait(500)
            flag = 0
        elif(front_sensor.distance()<450):
            car.brake()
            wait(500)
            flag = 2
        
        if(flag == 0):
            if(boja == "NARANCASTA"):
                strana = "LEFT"
            elif(boja == "PLAVA" ):
                strana = "RIGHT"
        elif(flag == 2):
            if(right_sensor.distance() > 1000):
                strana = "LEFT"
            else:
                strana = "RIGHT"
        wait(10)
    print(strana)


app = AppData([(0, 4)])

RED_HUE   = bytes([160])
GREEN_HUE = bytes([46])


# BOJE KOJE CE BITI NA NATJECANJU: ######
#RED_HUE   = bytes([177])
#GREEN_HUE = bytes([55])
#########

red_data   = (0, 0, 0, 0) # x, y, width, height
green_data = (0, 0, 0, 0) # x, y, width, height

def Ocitaj_crvenu():
    global red_data
    red_data   = (0, 0, 0, 0) # x, y, width, height
    # --- Read red ---
    app.configure(0, 0, RED_HUE)
    wait(500)  # give the phone time to switch and send a fresh frame
    red_data = unpack("BBBB", app.get_bytes(0))
    print("Red before check:",   red_data)
    xr,yr,wr,hr = red_data
    if(wr<10 and hr<10):
        red_data = (0,0,0,0)
    print("Red after check:",   red_data)

def Ocitaj_zelenu():
    # --- Read green ---
    global green_data
    green_data = (0, 0, 0, 0) # x, y, width, height
    app.configure(0, 0, GREEN_HUE)
    wait(500)
    green_data = unpack("BBBB", app.get_bytes(0))
    print("Green before check:", green_data)
    xg,yg,wg,hg = green_data
    if(wg<7 and hg<7):
        green_data = (0,0,0,0)
    print("Green after check:", green_data)

def Drive_until_boja():
    global red_data, green_data
    steer.run_target(300, 0)
    red_data = (0, 0, 0, 0)
    green_data = (0, 0, 0, 0)
    car.drive(170)
    boja = None
    while boja is None:
        car.correct(step=5)
        Ocitaj_crvenu()
        if red_data != (0, 0, 0, 0):
            boja = "RED"
            break
        Ocitaj_zelenu()
        if green_data != (0, 0, 0, 0):
            boja = "GREEN"
        wait(10)
    car.brake()
    return boja


def Zaobidji_boju():
    boja = Drive_until_boja()
    print("Detektirana boja:", boja)

    steer.run_target(200,0)
    wait(500)
    car.default_speed = 300
    car.use_gyro(False)
    car.straight(-150)
    wait(500)
    car.use_gyro(True)

    if boja == "RED":
        bypass_c_continuous2(car, S_arc=692, delta_max_deg=30, direction=1)
        car.turn(90,-30,speed_drive=250, speed_steer=200)
        steer.run_target(200,0)
        wait(500)
        #mozda tu treba paziti na distance od zida naprijed!!!
        car.turn(90,30,speed_drive=250, speed_steer=200)
    elif boja == "GREEN":

        bypass_c_continuous2(car, S_arc=692, delta_max_deg=30, direction=-1)
        car.turn(90,30,speed_drive=250, speed_steer=200)
        steer.run_target(200,0)
        wait(500)
        #mozda tu treba paziti na distance od zida naprijed!!!
        car.turn(90,-30,speed_drive=250, speed_steer=200)
    

def Odlazak_s_parkinga():
    steer.run_target(200,0)
    if(right_sensor.distance()>200):
        car.default_speed=190
        car.use_gyro(False)
        #car.straight(-90)
        #wait(3000)
        car.turn(-40,-40)
        wait(3000)
        car.turn(120,40)
        wait(3000)
        car.turn(120,-40)
        wait(3000)
        car.turn(40,40)
        car.straight(-500)
        wait(3000)
    else:
        car.default_speed=190
        car.use_gyro(False)
        #car.straight(-90)
        #wait(3000)
        car.turn(-40,40)
        wait(3000)
        car.turn(120,-40)
        wait(3000)
        car.turn(120,40)
        wait(3000)
       # car.turn(40,40)
        car.straight(-500)
        wait(3000)
       
        # car.turn(25,-40)
        # wait(3000)
        # car.turn(25,40)
        # wait(3000)
        # car.turn(20,20)
    car.use_gyro(True)   

#Odlazak_s_parkinga()



steer.run_target(200,20)
car._straight(300,_gyro=False)

# car.default_speed = 300
# boja = Drive_until_boja()
# print("Detektirana boja:", boja)
# distance = []
# for i in range(10):
#     distance.append(front_sensor.distance())
#     steer.run_angle(200,5)
#     wait(10)
# steer.run_target(300, 0)
# dist = min(distance)
# print(dist)
# car.bypass_obstacle(300,dist)
# print("Finished")


print("battery: ",hub.battery.voltage())
    


