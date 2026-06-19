from pybricks.pupdevices import Motor, UltrasonicSensor, ColorSensor
from pybricks.parameters import Port, Direction, Stop, Color, Axis
from pybricks.tools import wait
from pybricks.hubs import PrimeHub
from pybricks.tools import StopWatch, wait
from math import ceil, pi, tan, radians, sin, atan, degrees, floor, cos, log
from pupremote_hub import PUPRemoteHub
from hub_color_reader_openmv import get_color




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
        
        # print(
        #     f"Angle={angle:>4} | ",
        #     f"Gyro={gyro:>4} | ",
        #     f"Error={error:>4} | ",
        #     f"Ceil={ceil_value:>3} | ",
        #     f"Correction={correction:>4} | ",
        #     f"Limited={limited:>4}"
        # )


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
        speed = abs(speed)
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

    def turn(self, target_deg, step_deg, tolerance=1.5, speed_steer=None, speed_drive=None, reset_gyro=True):
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
        while abs(self.drive_motor.angle() - start) < abs(deg) and not (not abs(abs(self.hub.imu.heading()) - abs(target_deg)) > 3 * tolerance and reset_gyro):
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
        if reset_gyro:
            self.hub.imu.reset_heading((self.hub.imu.heading() - sa * target_deg))
            # print(self.hub.imu.heading())
            # print("-------------------------------------------")
        self.brake()
    
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



#################################################################
#################################################################
#################################################################
#################################################################

hub = PrimeHub(front_side=Axis.Y)

drive = Motor(Port.F, Direction.COUNTERCLOCKWISE)
steer = Motor(Port.D)

car = CarDriveBase(drive, steer, 62.4, 18, 250, 300, hub) #(drive, steer, 62.4, 10, 450, 300, hub)

#left_sensor = UltrasonicSensor(Port.E)
right_sensor = UltrasonicSensor(Port.A)
color_sensor = ColorSensor(Port.B)
#right_color_sensor = ColorSensor(Port.E)
front_sensor = UltrasonicSensor(Port.C)

print(hub.battery.voltage())

def test_gyro():
    # Test - samo ovo pokreni
    car.use_gyro(False)  # čisto enkoder, bez korekcije
    car.reset_gyro()
    car.turn(90, 30)
    print("Gyro nakon skretanja:", hub.imu.heading())



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


car.use_gyro(True)



def Drive_until_boja():
    global red_data, green_data
    steer.run_target(300, 0)
    red_data = (0, 0, 0, 0)
    green_data = (0, 0, 0, 0)
    car.drive(170)
    boja = None
    color = "BIJELA"
    while (boja is None and color == "BIJELA"):
        car.correct(step=5)
        bojaRaw = color_sensor.color()
        #print("bojaRAW:", bojaRaw)
        color = IMENA_BOJA.get(bojaRaw, str(bojaRaw))
        Ocitaj_crvenu()
        if red_data != (0, 0, 0, 0):
            boja = "RED"
        Ocitaj_zelenu()
        if green_data != (0, 0, 0, 0):
            boja = "GREEN"
        wait(10)
    car.brake()
    return boja


def Zaobidji_boju():
    detektirana_boja = Drive_until_boja()
    print("Detektirana boja:", detektirana_boja)

    steer.run_target(200,0)
    wait(500)
    car.default_speed = 300
    car.use_gyro(False)
    car.straight(-150)
    wait(500)
    car.use_gyro(True)


def scan_colors():
    RED_SAMPLES = 50
    
    r_areas = []
    g_areas = []
    r, rx, ry, rw, rh, g, gx, gy, gw, gh = 0,0,0,0,0,0,0,0,0,0
    for _ in range(RED_SAMPLES):
        r, rx, ry, rw, rh, g, gx, gy, gw, gh = get_color()
        if r:
            r_areas.append(rw * rh)
        if g:
            g_areas.append(gw * gh)
        r, rx, ry, rw, rh, g, gx, gy, gw, gh = 0,0,0,0,0,0,0,0,0,0
    
    has_red   = len(r_areas) > 25  # vidjena u vise od pola mjerenja
    has_green = len(g_areas) > 25

    if has_red and has_green:
        avg_r = sum(r_areas) / len(r_areas)
        avg_g = sum(g_areas) / len(g_areas)
        if avg_r >= avg_g:
            return "R1"  # crveni blizi
        else:
            return "G1"  # zeleni blizi
    elif has_red:
        return "R0"
    elif has_green:
        return "G0"
    else:
        return "NONE"


prvi = False
cnt = 0
def ocitaj():
    global cnt
    cnt += 1
    global prvi
    car.default_speed = 300
    car.brake()
    steer.run_target(300,0)
    wait(200)
    car.straight(-50)
    color_id = "NONE"
    color_id, x, y, w, h = get_color()
    if(color_id != "NONE"):
        if(color_id == "RED"):
            print("REDDD")
            car.use_gyro(False)
            wait(10)
            car.straight(-100)
            car.turn(45,20, reset_gyro=False)
            car.straight(130)
            car.turn(55,-20, reset_gyro=False)
            car.straight(80)
            car.turn(90,-35, reset_gyro=True)
            #car.straight(-10)
            car.turn(90,35, reset_gyro=True)  # tolerance=5
            wait(500)
            #car.straight(155)
        else:
            print("GREEN")
            car.use_gyro(False)
            wait(10)
            car.straight(-100)
            car.turn(55,-20, reset_gyro=False)
            car.straight(90)
            car.turn(55,20, reset_gyro=False)
            car.straight(80)
            # -------------------------
            car.use_gyro(True)
            car.turn(90,35, reset_gyro=True)
            car.use_gyro(False)
            #--------------------------------------

            #TEST: ZAUSTAVI SE PO ZIDU UMJESTO 20
            #car.straight(20)

            '''---- Treba pomaknuti prepreku do iste pozicije za skretanje:''' 
            if cnt == 1:
                dis=[]
                steer.run_target(200, -25)
                for i in range(10):
                    steer.run_angle(200, 5)
                    dis.append(front_sensor.distance())
                for i in range(3):
                    dis.remove(max(dis))
                    dis.remove(min(dis))
                distance=sum(dis)/4
                steer.run_target(200, 0)
                print("distance: ",distance)
                wait(200)
                car.straight(distance-600)
            else:
                car.straight(-170)
            '''----------------------------------------------------''' 
            car.use_gyro(True)
            car.turn(90,-35, reset_gyro=True)
            car.use_gyro(False)
            #car.straight(20)
            wait(500)
        wait(500)
        return True
    return False

def Stranica():
    global cnt
    cnt = 0
    global prvi
    prvi=False
    drugi=False
    car.use_gyro(False)
    global strana
    #steer.run_target(200,0)
    prvi = ocitaj()
    print(hub.imu.heading())
    if prvi:
        car.straight(-60)
        drugi = ocitaj()
        if(not drugi):
            car.straight(500)
    else:
        car.straight(500)
        drugi = ocitaj()
        if not drugi:
            car.use_gyro(True)
            car.straight(-500)
            car.straight(500)
            car.use_gyro(False)
            car.straight(500)
            ocitaj()
    hub.speaker.beep()
    print(hub.imu.heading())
    gumb()
    wait(3000)
    steer.run_target(200,0)
    car.straight(-(wall - st - front_sensor.distance()-450))
    car.use_gyro(False)
    strana = "RIGHT"
    if(strana == "RIGHT"):
        car.use_gyro(True)
        car.turn(-90,35, reset_gyro=True)
        car.use_gyro(False)
        steer.run_target(200,0)
        #car.use_gyro(True)
        car.drive(300)
        col_sens_boja = "BIJELA"
        while(col_sens_boja != "NARANCASTA"):
            car.correct()
            bojaRaw = color_sensor.color()
            col_sens_boja = IMENA_BOJA.get(bojaRaw, str(bojaRaw))
        car.brake()
        hub.speaker.beep()
    else:
        car.turn(-160,-35)
        steer.run_target(200,0)
        car.use_gyro(False)
        car.drive(300)
        col_sens_boja = "BIJELA"
        while(col_sens_boja != "PLAVA"):
            car.correct()
            bojaRaw = color_sensor.color()
            col_sens_boja = IMENA_BOJA.get(bojaRaw, str(bojaRaw))
        car.brake()
        hub.speaker.beep()
    car.use_gyro(False)

#_______________________________________--
#_______________________________________

def vozi_uz_zid(udaljenost_mm, target_dist=None, kp=2.0, max_steer=30, speed=None, ignore_below=None):
    """
    udaljenost_mm: koliko mm da preveze (enkoder)
    target_dist:   željena udaljenost od desnog zida u mm
    kp:            proporcionalni gain
    max_steer:     maksimalni steer kut korekcije
    ignore_below:  zanemaری očitanja ispod ovog praga (prepreka ispred zida)
                   npr. ignore_below=600 ako hoćeš biti 700mm od zida
    """
    if speed is None:
        speed = car.default_speed

    if target_dist is None:
        samples = []
        for _ in range(5):
            d = right_sensor.distance()
            if ignore_below is None or d >= ignore_below:
                samples.append(d)
            wait(10)
        if samples:
            target_dist = sum(samples) / len(samples)
        else:
            print("else")
            target_dist = ignore_below + 100  # fallback ako su sva očitanja blokirana
        print("Target dist:", target_dist)

    o = car.wheel_diameter * pi
    target_wheel_deg = udaljenost_mm / o * 360
    start = car.drive_motor.angle()

    last_valid_steer = 0  # zapamti zadnji steer kad je očitanje bilo validno

    car.drive_motor.run(speed)

    while abs(car.drive_motor.angle() - start) < target_wheel_deg:
        dist = right_sensor.distance()

        if ignore_below is None or dist >= ignore_below:
            # Validno očitanje — kalkuliraj korekciju normalno
            error = target_dist - dist
            steer = max(-max_steer, min(max_steer, kp * error))
            last_valid_steer = steer
        else:
            # Prepreka u putu — zadrži zadnji validni steer
            steer = last_valid_steer

        car.steer_motor.run_target(car.default_steer_speed, -steer, wait=False)
        wait(10)

    car.brake()
    car.steer_motor.run_target(car.default_steer_speed, 0)

# Uzme trenutnu udaljenost kao referencu i vozi 500mm
#vozi_uz_zid(500)

# # Ili zadaj točnu ciljnu udaljenost od zida
#vozi_uz_zid(500, target_dist=100)

# # Podesi agresivnost korekcije
#vozi_uz_zid(800, target_dist=100, kp=2.5, max_steer=20)

#______________________________________________________________
#______________________________________________________________

#Stranica()

# while True:
#     print(get_color())
# print(get_color())


#print(colors_exist)

# Odlazak_s_parkinga()
strana = ""


def start():
    global strana
    car.use_gyro(True)
    colors_exist=scan_colors()
    print(colors_exist)
    if(colors_exist == "R0" or colors_exist == "R1"):
        car.turn(45,30)
        car.straight(130)
        car.turn(45,-30)
        car.drive(300)
        boja = "BIJELA"
        flag = 1
        while flag == 1:
            car.correct()
            bojaRaw = color_sensor.color()
            #print("bojaRAW:", bojaRaw)
            boja = IMENA_BOJA.get(bojaRaw, str(bojaRaw))
            #print("boja:", boja)
            if(boja == "NARANCASTA" or boja == "PLAVA"):
                # hub.speaker.beep()
                car.brake()
                wait(100)
                flag = 0
        if(boja == "NARANCASTA"):
            strana = "LEFT"
        else: strana = "RIGHT"
        unutarnje_skretanje()

    elif(colors_exist == "G0" or colors_exist == "G1"):
        car.turn(40,-30)
        car.straight(120)
        car.turn(40,30)
        car.drive(300)
        boja = "BIJELA"
        flag = 1
        while flag == 1:
            car.correct()
            bojaRaw = color_sensor.color()
            #print("bojaRAW:", bojaRaw)
            boja = IMENA_BOJA.get(bojaRaw, str(bojaRaw))
            #print("boja:", boja)
            if(boja == "NARANCASTA" or boja == "PLAVA"):
                # hub.speaker.beep()
                car.brake()
                wait(100)
                flag = 0
        if(boja == "NARANCASTA"):
            strana = "LEFT"
        else: strana = "RIGHT"
        vanjsko_skretanje()
    elif(colors_exist == "NONE"):
        car.drive(300)
        boja = "BIJELA"
        flag = 1
        while flag == 1:
            car.correct()
            bojaRaw = color_sensor.color()
            #print("bojaRAW:", bojaRaw)
            boja = IMENA_BOJA.get(bojaRaw, str(bojaRaw))
            #print("boja:", boja)
            if(boja == "NARANCASTA" or boja == "PLAVA"):
                # hub.speaker.beep()
                car.brake()
                wait(100)
                flag = 0
        if(boja == "NARANCASTA"):
            strana = "LEFT"
            car.turn(90,30)
        else: 
            strana = "RIGHT"
            car.turn(90,-30)


    print("strana: ", strana)


def unutarnje_skretanje():
    global strana
    car.reset_gyro()
    #car.turn(10,30)
    car.reset_gyro()
    steer.run_target(300,0)
    d = []
    for i in range(20):
        d.append(front_sensor.distance())
    for i in range(5):
        d.remove(max(d))
        d.remove(min(d))
    dis = sum(d)/10
    car.use_gyro(True)
    car.straight(dis-50)
    car.use_gyro(True)
    car.reset_gyro()
    if (strana == "LEFT"):
        car.turn(-90,-30)
        car.brake()
    elif (strana == "RIGHT"):
        car.turn(-90,30)
        car.brake()
    car.use_gyro(True)
    steer.run_target(300,0)

def vanjsko_skretanje():
    global strana
    d = []
    for i in range(20):
        d.append(front_sensor.distance())
    for i in range(5):
        d.remove(max(d))
        d.remove(min(d))
    dis = sum(d)/10
    car.use_gyro(True)
    d = []
    for i in range(20):
        d.append(front_sensor.distance())
    for i in range(5):
        d.remove(max(d))
        d.remove(min(d))
    dis = sum(d)/10
    car.use_gyro(True)
    car.straight(dis-600)
    car.use_gyro(True)
    if(strana == "LEFT"):
        car.turn(90,30)
        car.brake()
    elif(strana == "RIGHT"):
        car.turn(90,-30)
        car.brake()
    car.use_gyro(True)
    steer.run_target(300,0)


#start()

def crvena_crvena():
    global strana
    car.turn(55,30)
    car.straight(170)
    car.turn(45,-30)
    vozi_uz_zid(800, target_dist=170, kp=2.5, max_steer=15)
    car.drive(300)
    boja = "BIJELA"
    flag = 1
    while flag == 1:
        #car.correct()
        bojaRaw = color_sensor.color()
        #print("bojaRAW:", bojaRaw)
        boja = IMENA_BOJA.get(bojaRaw, str(bojaRaw))
        #print("boja:", boja)
        if(boja == "NARANCASTA" and strana == "LEFT"):
            # hub.speaker.beep()
            car.brake()
            wait(100)
            flag = 0
            unutarnje_skretanje()
        elif(boja == "PLAVA" and strana == "RIGHT"):
            # hub.speaker.beep()
            car.brake()
            wait(100)
            flag = 0
            vanjsko_skretanje()
            

def zelena_zelena():
    global strana
    car.turn(40,-30)
    car.straight(170)
    car.turn(40,30)
    vozi_uz_zid(900,750,kp=2.5,max_steer=10,ignore_below=400)
    car.drive(300)
    boja = "BIJELA"
    flag = 1
    while flag == 1:
        #car.correct()
        bojaRaw = color_sensor.color()
        #print("bojaRAW:", bojaRaw)
        boja = IMENA_BOJA.get(bojaRaw, str(bojaRaw))
        #print("boja:", boja)
        if(boja == "NARANCASTA" and strana == "LEFT"):
            # hub.speaker.beep()
            car.brake()
            wait(100)
            flag = 0
            vanjsko_skretanje()
        elif(boja == "PLAVA" and strana == "RIGHT"):
            # hub.speaker.beep()
            car.brake()
            wait(100)
            flag = 0
            unutarnje_skretanje()

def crvena_zelena():
    global strana
    car.turn(40,30)
    car.straight(160)
    car.turn(40,-30)
    for i in range(2):
        car.turn(40,-30)
        car.straight(150)
        car.turn(40,30)
    car.drive(300)
    boja = "BIJELA"
    flag = 1
    while flag == 1:
        car.correct()
        bojaRaw = color_sensor.color()
        #print("bojaRAW:", bojaRaw)
        boja = IMENA_BOJA.get(bojaRaw, str(bojaRaw))
        #print("boja:", boja)
        if(boja == "NARANCASTA" and strana == "LEFT"):
            # hub.speaker.beep()
            car.brake()
            wait(100)
            flag = 0
            vanjsko_skretanje()
        elif(boja == "PLAVA" and strana == "RIGHT"):
            # hub.speaker.beep()
            car.brake()
            wait(100)
            flag = 0
            unutarnje_skretanje()
    
def zelena_crvena():
    global strana
    car.turn(40,-30)
    car.straight(170)
    car.turn(40,30)
    car.straight(-90)
    for i in range(2):
        car.turn(43,30)
        car.straight(150)
        car.turn(43,-30)
    car.use_gyro(True)
    car.drive(300)
    boja = "BIJELA"
    flag = 1
    while flag == 1:
        car.correct()
        bojaRaw = color_sensor.color()
        #print("bojaRAW:", bojaRaw)
        boja = IMENA_BOJA.get(bojaRaw, str(bojaRaw))
        #print("boja:", boja)
        if(boja == "NARANCASTA" and strana == "LEFT"):
            # hub.speaker.beep()
            car.brake()
            wait(100)
            flag = 0
            unutarnje_skretanje()
        elif(boja == "PLAVA" and strana == "RIGHT"):
            # hub.speaker.beep()
            car.brake()
            wait(100)
            flag = 0
            vanjsko_skretanje()


def krug():
    start()
    for i in range(10):
        car.straight(-40)
        exist = scan_colors()
        print(exist)
        if(exist == "G1"):
            zelena_crvena()
        elif(exist == "R1"):
            crvena_zelena()
        elif(exist == "G0"):
            zelena_zelena()
        elif(exist == "R0"):
            crvena_crvena()
        elif(exist == "NONE"):
            if(strana == "LEFT"):
                zelena_zelena()
            else:
                crvena_crvena()
    exist = scan_colors()
    if(exist == "R0" or exist == "R1"):
        car.turn(40,30)
        car.straight(160)
        car.turn(40,-30)
        car.straight(150)
    else:
        car.turn(40,-30)
        car.straight(170)
        car.turn(40,30)
        car.straight(150)

krug()


