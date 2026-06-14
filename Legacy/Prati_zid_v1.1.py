from pybricks.pupdevices import Motor, UltrasonicSensor
from pybricks.parameters import Port, Direction, Stop
from pybricks.tools import wait
from pybricks.hubs import PrimeHub
from pybricks.tools import StopWatch, wait
from math import pi, tan


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
FRONT_CRITICAL = 200   # glavni prag za hitni recovery
WALL_DETECTED = 600
GAIN = 1.05

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

def pratiZid(strana):
    print("PRATIM ZID: ", strana)
    l = left_sensor.distance()
    r = right_sensor.distance()

    distWall=400
    if strana=='L':
        error = 400 - l
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
        flag = 0
        while abs(kut)>3:
            # if((kut<1 and kut>-1 ) or ((l < 400) and (r < 400))):
            #     flag = 1
            drive_motor.dc(50)
            steer_motor.run_target(200, kut, wait=False)
            kut=kut*0.93
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
print(hub.battery.voltage())



while True:

    front_dist = front_sensor.distance()
    left_dist = left_sensor.distance()
    right_dist = right_sensor.distance()

    print("F:", front_dist, "L:", left_dist, "R:", right_dist, "STATE:", STATE)

    # # ---------------- EMERGENCY ----------------
    # if front_dist < FRONT_CRITICAL:
    #     STATE = "RECOVER"
    #     print("---------------RECOVERING -----------------------")
    # elif left_dist>right_dist and left_dist>500:
    #     STATE="LEFT_TURN"
    #     print("---------------TURNING LEFT -----------------------")

    # elif left_dist<right_dist and right_dist>500:
    #     STATE="RIGHT_TURN"
    #     print("---------------TURNING RIGHT -----------------------")
    # else: STATE="FORWARD"

    # # ---------------- STATE MACHINE ------------
    # if STATE == "RECOVER":
    #     recover()
    # elif STATE=="LEFT_TURN":
    #     turn("L")
    # elif STATE=="RIGHT_TURN":
    #     turn("R")
    # else:
    #     # normal driving
    #     center()
    #     drive_motor.dc(30)

    # wait(40)

    while front_sensor.distance()>WALL_DETECTED:
        drive_motor.dc(56)
        pratiZid("R")
        wait(40)
    drive_motor.dc(0)
    zavoj2("R")