from pybricks.pupdevices import Motor, UltrasonicSensor, ColorSensor
from pybricks.parameters import Port, Direction, Stop, Color, Axis
from pybricks.tools import wait
from pybricks.hubs import PrimeHub
from pybricks.tools import StopWatch, wait
from math import ceil, pi, tan, radians, sin, atan, degrees



class CarDriveBase:
    """Represents a car-like robot with independent drive and steering control."""
    def __init__(self, drive_motor, steer_motor, wheel_diameter,
                 axle_track, default_speed, default_steer_speed, hub):
        """Initialize the CarDriveBase with motors and configuration parameters.
        
        Args:
            drive_motor: Motor object for driving forward/backward.
            steer_motor: Motor object for steering control.
            wheel_diameter: Diameter of the drive wheels in millimeters.
            axle_track: Distance between the two front wheels in centimeters.
            default_speed: Default speed for driving operations in degrees per second.
            default_steer_speed: Default speed for steering operations in degrees per second.
            hub: PrimeHub object for access to IMU and other hub features.
        """
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
        """Enable or disable gyro-based heading correction.
        
        Args:
            use: Boolean indicating whether to use gyro for heading correction.
        """
        self.gyro = use
        
    def reset_gyro(self):
        """Reset the gyro heading to 0 degrees if gyro is enabled."""
        if self.gyro:
            self.hub.imu.reset_heading(0)
        
    def correct(self, angle=0, step=30, pr=False):
        """Correct steering to maintain a target heading angle.
        
        Args:
            angle: Target heading angle in degrees (default 0).
            step: Maximum steering angle adjustment per correction in degrees (default 30).
            pr: Boolean to enable debug printing (default False).
        """
        if self.gyro:
            # if pr:
            # print(max(-abs(step), min(abs(step), 15 * ceil((angle - self.hub.imu.heading()) / 15))))
            self.steer_motor.run_target(self.default_steer_speed, max(-abs(step), min(abs(step), 15 * ceil((angle - self.hub.imu.heading()) / 15))), wait=False)
            if pr:
                print("-c-", front_sensor.distance())
                print(angle, self.hub.imu.heading())

    def distance(self):
        """Calculate the distance traveled based on drive motor rotation.
        
        Returns:
            Distance traveled in millimeters.
        """
        return self.drive_motor.angle() * self.wheel_diameter * pi / 360

    def drive(self, speed=None):
        """Start driving at the specified or default speed.
        
        Args:
            speed: Speed in degrees per second (default is default_speed).
        """
        if speed is None:
            speed = self.default_speed
        self.drive_motor.run(speed)
        self.running = True

    def stop(self):
        """Stop the drive motor immediately without braking."""
        self.drive_motor.stop()
        self.running = False

    def brake(self):
        """Apply brakes to the drive motor."""
        self.drive_motor.brake()
        self.running = False

    def _straight(self, dist, turn_rate=0, speed=None, _gyro=True):
        """Internal method to drive straight for a specified distance.
        
        Args:
            dist: Distance to drive in millimeters.
            turn_rate: Target heading angle for gyro correction in degrees (default 0).
            speed: Speed in degrees per second (default is default_speed).
            _gyro: Boolean to enable gyro correction during movement (default True).
        """
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
        """Drive straight for a specified distance with wheels aligned.
        
        Args:
            dist: Distance to drive in millimeters.
            turn_rate: Target heading angle for gyro correction in degrees (default 0).
            speed: Speed in degrees per second (default is default_speed).
        """
        self.steer_motor.run_target(self.default_steer_speed, 0)
        self._straight(dist, turn_rate, speed)

    def turn(self, target_deg, step_deg, tolerance=1.5, speed_steer=None, speed_drive=None):
        """Turn the robot by a target angle using Ackermann steering geometry.
        
        Args:
            target_deg: Target rotation angle in degrees.
            step_deg: Steering wheel angle in degrees for the turn.
            tolerance: Heading tolerance in degrees (default 1.5).
            speed_steer: Steering motor speed in degrees per second (default is default_steer_speed).
            speed_drive: Drive motor speed in degrees per second (default is default_speed).
        """
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
        """Turn the robot by a target angle using a specified turning radius.
        
        Args:
            target_deg: Target rotation angle in degrees.
            radius: Turning radius in millimeters.
            tolerance: Heading tolerance in degrees (default 1.5).
            speed_steer: Steering motor speed in degrees per second (default is default_steer_speed).
            speed_drive: Drive motor speed in degrees per second (default is default_speed).
        """
        axle_track_mm = self.axle_track * 10  # cm to mm

        step_deg = degrees(2 * atan(axle_track_mm / (2 * radius)))
        

        # Preserve the sign of target_deg
        if target_deg < 0:
            step_deg = -step_deg
        print("step_deg",step_deg)
        self.turn(target_deg, step_deg, tolerance, speed_steer, speed_drive)
        print("turn radius stao")
        wait(100)

hub = PrimeHub(front_side=Axis.Y)
print(hub.battery.voltage())

drive = Motor(Port.F, Direction.COUNTERCLOCKWISE)
steer = Motor(Port.D)

car = CarDriveBase(drive, steer, 62.4, 11, 550, 300, hub) #(drive, steer, 62.4, 10, 450, 300, hub)

left_sensor = UltrasonicSensor(Port.E)
right_sensor = UltrasonicSensor(Port.A)
color_sensor = ColorSensor(Port.B)
front_sensor = UltrasonicSensor(Port.C)

def gumb():
    """Wait for any button on the hub to be pressed."""
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

def pocetak():
    """Initialize the robot by detecting the starting direction based on color sensor.
    
    The robot drives forward until it detects either orange (LEFT) or blue (RIGHT) color,
    then sets the global strana variable accordingly.
    """
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
    """Execute the left turn sequence, following the left wall and detecting orange color markers.
    
    The robot drives forward until reaching a wall, then turns left and follows the wall
    while detecting 11 orange color markers along a path.
    """
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
    wait(500)
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
        wait(500)        
        car.turn_radius(90, turn_r)
        


    car.drive()
    while front_sensor.distance() > wall + dist:
        car.correct()
        wait(5)
    car.brake()

    car.straight(200)


def okrenutDesno():
    """Execute the right turn sequence, following the right wall and detecting blue color markers.
    
    The robot drives forward until reaching a wall, then turns right and follows the wall
    while detecting 11 blue color markers along a path.
    """
    car.drive()
    while front_sensor.distance() > wall:
        car.correct()
        wait(5)
    car.brake()
    car.straight(-(wall - st - front_sensor.distance()))
    dist = car.distance()
    wait(500)
    car.turn_radius(90, -turn_r)

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
        # car.drive()
        # while front_sensor.distance() > wall:
        #     car.correct()
        #     wait(5)
        # car.brake()

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
        wait(500)
        car.turn_radius(90, -turn_r) #90 - i / 10


    car.straight(200)


try:
    sw = StopWatch()
    pocetak()
    if(strana == "LEFT"):
        okrenutLijevo()
    elif(strana == "RIGHT"):
        okrenutDesno()
    car.straight(100)
    print(sw/1000)
finally:
    print("battery: ",hub.battery.voltage())



# while True:
#     boja = color_sensor.color()
#     print(IMENA_BOJA.get(boja, str(boja)))

# for i in range(4):
#     car.turn(90, 30)
#     hub.speaker.beep()
#     car.straight(1000)
#     hub.speaker.beep()