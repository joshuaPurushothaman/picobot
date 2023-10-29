import math

# EZ MATH
def clamp(value: float, min_out: float, max_out: float):
    return max(min(value, max_out), min_out)


def map(value: float, in_min: float, in_max: float, out_min: float, out_max: float):
    """Scales values linearly, like Arduino map()"""
    
    """
    >>> map(75, 50, 100, 0, 100)

    >>> inScale = 100 - 50 = 50
    >>> outScale = 100 - 0 = 100

    >>> from0to1 = (75 - 50) / 50 = 25/50 = 1/2

    >>> return 1/2 * 100 = 50 # YUH
    """

    inScale = in_max - in_min
    outScale = out_max - out_min
    
    from0to1 = (value - in_min) / inScale

    return from0to1 * outScale

class Length(float):
    def __init__(self, meters: float):
        self = meters

    @staticmethod
    def from_meters(meters: float):
        return Length(meters)

    def to_meters(self):
        return self

    @staticmethod
    def from_mm(mm: float):
        return Length(mm / 1000.0)
    
    def to_mm(self):
        return self * 1000.0

    @staticmethod
    def from_inches(inches: float):
        return Length(inches * 0.0254)
    
    def to_inches(self):
        return self / 0.0254

    @staticmethod
    def from_feet(feet: float):
        return Length(feet * 0.3048)
    
    def to_feet(self):
        return self / 0.3048


class PIDController:
    iErr = 0
    lastError = 0

    def __init__(self,
                    kP=1.0, kI=0.0, kD=0.0,
                    min_output = float("-inf"), max_output = float("inf"),
                    i_zone=0, dt = 0.01):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.min_output = min_output
        self.max_output = max_output
        self.i_zone = i_zone
        self.dt = dt
    
    def calculate(self, setpoint: float, processVariable: float):
        error = setpoint - processVariable

        # integrate error
        self.iErr += error
        # apply iZone
        iTerm = (self.kI * self.iErr * self.dt) if (abs(error) < self.i_zone) else 0


        # calculate dErr
        dErr = error - self.lastError
        self.lastError = error


        # output
        output = (self.kP * error
                + iTerm
                + self.kD * dErr / self.dt)
            
        # clamp between min and max
        output = clamp(output, self.min_output, self.max_output)

        return output

class ProfiledPIDController:
    #FIXME
    def __init__(self, pid: PIDController):
        raise NotImplementedError("TODO")

class Pose2d:
    """
        Represents a two-dimensional position.
        
        Units: meters and radians.
        
        Conventions:
            forward: +x
            right: +y
            clockwise: +theta
    """
    x = 0.0
    y = 0.0
    theta = 0.0

    def reset(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

# TODO
# class Odometry:
#     pose = Pose2d()

#     def __init__(self, lMotor: Motor, rMotor: Motor, gyro: GyroSensor, TRACK_WIDTH: float, WHEEL_DIAMETER: float):
#         """
#         units in meters please!
#         """
#         self.lMotor = lMotor
#         self.rMotor = rMotor
#         self.gyro = gyro
#         self.TRACK_WIDTH = TRACK_WIDTH
#         self.WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * math.pi
    
#     def update(self) -> Pose2d:
#         """
#             call periodically plsZ!
#             thanks http://seattlerobotics.org/encoder/200610/Article3/IMU%20Odometry,%20by%20David%20Anderson.htm
        
#         """
#         # (1)	distance = (left_encoder + right_encoder) / 2.0
#         # (2)	theta = (left_encoder - right_encoder) / WHEEL_BASE;
#         # (3)	X_position = distance * sin(theta);
#         # (4)	Y_position = distance * cos(theta);

#         # calculate how far each side has gone
#         lRotMeters = (self.lMotor.position / self.lMotor.count_per_rot) * self.WHEEL_CIRCUMFERENCE
#         rRotMeters = (self.rMotor.position / self.rMotor.count_per_rot) * self.WHEEL_CIRCUMFERENCE

#         self.pose.x = math.sin((lRotMeters + rRotMeters) / 2.0)
#         self.pose.y = math.cos((lRotMeters + rRotMeters) / 2.0)

#         if self.gyro == None:
#             self.pose.theta = (lRotMeters - rRotMeters) / self.TRACK_WIDTH; # in radians!!
#         else:
#             self.pose.theta = self.gyro.angle()

#         return self.pose

def generate_trapezoid_profile(max_v , time_to_max_v , dt , goal):
    """ 
    From Controls Engineering in FIRST Robotics Competition, by Tyler Veness
    Creates a trapezoid profile with the given constraints.
    Returns:
    t_rec -- list of timestamps
    x_rec -- list of positions at each timestep
    v_rec -- list of velocities at each timestep
    a_rec -- list of accelerations at each timestep
    Keyword arguments :
    max_v -- maximum velocity of profile
    time_to_max_v -- time from rest to maximum velocity
    dt -- timestep
    goal -- final position when the profile is at rest
    """
    t_rec = [0.0]
    x_rec = [0.0]
    v_rec = [0.0]
    a_rec = [0.0]
    a = max_v / time_to_max_v
    time_at_max_v = goal / max_v - time_to_max_v
    # If profile is short
    if max_v * time_to_max_v > goal:
        time_to_max_v = math.sqrt(goal / a)
        time_from_max_v = time_to_max_v
        time_total = 2.0 * time_to_max_v
        profile_max_v = a * time_to_max_v
    else:
        time_from_max_v = time_to_max_v + time_at_max_v
        time_total = time_from_max_v + time_to_max_v
        profile_max_v = max_v
    while t_rec [-1] < time_total :
        t = t_rec [-1] + dt
        t_rec.append(t)
        if t < time_to_max_v :
            # Accelerate up
            a_rec.append(a)
            v_rec.append(a * t)
        elif t < time_from_max_v :
            # Maintain max velocity
            a_rec.append (0.0)
            v_rec.append( profile_max_v )
        elif t < time_total :
            # Accelerate down
            decel_time = t - time_from_max_v
            a_rec.append(-a)
            v_rec.append( profile_max_v - a * decel_time )
        else:
            a_rec.append (0.0)
            v_rec.append (0.0)
            x_rec.append(x_rec [-1] + v_rec [-1] * dt)
    return t_rec , x_rec , v_rec , a_rec

class TrapezoidProfile: #TODO
    def __init__(self, max_v: float, max_a: float, dt: float, goal: float):
        pass

    def calculate(self, t: float):
        pass
