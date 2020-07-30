from __future__ import division
import time
#from board import SCL, SDA
#import busio
import Adafruit_PCA9685



""" joint_key convention:
    R - right, L - left
    F - front, M - middle, B - back
    H - hip, K - knee, A - Ankle
    key : (channel, minimum_pulse_length, maximum_pulse_length) """

#USE YOUR VALUES OF MIN AND MAX PULSE LENGTHS
"""
joint_properties = {
    'LFH':  (10, 330, 480, -1),   #LFtibia
    'LFK':  (11, 200, 515, -1),   #LFfemur
    'LFA':  (12, 130, 610,  1),   #LFcoxa
    'RFH':  (13, 380, 530,  1),   #RFtibia
    'RFK':  (14, 300, 615,  1),   #RFfemur
    'RFA':  (15, 130, 610, -1),   #RFcoxa
    'LMH':  ( 4, 320, 470, -1),   #LMtibia
    'LMK':  ( 5, 251, 531, -1),   #LMfemur
    'LMA':  ( 6, 130, 610,  1),   #LMcoxa
    'RMH':  ( 7, 380, 530,  1),   #RMtibia
    'RMK':  ( 8, 290, 605,  1),   #RMfemur
    'RMA':  ( 9, 150, 630, -1),   #RMcoxa
    'LBH':  (17, 350, 500, -1),   #LBtibia
    'LBK':  ( 0, 200, 515, -1),   #LBfemur
    'LBA':  ( 1, 180, 660,  1),   #LBcoxa
    'RBH':  (18, 350, 500,  1),   #RBtibia
    'RBK':  ( 3, 300, 615,  1),   #RBfemur
    'RBA':  ( 2, 130, 610, -1),   #RBcoxa
    'N':    (19, 150, 650,  1)    #Neck
  }
"""
joint_properties = {
    'LFH':  (10, 110, 600, -1),   #LFtibia
    'LFK':  (11, 110, 600, -1),   #LFfemur
    'LFA':  (12, 110, 600,  1),   #LFcoxa
    'RFH':  (13, 110, 600,  1),   #RFtibia
    'RFK':  (14, 110, 600,  1),   #RFfemur
    'RFA':  (15, 110, 600, -1),   #RFcoxa
    'LMH':  ( 4, 110, 600, -1),   #LMtibia
    'LMK':  ( 5, 110, 600, -1),   #LMfemur
    'LMA':  ( 6, 110, 600,  1),   #LMcoxa
    'RMH':  ( 7, 110, 600,  1),   #RMtibia
    'RMK':  ( 8, 110, 600,  1),   #RMfemur
    'RMA':  ( 9, 110, 600, -1),   #RMcoxa
    'LBH':  (17, 110, 600, -1),   #LBtibia
    'LBK':  ( 0, 110, 600, -1),   #LBfemur
    'LBA':  ( 1, 110, 600,  1),   #LBcoxa
    'RBH':  (18, 110, 600,  1),   #RBtibia
    'RBK':  ( 3, 110, 600,  1),   #RBfemur
    'RBA':  ( 2, 110, 600, -1),   #RBcoxa
    'N':    (19, 110, 600,  1)    #Neck
  }

driver1 = Adafruit_PCA9685.PCA9685(address=0x40)
driver2 = Adafruit_PCA9685.PCA9685(address=0x41)

driver1.set_pwm_freq(60)
driver2.set_pwm_freq(60)

#servo_min = 110  # Min pulse length out of 4096
#servo_max = 600  # Max pulse length out of 4096

def drive(ch, val):
    driver = driver1 if ch < 16 else driver2
    ch = ch if ch < 16 else ch - 16    
    driver.set_pwm(ch, 0, val)


def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))


def remap(old_val, old_min, old_max, new_min, new_max):
    new_diff = (new_max - new_min)*(old_val - old_min) / float((old_max - old_min))
    return int(round(new_diff)) + new_min 


class HexapodCore:

    def __init__(self):

        self.neck = Joint("neck", 'N')

        self.left_front   = Leg(  'left front', 'LFH', 'LFK', 'LFA')
        self.right_front  = Leg( 'right front', 'RFH', 'RFK', 'RFA')

        self.left_middle  = Leg( 'left middle', 'LMH', 'LMK', 'LMA')
        self.right_middle = Leg('right middle', 'RMH', 'RMK', 'RMA')
        
        self.left_back    = Leg(   'left back', 'LBH', 'LBK', 'LBA')
        self.right_back   = Leg(  'right back', 'RBH', 'RBK', 'RBA')

        self.legs = [self.left_front, self.right_front,
                     self.left_middle, self.right_middle,
                     self.left_back, self.right_back]

        self.right_legs = [self.right_front, self.right_middle, self.right_back]
        self.left_legs = [self.left_front, self.left_middle, self.left_back]

        self.tripod1 = [self.left_front, self.right_middle, self.left_back]
        self.tripod2 = [self.right_front, self.left_middle, self.right_back]
        
        self.hips, self.knees, self.ankles = [], [], []

        for leg in self.legs:
            self.hips.append(leg.hip)
            self.knees.append(leg.knee)
            self.ankles.append(leg.ankle)

    def off(self):

        self.neck.off()
        
        for leg in self.legs:
            leg.off() 


class Leg:

    def __init__(self, name, hip_key, knee_key, ankle_key):

        max_hip, max_knee, knee_leeway = 45, 50, 10
        
        self.hip = Joint("hip", hip_key, max_hip)
        self.knee = Joint("knee", knee_key, max_knee, leeway = knee_leeway)
        self.ankle = Joint("ankle", ankle_key)

        self.name = name
        self.joints = [self.hip, self.knee, self.ankle]

    def pose(self, hip_angle = 0, knee_angle = 0, ankle_angle = 0):

        self.hip.pose(hip_angle)
        self.knee.pose(knee_angle)
        self.ankle.pose(ankle_angle)

    def move(self, knee_angle = None, hip_angle = None, offset = 100):
        """ knee_angle < 0 means thigh is raised, ankle's angle will be set to the specified 
            knee angle minus the offset. offset best between 80 and 110 """

        if knee_angle == None: knee_angle = self.knee.angle
        if hip_angle == None: hip_angle = self.hip.angle

        self.pose(hip_angle, knee_angle, knee_angle - offset)

    def replant(self, raised, floor, offset, t = 0.1):

        self.move(raised)
        time.sleep(t)

        self.move(floor, offset)
        time.sleep(t)

    def off(self):
        for joint in self.joints:
            joint.off()
        
    def __repr__(self):
        return 'leg: ' + self.name


class Joint:

    def __init__(self, joint_type, jkey, maxx = 90, leeway = 0):

        self.joint_type, self.name =  joint_type, jkey
        self.channel, self.min_pulse, self.max_pulse, self.direction = joint_properties[jkey]
        #self.channel, self.min_pulse, self.max_pulse = joint_properties[jkey] #meant for symmetric servo arrangement  
        self.max, self.leeway = maxx, leeway

        self.off()

    def pose(self, angle = 0):

        angle = constrain(angle, -(self.max + self.leeway), self.max + self.leeway)
        pulse = remap((angle * self.direction), (-self.max, self.max), (self.min_pulse, self.max_pulse))

        drive(self.channel, pulse)
        self.angle = angle
        
        #print(repr(self), ':', 'pulse', pulse)

    def off(self):
        drive(self.channel, 0)
        self.angle = None

    def __repr__(self):
        return 'joint: ' + self.joint_type + ' : ' + self.name + ' angle: ' + str(self.angle)
 
 