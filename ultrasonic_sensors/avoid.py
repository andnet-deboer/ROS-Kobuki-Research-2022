#!/usr/bin/env python
from __future__ import print_function
from __future__ import division
from time import sleep
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import Led
from std_msgs.msg import Float32
from math import radians
import time
import rospy
from kobuki_msgs.msg import BumperEvent
from kobuki_msgs.msg import WheelDropEvent
from kobuki_msgs.msg import Sound 


#move forward command
moveforward_cmd = Twist()
moveforward_cmd.linear.x = 0.3
#turn left command
moveback_cmd = Twist()
moveback_cmd.linear.x = -0.2
#stop command
stop_cmd = Twist()
stop_cmd.linear.x = 0.0

#Calculate Moving Average
def moving_average(msg):
    history = []
    history.append(msg)
    
    if len(history) > 2000:
        history = history[-2000:]

    average = sum(history) / float(len(history))
    return average
   
#Make Sound
def sound(int):
    arlosound = rospy.Publisher('/arlo/mobile_base/commands/sound', Sound, queue_size=10)
    tobiassound = rospy.Publisher('/tobias/mobile_base/commands/sound', Sound, queue_size=10)
    arlosound.publish(int)
    tobiassound.publish(int)

#Turn Left (0 turns arlo) & (1 turns arlo)
def move(self, id,linear, angular):
    """
    Function to turn robot

    Parameters:(id, linear, angular)

    -id (--integer): which robot to move | 0 for arlo | 1 for tobias |

    -linear (--float): linear vector of the robot controls the speed and direction 
                     postive is forward and negative is backwards

    -angular (--float): angular vector of the robot controls the speed and direction 
                     postive is clockwise and negative is counter-clockwise
                    
    """
    arlocmd_vel = rospy.Publisher('/arlo/mobile_base/commands/velocity', Twist, queue_size=10)
    tobiascmd_vel = rospy.Publisher('/tobias/mobile_base/commands/velocity', Twist, tcp_nodelay= True, queue_size=0)

    #turn left command
    move_cmd = Twist()
    move_cmd.linear.x = radians(linear)
    move_cmd.angular.z = radians(angular)

    while not rospy.is_shutdown():
        if id == 0:
            arlocmd_vel.publish(move_cmd)
        if id == 1 :
            tobiascmd_vel.publish(move_cmd)
            self.tobiasleftobjectdetected = False
        break
#Turn Left (0 turns arlo) & (1 turns arlo)
def move_for_time(id ,linear, angular, time):
    """
    Function to turn robot

    Parameters:(id, linear, angular)

    -id (--integer): which robot to move | 0 for arlo | 1 for tobias |

    -linear (--float): linear vector of the robot controls the speed and direction 
                     postive is forward and negative is backwards

    -angular (--float): angular vector of the robot controls the speed and direction 
                     postive is clockwise and negative is counter-clockwise

    -time (--float): controls the amount of time for the motion action in seconds            
    """
    arlocmd_vel = rospy.Publisher('/arlo/mobile_base/commands/velocity', Twist, queue_size=10)
    tobiascmd_vel = rospy.Publisher('/tobias/mobile_base/commands/velocity', Twist, queue_size=1)

    #turn left command
    move_cmd = Twist()
    move_cmd.linear.x = radians(linear)
    move_cmd.angular.z = radians(angular)

    while not rospy.is_shutdown() and  countdown(0,0,time) > 0:
        if id == 0:
            arlocmd_vel.publish(move_cmd)
        if id == 1 :
            tobiascmd_vel.publish(move_cmd)
        
        break
       
    
#Move Forward 
def moveforward(int):
    arlocmd_vel = rospy.Publisher("/arlo/mobile_base/commands/velocity", Twist, queue_size=10)
    tobiascmd_vel = rospy.Publisher("/tobias/mobile_base/commands/velocity", Twist, queue_size=1)
    
    while not rospy.is_shutdown():
        if int == 0:
            arlocmd_vel.publish(moveforward_cmd)
        if int == 1 :
            tobiascmd_vel.publish(moveforward_cmd)
        break

#Move Back
def moveback( int):
    arlocmd_vel = rospy.Publisher("/arlo/mobile_base/commands/velocity", Twist, queue_size=1)
    tobiascmd_vel = rospy.Publisher("/tobias/mobile_base/commands/velocity", Twist,tcp_nodelay= True,queue_size=0)
    
    while not rospy.is_shutdown():
        if int == 0:
            arlocmd_vel.publish(moveback_cmd)
            sleep(1)
        if int == 1 :
            tobiascmd_vel.publish(moveback_cmd)
            sleep(1)
        break

# Count Down Timer
def countdown(h, m, s):
 
    # Calculate the total number of seconds
    total_seconds = h * 3600 + m * 60 + s
    # While loop that checks if total_seconds reaches zero
    # If not zero, decrement total time by one second
    while total_seconds > 0:
        time.sleep(1)
        total_seconds -= 1
    return total_seconds

def turnonled(int):
        arlo_led1 = rospy.Publisher('/arlo/mobile_base/commands/led1', Led, queue_size=10)
        tobias_led1 = rospy.Publisher('/tobias//mobile_base/commands/led1', Led, queue_size=10)

        if(int == 0):
         arlo_led1.publish(Led.BLACK)
         tobias_led1.publish(Led.BLACK)
        elif(int == 1):
         arlo_led1.publish(Led.RED)
         tobias_led1.publish(Led.RED)
        elif(int == 2):
         arlo_led1.publish(Led.GREEN)
         tobias_led1.publish(Led.GREEN)
        elif(int == 3):
         arlo_led1.publish(Led.ORANGE)
         tobias_led1.publish(Led.ORANGE)

class avoid:
    #INTIALIZES SELF VARIABLES
    def __init__(self):

         #------------------------------------- Estevan
        self.estevanleftsensor = 0
        self.estevanrightsensor = 0

        self.estevanbumper = False
        self.estevanbumperpressed = False

        self.estevanleftobjectdetected = False
        self.estevanrightobjectdetected = False
        self.estevanwheeldrop = False

        #------------------------------------- Arlo
        self.arloleftsensor = 0
        self.arlorightsensor = 0

        self.arlobumper = False
        self.arlobumperpressed = False

        self.arloleftobjectdetected = False
        self.arlorightobjectdetected = False
        self.arlowheeldrop = False

        #------------------------------------ Tobias
        self.tobiasleftsensor = 0
        self.tobiasrightsensor = 0

        self.tobiasbumper = False
        self.tobiasbumperpressed = False

        self.tobiasleftobjectdetected = False
        self.tobiasrightobjectdetected = False
        self.tobiaswheeldrop = False
        
        #------------------------------------ System Variables
        self.distance = 30
        self.velocity = 0.2
        self.turnangle = 75

#======================== ESTEVAN ===========================#

    #RETRIEVES ESTEVAN LEFT ULTRASONIC SENSOR DATA
    def estevanleftsensor_callback(self, msg):
        # "Store" message received
        self.estevanleftsensor = msg.data
        # Compute stuff.
        self.estevan_compute()
    #RETRIEVES ESTEVAN RIGHT ULTRASONIC SENSOR DATA
    def estevanrightsensor_callback(self, msg):
        # "Store" the message received.
        self.estevanrightsensor = msg.data
        # Compute stuff.
        self.estevan_compute()
    #RETRIEVES ESTEVAN BUMPER DATA
    def estevanbumper_callback(self, msg):
       self.estevanbumper = msg.state
       self.estevan_compute()
    #RETRIEVES ESTEVAN WHEEL DROP DATA
    def estevanwheeldrop_callback(self, msg):
       self.estevanwheeldrop = msg.state
       self.estevan_compute()
    #PASSES ON ESTEVAN DATA TO COMPUTE FUNCTION AS PARAMETERS
    def estevan_compute(self):
        self.estevanleftobjectdetected,self.estevanrightobjectdetected, self.estevanbumperpressed  =  self.compute_stuff(
        0,
        self.estevanleftsensor,
        self.estevanrightsensor,
        self.estevanbumper, 
        self.estevanbumperpressed,
        self.estevanleftobjectdetected,
        self.estevanrightobjectdetected,
        self.estevanwheeldrop)

#======================== ARLO ===========================#

    #RETRIEVES ARLO LEFT ULTRASONIC SENSOR DATA
    def arloleftsensor_callback(self, msg):
        # "Store" message received
        self.arloleftsensor = msg.data
        # Compute stuff.
        
    #RETRIEVES ARLO RIGHT ULTRASONIC SENSOR DATA
    def arlorightsensor_callback(self, msg):
        # "Store" the message received.
        self.arlorightsensor = msg.data
        # Compute stuff.
        self.arlo_compute()

    #RETRIEVES ARLO BUMPER DATA
    def arlobumper_callback(self, msg):
       self.arlobumper = msg.state
       self.arlo_compute()

    #RETRIEVES Arlo WHEEL DROP DATA
    def arlowheeldrop_callback(self, msg):
       self.arlowheeldrop = msg.state
       self.arlo_compute()
       
    #PASSES ON ARLO DATA TO COMPUTE FUNCTION AS PARAMETERS
    def arlo_compute(self):
        self.arloleftobjectdetected,self.arlorightobjectdetected, self.arlobumperpressed  =  self.compute_stuff(
        0,
        self.arloleftsensor,
        self.arlorightsensor,
        self.arlobumper, 
        self.arlobumperpressed,
        self.arloleftobjectdetected,
        self.arlorightobjectdetected,
        self.arlowheeldrop)

#======================= Tobias ===========================# 
    #RETRIEVES TOBIAS LEFT ULTRASONIC SENSOR DATA
    def tobiasleftsensor_callback(self, msg):
        # "Store" message received
        self.tobiasleftsensor = msg.data
        # Compute stuff.
        self.tobias_compute()

    #RETRIEVES TOBIAS RIGHT ULTRASONIC SENSOR DATA
    def tobiasrightsensor_callback(self, msg):
        # "Store" the message received.
        self.tobiasrightsensor = msg.data
        # Compute stuff.
        self.tobias_compute()

    #RETRIEVS TOBIAS BUMPER DATA
    def tobiasbumper_callback(self, msg):
       self.tobiasbumper = msg.state
       self.tobias_compute

    #RETRIEVES TOBIAS WHEEL DROP DATA
    def tobiaswheeldrop_callback(self, msg):
       self.tobiaswheeldrop = msg.state
       self.tobias_compute()

    #PASSES ON TOBIAS DATA TO COMPUTE FUNCTION AS PARAMETERS
    def tobias_compute(self):
        self.tobiasleftobjectdetected,self.tobiasrightobjectdetected, self.tobiasbumperpressed  = self.compute_stuff(
        1,
        self.tobiasleftsensor,
        self.tobiasrightsensor,
        self.tobiasbumper, 
        self.tobiasbumperpressed,
        self.tobiasleftobjectdetected,
        self.tobiasrightobjectdetected,
        self.tobiaswheeldrop
        )

        #print("LEFT:-------", self.tobiasleftobjectdetected," --- BUMPER---|", self.tobiasbumperpressed, "|-----RIGHT---|",self.tobiasrightobjectdetected,"|---")

#=================== COMPUTE FUNCTION ======================#
    def compute_stuff(self , id ,leftsensor, rightsensor, bumper, bumperpressed, leftobjectdetected, rightobjectdetected, wheeldrop):
        #----------------------------------------------------------
        #print(bumperpressed)
        # LOOP RUNS WHILE SENSORS HAVE DATA
        if leftsensor is not None and rightsensor is not None:
            #-------------------------------------------
            # BUMPER
            if (bumper == BumperEvent.PRESSED):
                 bumperpressed = True
                 return leftobjectdetected , rightobjectdetected , bumperpressed

            elif (bumperpressed == True):
                while bumperpressed == True:
                    moveback(id)
                    move(self,id,0,self.turnangle)
                    sleep(1)
                    break
                bumperpressed = False 
                return leftobjectdetected , rightobjectdetected , bumperpressed
            #------------------------------------------- 
            # RIGHT ULTRASONIC SENSOR
            elif( rightsensor < self.distance):
                  rightobjectdetected = True
                  
            elif rightobjectdetected == True:
                  moveback(id)
                  while rightobjectdetected== True:
                    rightobjectdetected = False
                    move(self,id,0,self.turnangle) 
                    sleep(1)
                    break
                  rightobjectdetected = False
                  return False , False , False
            #-------------------------------------------- 
            # LEFT ULTRASONIC SENSOR
            elif( leftsensor < self.distance):
                  leftobjectdetected = True

            elif leftobjectdetected == True:
                  moveback(id)
                  while leftobjectdetected == True:
                    leftobjectdetected = False
                    print("turning left")
                    move(self,id,0,-self.turnangle) 
                    sleep(1)
                    break
                  leftobjectdetected = False
                  return False, False , False
            #---------------------------------------------
            # FORWARD  
           # if (rightsensor > self.distance and leftsensor > self.distance):
               # moveforward(id)
                
        # return leftobjectdetected , rightobjectdetected , bumperpressed
        #print("LEFT:-------", leftobjectdetected," --- BUMPER---|", bumperpressed, "|-----RIGHT---|",rightobjectdetected,"|---")
        moveforward(id)
        print("move forward")
        return leftobjectdetected, rightobjectdetected, bumperpressed

if __name__ == '__main__':
    rospy.init_node('avoid')

    avoid = avoid()

    #ARLO SUBSCRIBERS
    rospy.Subscriber('/arlo/ultrasonicsensor_1', Float32 , avoid.arloleftsensor_callback)
    rospy.Subscriber('/arlo/ultrasonicsensor_2', Float32, avoid.arlorightsensor_callback)
    rospy.Subscriber("/arlo/mobile_base/events/bumper", BumperEvent, avoid.arlobumper_callback)
    #rospy.Subscriber("/arlo/mobile_base/events/wheel_drop", BumperEvent, avoid.arlowheeldrop_callback)
    
    #TOBIAS SUBSCRIBERS
    rospy.Subscriber('/tobias/ultrasonicsensor_1', Float32 , avoid.tobiasleftsensor_callback)
    rospy.Subscriber('/tobias/ultrasonicsensor_2', Float32, avoid.tobiasrightsensor_callback)
    rospy.Subscriber("/tobias/mobile_base/events/bumper", BumperEvent, avoid.tobiasbumper_callback)
    #rospy.Subscriber("/tobias/mobile_base/events/wheel_drop", BumperEvent, avoid.tobiaswheeldrop_callback)

    #ESTEVAN SUBSCRIBERS
    rospy.Subscriber('/estevan/ultrasonicsensor_1', Float32 , avoid.estevanleftsensor_callback)
    rospy.Subscriber('/estevan/ultrasonicsensor_2', Float32, avoid.estevanrightsensor_callback)
    rospy.Subscriber("/estevan/mobile_base/events/bumper", BumperEvent, avoid.estevanbumper_callback)
    #rospy.Subscriber("/estevan/mobile_base/events/wheel_drop", BumperEvent, avoid.estevanwheeldrop_callback)
    

    rospy.spin()
