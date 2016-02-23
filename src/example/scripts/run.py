#!/usr/bin/env python

from common.srv import *
from std_msgs.msg import String, Empty, Float32
import rospy
import xml.etree.ElementTree as ET
import time, os, traceback, base64

import common.commons as common
# import additional modules for API to control hardware (PWM, GPIO, ADC etc.)
# module can be stored in "common" ros package
#from galileo import *

###Setup
# bot name visible in Snap
common.system_name = system_name = "ExampleBot"
# leave this default as it stands
common.work_dir = work_dir = os.path.dirname(os.path.realpath(__file__))+"/../data/"

# robot emergency stop procedure
def stop_callback(message):
    rospy.loginfo("Stop for bot requested!")

common.stop_callback = stop_callback
common.main()

###SERVICES
def service_move(req):
    val = float(req.request)
    if val > 0:
        print "Moving "+ str(val) + " steps forward"
    else:
        print "Moving "+ str(abs(val)) + " steps backward"
        
    time.sleep(2)
    return StringServiceResponse(str(val))

def service_turn(req):
    val = float(req.request)
    print "Turning "+ str(val) + " degrees"
        
    time.sleep(val/100)
    return StringServiceResponse("OK")

def service_sensor(req):
    try:
        sensors = (10, 20, 30, 40, 50)
        sensor = int(req.request)
        val = sensors[sensor]
        print "Sensor "+ str(sensor) + " value is: " + str(val)
        return StringServiceResponse(str(val))
    except Exception: return StringServiceResponse("N\A") 

def service_say(req):
    print "Saying: " + req.request
    return StringServiceResponse("OK")

def service_server():
    s1 = rospy.Service(system_name+'/move', StringService, service_move)
    s2 = rospy.Service(system_name+'/turn', StringService, service_turn)
    s2 = rospy.Service(system_name+'/sensor', StringService, service_sensor)
    s2 = rospy.Service(system_name+'/say', StringService, service_say)
    print "Services ready."
    
###TOPICS
def grab_callback(message):
    print "Grab"

def drop_callback(message):
    print "Drop"

def listeners():
    rospy.Subscriber(system_name+"/grab", Empty, grab_callback)
    rospy.Subscriber(system_name+"/drop", Empty, drop_callback)
    print "Topics ready."
    #print work_dir
    
def setup_hardware():
    # hw init procedure here
    pass
        
if __name__ == "__main__":
    try:
        setup_hardware()
        service_server()
        listeners()
        rospy.spin()
    except rospy.ROSInterruptException: pass 
    

    
