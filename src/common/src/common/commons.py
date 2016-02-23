#!/usr/bin/env python

from common.srv import *
from std_msgs.msg import String, Empty, Float32
import rospy
import xml.etree.ElementTree as ET
import time, os, traceback, base64
from subprocess import call

if 'system_name' not in globals():
    system_name = "SampleSystem"
if 'work_dir' not in globals():
    work_dir = os.path.dirname(os.path.realpath(__file__))+"/../../data/"

if "stop_callback" not in dir(os):
    def stop_callback(message):
        rospy.loginfo("Stop requested")
###
def play_sound(filename):
    print "playing: "+filename
    return call(["aplay", work_dir+'sounds/'+filename])
    
def handle_service(req):
    play_sound(req.request)
    return StringServiceResponse("OK")

def service_server():
    s = rospy.Service(system_name+'/play_sound', StringService, handle_service)
    print "Common services ready."
    
def sound_callback(message):
    play_sound(message.data)
    
def volume_callback(message):
    print "Setting volume to: "+str(message.data)
    os.system("amixer set Speaker "+str(message.data)+"%")
    
def listeners():
    pub = rospy.Publisher('snap_xml_listener', String, queue_size=10)
    def introduce_callback(message):     
        if(len(message.data) < 2):
            try:
                #send blocks
                f = open(work_dir+'blocks.xml', 'r')
                pub.publish(f.read())
                f.close()
                rospy.loginfo("Sending blocks")
                
                #send identity, blocks and data
                f = open(work_dir+'identity.xml', 'r')
                xml = f.read()
                f.close()
                
                root = ET.fromstring(xml)
                for sprite in root.iter('sprite'):
                    sprite.set('name', system_name)
                    f = open(work_dir+'sounds.xml', 'r')
                    xml = sprite.find(".//sounds")
                    xml.remove(xml.find("list"))
                    xml.append(ET.fromstring(f.read()))
                    f.close()
                pub.publish(ET.tostring(root, encoding='UTF-8').split("\n", 1)[1])
                
            except:
                print "Error opening blocks file:"
                traceback.print_exc()
        else:
            #process and save received xml data
            try:
                root = ET.fromstring(message.data)
                #data is not for this system
                if not root.get("name") == system_name:
                    return
                #save each sound to binary file
                for sound in root.iter('sound'):
                    f = open(work_dir+'sounds/'+sound.get('name'), 'w')
                    decoded = base64.b64decode(sound.get('sound').split(",",1)[1])
                    f.write(decoded)
                    f.close()
                    decoded = ""
                #save sound files xml data
                xml = root.find("./sounds/list")
                f = open(work_dir+'sounds.xml', 'w')
                f.write(ET.tostring(xml))
                f.close()
                print "Received xml processed"
            except:
                print "Error processing incoming data:"
                traceback.print_exc()
    rospy.Subscriber("/stop_all", Empty, stop_callback)
    rospy.Subscriber("/introduce_all", String, introduce_callback)
    rospy.Subscriber(system_name+"/play_sound", String, sound_callback)
    rospy.Subscriber(system_name+"/set_volume", Float32, volume_callback)
    print "Common topics ready."
    
def main():
    try:
        rospy.init_node(system_name+'_node')
        service_server()
        listeners()
    except rospy.ROSInterruptException: pass 

if __name__ == "__main__":
    main()
    rospy.spin()
    

    
