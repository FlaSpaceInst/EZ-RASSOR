#!/usr/bin/env python

#Camilo Lozano

import os
import rospy
import threading
import time
import json

from flask import Flask, request
from std_msgs.msg import Int16


def ros_callback(msg):
    print(msg)

#Thread makes it possible to run Flask within a ROS node
threading.Thread(target=lambda: rospy.init_node('example_node', disable_signals=True)).start()
pub = rospy.Publisher('/ezmain_topic', Int16, queue_size=10)

app = Flask(__name__)

#Global var used to kill hold func
kill = False

#Threaded func to continue publishing until killed
def hold(x):
    global kill
    global pub
    try:
        while not kill:
            pub.publish(x)
            time.sleep(0.1)
    except Exception as e:
        print(e)

#Manual override function to send binary single signal
@app.route('/manual', methods=['POST'])
def manual():
    msg = Int16()
    data = int(request.data,2)
    msg.data=data
    pub.publish(msg)
    return '{}'.format(data)

#Publishes corresponding discrete command
@app.route('/cmd',methods=['POST'])
def cmd():
    msg=Int16()
    data = int(request.data)
    msg.data=num_to_bin(data,0)
    pub.publish(msg)

    res = {'cmd':data,'bin':bin(msg.data)}
    return (json.dumps(res),200)


#Manual start to repeating message
@app.route('/start',methods=['POST'])
def start():
    global kill

    cmd = int(request.data)

    msg=Int16()
    msg.data=num_to_bin(cmd,0)

    kill=False
    tr = threading.Thread(target=lambda:hold(msg)).start()

    res = {'cmd':cmd,'bin':bin(msg.data)}
    return (json.dumps(res),200)

#Kills any active thread that is holding
@app.route('/kill',methods=['GET'])
def stop():
    global kill
    kill=True
    return ('Thread Killed',200)

#Translates input int from app (cmd) to respective binary
#Res is binary to be output, is or'ed to be able to be used 
# in the case of multiple commands
def num_to_bin(cmd,res):

    #Stop
    if cmd==0:
        global kill
        kill = True
    #Forward
    elif cmd==1:
        res |= 0b100000000000
    #Backward
    elif cmd==2:
        res |= 0b010000000000
    #Turn Left
    elif cmd==3:
        res |= 0b001000000000
    #Turn Right
    elif cmd==4:
        res |= 0b000100000000
    #Arm 1 - Move Up
    elif cmd==10:
        res |= 0b000010000000
    #Arm 2 - Move Up
    elif cmd==11:
        res |= 0b000000100000
    #Both Arms - Up
    elif cmd==12:
        res |= 0b000010100000
    #Arm 1 - Move Down
    elif cmd==13:
        res |= 0b000001000000
    #Arm 2 - Move Down
    elif cmd==14:
        res |= 0b000000010000
    #Both Arms - Down
    elif cmd==15:
        res |= 0b000001010000
    #Drum 1 - Excavate
    elif cmd==16:
        res |= 0b000000001000
    #Drum 2 - Excavate
    elif cmd==17:
        res |= 0b000000000010
    #Both Drums - Excavate
    elif cmd==18:
        res |= 0b000000001010
    #Drum 1 - Dump
    elif cmd==19:
        res |= 0b000000000100
    #Drum 2 - Dump
    elif cmd==20:
        res |= 0b000000000001
    #Both Drums - Dump
    elif cmd==21:
        res |= 0b000000000101

    ## Function have not been implemented yet
    #AUTO_DIG
    elif cmd==25:
        res |= 0b000000000000
    #AUTO_DUMP
    elif cmd==26:
        res |= 0b000000000000
    #SELF_RIGHT
    elif cmd==27:
        res |= 0b000000000000
    #Z_CONFIG
    elif cmd==28:
        res |= 0b000000000000
    #AUTO_DRIVE
    elif cmd==19:
        res |= 0b000000000000
    #Error
    else:
        res |= 0b000000000000
    return res

#Initializes Flask on local network, on default port of 5000
app.run('0.0.0.0')
