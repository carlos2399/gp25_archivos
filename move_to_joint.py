#!/usr/bin/env python
# license removed for brevity
import rospy
import time
import json
from std_msgs.msg import String
import requests
import os

global trama, lista_tramas, lista_tramas_aux, time_out
global ip_address, get_frequency, post_frequency, jsonData, status


def get_ip_address():
    global ip_address
    # os.getcwd() --> /home/ros/catkin_ws/ip_address.txt     
    f = open(os.getcwd()+ "/ip_address.txt")
    for linea in f:
        ip_address = linea
        f.close()
        return ip_address
        

def listener():
    global rospy
    # ###################################################
    #rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('chatter', String, callback)
    # ####################################################
    
def callback(data):
    global jsonData
    try:
        if data.data != None:
            jsonData = json.loads(data.data)
        else:
            jsonData = ''
    except Exception as e:
        print(e)
        

def leer_tramas():
    listener()


def enivar_tramas():
    global lista_tramas, time_out
    
    lista_tramas_aux = []
    
    for trama in lista_tramas:
        # trama[0] = json  trama[1] = json_time
        confirmacion = talker(trama[0])
        if not confirmacion:
            if time.time() - trama[1] < time_out:
                lista_tramas_aux.append(trama)
                
    lista_tramas = []
    lista_tramas = lista_tramas_aux
    return confirmacion

def talker(trama):
    global post_frequency
    global rospy
    confirmacion = False
    # ########################################################
    pub = rospy.Publisher('chatter_p', String, queue_size=10)
    #rospy.init_node('talker', anonymous=True)
    # ########################################################
    rate = rospy.Rate(post_frequency) # 10hz
    try:
        if not rospy.is_shutdown():
            pub.publish(trama)
            rate.sleep()
            confirmacion = True
    except Exception as e:
        confirmacion = False
        print(e)
    return confirmacion


if __name__ == '__main__':
    global trama, lista_tramas, liscleta_tramas_aux, time_out, ip_address, jsonData, status
    global rospy
    
    jsonData = ''
    lista_tramas = []
    lista_tramas_aux = []
    time_out = 10
    jsonData_aux = ''
    
    get_frequency = 2  # GET refresh time
    post_frequency = 1  # POST refresh time    
    
    ip_address = get_ip_address()
    rospy.init_node('listenr_talker', anonymous=True)
    while True:
        try:
            leer_tramas()
            if jsonData != '' and jsonData_aux != jsonData:
                print 'jsonData', jsonData
                time.sleep(1)
                jsonData_aux = jsonData
                lista_tramas.append([jsonData, time.time()])
            
            if len(lista_tramas) > 0:
                print 'lista_tramas ',lista_tramas 
                time.sleep(1)
                enivar_tramas()
                
            time.sleep(1)
        except rospy.ROSInterruptException:
            pass
