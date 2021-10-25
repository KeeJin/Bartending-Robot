import cv2
import requests
import time
import numpy as np
import pytesseract
import Queue as queue
#import queue
import threading
from genpy import message
import rospy
from bartending_server.srv import BartenderCocktailRequest
from requests.api import get
 
pytesseract.pytesseract.tesseract_cmd = '/usr/bin/tesseract'

TOKEN = "BBFF-2BoQtMzRrAltynSTpJK7JtbsB3oQ46" # Assign your Ubidots Token
DEVICE = "raspi" # Assign the device label to obtain the variable
DRINKNO = "drinkno" # Assign the variable label to obtain the variable value
TIMESTAMP = "timestamp" # Assign the variable label to obtain the variable value
DELAY = 1  # Delay in seconds

def get_val(device, variable):
    try:
        url = "http://industrial.api.ubidots.com/"
        url = url + \
            "api/v1.6/devices/{0}/{1}/".format(device, variable)
        headers = {"X-Auth-Token": TOKEN, "Content-Type": "application/json"}
        req = requests.get(url=url, headers=headers)
        return req.json()['last_value']['value']
    except:
        pass


def customer_request_client(order):
    #todo
    #message is a python list - if you want you can only publish the first item in the list
    
    #getting the alcohol and mixer values from order received
    alcohol = order%10
    mixer = order//10

    print("alcohol is {}\nmixer is {}\nstarting service now:".format(alcohol, mixer))

    rospy.wait_for_service('bartender/drink_request')
    try:
        drink_request = rospy.ServiceProxy('bartender/drink_request', BartenderCocktailRequest)
        response = drink_request(alcohol,mixer)
    except rospy.ServiceException as e:
        print("Customer Request Service call failed: %s"%e)
        return 0

    #print error message if drink isn't served
    if not response.success:
        print(response.message)
        return 0

def receive_order_thread():
    prev_timestamp = -1
    while True:
        timestamp = get_val(DEVICE,TIMESTAMP)
        #print(timestamp)
        if timestamp!=prev_timestamp:
            if prev_timestamp !=-1:
                prev_timestamp = timestamp
                result = get_val(DEVICE,DRINKNO) #relevant var
                #print("Value has been updated to: " + str(result))
                customer_requests.put(result)
                            #ros publisher function
                print(list(customer_requests.queue))
            else:
                prev_timestamp = timestamp
        time.sleep(DELAY)

def getImg(simulate):

    if simulate == True:                   #for testing data flow
        return -1

    vid = cv2.VideoCapture(2)              #change number in here as required

    if not (vid.isOpened()):
        print("Could not open video device")

    vid.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)

    time.sleep(0.1)
    ret, capture = vid.read()
    time.sleep(0.1)
    vid.release()

    #cv2.imshow('frame', capture)
    cv2.imwrite('capFinalUbi.jpg', capture)
    gray = cv2.cvtColor(capture, cv2.COLOR_BGR2GRAY)
    rect = cv2.rectangle(gray, (700, 170), (830, 250), (0, 255, 0), 2)
    rect = cv2.rectangle(gray, (470, 190), (600, 270), (0, 255, 0), 2)
    rect = cv2.rectangle(gray, (880, 170), (1020, 250), (0, 255, 0), 2)
    left = gray[190:270, 470:600]
    centre = gray[170:250, 700:830]
    right = gray[170:250,880:1020]
    #slot 1(left): 470,190 to 600,270
    #slot 2(centre): 700,170 to 830,250
    #slot 3(right): 
    #cv2.imshow("rect",rect)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    return([left, centre,right])

def drinksToMix(bottleOrder, drinknos):
    #bottleOrder is a list containing bottle codes in order from L to R
    #drinknos is an integer with 2 digits, each digit representing a drink

    #for testing use bottleOrder = -1, will just return drinknos
    drinkList = [drinknos//10, drinknos%10]

    if bottleOrder == -1:
        return ((drinkList[0]*10)+drinkList[1])

    else:
        retList = []
        for i in drinkList:
            retList.append(bottleOrder.index(i)+1)
        return ((retList[0]*10)+retList[1])

def processImgs(croppedImgList):
    if croppedImgList == -1 :           #will be true if simulation
        return -1
    else:
    #process colors
    #for ocr use processImgsOCR
        bottleOrder = []        #returns list containing bottle codes in order from L to R
        leftHSV = cv2.cvtColor(croppedImgList[0],cv2.COLOR_BGR2HSV)    
        centreHSV = cv2.cvtColor(croppedImgList[1],cv2.COLOR_BGR2HSV)
        rightHSV = cv2.cvtColor(croppedImgList[2],cv2.COLOR_BGR2HSV)
        hsvList = [leftHSV,centreHSV,rightHSV]
        lower_green = np.array([56,255,251])        
        upper_green = np.array([60,255,255])
        lower_red = np.array([0,255,251])        
        upper_red = np.array([5,255,255])
        lower_yellow = np.array([22,255,251])        
        upper_yellow = np.array([27,255,255])
        for i in range(3):
            green_mask = cv2.inRange(hsvList[i],lower_green ,upper_green)
            red_mask = cv2.inRange(hsvList[i], lower_red, upper_red)
            yellow_mask = cv2.inRange(hsvList[i], lower_yellow, upper_yellow)
            gpixels = cv2.countNonZero(green_mask)
            rpixels = cv2.countNonZero(red_mask)
            ypixels = cv2.countNonZero(yellow_mask)
            if gpixels>0:
                #print('greenDetectedIn '+str(i))
                bottleOrder[i]=1   
            if rpixels>0:
                #print('redDetectedIn'+ str(i))
                bottleOrder[i]=2
            if ypixels>0:
                #print('yellowDetectedIn'+str(i))
                bottleOrder[i]=3
    return(bottleOrder)     #for example if order is jaeger, other, redbull then [1,3,2]

def ocr_process(drinkNos):
    croppedImgList = getImg(False)                         #change to false when running in production
    bottleOrder = processImgs(croppedImgList)
    messageToSend = drinksToMix(bottleOrder, drinkNos)
    print(messageToSend)
    return messageToSend


def send_order_thread():
    while True:
        drinkNos = customer_requests.get()
        next_message = ocr_process(drinkNos)
        customer_request_client(next_message)

if __name__ == "__main__": 
    customer_requests = queue.Queue(maxsize=50)
    prev_timestamp = 0

    rospy.init_node('customer_request_client')

    receive_order_thread = threading.Thread(target=receive_order_thread)
    receive_order_thread.start()

    send_order_thread = threading.Thread(target=send_order_thread)
    send_order_thread.start()



    
