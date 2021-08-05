from imutils.video import VideoStream
from imutils.video import FPS
from threading import Thread
import time 
import re
import numpy as np
import imutils
import time
import cv2
import socket
import datetime;

circle_x = 0
circle_y = 0
circle_radius = 0
nearest_object_distance = 0

port = 13142
remote_client_is_connected = False
show_windows = False
should_reset_stop_sign_detection = False
time_elapsed_1 = time.time()
time_elapsed_2 = time.time() + 16

def wait_for_connection():
    global remote_client_is_connected
    s = socket.socket()         
    print ("Socket successfully created")
    s.bind(('127.0.0.1', port))         
    print ("socket binded to %s" %(port)) 
    s.listen(5)     
    print ("socket is listening")            
    client_connection, addr = s.accept()     
    print ('Got connection from', addr )
    remote_client_is_connected = True
    return client_connection




  

def log_sensor_metrics(file_writer, isStopSign, isRedlight):
    # ct stores current time
    ct = datetime.datetime.now()
    log_line =  str(ct) + "," +  str(circle_x) + "," + str(circle_y) + "," +  str(circle_radius) + "," + str(nearest_object_distance) + str(isStopSign) +","+ str(isRedlight) + "\n"
    print("log_line: {}".format(log_line))
    file_writer.write(log_line)



def socket_read_function(client_connectin):
    global remote_client_is_connected
    global nearest_object_distance
    if client_connectin:
        while remote_client_is_connected:
            try:
                data = str(client_connectin.recv(1024))
                # print(re.findall(r"\d+\.\d+", data))
                data = re.findall(r"\d+\.\d+", data)
                if len(data)> 0:
                    distance = data[-1]
                    nearest_object_distance = float(distance)
            except ConnectionResetError:
                remote_client_is_connected = False
                break
            except Exception as e:
                print("Error While Parsing Distance")
            time.sleep(0.5)

def report_stop_or_continue(c, isStopSign, isRedLight):
    global remote_client_is_connected
    global should_reset_stop_sign_detection
    global time_elapsed_2
    global time_elapsed_1

    if isStopSign:
        if time_elapsed_2 - time_elapsed_1 > 15 :
            print("Reseting the Timer")
            send_stop_sign_status = True
            time_elapsed_1 = time.time()
        else:
            print("Waiting for the Timer to reset")
            send_stop_sign_status = False
            time_elapsed_2 = time.time()

    try:
        if isStopSign and send_stop_sign_status:
            c.send(str.encode("Stop\0"))
            print("Stop")
            time.sleep(5)
            c.send(str.encode("Continue\0"))
            time_elapsed_1 = time.time()
            time_elapsed_2 = time.time()
        elif isRedLight:
            c.send(str.encode("Stop\0"))    
        else:
            print("Continue")
            c.send(str.encode("Continue\0"))
    except:
        print("Looks Like the Client disconnected")
        remote_client_is_connected = False
    

def is_stop_sign_detected(frame, SS_cascade):
    signdetected = False
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    SSs = SS_cascade.detectMultiScale(gray, 1.3, 5)
    if len(SSs) > 0: 
        print ("Stop Sign Detected ")
        signdetected = True
    return signdetected


def is_red_light_detected(frame):
    signdetected = False
    result = mask = np.zeros(frame.shape, dtype=np.uint8)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 100)            
    global circle_x
    global circle_y
    global circle_radius
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        # loop over the (x, y) coordinates and radius of the circles
        for (x, y, r) in circles:

            # draw the circle in the output image, then draw a rectangle
            # corresponding to the center of the circle
            # print(x, y)
            res = mask = np.zeros(frame.shape, dtype=np.uint8)
            cv2.circle(mask, (x, y), r, (255,255,255), -1)
            # Bitwise-and for ROI
            ROI = cv2.bitwise_and(frame, mask)
            # Crop mask and turn background white
            mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
            x_,y_,w,h = cv2.boundingRect(mask)
            result = ROI[y_:y_+h,x_:x_+w]
            mask = mask[y_:y_+h,x_:x_+w]


            # Color Detection
            hsv = cv2.cvtColor(result, cv2.COLOR_BGR2HSV)
            # lower mask (0-10)
            lower_red = np.array([0,50,50])
            upper_red = np.array([10,255,255])
            mask0 = cv2.inRange(hsv, lower_red, upper_red)

            # upper mask (170-180)
            lower_red = np.array([170,50,50])
            upper_red = np.array([180,255,255])
            mask1 = cv2.inRange(hsv, lower_red, upper_red)

            cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(frame, (x-1, y-1), (x + 1, y + 1), (0, 128, 255), -1)
            cv2.rectangle(frame, (x - (r+1), y - (r+1)), (x + (r+1), y + (r+1)), (0, 128, 255), 2)
            text_output = "(" + str(x) + "," + str(y) + "), Radius: " + str(r)
            cv2.putText(frame,text_output, (x - 1, y - 1), cv2.FONT_HERSHEY_PLAIN, 1, 255)
            if nearest_object_distance > 0:
                text_output = "distance: " + str(nearest_object_distance)
                cv2.putText(frame,text_output, (x - (r+1), y - (r+1)), cv2.FONT_HERSHEY_PLAIN, 1, 255)
            area = w * h
            print(area)
            if cv2.countNonZero(mask1) > 0 :
                signdetected = True
                print('Red is present!')
                circle_x = x
                circle_y = y
                circle_radius = r
            else:
                signdetected = False
                print('Red is not present!')
            if show_windows:
                cv2.imshow('Mask', mask)
                cv2.imshow('Result', result)
    else:
        return None
    return signdetected
               

if __name__ == "__main__":
    # wait for the clients to connect.
    
    
    #reduce the resolution to increase the FPS
    print("[INFO] starting video stream...")
    vs = VideoStream(src=0).start()
    time.sleep(2.0)
    fps = FPS().start()
    # Initialize the module to write video frames to file
    video_writer = None
    writer = None
    (h, w) = (None, None)
    zeros = None
    # Load the Stop Sign identifier Model
    SS_cascade = cv2.CascadeClassifier('stopsign_classifier.xml')
    client_connection = wait_for_connection()
    fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
    file_name = "traffic_light_detectin" + time.strftime("%Y%m%d-%H%M%S") + ".csv"
    file_writer = open(file_name,'w')
    file_writer.write("TIME,X,Y,R,Object-Distance,isStopSign,isRedLight\n")
    
    # Spin off a thread to read the distance values and exit status!
    thread = Thread(target = socket_read_function, args = (client_connection,))
    thread.start()

    while remote_client_is_connected:
        circle_x = circle_y = circle_radius = 0
        stop_sign_detected = False
        red_light_is_detected = False
        frame = vs.read()
        frame = imutils.resize(frame, width=400)
        if frame is None:
            pass
        else:
            if video_writer is None:
                h, w =  frame.shape[:2]
                video_writer = cv2.VideoWriter('/home/pi/Desktop/output.mp4', fourcc , 20.0,(h, w))
                
            stop_sign_detected = is_stop_sign_detected(frame, SS_cascade)
            
            if stop_sign_detected:
                report_stop_or_continue(client_connection, True, False)
                log_sensor_metrics(file_writer, stop_sign_detected, red_light_is_detected)
            else:
                red_light_is_detected = is_red_light_detected(frame)
                # Determine if the color exists on the image
                if red_light_is_detected is None:
                    pass
                if red_light_is_detected:
                    # client connection, isStopSign, isRedLight
                    report_stop_or_continue(client_connection, False, True)
                    log_sensor_metrics(file_writer, stop_sign_detected, red_light_is_detected)
                elif red_light_is_detected != None:
                    # client connection, isStopSign, isRedLight
                    report_stop_or_continue(client_connection, False, True)
                    log_sensor_metrics(file_writer, stop_sign_detected, red_light_is_detected)
                
            if show_windows:
                cv2.imshow('Frame_Original', frame)
            video_writer.write(frame)
            cv2.imwrite("./images/" + time.strftime("%Y%m%d-%H%M%S") + ".jpg", frame) 
        if not (stop_sign_detected or red_light_is_detected):
            report_stop_or_continue(client_connection, False, False)
        key = cv2.waitKey(1) & 0xFF
    print("Closing the video Stream")
    cv2.destroyAllWindows()
    vs.stop()
    video_writer.release()
    file_writer.close()
    thread.join()
    print("thread finished...exiting")