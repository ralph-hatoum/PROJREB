#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
import cv2
# from ar_finder import *
from naoqi import ALProxy
from cmath import exp
import sys
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf
from tf.transformations import euler_from_quaternion
from math import pi
import threading

## Variables quelconques utilisées dans le code ##

flag = -1
counter = 0
lamold = 0
lamnew = 0
frames = 0
lock = threading.Lock()

## FONCTIONS DE TRAITEMENT D'IMAGE ##

def putCube(frame):
    # Find contours
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    ret,thresh = cv2.threshold(gray, 240, 255, cv2.THRESH_BINARY)
    _,contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    minPeri = 90
    maxPeri = 900
    inside = 0
    j = 0
    k = 0
    # Find parent contours and then detect tag
    mids =  []
    # print(type(hierarchy))
    if type(hierarchy) != type(None):
        for contourInfo in hierarchy[0]:
            if(contourInfo[3] == -1 and findChildren(k,hierarchy,0) >= 2):
                max_index = k
                for i in hierarchy[0]:
                    if(i[3] == max_index):
                        perimeter = cv2.arcLength(contours[j],True)
                        if(perimeter > minPeri and perimeter < maxPeri):
                            # print("perimeter ", perimeter)
                            inside = 1
                            cnt = contours[j]
                            frame, status, mid = putCubeOnContour(cnt, frame)
                            mids.append(mid)
                            j = 0
                            break
                    j = j+1
            j = 0
            k = k + 1
    if(inside == 1):
        return(frame,inside, mids)
    else:
        return(frame, 0, [])

def putCubeOnContour(cnt, frame):
    global flag 
    global counter
    # Read Marker Image
    refMarker =  cv2.imread('ref_marker.png')
    # Store ref mareker's dimensions
    ht,wt, channel = refMarker.shape
    # Store corners in image dimensions
    x,y,w,h = cv2.boundingRect(cnt)
    # Find important contour points only
    approx = cv2.approxPolyDP(cnt,0.05*cv2.arcLength(cnt,True),True)
    pts1 = np.zeros([4,2],dtype = 'float32')
    # check if the contour is a rectangle
    if(len(approx) == 4):   
        n = 0
        # put the image points in an array
        for j in approx:
            if(n<4):
                pts1[n][0] = j[0][0]
                pts1[n][1] = j[0][1]
            n += 1
        # points of upright tag
        pts2 = np.float32([[0,0],[w-1,0],[w-1,h-1],[0,h-1]])
        # world coordinates
        pts3 = np.float32([[0,0],[wt-1,0],[wt-1,ht-1],[0,ht-1]])
        # print("pts3 ",pts3)
        # find the H matrix
        H = findHMatrix(pts2,pts1) # Transforming second to first
        # Make the tag upright
        uprightTag = cv2.warpPerspective(frame,H,(w-1,h-1))
        # Convert it to grayscale
        grayUprightTag  = cv2.cvtColor(uprightTag , cv2.COLOR_BGR2GRAY)
        # Convert to binary
        ret,binaryUprightTag  = cv2.threshold(grayUprightTag ,240,255,cv2.THRESH_BINARY)
        # Smoothen the edges
        binaryUprightTag = cv2.blur(binaryUprightTag,(5,5))
        binaryUprightTag = cv2.bilateralFilter(binaryUprightTag,5,100,100)
        # Align the tag
        pts5, index = alignTag(uprightTag, binaryUprightTag, pts2, pts3)
        pts4 = np.roll(pts2, index, axis = 0)
        HForTag =  findHMatrix(pts4,pts2)
        rotatedTag = cv2.warpPerspective(uprightTag,HForTag,(w-1,h-1))
        # Calculate tag ID
        tagID = giveTag(rotatedTag)
        flag = tagID
        font = cv2.FONT_HERSHEY_SIMPLEX
        # cv2.putText(frame,"tagID = " + str(flag),(pts1[0][0],pts1[0][1]), font, 1.2,(0,0,255),3,cv2.LINE_AA)
        # find the H matrix
        H = findHMatrix(pts1,pts3) # Transforming second to first
        # Find Projection matrix
        upperLayerCubePoints = projectionMatrix(H,wt,ht)
        # print("upper layer cube points ",upperLayerCubePoints)  
        mid = computeCubeCenter(pts1)
        # draw cube 
        # return(draw(frame,pts1,upperLayerCubePoints), 1)
        return(drawMid(frame,pts1,mid), 1, mid)
    else:
        return(frame, 0, (0,0))

def computeCubeCenter(upperLayerCubePoints):

    mid = [0,0,1.0]

    mid[0] = (upperLayerCubePoints[1][0] + upperLayerCubePoints[3][0])/2
    mid[1] = (upperLayerCubePoints[1][1] + upperLayerCubePoints[3][1])/2

    mid = np.float32(mid)

    return mid

def drawMid(img, pts1, xc):
    cv2.circle(img, tuple(xc[:-1].astype(np.int32)), 10, (0, 0, 255), 5)
    return img

def projectionMatrix(b,w,h): 
    #Calibration matrix (Transposed)
    global  lamold, lamnew, frames
    # Calibration matrix K
    KTrans =[1406.08415449821,0,0,2.20679787308599, 1417.99930662800,0,1014.13643417416, 566.347754321696,1]
    K = (np.reshape(KTrans,(-1,3))).T #bt have to take transpose
    lamnew = 2/(np.linalg.norm(np.dot(np.linalg.inv(K),b[:,0]),2)+np.linalg.norm(np.dot(np.linalg.inv(K),b[:,1]),2))
    Btilde = np.dot(np.linalg.inv(K),b)
    if(frames > 0):
        lam = (lamold+lamnew)/2
    else:
        lam = lamnew
    lamold = lamnew

    if(np.linalg.det(Btilde)< 0):
        constant = -1
    else:
        constant = 1

    B = np.dot(Btilde,constant)
    # Calculating the Rotation and translation values 
    r1 = np.dot(lam,B[:,0])
    r2 = np.dot(lam,B[:,1])
    r3 = np.cross(r1,r2)
    t = np.dot(lam,B[:,2])
    
    Rt = np.zeros((3,4))
    # Generating 3x4 Rotation-translation matrix
    Rt[:,0] = r1 
    Rt[:,1] = r2 
    Rt[:,2] = r3 
    Rt[:,3] = t 

    # Calculating the Projection matrix
    P = np.dot(K,Rt)
    xw = np.zeros((4,4))
    
    # Assigning world coordinates
    xw[2,:],xw[3,:] = -w,1 
    xw[0][0],xw[1][0] = 0,0 
    xw[0][1],xw[1][1] = w,0 
    xw[0][2],xw[1][2] = w,h 
    xw[0][3],xw[1][3] = 0,h 

    xc = np.zeros((0,0))
    
    # Calculating image plane coordinates
    # using projection matrix and world coordinates 
    for i in range(0,4):
        xc = np.append(xc,np.dot(P,xw[:,i]))

    #print('np.dot(P,xw[:,i])',np.dot(P,xw[:,i]),'xw[:,i]',xw[:,i])
    xc = np.reshape(xc,(-1,3))
    for i in range(0,4):
        xc[i][0],xc[i][1],xc[i][2] = xc[i][0]/xc[i][2],xc[i][1]/xc[i][2],xc[i][2]/xc[i][2]
    return xc

def draw(img,pts1, xc):
     xc =xc[:,0:xc.shape[1]-1]
     # Drawing bottom plane of the cube 
     cv2.drawContours(img, [pts1.astype(np.int32)] , -1, (0,255,0), 30)
     # Drawing lines between the bottom and the top plane
     for i in range(0,4):
         img = cv2.line(img, tuple(pts1[i].astype(np.int32)), tuple(xc[i].astype(np.int32)),(255),30)
     # Drawing upper plane
     img = cv2.drawContours(img,[xc.astype(np.int32)],-1,(0,0,255),30)
     return img

def giveTag(dstTag):
    # divide image into eight parts
    row1 = int(dstTag.shape[0]/8)
    col1  = int(dstTag.shape[1]/8)
    reqRegion = np.zeros((4,2),dtype = 'int32')
    reqRegion[0][0] = 3*row1
    reqRegion[0][1] = 3*col1
    reqRegion[3][0] = 4*row1
    reqRegion[3][1] = 3*col1
    reqRegion[2][0] = 4*row1
    reqRegion[2][1] = 4*col1
    reqRegion[1][0] = 3*row1
    reqRegion[1][1] = 4*col1
    lst = []
    # Check the values of the encoding region
    for i in reqRegion:
            ROI = dstTag[i[0]:i[0]+row1,i[1]:i[1]+col1]
            meanL = ROI.mean(axis=0).mean(axis=0)
            mean = meanL.sum()/3
            if(mean > 240):
                    lst.append(1)
            else:
                    lst.append(0)
    ans = lst[0]*1 + lst[1]*2 + lst[2]*4 + lst[3]*8          
    return ans

def alignTag(uprightTag, binaryImage, pts, pts1):
    # Find contours in upright tag image
    _, tagContours, hierarchy = cv2.findContours(binaryImage, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # tagContours, hierarchy = cv2.findContours(binaryImage, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # Find biggest contour
    areas = [cv2.contourArea(c) for c in tagContours]
    max_index = np.argmax(areas)
    cnt = tagContours[max_index]
    # Smoothen the contour to find the corner points
    approx = cv2.approxPolyDP(cnt,0.1*cv2.arcLength(cnt,True),True)
    # Find the corner associated with rotation
    minDistance = 0
    firstTime = 1
    index = 0
    for corners in approx:
        x,y = corners.ravel()
        #cv2.circle(image,(x,y),2,[0,255,0],-1)
        i = 0
        for points in pts:
            i = i+1
            borderX,borderY = points
            distance = math.sqrt((borderX - x)**2 + (borderY - y)**2)
            if  distance < minDistance or firstTime:
                firstTime = 0
                minDistance = distance
                index = i
    pts1 = np.roll(pts1, index-3, axis = 0)
    return (pts1, index - 3)

def findHMatrix(pts1,pts2):
        A = []
        # Image coordinates
        xc1,yc1 = pts1[0]
        xc2,yc2 = pts1[1]
        xc3,yc3 = pts1[2]
        xc4,yc4 = pts1[3]

        # World Coordinates
        xw1,yw1 = pts2[0]
        xw2,yw2 = pts2[1]
        xw3,yw3 = pts2[2]
        xw4,yw4 = pts2[3]

        # Transforming World to Image coordinates
        A = [[xw1,yw1,1,0,0,0,-xc1*xw1,-xc1*yw1,-xc1],
             [0,0,0,xw1,yw1,1,-yc1*xw1,-yc1*yw1,-yc1],
             [xw2,yw2,1,0,0,0,-xc2*xw2,-xc2*yw2,-xc2],
             [0,0,0,xw2,yw2,1,-yc2*xw2,-yc2*yw2,-yc2],
             [xw3,yw3,1,0,0,0,-xc3*xw3,-xc3*yw3,-xc3], 
             [0,0,0,xw3,yw3,1,-yc3*xw3,-yc3*yw3,-yc3],
             [xw4,yw4,1,0,0,0,-xc4*xw4,-xc4*yw4,-xc4],
             [0,0,0,xw4,yw4,1,-yc4*xw4,-yc4*yw4,-yc4]]
        u, s, V = np.linalg.svd(A, full_matrices = True)

        #Converting to Hommogeneous coordinates
        a = []
        if V[8][8] == 1:
            for i in range(0,9):
                a.append(V[8][i])
        else:
            for i in range(0,9):
                a.append(V[8][i]/V[8][8])

        # H matrix in 3X3 shape
        b = np.reshape(a, (3, 3))
        return b


def findChildren(k, hierarchy,childs):
    n = 0
    # find childrens of given parent
    for row in hierarchy[0]:
        if(row[3] == k):
            childs = childs + 1
            childs = findChildren(n, hierarchy,childs)
        n = n+1
    return childs

## FIN FONCTIONS DE TRAITEMENT D'IMAGE ##

## DETECTION BOITE ET AJUSTEMENT VITESSE ##

def detectWhichSectorBoxIsIn(mids, limit1, limit2, tts):
    # Detection de la boite 
    # Actuellement, les flags BOX_ENCOUNTERED et flagPushMode sont redondents.
    # Le flag BOX_ENCOUNTERED a vocation à disparaitre, pour permettre la détection d'autres boites sur le chemin

    global flagPushMode
    global VITESSE_COURANTE
    global BOX_ENCOUNTERED
    if not(BOX_ENCOUNTERED):
        if not(flagPushMode):
            if len(mids)==0:
                # Pas de boite detectee
                print("No box detected")
            else :
                # On récupère la liste des tags AR detectes dans l'image par mids, on obtient une liste de 2-uplets
                mids = list(map(lambda x : x[1], mids)) # On traite uniquement la coordonnées y donc on récupère seulement les 2e éléments des 2-uples
                mids.sort() # On les trie, de cette façon on sait que le dernier élement de la liste est le plus proche du robot
                y_to_check = mids[-1]
                if y_to_check<limit1:
                    # Box detectee en safe zone
                    print("Box detected in safe zone - not changing behavior")
                    
                elif y_to_check>=limit1 and y_to_check<=limit2:
                    # Box detectee en zone intermediaire
                    print("Box detected closeby - lowering speed")
                    # MODIFICATION DE LA VITESSE COURANTE 
                    lock.acquire()
                    VITESSE_COURANTE = updateSpeed(y_to_check, NORMAL_SPEED, PUSH_MODE_SPEED,limit1, limit2)
                    lock.release()
                    print("speed will be : ", VITESSE_COURANTE)
                elif y_to_check>limit2 : 
                    # Passage en pushmode
                    print("Almost at box - push mode")
                    tts.say("Boite, je la pousse") # Signal vocal 
                    flagPushMode = True
                    BOX_ENCOUNTERED = True
        else: 
            print("Currently in push mode")

def updateSpeed(y_value, initialSpeed, goal,limit1,limit2):
    # Need to update speed according to y_value
    # changement de la vitesse pour la zone intermediaire
    a = (initialSpeed-goal)/(limit1-limit2)
    b = initialSpeed - limit1*((initialSpeed-goal)/(limit1-limit2))
    # print(a,b)

    return a*y_value+b

## FIN DETECTION BOITE ET AJUSTEMENT VITESSE ##


tts = ALProxy("ALTextToSpeech", "127.0.0.1", 9559) # pour se connecter a l'interface Text2Speech de Pepper

topic_camera_up = "/pepper_robot/camera/front/image_raw" # Topic de la camera frontale du robot
# topic_head = "/pepper_robot/pose/joint_angles" 
# joint_message_type = "naoqi_bridge_msgs/JointAnglesWithSpeed"


# FLAG + VITESSES NORMALES ET VITESSE EN PUSHMODE
flagPushMode = False

NORMAL_SPEED = 0.07
PUSH_MODE_SPEED = 0.035

VITESSE_COURANTE = NORMAL_SPEED

BOX_ENCOUNTERED = False

# L1 et L2 qui permettent de definir les zones safe, ralentissement et push mode

L1 = 70
L2 = 180

def box_detected_behavior():
    print("box detected")

def message_callback(msg):
    global flagPushMode
    bridge = CvBridge()
    cv_image= bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    image_array = np.array(cv_image)
    middles = putCube(cv_image)
    middles = middles[2]
    detectWhichSectorBoxIsIn(middles, L1, L2, tts)

def odom_callback(msg):
    global current_orientation_z
    global current_position_x
    global current_position_y
    global cmd_vel_pub
    global targets
    global target_nbr
    global orientation_reached
    global start_x
    global start_y
    global VITESSE_COURANTE
    global flagPushMode
    global cpt

    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    )
    _, _, current_orientation_z = euler_from_quaternion(quaternion)
    current_position_x = msg.pose.pose.position.x
    current_position_y = msg.pose.pose.position.y
    #waypoint list

    angular_speed = 0.2  # Adjust the angular speed as needed
    lock.acquire()
    linear_speed = VITESSE_COURANTE
    lock.release()

    cmd_vel_msg = Twist()

    if start_x==0.0 and start_y ==0.0 : #initialisation au premier tour
        start_x = current_position_x
        start_y = current_position_y

    if target_nbr == len(targets):
        print("END")
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.angular.z = 0.0
    else:
        delta_x=targets[target_nbr][0] - start_x
        delta_y=targets[target_nbr][1] - start_y
        target_orientation_z = math.atan2(delta_y, delta_x)  # Adjust the target orientation as needed
        norme_start_target = math.pow(math.pow(delta_x, 2) + math.pow(delta_y,2) ,0.5)
        #print(target_orientation_z)
        '''
        if target_orientation_z - current_orientation_z > pi or (target_orientation_z - current_orientation_z < pi and target_orientation_z - current_orientation_z < 0):
            angular_speed = - angular_speed
        
        
        x = target_orientation_z - current_orientation_z
        if x > pi or (x < pi and x < 0) or (x<pi and x>0 and target_orientation_z<0 and current_orientation_z<0):
            angular_speed = - angular_speed
        cmd_vel_msg.angular.z = angular_speed  # Set angular velocity
        '''
        x=0
        if targets[target_nbr][0]>current_position_x:
            x=math.atan(delta_y/delta_x)
        else:
            x=math.atan(delta_y/delta_x)+pi
        
        if x>=0 and x<=pi:
            pass
        else:
            angular_speed = - angular_speed
        on_target = abs(current_position_x - targets[target_nbr][0]) < 0.2 and abs(current_position_y - targets[target_nbr][1]) < 0.2
        orientation_ok = abs(current_orientation_z - target_orientation_z) < 0.05
        
        if on_target==True:
            cmd_vel_msg.linear.x = 0.0
            target_nbr+=1
            orientation_reached = False
            start_y = current_position_y
            start_x = current_position_x
            flagPushMode=False
            VITESSE_COURANTE = NORMAL_SPEED
            cpt=0
            print("current postition : ", current_position_x, " ", current_position_y, " Target : ", targets[target_nbr][0], " ", targets[target_nbr][1])
        elif orientation_reached == True :
            cmd_vel_msg.angular.z = 0.0  # Set angular velocity
            cmd_vel_msg.linear.x = linear_speed
            delta_x=current_position_x - start_x
            delta_y=current_position_y - start_y
            norme = math.pow(math.pow(delta_x, 2) + math.pow(delta_y,2) ,0.5)
            if abs(norme - 0.75*norme_start_target) <= 0.1 and cpt==0: #and abs(current_orientation_z - target_orientation_z) < 0.05:
                orientation_reached=False
                cpt+=1
                print("correction ###########################################")
            print(orientation_reached)
            print("current postition : ", current_position_x, " ", current_position_y, " Target : ", targets[target_nbr][0], " ", targets[target_nbr][1])
        elif orientation_ok==True:
            cmd_vel_msg.angular.z = 0.0  # Set angular velocity
            cmd_vel_msg.linear.x = linear_speed
            #start_y = current_position_y
            #start_x = current_position_x
            orientation_reached=True
            print("current postition : ", current_position_x, " ", current_position_y, " Target : ", targets[target_nbr][0], " ", targets[target_nbr][1])
        else:
            cmd_vel_msg.angular.z = angular_speed
            cmd_vel_msg.linear.x = 0.0

    cmd_vel_pub.publish(cmd_vel_msg)

def listener():
    global cmd_vel_pub
    global targets
    global target_nbr
    global orientation_reached
    global start_x
    global start_y
    global cpt

    targets = [(2,0),(3,1),(1,-0.2),(0,0)]
    target_nbr = 0
    orientation_reached = False

    start_x = 0.0
    start_y = 0.0
    cpt=0


    rospy.init_node('rotate_robot_node', anonymous=True)
    # Subscribe to odometry topic
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/pepper_robot/odom', Odometry, odom_callback)
    #rospy.init_node('message_listener', anonymous=True)
    rospy.Subscriber(topic_camera_up, Image, message_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
