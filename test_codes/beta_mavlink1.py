#import mavutil
from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink1
import numpy as np
import time
#from numpy.polynomial import Polynomial as Poly
import warnings
warnings.simplefilter('ignore', np.RankWarning)
# Create the connection
master = mavutil.mavlink_connection('/dev/ttyACM0')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

IN = [[0]*6]*6
R_in = [[0]*4]*5
Y_in = [[0]*4]*5
P_in = [[0]*4]*5
i = 0

g,h = 0,0

def check():
    P = np.array([[-0.52653864,-0.58066964,0.43602782,-0.44210809]]).transpose()
    Y = np.array([[ 0.52806954,0.5559614,-0.47781378,-0.42865309]]).transpose()
    R = np.array([[-0.55982574,-0.51645258,0.43707775,-0.47836692]]).transpose()

    result_R = np.dot(all_in[0],R).transpose()
    result_Y = np.dot(all_in[1],Y).transpose()
    result_P = np.dot(all_in[2],P).transpose()
    #print(result_P,' \n', result_R,' \n', result_Y)
    
    """
    samp = [1,2,3,4,5]
    #print(result_R[0])
    R_R = np.polyfit(result_R[0],samp,1)[0]
    Y_R = np.polyfit(result_Y[0],samp,1)[0]
    P_R = np.polyfit(result_P[0],samp,1)[0]
    #R_R = Poly.fit(result_R[0],samp,0)
    """
    h = time.time()
    e = np.mean(result_R[0])
    w = np.mean(result_P[0])
    q = np.mean(result_Y[0])

    print(str(q) +' '+ str(w) +' '+str(e)+' ')
    #print('After: ' + str(h-g) + ' sec \n')
    if(q>10 or w>10 or e>10 ):
        print('Fault Detected')
        while True:
            print("Fault")
            time.sleep(2)
    else:
        print('No fault in last 5 samples')
    

while True: 
    g = time.time()
    master.mav.command_long_send(master.target_system, master.target_component, mavlink1.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mavlink1.MAVLINK_MSG_ID_ATTITUDE, 20000, 0, 0, 0, 0 ,0)
    msg = master.recv_match()
    #If we have a valid message
    #print(msg)
    if msg is not None:
        #print msg.get_type()
        if msg.get_type() == "ATTITUDE_TARGET":
            IN[i][0] = msg.body_roll_rate
            IN[i][1] = msg.body_yaw_rate
            IN[i][2] = msg.body_pitch_rate
        elif msg.get_type() == "ATTITUDE":
            IN[i][3] = msg.roll
            IN[i][4] = msg.yaw
            IN[i][5] = msg.pitch
    i = i+1
    if i>5:
        i = 0
        check()


    #print("INPUT TAKEN")
    t = np.array(IN)
    #print(t)
    if i>0:
        R_in[i-1] = np.array([IN[i][3], IN[i-1][3], IN[i][4], IN[i][5]])
        Y_in[i-1] = np.array([IN[i][4], IN[i-1][4], IN[i][5], IN[i][3]])
        P_in[i-1] = np.array([IN[i][5], IN[i-1][5], IN[i][3], IN[i][4]])
        
        all_in = np.array([R_in, Y_in, P_in])
        #print(all_in)
