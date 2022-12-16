import time
# Import mavutil
from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink1 
# Create the connection
master = mavutil.mavlink_connection('/dev/ttyACM0')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

"""
while True:
    master.mav.command_long_send(master.target_system, master.target_component, mavlink1.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mavlink1.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT, 20000, 0, 0, 0, 0 ,0)
    msg = master.recv_match()
    #If we have a valid message
    print(msg)
    ""
    if msg is not None:
        #print msg.get_type()
        if msg.get_type() == "HEARTBEAT":
            print (str(msg)+' HI')
        elif msg.get_type() == "NAV_CONTROLLER_OUTPUT":
            print (msg)
"""
def request_message_interval(message_id: int, frequency_hz: float):
    """   
    Request MAVLink message in a desired frequency,
    documentation for SET_MESSAGE_INTERVAL:
        https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

    Args:
        message_id (int): MAVLink message ID
        frequency_hz (float): Desired frequency in Hz
"""   
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, # The MAVLink message ID
        1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
        0, 0, 0, 0, # Unused parameters
        0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
    )

# Configure AHRS2 message to be sent at 1Hz
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT,1)

# Configure ATTITUDE message to be sent at 2Hz
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 2)

# Get some information !
while True:
    try:
        print(master.recv_match().to_dict())
    except:
        pass
    time.sleep(0.1)
