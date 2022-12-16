
"""
Example of how to change flight modes using pymavlink

import sys
# Import mavutil
from pymavlink import mavutil

# Create the connection
master = mavutil.mavlink_connection('/dev/ttyACM0')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# Choose a mode
mode = 'STABILIZED'

# Check if mode is available
if mode not in master.mode_mapping():
    print('Unknown mode : {}'.format(mode))
    print('Try:', list(master.mode_mapping().keys()))
    sys.exit(1)

# Get mode ID
mode_id = master.mode_mapping()[mode]
# Set new mode
# master.mav.command_long_send(
#    master.target_system, master.target_component,
#    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
#    0, mode_id, 0, 0, 0, 0, 0) or:
# master.set_mode(mode_id) or:
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)

while True:
    # Wait for ACK command
    # Would be good to add mechanism to avoid endlessly blocking
    # if the autopilot sends a NACK or never receives the message
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    ack_msg = ack_msg.to_dict()

    # Continue waiting if the acknowledged command is not `set_mode`
    if ack_msg['command'] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
        continue

    # Print the ACK result !
    print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
    break



"""
from pymavlink import mavutil

# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('/dev/ttyACM0')

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

mode_id = the_connection.mode_mapping()['STABILIZED']
the_connection.mav.set_mode_send(the_connection.target_system,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)

try: 
    altitude = the_connection.messages['GPS_RAW_INT'].alt  # Note, you can access message fields as attributes!
    timestamp = the_connection.time_since('GPS_RAW_INT')
    print(altitude)
except:
    print('No GPS_RAW_INT message received')

try: 
    altitude = the_connection.messages['NAV_CONTROLLER_OUTPUT']  # Note, you can access message fields as attributes!
    timestamp = the_connection.time_since('NAV_CONTROLLER_OUTPUT')
    print(altitude)
except:
    print('No NAV message received')

"""
msg = the_connection.recv_match(type='SYS_STATUS')
print(msg)
altitude = the_connection.messages['GPS_RAW_INT'].alt  # Note, you can access message fields as attributes!
timestamp = the_connection.time_since('GPS_RAW_INT')
print(str(altitude) +' '+ str(timestamp))
print('No GPS_RAW_INT message received)
"""
"""
msg = the_connection.recv_match(type='SYS_STATUS', condition='SYS_STATUS.mode==2 and SYS_STATUS.nav_mode==4', blocking=True).mode
print(msg.mode)

roll = the_connection.messages['ATTITUDE_QUATERNION_COV'].rollspeed
print(roll)
"""
