import math
import time
from pymavlink import mavutil

# ==========================================
# CONNECTION
# ==========================================

master = mavutil.mavlink_connection('udp:172.25.0.1:14551')

print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Connected")

# ==========================================
# GET HOME POSITION
# ==========================================

def get_home():
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        if lat != 0:
            return lat, lon

# ==========================================
# METER OFFSET
# ==========================================

def meter_offset(lat, north, east):
    dlat = north / 111111
    dlon = east / (111111 * math.cos(math.radians(lat)))
    return dlat, dlon

# ==========================================
# SQUARE GENERATOR WITH CYCLES
# ==========================================

def generate_square(home_lat, home_lon, size, altitude, cycles):
    half = size / 2
    base_square = [
        (0,0),
        (size,0),
        (size,size),
        (0,size),
        (0,0)
    ]
    mission = []
    for cycle in range(cycles):
        for i,(north,east) in enumerate(base_square):
            if cycle > 0 and i == 0:
                continue
            dlat, dlon = meter_offset(home_lat, north, east)
            lat = home_lat + dlat
            lon = home_lon + dlon
            mission.append((lat, lon, altitude))
    return mission

# ==========================================
# CLEAR MISSION
# ==========================================

def clear_mission():
    master.mav.mission_clear_all_send(
        master.target_system,
        master.target_component
    )
    master.recv_match(type='MISSION_ACK', blocking=True)
    print("Mission cleared")

# ==========================================
# UPLOAD MISSION (with RTL + LAND)
# ==========================================

def upload_mission(waypoints, home_lat, home_lon):
    total = len(waypoints) + 2  # Add RTL + LAND
    master.mav.mission_count_send(
        master.target_system,
        master.target_component,
        total
    )
    for i in range(total):
        master.recv_match(type=['MISSION_REQUEST','MISSION_REQUEST_INT'], blocking=True)

        # NORMAL WAYPOINTS
        if i < len(waypoints):
            lat, lon, alt = waypoints[i]
            master.mav.mission_item_int_send(
                master.target_system,
                master.target_component,
                i,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,
                1,
                0,0,0,0,
                int(lat * 1e7),
                int(lon * 1e7),
                alt
            )
        # RTL
        elif i == len(waypoints):
            master.mav.mission_item_int_send(
                master.target_system,
                master.target_component,
                i,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                0,
                1,
                0,0,0,0,
                0,0,0
            )
        # LAND
        else:
            master.mav.mission_item_int_send(
                master.target_system,
                master.target_component,
                i,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0,
                1,
                0,0,0,0,
                int(home_lat * 1e7),
                int(home_lon * 1e7),
                0
            )

        print("Uploaded WP", i)

    master.recv_match(type='MISSION_ACK', blocking=True)
    print("Mission uploaded")

# ==========================================
# ARM + TAKEOFF
# ==========================================

def arm_takeoff(alt):
    master.set_mode_apm("GUIDED")
    master.arducopter_arm()
    master.motors_armed_wait()
    print("Armed")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,0,0,0,0,0,alt
    )
    time.sleep(8)

# ==========================================
# MONITOR MISSION (with GPS printing)
# ==========================================

def monitor():
    last_wp = -1
    while True:
        msg = master.recv_match(type='MISSION_CURRENT', blocking=True)
        seq = msg.seq
        if seq != last_wp:
            # Read current global position
            pos_msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            lat = pos_msg.lat / 1e7
            lon = pos_msg.lon / 1e7
            alt = pos_msg.relative_alt / 1000  # meters
            print(f"Reached WP {seq}: Lat={lat:.7f}, Lon={lon:.7f}, Alt={alt:.2f}m")
            last_wp = seq

# ==========================================
# PARAMETERS (simulate UI sliders)
# ==========================================

SIZE = 30
ALTITUDE = 10
CYCLES = 3

# ==========================================
# MAIN FLOW
# ==========================================

home_lat, home_lon = get_home()

mission = generate_square(home_lat, home_lon, SIZE, ALTITUDE, CYCLES)

print("Mission waypoints:", len(mission))

clear_mission()

upload_mission(mission, home_lat, home_lon)

arm_takeoff(ALTITUDE)

master.set_mode_apm("AUTO")

monitor()