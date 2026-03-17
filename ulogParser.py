import csv
from math import radians, cos, sin, asin, sqrt
import bisect

from pyulog import ULog
from scipy.spatial.transform import Rotation

from px4_flightmodes import NAV_STATE_MAP

"""Return first matching topic or None."""
def get_topic(ulog, topic_name):
    for t in ulog.data_list:
        if t.name == topic_name:
            return t
    return None

"""Return field value if available, else empty string."""
def get(data, field, idx):
    try:
        return data[field][idx]
    except Exception:
        return ""

""" Calculate distance between two positions of (lat,lon) using haversine distance formula"""
def calculate_distance(lat1, lon1, lat2, lon2):
    try:
        # Earth radius
        r = 6371.0088 * 1000.0 # convert from km to meter

        # convert decimal degrees to radians
        lat1 = radians(lat1)
        lon1 = radians(lon1)
        lat2 = radians(lat2)
        lon2 = radians(lon2)

        # haversin formula
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = sin(dlat*0.5)**2 + cos(lat1) * cos(lat2) * sin(dlon * 0.5)**2
        c = 2 * r * asin(sqrt(a))

        return c
    except Exception:
        return ""

""" Calculate alt over home point """
def calculate_alt_from_home(home_alt, curr_alt):
    try:
        # calculate alt over home-point, if needed clamp it at 0, so we can't go "negative". Although in reality we could takeoff from roof and fly under it...
        return round(curr_alt - home_alt, 2)
    except Exception:
        return ""

""" Transform quat to RPY using scipy. Quat input is expected as [q0, q1, q2, q3]"""
def quats_to_rpy_degree(quats):
    try:
        r = Rotation.from_quat(quats)
        return r.as_euler('xyz', degrees=True)
    except Exception:
        return ["","",""]

""" Function to get array that is in different fields with field[] notation into an array """
def get_array(data, key, index):
    try:
        # make array to push them to
        arr = []
        n = 0

        # loop until we have gotten all data for array
        while True:
            # get element
            element = get(data, "{}[{}]".format(key, n), index)

            # if we fail getting the data, we are done and return the array
            if element == "":
                break

            # push to array, make sure to cast to python float instead of numpy for proper csv writing
            arr.append(float(element))

            # increment n
            n += 1

        return arr

    except Exception:
        return []

""" Convert flightmode to string """
def convert_flightmode_to_string(flightmode):
    try:
        # TODO
        return ""
    except Exception:
        return ""

""" Find the nearest index based on our main clock timestamp. Required as topics are published at different speeds """
""" timestamps: the list to search through"""
""" target: the target time to find closest index to"""
def nearest_index(timestamps, target):
    pos = bisect.bisect_left(timestamps, target)
    if pos == 0:
        return 0
    if pos == len(timestamps):
        return len(timestamps) - 1
    before = timestamps[pos - 1]
    after = timestamps[pos]
    # check the difference between after and before, to find the closest index
    return pos if abs(after - target) < abs(target - before) else pos - 1

""" Main script """
def main(ulog_path, csv_path):
    ulog = ULog(ulog_path)

    # Try to load topics
    gps = get_topic(ulog, "vehicle_gps_position")
    global_pos = get_topic(ulog, "vehicle_global_position")
    home = get_topic(ulog, "home_position")
    local_pos = get_topic(ulog, "vehicle_local_position")
    battery = get_topic(ulog, "battery_status")
    attitude = get_topic(ulog, "vehicle_attitude")
    vehicle_status = get_topic(ulog, "vehicle_status")
    rc = get_topic(ulog, "input_rc")

    # Extract home lat/lon/alt if available for relative distance and altitude calculations
    home_lat = home.data["lat"][0] if home and "lat" in home.data else None
    home_lon = home.data["lon"][0] if home and "lon" in home.data else None
    home_alt = home.data["alt"][0] if home and "alt" in home.data else None

    # Get initial parameters
    params = ulog.initial_parameters

    # Set RC channel mapping based on parameters. Subtract one for 0-index
    ail_ch = params.get("RC_MAP_ROLL") - 1      # roll
    ele_ch = params.get("RC_MAP_PITCH") -1    # pitch
    rud_ch = params.get("RC_MAP_YAW") - 1       # yaw
    thr_ch = params.get("RC_MAP_THROTTLE") - 1  # thrust

    # Board serial
    fc_serial = ulog.msg_info_dict.get("sys_uuid") or ""
    fc_hw = ulog.msg_info_dict.get("ver_hw") or ""

    # TODO add as metadata in first data-row - Build meta-data fields #

    # Determine which topic to use for timestamps, primarily we want to use global_position as its the fused position
    primary = global_pos or gps
    if primary is None:
        print("No usable GPS or global position data found.")
        return

    # select source for timestamps, GPS if available
    timestamps = primary.data["timestamp"]
    n = len(timestamps)

    # Establish t=0 at the first timestamp
    start_time = timestamps[0]

    # Fields for CSV
    fields = [
        "time_s", "lat", "lon", "alt_m", "distance_to_home_m",
        "speed_ms", "velocity_x_ms", "velocity_y_ms", "velocity_z_ms",
        "battery_percent", "battery_voltage_v", "battery_temp_c", "cell_voltages",
        "pitch_deg", "roll_deg", "yaw_deg",
        "flight_mode",
        "rc_aileron", "rc_elevator", "rc_throttle", "rc_rudder",
        "satellites", "rc_signal"
    ]

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(fields)

        for i in range(n):
            # Convert to seconds since start of log
            time_s = (timestamps[i] - start_time) / 1e6

            ########## Mandatory ##########

            # Position fields (if available) - Don't need to find timestamps as MAIN clock is global_pos
            lat = get(global_pos.data, "lat", i) if global_pos else ""
            lon = get(global_pos.data, "lon", i) if global_pos else ""
            alt = get(global_pos.data, "alt", i) if global_pos else ""

            # convert alt to alt from takeoff
            if home_alt is not None and alt != "":
                alt_from_home = calculate_alt_from_home(home_alt, alt)
            else:
                alt_from_home = ""

            # Distance to home (calculated from current position to home position)
            if home_lat is not None and home_lon is not None and lat != "" and lon != "":
                dist_from_home = calculate_distance(lat, lon, home_lat, home_lon)
            else:
                dist_from_home = ""

            ########## Optionals ##########
            # Here we need to find the closest timestamp for our data

            # TODO finish getting it to work with syncronised timestamps

            # GPS
            gps_index = nearest_index(gps.data['timestamp'], timestamps[i])
            satellites = get(gps.data, "satellites_used", gps_index) if gps else ""
            ground_speed_m_s = get(gps.data, "vel_m_s", gps_index) if gps else ""

            # Local velocities
            local_pos_index = nearest_index(local_pos.data['timestamp'], timestamps[i])
            vx = get(local_pos.data, "vx", local_pos_index) if local_pos else ""
            vy = get(local_pos.data, "vy", local_pos_index) if local_pos else ""
            vz = get(local_pos.data, "vz", local_pos_index) if local_pos else ""

            # Attitude (Quaternions to rpy as degrees)
            attitude_index = nearest_index(attitude.data['timestamp'], timestamps[i])
            roll, pitch, yaw = quats_to_rpy_degree(
                [
                    get(attitude.data, "q[0]", attitude_index),
                    get(attitude.data, "q[1]", attitude_index),
                    get(attitude.data, "q[2]", attitude_index),
                    get(attitude.data, "q[3]", attitude_index)
                ]
            )

            # Battery
            battery_index = nearest_index(battery.data['timestamp'], timestamps[i])
            batt_percent = get(battery.data, "remaining", battery_index) if battery else ""
            batt_volt = get(battery.data, "voltage_v", battery_index) if battery else ""
            batt_temp = get(battery.data, "temperature", battery_index) if battery else ""
            cell_volt = get_array(battery.data, "voltage_cell_v", battery_index) if battery else "[]"

            # Flight mode - We map the raw value to PX4 enum, if we fail to map we set it as unknown and the raw value
            vehicle_status_index = nearest_index(vehicle_status.data['timestamp'], timestamps[i])
            flight_mode_raw = get(vehicle_status.data, "nav_state", vehicle_status_index)
            flight_mode = NAV_STATE_MAP.get(flight_mode_raw, f"UNKNOWN({flight_mode_raw})") if flight_mode_raw != "" else ""

            # RC
            rc_data_index = nearest_index(rc.data['timestamp'], timestamps[i])
            rc_ail = get(rc.data, "values[{}]".format(ail_ch), rc_data_index) if rc else ""
            rc_ele = get(rc.data, "values[{}]".format(ele_ch), rc_data_index) if rc else ""
            rc_thr = get(rc.data, "values[{}]".format(thr_ch), rc_data_index) if rc else ""
            rc_rud = get(rc.data, "values[{}]".format(rud_ch), rc_data_index) if rc else ""
            rc_signal = get(rc.data, "rssi", rc_data_index) if rc else ""

            # write it to csv file
            writer.writerow([
                time_s, lat, lon, alt_from_home, dist_from_home,
                ground_speed_m_s, vx, vy, vz,
                batt_percent, batt_volt, batt_temp, cell_volt,
                pitch, roll, yaw,
                flight_mode,
                rc_ail, rc_ele, rc_thr, rc_rud,
                satellites, rc_signal
            ])

    print(f"CSV written to {csv_path}")

if __name__ == "__main__":
    main("test_file/log_54_2026-3-12-11-16-12.ulg", "test_file/output.csv")
