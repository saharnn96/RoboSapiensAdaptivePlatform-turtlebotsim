import time
class Stamp():
    sec=0
    nanosec=0
class Header():
    stamp = Stamp()
    frame_id = ''


class BatteryMsg():
    header=Header()
    voltage=0.0
    temperature = 0.0
    current = 0.0
    charge = 0.0
    capacity = 0.0
    design_capacity = 0.0
    percentage = 1.0
    power_supply_status = 0
    power_supply_health = 0
    power_supply_technology = 0
    present = True
    cell_voltage = []
    cell_temperature = []
    location = ''
    serial_number = ''


class LidarMsg():
    header=Header()
    angle_min= -3.12
    angle_max = 3.14
    angle_increment = 0.0087
    time_increment = 0.000187
    scan_time = 0.13464
    range_min = 0.15
    range_max = 12.0
    ranges = []






def composeHeader(frame_id):

    _stamp = Stamp()
    _stamp.sec = int(time.time())
    _stamp.nanosec = 1

    _header = Header()
    _header.stamp = _stamp
    _header.frame_id = frame_id

    return _header