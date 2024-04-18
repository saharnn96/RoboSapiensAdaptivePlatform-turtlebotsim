#**********************************************************************************
# * Copyright (C) 2024-present Bert Van Acker (B.MKR) <bert.vanacker@uantwerpen.be>
# *
# * This file is part of the roboarch R&D project.
# *
# * RAP R&D concepts can not be copied and/or distributed without the express
# * permission of Bert Van Acker
# **********************************************************************************
from turtlebot4_sim.turtlebot4.Messages import BatteryMsg,composeHeader
import time
import json

class Battery:
    def __init__(self):
        # --- Static battery parameters ---
        self._design_capacity = 0.0
        self._power_supply_technology = 0
        self._present = True
        self._location = ''
        self._serial_number = ''

        # --- dynamic battery parameters ---
        self._percentage = 1.0    #100%
        self._voltage = 0.0
        self._temperature = 0.0
        self._current = 0.0
        self._charge = 0.0
        self._capacity = 0.0
        self._power_supply_status = 0
        self._power_supply_health = 0
        self._cell_voltage = []
        self._cell_temperature = []

        # --- exposed battery message ---
        self.batteryMessage = BatteryMsg()

    def spin_once(self):

        # --- run battery model ---

            #DOES NOTHING FOR NOW

        # --- compose battery message ---

        _header = composeHeader(frame_id='')

        self.batteryMessage.header = _header

        self.batteryMessage.percentage = 0.99   #99%
        self.batteryMessage.voltage = 0.0
        self.batteryMessage.temperature = 0.0
        self.batteryMessage.current = 0.0
        self.batteryMessage.charge = 0.0
        self.batteryMessage.capacity = 0.0
        self.batteryMessage.power_supply_status = 0
        self.batteryMessage.power_supply_health = 0
        self.batteryMessage.cell_voltage = []
        self.batteryMessage.cell_temperature = []

        # --- Static battery parameters ---
        self.batteryMessage.design_capacity = 0.0
        self.batteryMessage.power_supply_technology = 0
        self.batteryMessage.present = True
        self.batteryMessage.location = ''
        self.batteryMessage.serial_number = ''

        #print(json.dumps(self.batteryMessage, default=lambda o: o.__dict__,sort_keys=True, indent=4))


