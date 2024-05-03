# **********************************************************************************
# * Copyright (C) 2024-present Bert Van Acker (B.MKR) <bert.vanacker@uantwerpen.be>
# *
# * This file is part of the roboarch R&D project.
# *
# * RAP R&D concepts can not be copied and/or distributed without the express
# * permission of Bert Van Acker
# **********************************************************************************
import json
from RoboSapiensAdaptivePlatform.utils.nodes import TriggeredNode
from RoboSapiensAdaptivePlatform.Communication.Messages.messages import ComponentStatus
from RoboSapiensAdaptivePlatform.utils.constants import *



class monitor(TriggeredNode):

    def __init__(self, logger = None,knowledgeBase = None,verbose=False):
        super().__init__(logger=logger,knowledge = knowledgeBase,verbose=verbose)

        self._name = "monitor"
        self._counter = 0       #DUMMY ANOMALY!

    # ------------------------------------------------------------------------------------------------
    # -------------------------------------INTERNAL FUNCTIONS----------------------------------------
    # ------------------------------------------------------------------------------------------------
    def _SpinOnceFcn(self, args):

        # 1. FETCH robot odometry and detected persons from KB
        ROB_ODO,ROB_ODO_history = self.knowledge.read("RobotOdometry",queueSize=2)
        PERSON_DETECT, PERSON_DETECT_history = self.knowledge.read("DetectedPersons", queueSize=2)

        # 2. PERFORM MONITORING
        #!!--------------USER IMPLEMENTATION---------------!!
        self._counter = self._counter+1
        if self._counter == 4:
            _status = monitorStatus.ANOMALY
            _accuracy = 1.0
        else:
            _status = monitorStatus.NORMAL
            _accuracy = 1.0

        #!!--------------USER IMPLEMENTATION--------------!!

        # 3. SIGNAL MONITORING STATE VIA KNOWLEDGE
        self.RaPSignalStatus(component=adaptivityComponents.MONITOR,status=_status,accuracy=_accuracy)


        self.logger.log("["+self._name+"] - "+"Monitoring robot odometry and detected persons")

        # !!FOR TESTING!!
        #print(json.dumps(ROB_ODO, default=lambda o: o.__dict__,sort_keys=True, indent=4))

        # 4. return status of execution (fail = False, success = True)
        return True

    def _EnterInitializationModeFcn(self):
        if self._verbose: print("["+self._name+"] - "+"Enter initializationModeFcn not implemented")

    def _ExitInitializationModeFcn(self):
        # initial signal after startup
        self.RaPSignalStatus(component=adaptivityComponents.MONITOR,status=monitorStatus.NORMAL,accuracy=1.0)

    def _EnterConfigurationModeFcn(self):
        if self._verbose: print("["+self._name+"] - "+"Enter configurationModeFcn not implemented")

