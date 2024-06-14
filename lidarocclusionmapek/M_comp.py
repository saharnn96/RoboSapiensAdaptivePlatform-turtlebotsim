# **********************************************************************************
# * Copyright (C) 2024-present Bert Van Acker (B.MKR) <bert.vanacker@uantwerpen.be>
# *
# * This file is part of the roboarch R&D project.
# *
# * RAP R&D concepts can not be copied and/or distributed without the express
# * permission of Bert Van Acker
# **********************************************************************************
import json
import itertools
from fractions import Fraction

import numpy as np
from matplotlib import pyplot as plt

from RoboSapiensAdaptivePlatform.utils.nodes import TriggeredNode
from RoboSapiensAdaptivePlatform.Communication.Messages.messages import ComponentStatus
from RoboSapiensAdaptivePlatform.utils.constants import *
from lidarocclusion.masks import BoolLidarMask, ProbLidarMask
from lidarocclusion.sliding_lidar_masks import sliding_lidar_mask, sliding_prob_lidar_mask
from RoboSapiensAdaptivePlatform.Communication.Messages.messages import LidarRange


def lidar_mask_from_scan(scan: LidarRange) -> BoolLidarMask:
    scan_ranges = np.array(scan.rangeList)
    return BoolLidarMask(
        (scan_ranges != np.inf) & (scan_ranges != -np.inf),
        base_angle=Fraction(2, len(scan.rangeList)),
        # base_angle=scan.angleIncrement/180,
    )

import matplotlib
matplotlib.use('Agg')


class Monitor(TriggeredNode):

    def __init__(self, logger = None,knowledgeBase = None,verbose=False):
        super().__init__(logger=logger,knowledge = knowledgeBase,verbose=verbose)

        self._name = "monitor"
        self._counter = 0       #DUMMY ANOMALY!

    # ------------------------------------------------------------------------------------------------
    # -------------------------------------INTERNAL FUNCTIONS----------------------------------------
    # ------------------------------------------------------------------------------------------------
    def _SpinOnceFcn(self, args):
        lidar_data,lidar_data_history = self.knowledge.read("LidarRange",queueSize=2)

        self.logger.log(f"[{self._name}] - Knowledge = {self.knowledge._PropertyList}")

        if not isinstance(lidar_data, LidarRange):
            self.logger.log("["+self._name+"] - "+"No lidar data available")
            return
        self.logger.log("["+self._name+"] - "+"Acquired lidar data: " + repr(lidar_data)) 
        self.logger.log("["+self._name+"] - "+"Lidar Data angleMin: " + repr(lidar_data.angleMin))
        self.logger.log("["+self._name+"] - "+"Lidar Data angleMax: " + repr(lidar_data.angleMax))
        self.logger.log("["+self._name+"] - "+"Lidar Data Len: " + repr(len(lidar_data.rangeList)))
        self.logger.log("["+self._name+"] - "+"Lidar Data Raw: " + repr(lidar_data.rangeList))
        self.logger.log("["+self._name+"] - "+"Lidar Scan Angle Increment: " + repr(lidar_data.angleIncrement))
        self.logger.log("["+self._name+"] - "+"TEST!")

        # 2. PERFORM MONITORING
        #!!--------------USER IMPLEMENTATION---------------!!
        self._scans.append(lidar_data)

        # Add the next sliding boolean lidar mask to the knowledge base
        lidar_mask = next(self._sliding_lidar_masks)
        self.logger.log(f"[{self._name}] - Lidar mask: {lidar_mask}")
        # self.knowledge.write("LidarMask", next(self._sliding_lidar_masks)) 

        # Add the next sliding probabilistic lidar mask to the knowledge base
        prob_lidar_mask = next(self._sliding_prob_lidar_masks)
        self.logger.log(f"[{self._name}] - Prob lidar mask: {prob_lidar_mask}")

        # Plot the prob lidar mask to a file
        prob_lidar_mask.plot()
        plt.savefig("prob_lidar_mask.png")

        _status = monitorStatus.NORMAL
        # _accuracy = 1.0
        self.knowledge.write("ProbLidarMask", prob_lidar_mask)

        # Plot the prob lidar mask
        

        # plt.imshow(prob_lidar_mask)

        # In this case monitoring consists of building a sliding lidar occlusion map

        # self._counter = self._counter+1
        # if self._counter == 4:
        #     _status = monitorStatus.ANOMALY
        #     _accuracy = 1.0
        # else:
        #     _status = monitorStatus.NORMAL
        #     _accuracy = 1.0

        # plt.figure()
        # prob_lidar_mask.plot()
        # plt.savefig("prob_lidar_mask.png")

        #!!--------------USER IMPLEMENTATION--------------!!

        # 3. SIGNAL MONITORING STATE VIA KNOWLEDGE
        self.RaPSignalStatus(component=adaptivityComponents.MONITOR,status=_status,accuracy=_accuracy)


        self.logger.log("["+self._name+"] - "+"Monitoring robot odometry and detected persons")

        # !!FOR TESTING!!
        #print(json.dumps(ROB_ODO, default=lambda o: o.__dict__,sort_keys=True, indent=4))

        # 4. return status of execution (fail = False, success = True)
        return True

    def _EnterInitializationModeFcn(self):
        self._scans = []

        def scans():
            while True:
                for scan in self._scans:
                    yield scan
                
                self._scans = []

        def raw_lidar_masks():
            for scan in scans():
                yield lidar_mask_from_scan(scan)
        
        raw_masks1, raw_masks2 = itertools.tee(raw_lidar_masks(), 2)

        self._sliding_lidar_masks = sliding_lidar_mask(
            raw_masks1,
            window_size=5,
            cutoff=0.8,
        )
        self._sliding_prob_lidar_masks = sliding_prob_lidar_mask(
            raw_masks2,
            window_size=5,
        )


        # if self._verbose: print("["+self._name+"] - "+"Enter initializationModeFcn not implemented")

    def _ExitInitializationModeFcn(self):
        # initial signal after startup
        self.RaPSignalStatus(component=adaptivityComponents.MONITOR,status=monitorStatus.NORMAL,accuracy=1.0)

    def _EnterConfigurationModeFcn(self):
        if self._verbose: print("["+self._name+"] - "+"Enter configurationModeFcn not implemented")

