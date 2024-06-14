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
from RoboSapiensAdaptivePlatform.Communication.Messages.messages import Action
from RoboSapiensAdaptivePlatform.utils.constants import *
from lidarocclusion.masks import BoolLidarMask, ProbLidarMask
from lidarocclusion.sliding_lidar_masks import sliding_lidar_mask, sliding_prob_lidar_mask
from RoboSapiensAdaptivePlatform.Communication.Messages.messages import LidarRange

# Probability threshold for detecting a lidar occlusion
OCCLUSION_THRESHOLD = 0.3
# Number of scans to use for the sliding window
SLIDING_WINDOW_SIZE = 3
# Lidar mask sensitivity (ignore small occlusions below this angle)
OCCLUSION_SENSITIVITY = Fraction(1, 48)

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

        # 2. PERFORM MONITORING
        #!!--------------USER IMPLEMENTATION---------------!!
        self._scans.append(lidar_data)

        # self.knowledge.write("LidarMask", next(self._sliding_lidar_masks)) 

        # Add the next sliding probabilistic lidar mask to the knowledge base
        prob_lidar_mask = next(self._sliding_prob_lidar_masks)
        self.logger.log(f"[{self._name}] - Prob lidar mask: {prob_lidar_mask}")
        
        # Rotate the probabilistic lidar mask to align with the robot's orientation
        prob_lidar_mask = prob_lidar_mask.rotate(-Fraction(1, 2))

        # Plot the prob lidar mask to a file
        prob_lidar_mask.plot()
        plt.savefig("prob_lidar_mask.png")
        # self.knowledge.write("ProbLidarMask", prob_lidar_mask)

        # Compute the boolean lidar mask
        # Apply the occlusion threshold
        lidar_mask = (prob_lidar_mask >= OCCLUSION_THRESHOLD)
        # Weaken lidar masks to threshold
        lidar_mask = lidar_mask.weaken(OCCLUSION_SENSITIVITY)
        lidar_mask = lidar_mask.weaken(-OCCLUSION_SENSITIVITY)
        lidar_mask = lidar_mask.strengthen(OCCLUSION_SENSITIVITY)
        lidar_mask = lidar_mask.strengthen(-OCCLUSION_SENSITIVITY)
        self.logger.log(f"[{self._name}] - Lidar mask: {lidar_mask}")
        
        lidar_mask.plot()
        plt.savefig("bool_lidar_mask.png")

        # We don't care if there is an occlusion directly behind the robot
        ignore_lidar_region = BoolLidarMask(
            [(3*np.pi/4, 5*np.pi/4)],
            lidar_mask.base_angle,
        )
        ignore_lidar_region.plot()
        plt.savefig("ignore_lidar_region.png")

        # Mask out the ignored region
        lidar_mask_reduced = lidar_mask | ignore_lidar_region
        self.logger.log(f"[{self._name}] - Reduced lidar mask: {lidar_mask_reduced}")
        lidar_mask_reduced.plot()
        plt.savefig("bool_lidar_mask_reduced.png")

        # Add the next sliding boolean lidar mask to the knowledge base
        self.logger.log(f"[{self._name}] - Lidar mask: {lidar_mask}")
        # raise Exception("I object!")

        # Set the monitor status to mark an anomaly if the there is any
        # occlusion outside of the ignored region
        status = (monitorStatus.NORMAL
                  if lidar_mask_reduced._values.all()
                  else monitorStatus.ANOMALY)
        self.logger.log(f"[Monitor] status was {status}")
        # TODO: figure out what this should be
        accuracy = 1.0

        #!!--------------USER IMPLEMENTATION--------------!!

        # 3. SIGNAL MONITORING STATE VIA KNOWLEDGE
        self.RaPSignalStatus(component=adaptivityComponents.MONITOR,status=status, accuracy=accuracy)

        self.logger.log("["+self._name+"] - "+"Monitoring")

        # Hack to get the lidar mask into the knowledge base
        plan = Action()
        plan.name = 'SpeedAdaptationAction'
        plan.ID = actionType.ADAPTATIONTYPE
        plan.description = "SPEED ADAPTATION"
        plan.propertyList = [{"mask": lidar_mask_reduced, "directions": None}]
        self.knowledge.write(plan)

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
        
        self._sliding_prob_lidar_masks = sliding_prob_lidar_mask(
            raw_lidar_masks(),
            window_size=SLIDING_WINDOW_SIZE,
        )


        # if self._verbose: print("["+self._name+"] - "+"Enter initializationModeFcn not implemented")

    def _ExitInitializationModeFcn(self):
        # initial signal after startup
        self.RaPSignalStatus(component=adaptivityComponents.MONITOR,status=monitorStatus.NORMAL,accuracy=1.0)

    def _EnterConfigurationModeFcn(self):
        if self._verbose: print("["+self._name+"] - "+"Enter configurationModeFcn not implemented")

