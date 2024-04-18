#**********************************************************************************
# * Copyright (C) 2024-present Bert Van Acker (B.MKR) <bert.vanacker@uantwerpen.be>
# *
# * This file is part of the roboarch R&D project.
# *
# * RAP R&D concepts can not be copied and/or distributed without the express
# * permission of Bert Van Acker
# **********************************************************************************
from RoboSapiensAdaptivePlatform.utils.nodes import TriggeredNode
from RoboSapiensAdaptivePlatform.Communication.Messages.messages import *
from RoboSapiensAdaptivePlatform.utils.constants import *

class Telegraf(TriggeredNode):

    def __init__(self,logger = None,knowledgeManagement = None,verbose=False):
        super().__init__(logger=logger,knowledge = knowledgeManagement,verbose=verbose)
        """Initialize the generic Telegraf component.

                Parameters
                ----------
                verbose : bool
                    component verbose execution

        See Also
        --------
        ..

        Examples
        --------
        >> cm = Telegraf(verbose=False)

        """

        self._name = "Telegraf"
        self._description = "The telegraf handles the collection of data and metrics from the managed system to be used within the RoboSapiens Adaptive Platform."
        self._gatewayMatrix = [{"name" : "property1","min" : 0.0,"max" : 0.0,"type":"property"}]

    @property
    def gatewayMatrix(self):
        """The telegraf gateway matrix component (read-only)."""
        return self._gatewayMatrix

    @gatewayMatrix.setter
    def gatewayMatrix(self,value):
        self._gatewayMatrix = value
    # ------------------------------------------------------------------------------------------------
    # -------------------------------------INTERFACE FUNCTIONS----------------------------------------
    # ------------------------------------------------------------------------------------------------
    def digest(self,name,value):
        found = False
        for route in self._gatewayMatrix:
            if route["name"] == name:
                if route["type"]==knowledgeType.PROPERTY:  #TODO: extend to support other message types
                    found=True
                    poi = Proptery()
                    poi.name = name
                    poi.value = value
                    poi.min = route["min"]
                    poi.max = route["max"]
                    self.knowledge.registerKnowledge(poi)
                    self.logger.log("["+self._name+"] - "+"Data digested: " + poi.name + " with value:" + poi.value.__str__()+" - type:"+route["type"])

                else:   #if not a property, a class is passed as value
                    found = True
                    self.knowledge.registerKnowledge(value)
                    self.logger.log("[" + self._name + "] - " + "Data digested: " + name + " passed as class - type:" +route["type"])

        if not found:
            self.logger.log("["+self._name+"] - "+"Data digested FAILED: property "+ name+" not in telegraf gateway matrix")

    # ------------------------------------------------------------------------------------------------
    # -------------------------------------INTERNAL FUNCTIONS----------------------------------------
    # ------------------------------------------------------------------------------------------------
    def _SpinOnceFcn(self, args=None):
        if self._verbose: print("Spin once function call not implemented - CAN BE USED FOR BATCH WRITING")
        # 1. FETCH DATA FROM MANAGED SYSTEM

        # 2. COMPOSE KNOWLEDGE

        # 3. WRITE KNOWLEDGE TO KNOWLEDGE BASE VIA KNOWLEDGE MANAGEMENT


