#**********************************************************************************
# * Copyright (C) 2024-present Bert Van Acker (B.MKR) <bert.vanacker@uantwerpen.be>
# *
# * This file is part of the roboarch R&D project.
# *
# * RAP R&D concepts can not be copied and/or distributed without the express
# * permission of Bert Van Acker
# **********************************************************************************
from RoboSapiensAdaptivePlatform.utils.constants import *
from RoboSapiensAdaptivePlatform.utils.nodes import Node
from RoboSapiensAdaptivePlatform.Knowledge.KnowledgeBase import KnowledgeBase

class KnowledgeManager(Node):

    def __init__(self,logger = None,verbose=False):
        super().__init__(logger=logger,verbose=verbose)
        """Initialize the generic KnowledgeManager component.

                Parameters
                ----------
                verbose : bool
                    component verbose execution

        See Also
        --------
        ..

        Examples
        --------
        >> cm = KnowledgeManager(verbose=False)

        """

        self._name = "Knowledge Manager"
        self._description = "The knowledge manager handles the (cross-device) knowledge within the RoboSapiens Adaptive Platform."
        self._verbose = verbose

        #Knowledge base is application-specific, set by the adaptivity application
        self._knowledgeBase = None


    @property
    def knowledgeBase(self):
        return self._knowledgeBase

    @knowledgeBase.setter
    def knowledgeBase(self,value):
        self._knowledgeBase = value
    #------------------------------------------------------------------------------------------------
    # -------------------------------------INTERFACE FUNCTIONS---------------------------------------
    # ------------------------------------------------------------------------------------------------
    def registerKnowledge(self, cls):
        """Function to write knowledge to the knowlegde base."""

        #TODO: ADD TRUSTWORTHINESS/SANITY CHECKING BEFORE WRITING

        # self.logger.log("[" + self._name + "] - " + "Knowledge registered: " + cls.name )
        self._knowledgeBase.write(cls)

    def read(self,name='TBD',queueSize=10):
        """Function to read knowledge from the knowlegde base."""
        cls,historyQueue = self._knowledgeBase.read(name=name, queueSize=queueSize)
        self.logger.log("[" + self._name + "] - " + "Knowledge read: " + cls.name)
        return cls,historyQueue


    # ------------------------------------------------------------------------------------------------
    # -------------------------------------INTERNAL FUNCTIONS----------------------------------------
    # ------------------------------------------------------------------------------------------------
    def _EnterInitializationModeFcn(self):
        if self._verbose: print("Enter initializationModeFcn not implemented")

    def _ExitInitializationModeFcn(self):
        if self._verbose: print("Exit initializationModeFcn not implemented")

    def _EnterConfigurationModeFcn(self):
        if self._verbose: print("Enter configurationModeFcn not implemented")

