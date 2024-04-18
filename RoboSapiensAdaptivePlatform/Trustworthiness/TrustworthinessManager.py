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

class TrustworthinessManager(Node):

    def __init__(self,logger=None,knowledgeManagement = None,adaptationManagement = None, trustworthinessExecutor= None,verbose=False):
        super().__init__(logger=logger, verbose=verbose)
        """Initialize the generic TrustworthinessManager component.

                Parameters
                ----------
                verbose : bool
                    component verbose execution

        See Also
        --------
        ..

        Examples
        --------
        >> cm = TrustworthinessManager(verbose=False)

        """

        self._name = "Trustworthiness Manager"
        self._description = "The trusworthiness manager handles the trustworthiness of the MAPVE-K execution and data within the RoboSapiens Adaptive Platform."
        self._verbose = verbose
        self._state = genericStates.STARTUP

        self._trustworthinessExecutor = trustworthinessExecutor
        self._adaptationManagement = adaptationManagement
        self._knowledgeManagement = knowledgeManagement

    @property
    def name(self):
        """The name property (read-only)."""
        return self._name

    @property
    def description(self):
        """The description property (read-only)."""
        return self._description

    @property
    def knowledgeManagement(self):
        """The knowledge management node (read-only)."""
        return self._knowledgeManagement

    @property
    def adaptationManagement(self):
        """The adaptation management node (read-only)."""
        return self._adaptationManagement

    @property
    def trustworthinessExecutor(self):
        """The trustworthiness executor (read-only)."""
        return self._trustworthinessExecutor

    @trustworthinessExecutor.setter
    def trustworthinessExecutor(self,cmp):
        """The trustworthiness executor (write)."""
        self._trustworthinessExecutor = cmp

    #------------------------------------------------------------------------------------------------
    # -------------------------------------INTERFACE FUNCTIONS---------------------------------------
    # ------------------------------------------------------------------------------------------------
    def checkTrustworthiness(self, action):
        self.logger.log("["+self._name+"] - "+"Checking trustworthiness of action "+action.description)

        # TODO: IMPLEMENT TRUSTWORTHINESS CHECK

        return True

    # ------------------------------------------------------------------------------------------------
    # -------------------------------------INTERNAL FUNCTIONS----------------------------------------
    # ------------------------------------------------------------------------------------------------
    def _EnterInitializationModeFcn(self):
        if self._verbose: print("Enter initializationModeFcn not implemented")

    def _ExitInitializationModeFcn(self):
        if self._verbose: print("Exit initializationModeFcn not implemented")

    def _EnterConfigurationModeFcn(self):
        if self._verbose: print("Enter configurationModeFcn not implemented")