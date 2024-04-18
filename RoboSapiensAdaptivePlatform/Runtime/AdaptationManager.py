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

class AdaptationManager(Node):

    def __init__(self,logger = None,knowledgeManagement = None, trustworthinessManagement = None, trusworthinessChecker = None,adaptationOrchestrator = None, verbose=False):
        super().__init__(logger=logger, verbose=verbose)
        """Initialize the generic AdaptationManager component.

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

        self._name = "Adaptation Manager"
        self._description = "The adaptation manager handles the execution of MAPVE-K and resulting adaptations within the RoboSapiens Adaptive Platform."
        self._verbose = verbose
        self._state = genericStates.STARTUP

        # RaP logger component
        self._RaPlogger = logger

        # knowledge Management
        self._knowledgeManagement = knowledgeManagement

        # trustworthiness Management and Checker
        self._trustworthinessManagement = trustworthinessManagement
        self._trustworthinessChecker = trusworthinessChecker

        # adaptation orchestrator
        self._adaptationOrchestrator = adaptationOrchestrator

    @property
    def name(self):
        """The name property (read-only)."""
        return self._name

    @property
    def description(self):
        """The description property (read-only)."""
        return self._description

    @property
    def trustworthinessChecker(self):
        """The trustworthiness checker (read-only)."""
        return self._trustworthinessChecker

    @property
    def trustworthinessManagement(self):
        """The trustworthiness management node (read-only)."""
        return self._trustworthinessManagement

    @trustworthinessManagement.setter
    def trustworthinessManagement(self,cmp):
        """The trustworthiness management node (write)."""
        self._trustworthinessManagement = cmp
    #------------------------------------------------------------------------------------------------
    # -------------------------------------INTERFACE FUNCTIONS---------------------------------------
    # ------------------------------------------------------------------------------------------------
    def performDiagnosis(self,action):
        """Function to execute a diagnosis action."""
        self.logger.log("["+self._name+"] - "+"Diagnosis action " + action.description + " requested to be executed on the managed system")

        # --- TODO: CHANGE WITH YAKINDU STATEMACHINE ---


        # PERFORM TRUSTWORTHINESS ON THE ACTION
        self._isTrusworthy = self._trustworthinessManagement.checkTrustworthiness(action)

        # PERFORM TRUSTED ACTION
        if self._isTrusworthy:
            self.logger.log("["+self._name+"] - "+"Action " + action.description + " is trustworthy")
            self._actionStatus = actionStatus.TRUSTED

            # REGISTER THE DIANOSIS ACTION
            self._actionStatus = actionStatus.REGISTERED
            self._actionStatus = self.trustworthinessChecker.RegisterAction(action)

        else:
            self.logger.log("["+self._name+"] - "+"Action " + action.description + " is not trustworthy, not registered to the trustworthiness checker!")
            self._action = None
            self._isTrusworthy = False
            self._actionStatus = actionStatus.DROPPED

        return self._actionStatus

    def performAdaptation(self,action):
        """Function to execute a diagnosis action."""
        self.logger.log("["+self._name+"] - "+"Adaptation action " + action.description + " requested to be executed on the managed system")

        # --- TODO: CHANGE WITH YAKINDU STATEMACHINE ---

        # PERFORM TRUSTWORTHINESS ON THE ACTION
        self._isTrusworthy = self._trustworthinessManagement.checkTrustworthiness(action)

        # PERFORM TRUSTED ACTION
        if self._isTrusworthy:
            self.logger.log("["+self._name+"] - "+"Action " + action.description + " is trustworthy")
            self._actionStatus = actionStatus.TRUSTED

            # REGISTER THE DIANOSIS ACTION
            self._actionStatus = actionStatus.REGISTERED
            self._actionStatus = self.trustworthinessChecker.RegisterAction(action)

        else:
            self.logger.log("["+self._name+"] - "+"Action " + action.description + " is not trustworthy, not registered to the trustworthiness checker!")
            self._action = None
            self._isTrusworthy = False
            self._actionStatus = actionStatus.DROPPED

        return self._actionStatus


    # ------------------------------------------------------------------------------------------------
    # -------------------------------------INTERNAL FUNCTIONS----------------------------------------
    # ------------------------------------------------------------------------------------------------
    def _EnterInitializationModeFcn(self):
        if self._verbose: print("Enter initializationModeFcn not implemented")

    def _ExitInitializationModeFcn(self):
        if self._verbose: print("Exit initializationModeFcn not implemented")

    def _EnterConfigurationModeFcn(self):
        if self._verbose: print("Enter configurationModeFcn not implemented")