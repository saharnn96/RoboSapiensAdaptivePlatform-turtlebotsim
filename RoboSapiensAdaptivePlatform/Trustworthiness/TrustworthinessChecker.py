#**********************************************************************************
# * Copyright (C) 2024-present Bert Van Acker (B.MKR) <bert.vanacker@uantwerpen.be>
# *
# * This file is part of the roboarch R&D project.
# *
# * RAP R&D concepts can not be copied and/or distributed without the express
# * permission of Bert Van Acker
# **********************************************************************************
from RoboSapiensAdaptivePlatform.utils.constants import *
from RoboSapiensAdaptivePlatform.utils.nodes import TriggeredNode

class TrustworthinessChecker(TriggeredNode):

    def __init__(self,adaptationExecutor=None,trustworthinessExecutor=None,effector=None,logger = None,knowledgeManagement = None,verbose=False):
        super().__init__(logger=logger, knowledge=knowledgeManagement, verbose=verbose)
        """Initialize the generic TrustworthinessChecker component.

                Parameters
                ----------
                verbose : bool
                    component verbose execution

        See Also
        --------
        ..

        Examples
        --------
        >> cm = TrustworthinessChecker(adaptationExecutor=None,trustworthinessExecutor=None,verbose=False)

        """

        self._name = "Trustworthiness Checker"
        self._description = "The trusworthiness checker is the collection of components that handles the trustworthiness of the execution of the adaptations and/or diagnosis routines on the managed system, triggered from within the RoboSapiens Adaptive Platform."
        self._verbose = verbose
        self._state = genericStates.STARTUP

        # --- components of trusworthiness checker ---
        self._adaptationExecutor = adaptationExecutor
        self._trustworthinessExecutor =trustworthinessExecutor
        self._adaptationExecutor.effector = effector

        # --- action to-be-executed
        self._action = None
        self._isTrusworthy = False
        self._actionStatus = actionStatus.IDLE


    @property
    def name(self):
        """The name property (read-only)."""
        return self._name

    @property
    def description(self):
        """The description property (read-only)."""
        return self._description

    @property
    def effector(self):
        """The effector class (read-only)."""
        return self._adaptationExecutor.effector

    @effector.setter
    def effector(self, cmp):
        """The effector class (write)."""
        self._adaptationExecutor.effector = cmp

    @property
    def trustworthinessExecutor(self):
        """The trustworthiness executor class (read-only)."""
        return self._trustworthinessExecutor

    @trustworthinessExecutor.setter
    def trustworthinessExecutor(self, cmp):
        """The _trustworthiness executor class (write)."""
        self._trustworthinessExecutor = cmp
    #------------------------------------------------------------------------------------------------
    # -------------------------------------INTERFACE FUNCTIONS---------------------------------------
    # ------------------------------------------------------------------------------------------------
    def RegisterAction(self,action):
        self.logger.log("["+self._name+"] - "+"Action registered")
        self._actionStatus = actionStatus.REGISTERED
        self._isTrusworthy = False

        # --- TODO: CHANGE WITH YAKINDU STATEMACHINE ---

        # PERFORM TRUSTWORTHINESS ON THE REGISTERED ACTION
        self._isTrusworthy = self._trustworthinessExecutor.checkTrustworthiness(action)

        # PERFORM TRUSTED ACTION
        if self._isTrusworthy:
            self.logger.log("["+self._name+"] - "+"Action "+action.description+" is trustworthy")
            self._actionStatus = actionStatus.TRUSTED

            # SET ACTION TO BE PERFORMED BY THE ADAPTATION EXECUTOR
            self._actionStatus = actionStatus.EXECUTING
            self._actionStatus = self._adaptationExecutor.execute(action)

        else:
            self.logger.log("["+self._name+"] - "+"Action " + action.description + " is not trustworthy, dropping the action")
            self._action = None
            self._isTrusworthy = False
            self._actionStatus = actionStatus.DROPPED

        return self._actionStatus
