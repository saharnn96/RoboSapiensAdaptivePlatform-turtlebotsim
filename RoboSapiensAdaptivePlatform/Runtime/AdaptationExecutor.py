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

class AdaptationExecutor(TriggeredNode):

    def __init__(self, logger=None,knowledgeManagement=None, verbose=False):
        super().__init__(logger=logger, knowledge=knowledgeManagement, verbose=verbose)
        """Initialize the generic AdaptationExecutor component.

                Parameters
                ----------
                verbose : bool
                    component verbose execution

        See Also
        --------
        ..

        Examples
        --------
        >> cm = AdaptationExecutor(verbose=False)

        """

        self._name = "Adaptation Executor"
        self._description = "The adaptation executor handles the execution of the determined adaptation of the managed system, triggered from within the RoboSapiens Adaptive Platform."
        self._verbose = verbose
        self._state = genericStates.STARTUP
        self._effector = None

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
        return self._effector

    @effector.setter
    def effector(self, cmp):
        """The effector class (write)."""
        self._effector = cmp


    #------------------------------------------------------------------------------------------------
    # -------------------------------------INTERFACE FUNCTIONS---------------------------------------
    # ------------------------------------------------------------------------------------------------
    def execute(self, action):
        status = -1
        self.logger.log("["+self._name+"] - "+"Execution routine for action "+action.description)

        # PERFORM PRE-ACTIONS

        # SEND ACTION TO EFFECTOR
        try:
            self._effector.RaPSpin_once(action)
            status = actionStatus.FINISHED
        except:
            self.logger.log("["+self._name+"] - "+"Failed to execute the action on managed system effector")
            status = actionStatus.DROPPED


        # PERFORM POST-ACTIONS, e.g. startup

        return status