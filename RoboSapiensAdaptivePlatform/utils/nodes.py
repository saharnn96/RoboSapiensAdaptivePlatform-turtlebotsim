# **********************************************************************************
# * Copyright (C) 2024-present Bert Van Acker (B.MKR) <bert.vanacker@uantwerpen.be>
# *
# * This file is part of the roboarch R&D project.
# *
# * RAP R&D concepts can not be copied and/or distributed without the express
# * permission of Bert Van Acker
# **********************************************************************************
import os
import logging
import yaml

from RoboSapiensAdaptivePlatform.utils.constants import *
from RoboSapiensAdaptivePlatform.Communication.Messages.messages import ComponentStatus
from RoboSapiensAdaptivePlatform.utils.timer import perpetualTimer, StoppableThread
import threading


class Node(object):

    def __init__(self, logger = None,verbose=False):
        """Initialize the generic Node component.

				Parameters
				----------
				verbose : bool
					component verbose execution

		See Also
		--------
		..

		Examples
		--------
		>> node = Node(verbose=False)

		"""

        self._name = "Generic Node"
        self._description = "Generic node used within RoboSapiens Adaptive Platform."
        self._verbose = verbose
        self._state = genericNodeStates.INSTANTIATED
        self._RaPlogger = logger

    @property
    def name(self):
        """The name property (read-only)."""
        return self._name

    @property
    def description(self):
        """The description property (read-only)."""
        return self._description

    @property
    def state(self):
        """The state property (read-only)."""
        return self._state

    @property
    def logger(self):
        """The logger component (read-only)."""
        return self._RaPlogger

    # ------------------------------------------------------------------------------------------------
    # -------------------------------------INTERFACE FUNCTIONS----------------------------------------
    # ------------------------------------------------------------------------------------------------
    def RaPEnterInitializationMode(self):
        """Function to enter the initialization mode of a genericNode.

			:return: `True` if enter initialization mode was successful, `False` otherwise
			:rtype: bool
			"""
        try:
            self._EnterInitializationModeFcn()
            self._state = genericNodeStates.INITIALIZATION_MODE
            return True
        except:
            return False

    def RaPExitInitializationMode(self):
        """Function to exit the initialization mode of a genericNode.

			:return: `True` if exit initialization mode was successful, `False` otherwise
			:rtype: bool
			"""
        try:
            self._ExitInitializationModeFcn()
            self._state = genericNodeStates.INITIALIZED
            return True
        except:
            return False

    def RaPEnterConfigurationMode(self):
        """Function to enter the configuration mode of a genericNode.

			:return: `True` if enter configuration mode was successful, `False` otherwise
			:rtype: bool
			"""
        try:
            self._state = genericNodeStates.CONFIGURATION_MODE
            self._EnterConfigurationModeFcn()
            return True
        except:
            self._state = genericNodeStates.INSTANTIATED
            return False

    def RaPExitConfigurationMode(self):
        """Function to exit the configuration mode of a genericNode.

		:return: `True` if exit configuration mode was successful, `False` otherwise
		:rtype: bool
		"""
        try:
            self._ExitConfigurationModeFcn()
            self._state = genericNodeStates.INSTANTIATED
            return True
        except:
            return False

    def RaPTerminate(self):
        """Function to terminate a genericNode.

			:return: `True` if node termination was successful, `False` otherwise
			:rtype: bool
			"""
        try:
            self._TerminateFcn()
            self._state = genericNodeStates.TERMINATED
            return True
        except:
            return False

    def RaPFreeInstance(self):
        """Function to terminate a genericNode.
			"""
        try:
            self._DestroyFcn()
            self._state = genericNodeStates.TERMINATED
            return True
        except:
            return False

    def RaPReset(self):
        """Function to reset a genericNode.
			"""
        try:
            self._ResetFcn()
            self._state = genericNodeStates.INSTANTIATED
            return True
        except:
            return False

    # ------------------------------------------------------------------------------------------------
    # -------------------------------------INTERNAL FUNCTIONS----------------------------------------
    # ------------------------------------------------------------------------------------------------
    def _EnterInitializationModeFcn(self):
        if self._verbose: print("Enter initialization mode function call not implemented")

    def _ExitInitializationModeFcn(self):
        if self._verbose: print("Exit initialization mode function call not implemented")

    def _EnterConfigurationModeFcn(self):
        if self._verbose: print("Enter configuration mode function call not implemented")

    def _ExitConfigurationModeFcn(self):
        if self._verbose: print("Exit configuration mode function call not implemented")

    def _TerminateFcn(self):
        if self._verbose: print("Terminate function call not implemented")

    def _DestroyFcn(self):
        if self._verbose: print("Destroy function call not implemented")

    def _ResetFcn(self):
        if self._verbose: print("Reset function call not implemented")


class TriggeredNode(Node):

    def __init__(self, logger = None,knowledge=None,verbose=False):
        super().__init__(verbose=verbose)
        """Initialize the generic adaptation node component.

        Parameters
        ----------
        logger : RaP logger
            Logger component used within the RaP
            
        knowledge : RaP knowledge management
            knowledge management component used within the RaP
            
        verbose : bool
            component verbose execution

		See Also
		--------
		..

		Examples
		--------
		>> node = Node(verbose=False)

		"""

        self._name = "Generic adaptation Node"
        self._description = "Generic adaptation node used within RoboSapiens Adaptive Platform."
        self._RaPlogger = logger
        self._kb = knowledge

    @property
    def logger(self):
        """The logger component (read-only)."""
        return self._RaPlogger

    @property
    def knowledge(self):
        """The knowledge component (read-only)."""
        return self._kb

    # ------------------------------------------------------------------------------------------------
    # -------------------------------------INTERFACE FUNCTIONS----------------------------------------
    # ------------------------------------------------------------------------------------------------
    def RaPSpin_once(self, args=None):
        """Function to enter the running mode of a adaptation node.

		:param args: generic arguments that can be passed to the spin_once method
        :type args: list

		:return: `True` if enter running mode was successful, `False` otherwise
		:rtype: bool
		"""
        if self._state == genericNodeStates.IDLE or self._state == genericNodeStates.INITIALIZED:
            try:
                self._state = genericNodeStates.RUNNING     #TODO Sahar: Push to the state management, state management stores state in list (per component) with timestamp
                _status = self._SpinOnceFcn(args=args)
                self._state = genericNodeStates.IDLE        #TODO Sahar: Push to the state management, state management stores state in list (per component) with timestamp
                return _status
            except Exception as e:
                if self._verbose:print('Faillure reason: %s', repr(e))
                self._state = genericNodeStates.IDLE
                return False
        else:
            self._state = genericNodeStates.IDLE
            if self._verbose: print("ERROR - adaptation node not initialized")
            return False

    def RaPSignalStatus(self,component,status,accuracy):
        _status = ComponentStatus()
        _status.component = component
        _status.status = status
        _status.accuracy = accuracy
        self.knowledge.write(_status)

    # ------------------------------------------------------------------------------------------------
    # -------------------------------------INTERNAL FUNCTIONS----------------------------------------
    # ------------------------------------------------------------------------------------------------
    def _SpinOnceFcn(self, args):
        if self._verbose: print("Spin once function call not implemented")


class OrchestratorNode(TriggeredNode):

    def __init__(self,dt=0.5,M=None,A=None,P=None,L=None,E=None, logger=None, knowledge=None, verbose=False):
        super().__init__(logger=logger,knowledge=knowledge,verbose=verbose)
        """Initialize the orchestrator node component.

        Parameters
        ----------
        logger : RaP logger
            Logger component used within the RaP

        knowledge : RaP knowledge management
            knowledge management component used within the RaP

        verbose : bool
            component verbose execution

		See Also
		--------
		..

		Examples
		--------
		>> node = Node(verbose=False)

		"""

        self._name = "Orchestrator Node"
        self._description = "Generic orchestrator node for execution of the MAPLE"
        self._RaPlogger = logger
        self._kb = knowledge
        self._orchestrationStatus = orchestrationStatus.IDLE

        # MAPLE elements
        self._monitor = M
        self._analysis = A
        self._plan = P
        self._legitimize = L
        self._execute = E

        # --- periodic monitoring ---
        self._dt = dt
        self._MonitorExecutor = perpetualTimer(self._dt, self.monitor.RaPSpin_once)
        # --- set the orchestration function ---
        self.orchestrationExecutor = StoppableThread()
        self.orchestrationExecutor._runFcn = self._OrchestrationLoop

    @property
    def logger(self):
        """The logger component (read-only)."""
        return self._RaPlogger

    @property
    def knowledge(self):
        """The knowledge component (read-only)."""
        return self._kb

    @property
    def monitor(self):
        """The monitor class (read-only)."""
        return self._monitor

    @monitor.setter
    def monitor(self, cmp):
        """The monitor class (write)."""
        self._monitor = cmp

    @property
    def analysis(self):
        """The communication management class (read-only)."""
        return self._analysis

    @analysis.setter
    def analysis(self, cmp):
        """The analysis class (write)."""
        self._analysis = cmp

    @property
    def plan(self):
        """The communication management class (read-only)."""
        return self._plan

    @plan.setter
    def plan(self, cmp):
        """The plan class (write)."""
        self._plan = cmp

    @property
    def legitimize(self):
        """The legitimize class (read-only)."""
        return self._legitimize

    @legitimize.setter
    def legitimize(self, cmp):
        """The legitimize class (write)."""
        self._legitimize = cmp

    @property
    def execute(self):
        """The execute class (read-only)."""
        return self._execute

    @execute.setter
    def execute(self, cmp):
        """The execute class (write)."""
        self._execute = cmp

    @property
    def orchestrationFunction(self):
        """The orchestration function (read-only)."""
        return self._OrchestrationFcn

    @orchestrationFunction.setter
    def orchestrationFunction(self, fn):
        """The orchestration function (write)."""
        self._OrchestrationFcn = fn

    # ------------------------------------------------------------------------------------------------
    # -------------------------------------INTERFACE FUNCTIONS----------------------------------------
    # ------------------------------------------------------------------------------------------------
    def RaPStartOrchestration(self):
        """Function to start the orchestration.

            :param args: generic arguments that can be passed to the spin_once method
            :type args: list

            :return: `True` if enter running mode was successful, `False` otherwise
            :rtype: bool
            """
        try:
            self.orchestrationExecutor.run()
        except:
            self.logger.log("Unable to start orchestration")

    def RaPStopOrchestration(self):
        """Function to stop the orchestration.

            :param args: generic arguments that can be passed to the spin_once method
            :type args: list

            :return: `True` if enter running mode was successful, `False` otherwise
            :rtype: bool
            """
        self.orchestrationExecutor.stop()

    def RaPSpin_once(self, args=None):
        """Function to execute a single orchestration loop.

		:param args: generic arguments that can be passed to the spin_once method
        :type args: list

		:return: `True` if enter running mode was successful, `False` otherwise
		:rtype: bool
		"""
        if self._state == genericNodeStates.IDLE or self._state == genericNodeStates.INITIALIZED:
            try:
                self._state = genericNodeStates.RUNNING
                _status = self._OrchestrationFcn(args=args)
                self._state = genericNodeStates.IDLE
                return _status
            except:
                return False
        else:
            if self._verbose: print("ERROR - orchestration node not initialized")
            return False
    # ------------------------------------------------------------------------------------------------
    # -------------------------------------INTERNAL FUNCTIONS----------------------------------------
    # ------------------------------------------------------------------------------------------------
    def _OrchestrationLoop(self):

        # --- SEQUENTIAL MAPLE EXECUTION
        if self._orchestrationStatus == orchestrationStatus.IDLE:
            # 1. start the periodic monitoring
            self._orchestrationStatus = orchestrationStatus.MONITORING
            self._MonitorExecutor.start()


        elif self._orchestrationStatus == orchestrationStatus.MONITORING:
            # 1. check status of monitor
            status_M, history_M = self.knowledge.read(name=adaptivityComponents.MONITOR, queueSize=1)
            if status_M.status == monitorStatus.ANOMALY:
                # proceed with analyzing if anomaly is detected
                self._orchestrationStatus = orchestrationStatus.ANALYZING

        if self._orchestrationStatus == orchestrationStatus.ANALYZING:
            # 1. execute analysis
            status = self.analysis.RaPSpin_once()
            if status:
                status_A, history_A = self.knowledge.read(name=adaptivityComponents.ANALYSIS, queueSize=1)
                if status_A.status == analysisStatus.ANOMALY:
                    # proceed with plan if anomaly is analyzed
                    self._orchestrationStatus = orchestrationStatus.PLANNING

        if self._orchestrationStatus == orchestrationStatus.PLANNING:
            # 1. execute plan
            status = self.plan.RaPSpin_once()
            if status:
                status_P, history_P = self.knowledge.read(name=adaptivityComponents.PLAN, queueSize=1)
                if status_P.status == planStatus.PLANNED:
                    # proceed with legitimize if plan is determined
                    self._orchestrationStatus = orchestrationStatus.LEGITIMIZING

        if self._orchestrationStatus == orchestrationStatus.LEGITIMIZING:
            # 1. execute plan
            status = self.legitimize.RaPSpin_once()
            if status:
                status_L, history_L = self.knowledge.read(name=adaptivityComponents.LEGITIMIZE, queueSize=1)
                if status_L.status != legitimizeStatus.IDLE:
                    # check if plan is valid and proceed to execute
                    if status_L.status == legitimizeStatus.VALID:
                        self._orchestrationStatus = orchestrationStatus.EXECUTING
                    else:
                        self.logger.log(
                            "Plan invalidated by the legitimize component, plan dropped")  # TODO: what to do when not valid, replan?
                        self._orchestrationStatus = orchestrationStatus.MONITORING

        if self._orchestrationStatus == orchestrationStatus.EXECUTING:
            # 1. execute
            status = self.execute.RaPSpin_once()
            if status:
                status_E, history_E = self.knowledge.read(name=adaptivityComponents.EXECUTE, queueSize=1)
                if status_E.status == executeStatus.EXECUTION:
                    # proceed for the action complete
                    self._orchestrationStatus = orchestrationStatus.WAITING_FOR_ACTION_COMPLETE

        if self._orchestrationStatus == orchestrationStatus.WAITING_FOR_ACTION_COMPLETE:
            # TODO: ADAPTATION MANAGEMENT NEEDS TO RELEASE FROM THIS STATE??
            # PROPOSAL: RETURN TO MONITORING WHEN ANOMALY IS GONE
            status_M, history_M = self.knowledge.read(name=adaptivityComponents.MONITOR, queueSize=1)
            if status_M.status == monitorStatus.NORMAL:
                self._orchestrationStatus = orchestrationStatus.MONITORING

