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
from RoboSapiensAdaptivePlatform.Runtime.Logger import Logger
from RoboSapiensAdaptivePlatform.Communication.CommunicationManager import CommunicationManager
from RoboSapiensAdaptivePlatform.Communication.CommunicationEndpoint import CommunicationEndpoint
from RoboSapiensAdaptivePlatform.Knowledge.KnowledgeManager import KnowledgeManager
from RoboSapiensAdaptivePlatform.utils.timer import perpetualTimer
from RoboSapiensAdaptivePlatform.Trustworthiness.TrustworthinessChecker import TrustworthinessChecker
from RoboSapiensAdaptivePlatform.Runtime.AdaptationExecutor import AdaptationExecutor
from RoboSapiensAdaptivePlatform.Trustworthiness.TrustworthinessExecutor import TrustworthinessExecutor
from RoboSapiensAdaptivePlatform.Trustworthiness.TrustworthinessManager import TrustworthinessManager
from RoboSapiensAdaptivePlatform.Runtime.AdaptationManager import AdaptationManager
from RoboSapiensAdaptivePlatform.ManagedSystem.Effector import Effector
from RoboSapiensAdaptivePlatform.ManagedSystem.Probe import Probe
from RoboSapiensAdaptivePlatform.Communication.Messages.messages import *
import json

class Core(Node):

    def __init__(self,mode = comMode.MAPE,verbose=False):
        super().__init__(verbose=verbose)
        """Initialize the Core component.

                Parameters
                ----------
                mode: enum
                    Communication mode (application|MAPE)
                    
                verbose : bool
                    component verbose execution

        See Also
        --------
        ..

        Examples
        --------
        >> core = Core(verbose=False)

        """

        self._name = "Core"
        self._description = "The core handles the (de-)initialization of all the RoboSapiens Adaptive Platform components as well as the abort handling."
        self._mode = mode
        self._verbose = verbose

        # --- RoboSapiens Adaptive Platform building blocks ---
        self._updateRate = 0.1

        # --- LOGGER instantiated at startup of node ---
        self._logger = Logger(config="input/config.yaml", verbose=True)
        self._logger.RaPEnterConfigurationMode()
        self._logger.RaPExitConfigurationMode()
        self._logger.RaPEnterInitializationMode()
        self._logger.RaPExitInitializationMode()

        # --- COMMUNICATION ---
        self._com = CommunicationManager(mode=self._mode,config="input/config.yaml",communicationProtocol=communicationProtocol.MQTT,logger=self._logger,verbose=False)
        self._comExecutor = None
        # --- KNOWLEDGE ---
        self._km = KnowledgeManager(logger=self._logger,verbose=self._verbose)

    #------------------------------------------------------------------------------------------------
    #-------------------------------------INTERFACE FUNCTIONS---------------------------------------
    #------------------------------------------------------------------------------------------------
    @property
    def logger(self):
        """The logger class (read-only)."""
        return self._logger

    @property
    def communicationManagement(self):
        """The communication management class (read-only)."""
        return self._com

    @property
    def knowledgeManagement(self):
        """The knowledge management class (read-only)."""
        return self._km

    # ------------------------------------------------------------------------------------------------
    # -------------------------------------INTERNAL FUNCTIONS----------------------------------------
    # ------------------------------------------------------------------------------------------------
    def _dispatch(self):
        """Function to dispatch the remote commands."""
        messages = self.communicationManagement.getQueuedMessages()
        if messages != -1:
            for message in messages:
                for subscription in self.communicationManagement._subscriptionList:
                    if subscription["topic"] == message.topic:
                        if subscription["class"]== "Command":
                            print("TODO: implement command dispatcher")
                        elif subscription["class"] == "Property":
                            print("TODO: Trigger set function of knowledge")

                #TODO: add the routings depending on the received message (command/property/...)



    def _EnterInitializationModeFcn(self):


        # --- COMMUNICATION MANAGEMENT ---
        self.communicationManagement.RaPEnterInitializationMode()
        self.communicationManagement.RaPExitInitializationMode()
        self._comExecutor.start()

        # --- KNOWLEDGE MANAGEMENT ---
        self.knowledgeManagement.RaPEnterInitializationMode()
        self.knowledgeManagement.RaPExitInitializationMode()



    def _EnterConfigurationModeFcn(self):


        # --- COMMUNICATION MANAGEMENT ---
        self.communicationManagement.RaPEnterConfigurationMode()
        self.communicationManagement.RaPExitConfigurationMode()
        self._comExecutor = perpetualTimer(self._updateRate, self._dispatch)

        # --- KNOWLEDGE MANAGEMENT ---
        self.knowledgeManagement.RaPEnterConfigurationMode()
        self.knowledgeManagement.RaPExitConfigurationMode()


class localCore(Node):

    def __init__(self, verbose=False):
        super().__init__(verbose=verbose)
        """Initialize the local core component.

                Parameters
                ----------

                verbose : bool
                    component verbose execution

        See Also
        --------
        ..

        Examples
        --------
        >> core = localCore(verbose=False)

        """

        self._name = "Local core"
        self._description = "The local core handles the (de-)initialization of all the RoboSapiens Adaptive Platform components as well as the abort handling."
        self._verbose = verbose


        # --- LOGGER instantiated at startup of node ---
        self._logger = Logger(config="00_input/config.yaml", verbose=verbose)
        self._logger.RaPEnterConfigurationMode()
        self._logger.RaPExitConfigurationMode()
        self._logger.RaPEnterInitializationMode()
        self._logger.RaPExitInitializationMode()

        # --- COMMUNICATION ---
        self._com = None    # NO COMMUNICATION NODE!
        self._comExecutor = None

        # --- KNOWLEDGE ---
        self._km = KnowledgeManager(logger=self.logger,verbose=self._verbose)

        # --- TELEGRAF ---
        self._telegraf = None

        # --- MAPE components ---
        self._monitor = None
        self._analysis = None
        self._plan = None
        self._legitimize = None
        self._execute = None
        self._adaptationOrchestrator = None

        # --- MANAGED SYSTEM components ---
        self._effector = Effector(logger=self.logger, knowledgeBase=None, verbose=verbose)
        self._probe = Probe(telegraf=self.telegraf,verbose=verbose)

        # --- TRUSTWORTHINESS CHECKER ---
        self._adaptationExecutor = AdaptationExecutor(logger=self.logger,knowledgeManagement=self.knowledgeManagement,verbose=self._verbose)
        self._trusworthinessExecutor = TrustworthinessExecutor(logger=self.logger, knowledgeManagement=self.knowledgeManagement,verbose=self._verbose)
        self._trustworthinessChecker = TrustworthinessChecker(adaptationExecutor=self._adaptationExecutor,trustworthinessExecutor=self._trusworthinessExecutor,effector=None,logger=self.logger,knowledgeManagement=self.knowledgeManagement,verbose=self._verbose)

        # --- ADAPTATION MANAGEMENT ---
        self._adaptationManagement = AdaptationManager(logger=self.logger,knowledgeManagement=self.knowledgeManagement,trusworthinessChecker=self.trustworthinessChecker,verbose=self._verbose)

        # --- TRUSTWORTHINESS MANAGEMENT ---
        self._trustworthinessManagement = TrustworthinessManager(logger=self.logger,knowledgeManagement=self.knowledgeManagement,adaptationManagement=self.adaptationManagement,verbose=self._verbose)
        self._adaptationManagement.trustworthinessManagement = self.trustworthinessManagement

    # ------------------------------------------------------------------------------------------------
    # -------------------------------------INTERFACE FUNCTIONS---------------------------------------
    # ------------------------------------------------------------------------------------------------
    @property
    def logger(self):
        """The logger class (read-only)."""
        return self._logger

    @property
    def communicationManagement(self):
        """The communication management class (read-only)."""
        return self._com

    @property
    def knowledgeManagement(self):
        """The knowledge management class (read-only)."""
        return self._km

    @property
    def knowledgeBase(self):
        return self._km.knowledgeBase

    @knowledgeBase.setter
    def knowledgeBase(self, value):
        self._km.knowledgeBase = value

    @property
    def monitor(self):
        """The monitor class (read-only)."""
        return self._monitor

    @monitor.setter
    def monitor(self,cmp):
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
    def telegraf(self):
        """The telegraf class (read-only)."""
        return self._telegraf

    @telegraf.setter
    def telegraf(self, cmp):
        """The telegraf class (write)."""
        self._telegraf = cmp
        self.probe.telegraf = self.telegraf

    @property
    def effector(self):
        """The effector class (read-only)."""
        return self._effector

    @effector.setter
    def effector(self, cmp):
        """The effector class (write)."""
        self._effector = cmp

    @property
    def probe(self):
        """The probe class (read-only)."""
        return self._probe

    @probe.setter
    def probe(self, cmp):
        """The probe class (write)."""
        self._probe = cmp

    @property
    def trustworthinessChecker(self):
        """The trustworthinessChecker class (read-only)."""
        return self._trustworthinessChecker

    @trustworthinessChecker.setter
    def trustworthinessChecker(self, cmp):
        """The trustworthinessChecker class (write)."""
        self._trustworthinessChecker = cmp
        #TODO: CHECK IF BEST PLACE
        self.trustworthinessManagement.trustworthinessExecutor = self._trustworthinessChecker.trustworthinessExecutor

    @property
    def adaptationManagement(self):
        """The adaptation management class (read-only)."""
        return self._adaptationManagement

    @adaptationManagement.setter
    def adaptationManagement(self, cmp):
        """The adaptation management class (write)."""
        self._adaptationManagement = cmp

    @property
    def trustworthinessManagement(self):
        """The trustworthiness management class (read-only)."""
        return self._trustworthinessManagement

    @trustworthinessManagement.setter
    def trustworthinessManagement(self, cmp):
        """The trustworthiness management class (write)."""
        self._trustworthinessManagement = cmp

    @property
    def adaptationOrchestrator(self):
        """The adaptation orchestrator class (read-only)."""
        return self._adaptationOrchestrator

    @adaptationOrchestrator.setter
    def adaptationOrchestrator(self, cmp):
        """The adaptation orchestrator class (write)."""
        self._adaptationOrchestrator = cmp

    # ------------------------------------------------------------------------------------------------
    # -------------------------------------INTERNAL FUNCTIONS----------------------------------------
    # ------------------------------------------------------------------------------------------------

    def _EnterInitializationModeFcn(self):

        # --- INITIALIZE MAPE components ---
        self.monitor.RaPEnterInitializationMode()
        self.monitor.RaPExitInitializationMode()

        self.analysis.RaPEnterInitializationMode()
        self.analysis.RaPExitInitializationMode()

        self.plan.RaPEnterInitializationMode()
        self.plan.RaPExitInitializationMode()

        self.legitimize.RaPEnterInitializationMode()
        self.legitimize.RaPExitInitializationMode()

        self.execute.RaPEnterInitializationMode()
        self.execute.RaPExitInitializationMode()

        # --- INITIALIZE MANAGED SYSTEM components ---
        self.effector.RaPEnterInitializationMode()
        self.effector.RaPExitInitializationMode()

        # --- KNOWLEDGE MANAGEMENT ---
        self.knowledgeManagement.RaPEnterInitializationMode()
        self.knowledgeManagement.RaPExitInitializationMode()

        # --- TRUSTWORTHINESS CHECKER ---
        self.trustworthinessChecker.effector = self.effector
        self.trustworthinessChecker.RaPEnterInitializationMode()
        self.trustworthinessChecker.RaPExitInitializationMode()

        # --- ADAPTATION MANAGEMENT ---
        self.adaptationManagement.RaPEnterInitializationMode()
        self.adaptationManagement.RaPExitInitializationMode()

        # --- TRUSTWORTHINESS MANAGEMENT ---
        self.trustworthinessManagement.RaPEnterInitializationMode()
        self.trustworthinessManagement.RaPExitInitializationMode()

        # --- ADAPTATION ORCHESTRATOR ---
        self.adaptationOrchestrator.RaPEnterInitializationMode()
        self.adaptationOrchestrator.RaPExitInitializationMode()

    def _EnterConfigurationModeFcn(self):

        # --- CONFIGURE MAPE components ---
        self.monitor.RaPEnterConfigurationMode()
        self.monitor.RaPExitConfigurationMode()

        self.analysis.RaPEnterConfigurationMode()
        self.analysis.RaPExitConfigurationMode()

        self.plan.RaPEnterConfigurationMode()
        self.plan.RaPExitConfigurationMode()

        self.legitimize.RaPEnterConfigurationMode()
        self.legitimize.RaPExitConfigurationMode()

        self.execute.RaPEnterConfigurationMode()
        self.execute.RaPExitConfigurationMode()

        # --- CONFIGURE MANAGED SYSTEM components ---
        self.effector.RaPEnterConfigurationMode()
        self.effector.RaPExitConfigurationMode()

        # --- KNOWLEDGE MANAGEMENT ---
        self.knowledgeManagement.RaPEnterConfigurationMode()
        self.knowledgeManagement.RaPExitConfigurationMode()

        # --- TRUSTWORTHINESS CHECKER ---
        self.trustworthinessChecker.RaPEnterConfigurationMode()
        self.trustworthinessChecker.RaPExitConfigurationMode()

        # --- ADAPTATION MANAGEMENT ---
        self.adaptationManagement.RaPEnterConfigurationMode()
        self.adaptationManagement.RaPExitConfigurationMode()

        # --- TRUSTWORTHINESS MANAGEMENT ---
        self.trustworthinessManagement.RaPEnterConfigurationMode()
        self.trustworthinessManagement.RaPExitConfigurationMode()

        # --- ADAPTATION ORCHESTRATOR ---
        self.adaptationOrchestrator.RaPEnterConfigurationMode()
        self.adaptationOrchestrator.RaPExitConfigurationMode()


class remoteCore(Node):

    def __init__(self, verbose=False):
        super().__init__(verbose=verbose)
        """Initialize the remote core component.

                Parameters
                ----------

                verbose : bool
                    component verbose execution

        See Also
        --------
        ..

        Examples
        --------
        >> core = remoteCore(verbose=False)

        """

        self._name = "Remote core"
        self._description = "The remote core handles the (de-)initialization of all the RoboSapiens Adaptive Platform components as well as the abort handling."
        self._verbose = verbose


        # --- LOGGER instantiated at startup of node ---
        self._logger = Logger(config="00_input/config.yaml", verbose=verbose)
        self._logger.RaPEnterConfigurationMode()
        self._logger.RaPExitConfigurationMode()
        self._logger.RaPEnterInitializationMode()
        self._logger.RaPExitInitializationMode()

        # --- COMMUNICATION ---
        self._com = CommunicationManager(mode=comMode.MAPE,config="00_input/config.yaml",communicationProtocol=communicationProtocol.MQTT,logger=self._logger,verbose=False)    # MQTT BASED COMMUNICATION MANAGEMENT
        self._com._dispatch = self._dispatch

        # --- KNOWLEDGE ---
        self._km = KnowledgeManager(logger=self.logger,verbose=self._verbose)

        # --- TELEGRAF ---
        self._telegraf = None

        # --- MAPE components ---
        self._monitor = None
        self._analysis = None
        self._plan = None
        self._legitimize = None
        self._execute = None
        self._adaptationOrchestrator = None

        # --- MANAGED SYSTEM components via client library, see adaptiveTurtlebot4 ---
        self._effector = CommunicationEndpoint(logger=self.logger, knowledgeBase=None, communicationManagement=self.communicationManagement, verbose=verbose) #EFFECTOR IS NOW A COMMUNICATION ENDPOINT, SENDING TO COMMUNICATION MANAGEMENT

        # --- TRUSTWORTHINESS CHECKER ---
        self._adaptationExecutor = AdaptationExecutor(logger=self.logger,knowledgeManagement=self.knowledgeManagement,verbose=self._verbose)
        self._trusworthinessExecutor = TrustworthinessExecutor(logger=self.logger, knowledgeManagement=self.knowledgeManagement,verbose=self._verbose)
        self._trustworthinessChecker = TrustworthinessChecker(adaptationExecutor=self._adaptationExecutor,trustworthinessExecutor=self._trusworthinessExecutor,effector=None,logger=self.logger,knowledgeManagement=self.knowledgeManagement,verbose=self._verbose)

        # --- ADAPTATION MANAGEMENT ---
        self._adaptationManagement = AdaptationManager(logger=self.logger,knowledgeManagement=self.knowledgeManagement,trusworthinessChecker=self.trustworthinessChecker,verbose=self._verbose)

        # --- TRUSTWORTHINESS MANAGEMENT ---
        self._trustworthinessManagement = TrustworthinessManager(logger=self.logger,knowledgeManagement=self.knowledgeManagement,adaptationManagement=self.adaptationManagement,verbose=self._verbose)
        self._adaptationManagement.trustworthinessManagement = self.trustworthinessManagement

    # ------------------------------------------------------------------------------------------------
    # -------------------------------------INTERFACE FUNCTIONS---------------------------------------
    # ------------------------------------------------------------------------------------------------
    @property
    def logger(self):
        """The logger class (read-only)."""
        return self._logger

    @property
    def communicationManagement(self):
        """The communication management class (read-only)."""
        return self._com

    @property
    def knowledgeManagement(self):
        """The knowledge management class (read-only)."""
        return self._km

    @property
    def knowledgeBase(self):
        return self._km.knowledgeBase

    @knowledgeBase.setter
    def knowledgeBase(self, value):
        self._km.knowledgeBase = value

    @property
    def monitor(self):
        """The monitor class (read-only)."""
        return self._monitor

    @monitor.setter
    def monitor(self,cmp):
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
    def telegraf(self):
        """The telegraf class (read-only)."""
        return self._telegraf

    @telegraf.setter
    def telegraf(self, cmp):
        """The telegraf class (write)."""
        self._telegraf = cmp


    @property
    def effector(self):
        """The effector class (read-only)."""
        return self._effector

    @effector.setter
    def effector(self, cmp):
        """The effector class (write)."""
        self._effector = cmp

    @property
    def probe(self):
        """The probe class (read-only)."""
        return self._probe

    @probe.setter
    def probe(self, cmp):
        """The probe class (write)."""
        self._probe = cmp

    @property
    def trustworthinessChecker(self):
        """The trustworthinessChecker class (read-only)."""
        return self._trustworthinessChecker

    @trustworthinessChecker.setter
    def trustworthinessChecker(self, cmp):
        """The trustworthinessChecker class (write)."""
        self._trustworthinessChecker = cmp
        #TODO: CHECK IF BEST PLACE
        self.trustworthinessManagement.trustworthinessExecutor = self._trustworthinessChecker.trustworthinessExecutor

    @property
    def adaptationManagement(self):
        """The adaptation management class (read-only)."""
        return self._adaptationManagement

    @adaptationManagement.setter
    def adaptationManagement(self, cmp):
        """The adaptation management class (write)."""
        self._adaptationManagement = cmp

    @property
    def trustworthinessManagement(self):
        """The trustworthiness management class (read-only)."""
        return self._trustworthinessManagement

    @trustworthinessManagement.setter
    def trustworthinessManagement(self, cmp):
        """The trustworthiness management class (write)."""
        self._trustworthinessManagement = cmp

    @property
    def adaptationOrchestrator(self):
        """The adaptation orchestrator class (read-only)."""
        return self._adaptationOrchestrator

    @adaptationOrchestrator.setter
    def adaptationOrchestrator(self, cmp):
        """The adaptation orchestrator class (write)."""
        self._adaptationOrchestrator = cmp

    # ------------------------------------------------------------------------------------------------
    # -------------------------------------INTERNAL FUNCTIONS----------------------------------------
    # ------------------------------------------------------------------------------------------------
    def _dispatch(self,args):
        """Function to dispatch the remote commands."""
        if self._verbose:print("DISPATCHING RECEIVED MESSAGE")
        topic=args[0]
        messageRAW = args[1]

        # --- CUSTOM MATRIX TO PASS TO KNOWLEDGE BASE ---       #TODO: THIS WILL BE GENERATED BASED ON THE DEPLOYMENT!
        if topic == "/RobotOdometry":
            odometry = RobotPose()
            odometry.instantiate(messageRAW)
            self.telegraf.digest(odometry.name, odometry)
        if topic == "/DetectedPersons":
            detections = ObjectsStamped()
            detections.instantiate(messageRAW)
            self.telegraf.digest(detections.name, detections)
        if topic == "/ManagedSystemLog":
            logmsg = LogMessage()
            logmsg.instantiate(messageRAW)
            self.logger.log(msg=logmsg.message)

    def _EnterInitializationModeFcn(self):

        # --- INITIALIZE THE COMMUNICATION MANAGEMENT component ---
        self.communicationManagement.RaPEnterInitializationMode()
        self.communicationManagement.RaPExitInitializationMode()

        # --- INITIALIZE MAPE components ---
        self.monitor.RaPEnterInitializationMode()
        self.monitor.RaPExitInitializationMode()

        self.analysis.RaPEnterInitializationMode()
        self.analysis.RaPExitInitializationMode()

        self.plan.RaPEnterInitializationMode()
        self.plan.RaPExitInitializationMode()

        self.legitimize.RaPEnterInitializationMode()
        self.legitimize.RaPExitInitializationMode()

        self.execute.RaPEnterInitializationMode()
        self.execute.RaPExitInitializationMode()

        # --- INITIALIZE MANAGED SYSTEM components ---
        self.effector.RaPEnterInitializationMode()
        self.effector.RaPExitInitializationMode()

        # --- KNOWLEDGE MANAGEMENT ---
        self.knowledgeManagement.RaPEnterInitializationMode()
        self.knowledgeManagement.RaPExitInitializationMode()

        # --- TRUSTWORTHINESS CHECKER ---
        self.trustworthinessChecker.effector = self.effector
        self.trustworthinessChecker.RaPEnterInitializationMode()
        self.trustworthinessChecker.RaPExitInitializationMode()

        # --- ADAPTATION MANAGEMENT ---
        self.adaptationManagement.RaPEnterInitializationMode()
        self.adaptationManagement.RaPExitInitializationMode()

        # --- TRUSTWORTHINESS MANAGEMENT ---
        self.trustworthinessManagement.RaPEnterInitializationMode()
        self.trustworthinessManagement.RaPExitInitializationMode()

        # --- ADAPTATION ORCHESTRATOR ---
        self.adaptationOrchestrator.RaPEnterInitializationMode()
        self.adaptationOrchestrator.RaPExitInitializationMode()

    def _EnterConfigurationModeFcn(self):

        # --- CONFIGURE THE COMMUNICATION MANAGEMENT component ---
        self.communicationManagement.RaPEnterConfigurationMode()
        self.communicationManagement.RaPExitConfigurationMode()

        # --- CONFIGURE MAPE components ---
        self.monitor.RaPEnterConfigurationMode()
        self.monitor.RaPExitConfigurationMode()

        self.analysis.RaPEnterConfigurationMode()
        self.analysis.RaPExitConfigurationMode()

        self.plan.RaPEnterConfigurationMode()
        self.plan.RaPExitConfigurationMode()

        self.legitimize.RaPEnterConfigurationMode()
        self.legitimize.RaPExitConfigurationMode()

        self.execute.RaPEnterConfigurationMode()
        self.execute.RaPExitConfigurationMode()

        # --- CONFIGURE MANAGED SYSTEM components ---
        self.effector.RaPEnterConfigurationMode()
        self.effector.RaPExitConfigurationMode()

        # --- KNOWLEDGE MANAGEMENT ---
        self.knowledgeManagement.RaPEnterConfigurationMode()
        self.knowledgeManagement.RaPExitConfigurationMode()

        # --- TRUSTWORTHINESS CHECKER ---
        self.trustworthinessChecker.RaPEnterConfigurationMode()
        self.trustworthinessChecker.RaPExitConfigurationMode()

        # --- ADAPTATION MANAGEMENT ---
        self.adaptationManagement.RaPEnterConfigurationMode()
        self.adaptationManagement.RaPExitConfigurationMode()

        # --- TRUSTWORTHINESS MANAGEMENT ---
        self.trustworthinessManagement.RaPEnterConfigurationMode()
        self.trustworthinessManagement.RaPExitConfigurationMode()

        # --- ADAPTATION ORCHESTRATOR ---
        self.adaptationOrchestrator.RaPEnterConfigurationMode()
        self.adaptationOrchestrator.RaPExitConfigurationMode()
