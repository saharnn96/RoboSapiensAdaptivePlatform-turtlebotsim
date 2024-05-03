# **********************************************************************************
# * Copyright (C) 2024-present Bert Van Acker (B.MKR) <bert.vanacker@uantwerpen.be>
# *
# * This file is part of the roboarch R&D project.
# *
# * RAP R&D concepts can not be copied and/or distributed without the express
# * permission of Bert Van Acker
# **********************************************************************************
from RoboSapiensAdaptivePlatform.utils.constants import *
from RoboSapiensAdaptivePlatform.utils.timer import perpetualTimer
from RoboSapiensAdaptivePlatform.ManagedSystem.RemoteProbes import MQTTProbe
from RoboSapiensAdaptivePlatform.ManagedSystem.RemoteEffectors import MQTTEffector
from RoboSapiensAdaptivePlatform.Runtime.RemoteLoggers import MQTTLogger


class localManagedSystem(object):

    def __init__(self,name = "Local managed system Templates",description = "A local managed system Templates for RaP.",logger=None,effector = None,probe=None,verbose=False):
        self._name = name
        self._description = description
        self._verbose = verbose

        # RoboSapiens Adaptive Platform elements
        self._RaPLogger = logger
        self._RaPEffector = effector
        self._RaPProbe = probe

    @property
    def RaPLogger(self):
        """The RaP logger class (read-only)."""
        return self._RaPLogger

    @RaPLogger.setter
    def RaPLogger(self, cmp):
        """The RaP logger class (write)."""
        self._RaPLogger = cmp

    @property
    def RaPEffector(self):
        """The RaP effector class (read-only)."""
        return self._RaPEffector

    @RaPEffector.setter
    def RaPEffector(self, cmp):
        """The RaP effector class (write)."""
        self._RaPEffector = cmp

    @property
    def RaPProbe(self):
        """The RaP probe class (read-only)."""
        return self._RaPProbe

    @RaPProbe.setter
    def RaPProbe(self, cmp):
        """The RaP effector class (write)."""
        self._RaPProbe = cmp
        # Auto-start probing executor
        #self._probe_executor.start()


    def _effector_spin_once(self,action):
        self.RaPLogger.log("[" + self._name + "] - " + 'IMPLEMENTATION REQUIRED FOR THE EFFECTOR SPIN_ONCE FUNCTION (_effector_spin_once)"')
        return -1


class remoteManagedSystem(object):

    def __init__(self,name = "Remote managed system Templates (MQTT)",description = "A remote managed system Templates for RaP.",logger=None,effector = None,probe=None,config='00_input/config.yaml',verbose=False):
        self._name = name
        self._description = description
        self._verbose = verbose

        # RoboSapiens Adaptive Platform elements
        self._RaPLogger = MQTTLogger(config=config,verbose=verbose)
        self._RaPEffector = MQTTEffector(logger=logger,knowledgeBase=None,config=config,verbose=verbose)
        self._RaPProbe =  MQTTProbe(config=config,verbose=verbose)

        # AUTO-CONFIG
        self._RaPLogger.RaPEnterConfigurationMode()
        self._RaPLogger.RaPExitConfigurationMode()
        self._RaPLogger.RaPEnterInitializationMode()
        self._RaPLogger.RaPExitInitializationMode()

        self._RaPProbe.RaPEnterConfigurationMode()
        self._RaPProbe.RaPExitConfigurationMode()
        self._RaPProbe.RaPEnterInitializationMode()
        self._RaPProbe.RaPExitInitializationMode()

        self._RaPEffector.RaPEnterConfigurationMode()
        self._RaPEffector.RaPExitConfigurationMode()
        self._RaPEffector.RaPEnterInitializationMode()
        self._RaPEffector.RaPExitInitializationMode()

    @property
    def RaPLogger(self):
        """The RaP logger class (read-only)."""
        return self._RaPLogger

    @RaPLogger.setter
    def RaPLogger(self, cmp):
        """The RaP logger class (write)."""
        self._RaPLogger = cmp

    @property
    def RaPEffector(self):
        """The RaP effector class (read-only)."""
        return self._RaPEffector

    @RaPEffector.setter
    def RaPEffector(self, cmp):
        """The RaP effector class (write)."""
        self._RaPEffector = cmp

    @property
    def RaPProbe(self):
        """The RaP probe class (read-only)."""
        return self._RaPProbe

    @RaPProbe.setter
    def RaPProbe(self, cmp):
        """The RaP effector class (write)."""
        self._RaPProbe = cmp
        # Auto-start probing executor
        #self._probe_executor.start()
