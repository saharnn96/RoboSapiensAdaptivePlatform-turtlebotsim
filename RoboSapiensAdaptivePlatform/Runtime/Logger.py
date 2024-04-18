#**********************************************************************************
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
from RoboSapiensAdaptivePlatform.utils.nodes import Node


class Logger(Node):

    def __init__(self,config="default",verbose=False):
        super().__init__(verbose=verbose)
        """Initialize the generic Logger component.

                Parameters
                ----------
                config : string
                    Path to the YAML config file
                
                verbose : bool
                    component verbose execution

        See Also
        --------
        ..

        Examples
        --------
        >> log = Logger(config="",verbose=False)

        """

        self._name = "Logger"
        self._description = "The logger handles all logging and tracking data within the RoboSapiens Adaptive Platform."
        self._verbose = verbose


        # --- LOGGER SETUP USING CONFIG ---
        self._cfg = config
        self._systemLog = None
        self._syslogger = None

    # ------------------------------------------------------------------------------------------------
    # -------------------------------------INTERFACE FUNCTIONS----------------------------------------
    # ------------------------------------------------------------------------------------------------
    def IsEnabled(self):
        """Function to check if logger is enabled.

        :return: `enabled` if logger is enabled, `disabled` otherwise
        :rtype: string
        """
        if self._state == genericNodeStates.INITIALIZED:
            return True
        else:
            return False

    def log(self, msg):
        """Function to log a message using the default level.

        :param msg: Message to be logged
        :type msg: string

        :return: `True` if logging was successful, `False` otherwise
        :rtype: bool
        """
        try:
            self._syslogger.info(msg)
            if self._verbose:print(msg)
            #TODO Sahar: add push to dashboard databas (e.g. influxdb)
            return True
        except:
            if self._verbose: print("Logging failed")
            return False


    # ------------------------------------------------------------------------------------------------
    # -------------------------------------INTERNAL FUNCTIONS----------------------------------------
    # ------------------------------------------------------------------------------------------------
    def _setup_logger(self,name, log_file, level=logging.INFO):
        """Setup of single loggers"""
        formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
        handler = logging.FileHandler(log_file)
        handler.setFormatter(formatter)
        logger = logging.getLogger(name)
        logger.setLevel(level)
        logger.addHandler(handler)

        return logger

    def _EnterInitializationModeFcn(self):
        self._syslogger = self._setup_logger(name="sysLog", log_file=self._systemLog, level=logging.INFO)

    def _EnterConfigurationModeFcn(self):
        # --- CONFIGURE THE LOGGER ---
        with open(self._cfg, 'r') as file:
            cfg = yaml.safe_load(file)
            self._systemLog = cfg["logger"]["path"]








