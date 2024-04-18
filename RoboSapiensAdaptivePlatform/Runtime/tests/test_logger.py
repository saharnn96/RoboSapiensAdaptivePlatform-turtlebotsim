from unittest import TestCase
from RoboSapiensAdaptivePlatform.Runtime.Logger import Logger
from RoboSapiensAdaptivePlatform.utils.constants import *



class testLogger(TestCase):

    def setup(self):
        self.assertTrue(True)


    def test_initialization(self):
        logger = Logger(config="input/config.yaml",verbose=True)
        #CONFIGURE THE LOGGER
        logger.RaPEnterConfigurationMode()
        logger.RaPExitConfigurationMode()
        #INITIALIZE THE LOGGER
        logger.RaPEnterInitializationMode()
        logger.RaPExitInitializationMode()

        #CHECKS
        self.assertTrue(logger.IsEnabled(),True)
        self.assertTrue(logger.state, genericNodeStates.INITIALIZED)

    def test_logging(self):
        logger = Logger(config="input/config.yaml", verbose=True)
        # CONFIGURE THE LOGGER
        logger.RaPEnterConfigurationMode()
        logger.RaPExitConfigurationMode()
        # INITIALIZE THE LOGGER
        logger.RaPEnterInitializationMode()
        logger.RaPExitInitializationMode()

        isLogged = logger.log("this is a simple log")
        #CHECKS
        self.assertTrue(logger.IsEnabled(),True)
        self.assertTrue(isLogged, True)










