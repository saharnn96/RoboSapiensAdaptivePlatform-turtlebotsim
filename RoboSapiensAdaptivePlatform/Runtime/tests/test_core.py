from unittest import TestCase
from RoboSapiensAdaptivePlatform.Runtime.Core import Core
from RoboSapiensAdaptivePlatform.Communication.Messages.messages import *
from RoboSapiensAdaptivePlatform.utils.constants import *



class testCore(TestCase):

    def setup(self):
        self.assertTrue(True)


    def test_initialization(self):
        RaP = Core(verbose=True)
        #CONFIGURE THE CORE
        RaP.RaPEnterConfigurationMode()
        RaP.RaPExitConfigurationMode()
        #INITIALIZE THE LOGGER
        RaP.RaPEnterInitializationMode()
        RaP.RaPExitInitializationMode()

        #CHECKS
        self.assertTrue(RaP.logger.IsEnabled(),True)
        self.assertTrue(RaP.logger.state, genericNodeStates.INITIALIZED)

    def test_logging(self):
        RaP = Core(verbose=True)
        #CONFIGURE THE CORE
        RaP.RaPEnterConfigurationMode()
        RaP.RaPExitConfigurationMode()
        #INITIALIZE THE LOGGER
        RaP.RaPEnterInitializationMode()
        RaP.RaPExitInitializationMode()

        isLogged = RaP.logger.log("this is a simple log")
        #CHECKS
        self.assertTrue(RaP.logger.IsEnabled(),True)
        self.assertTrue(isLogged, True)

    def test_COM_publish(self):
        RaP = Core(mode= comMode.APPLICATION,verbose=True)
        #CONFIGURE THE CORE
        RaP.RaPEnterConfigurationMode()
        RaP.RaPExitConfigurationMode()
        #INITIALIZE THE LOGGER
        RaP.RaPEnterInitializationMode()
        RaP.RaPExitInitializationMode()

        poi = Property()
        poi.name = "temperature_average"
        poi.value = 15.9
        poi.min = 0.0
        poi.max = 50.0
        RaP.communicationManagement.publish(cls=poi)

        while True:
            x=10


        #CHECKS
        #self.assertTrue(RaP.logger.IsEnabled(),True)
        #self.assertTrue(isLogged, True)










