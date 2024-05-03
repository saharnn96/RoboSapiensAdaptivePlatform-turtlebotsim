import time
from unittest import TestCase
from RoboSapiensAdaptivePlatform.Communication.CommunicationManager import CommunicationManager
from RoboSapiensAdaptivePlatform.Communication.Messages.messages import *
from RoboSapiensAdaptivePlatform.utils.constants import *



class testCom(TestCase):

    def setup(self):
        self.assertTrue(True)


    def test_initialization(self):
        com = CommunicationManager(mode=comMode.APPLICATION,config="input/config.yaml",communicationProtocol=communicationProtocol.MQTT)

        #CONFIGURE THE LOGGER
        com.RaPEnterConfigurationMode()
        com.RaPExitConfigurationMode()
        #INITIALIZE THE LOGGER
        com.RaPEnterInitializationMode()
        com.RaPExitInitializationMode()

        poi = Property()
        poi.name = 'target_speed'
        poi.value = 1.0
        poi.description = 'Slow down robot'

        action2 = Action()
        action2.name = 'SpeedAdaptationAction'
        action2.ID = actionType.ADAPTATIONTYPE
        action2.description = "SPEED ADAPTATION"
        action2.propertyList = [poi]

        com.publish(cls=action2)

        #CHECKS
        #self.assertTrue(logger.IsEnabled(),True)
        #self.assertTrue(logger.state, loggerStates.ENABLED)



    def test_Receive(self):
        com = CommunicationManager(mode=comMode.APPLICATION,config="input/config.yaml",communicationProtocol=communicationProtocol.MQTT,verbose=True)

        #CONFIGURE THE COM
        com.RaPEnterConfigurationMode()
        com.RaPExitConfigurationMode()
        #INITIALIZE THE COM
        com.RaPEnterInitializationMode()
        com.RaPExitInitializationMode()

        # POLLING FOR MESSAGES
        while True:
            messages = com.getQueuedMessages()
            if messages!= -1:
                print(messages[0].payload)
            time.sleep(2)

        #CHECKS
        #self.assertTrue(logger.IsEnabled(),True)
        #self.assertTrue(logger.state, loggerStates.ENABLED)







