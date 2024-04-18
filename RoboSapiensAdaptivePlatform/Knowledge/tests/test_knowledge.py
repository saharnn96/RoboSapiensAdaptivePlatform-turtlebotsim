from unittest import TestCase
from RoboSapiensAdaptivePlatform.Communication.Messages.messages import *
from RoboSapiensAdaptivePlatform.Knowledge.KnowledgeManager import KnowledgeManager
from RoboSapiensAdaptivePlatform.utils.constants import *



class testCore(TestCase):

    def setup(self):
        self.assertTrue(True)


    def test_initialization(self):

        KM = KnowledgeManager(verbose=True)

        #CONFIGURE THE Knowledge
        KM.RaPEnterConfigurationMode()
        KM.RaPExitConfigurationMode()
        #INITIALIZE THE LOGGER
        KM.RaPEnterInitializationMode()
        KM.RaPExitInitializationMode()

        poi = Proptery()
        poi.name = "temperature_average"
        poi.value = 55.0
        poi.min = 0.0
        poi.max = 50.0

        fan = Proptery()
        fan.name = "fan_control"
        fan.value = 0
        fan.min = 0
        fan.max = 1

        #write to the knowledge base
        KM.registerKnowledge(fan)
        for i in range(0,36,1):
            KM.registerKnowledge(poi)
            poi.value = poi.value + 1
            if poi.value > 85:
                fan.value = 1
                KM.registerKnowledge(fan)

        poi_read,poi_history = KM.read(name="temperature_average", queueSize=10)
        fan_read, fan_history = KM.read(name="fan_control", queueSize=10)
        print(poi_read.value)
        print(poi_history)
        print(fan_read.value)
        print(fan_history)
        self.assertTrue(len(poi_history), 10)
        self.assertTrue(len(fan_history), 7)









