# **********************************************************************************
# * Copyright (C) 2024-present Bert Van Acker (B.MKR) <bert.vanacker@uantwerpen.be>
# *
# * This file is part of the roboarch R&D project.
# *
# * RAP R&D concepts can not be copied and/or distributed without the express
# * permission of Bert Van Acker
# **********************************************************************************
import json
import yaml
from RoboSapiensAdaptivePlatform.utils.nodes import Node
from RoboSapiensAdaptivePlatform.utils.CommunicationInterfaces import MQTTInterface
from RoboSapiensAdaptivePlatform.utils.constants import *



class MQTTProbe(Node):

    def __init__(self, config="default",verbose=False):
        super().__init__(verbose=verbose)

        # configurion file
        self._cfg = config

        # setup MQTT publisher
        self._publisher = None
        self._publishList = []


    # ------------------------------------------------------------------------------------------------
    # -------------------------------------EXTERNAL FUNCTIONS----------------------------------------
    # ------------------------------------------------------------------------------------------------
    def push(self,name,value):
        for publisher in self._publishList:
            if name == publisher["name"]:
                message = self._encode(message=value)   #CLASS GIVEN AS VALUE!
                self._publisher.push(topic=publisher["topic"],value=message)

    # ------------------------------------------------------------------------------------------------
    # -------------------------------------INTERNAL FUNCTIONS----------------------------------------
    # ------------------------------------------------------------------------------------------------
    def _encode(self,message):
        return json.dumps(message, default=lambda o: o.__dict__,sort_keys=True, indent=4)

    def _create_publisher(self,cls=None,name="",topic='topic',QoS=10):
        if self._state == genericNodeStates.CONFIGURATION_MODE:
            self._publishList.append({'name': name, 'class': cls, 'topic': topic})
        else:
            self._verbose:print("Publisher cannot be added, com module not in config mode")

    def _EnterInitializationModeFcn(self):
        self._publisher = MQTTInterface(name="MQTTPublisher", VERBOSE=self._verbose)

    def _ExitInitializationModeFcn(self):
        self._publisher._mqttClient = self._publisher._mqtt_setup()
        self._publisher.start()

    def _EnterConfigurationModeFcn(self):
        with open(self._cfg, 'r') as file:
            cfg = yaml.safe_load(file)
            for msg in cfg["probe"]["properties"]:
                p = msg["property"]
                self._create_publisher(cls=p["class"], name=p["name"], topic=p["topic"],QoS=p["QoS"])



