# **********************************************************************************
# * Copyright (C) 2024-present Bert Van Acker (B.MKR) <bert.vanacker@uantwerpen.be>
# *
# * This file is part of the roboarch R&D project.
# *
# * RAP R&D concepts can not be copied and/or distributed without the express
# * permission of Bert Van Acker
# **********************************************************************************
from RoboSapiensAdaptivePlatform.utils.nodes import TriggeredNode
from RoboSapiensAdaptivePlatform.Communication.Messages.messages import ComponentStatus
from RoboSapiensAdaptivePlatform.utils.CommunicationInterfaces import MQTTInterface
from RoboSapiensAdaptivePlatform.utils.constants import *
from RoboSapiensAdaptivePlatform.Communication.Messages.messages import *

import json
import yaml



class MQTTEffector(TriggeredNode):

    def __init__(self, logger = None,knowledgeBase = None,config="default",verbose=False):
        super().__init__(logger=logger,knowledge = knowledgeBase,verbose=verbose)

        # configurion file
        self._cfg = config
        self._endpointList = []

        # setup MQTT subscriber
        self._subscriber = None
        self._subscriptionList = []
        self._subscriptionListTopics = []

    # ------------------------------------------------------------------------------------------------
    # -------------------------------------EXTERNAL FUNCTIONS----------------------------------------
    # ------------------------------------------------------------------------------------------------
    def getActions(self,endpointName):
        actionList = []
        messages = self._subscriber.fetch(blocking=False)
        if len(messages) != 0:
            #resolve endpoint
            _endpointTopic = None
            for endpoint in self._endpointList:
                if endpoint[0] == endpointName:
                    _endpointTopic = endpoint[1]

            #decode action messages
            for message in messages:
                if _endpointTopic is not None:
                    actionList.append(self._decode(topic=_endpointTopic,message=message))
            return actionList
        else:
            return -1

    # ------------------------------------------------------------------------------------------------
    # -------------------------------------INTERNAL FUNCTIONS----------------------------------------
    # ------------------------------------------------------------------------------------------------

    def _create_subscription(self,cls=None,name="",topic='topic',QoS=10):
        if self._state == genericNodeStates.CONFIGURATION_MODE:
            self._subscriptionList.append({'name': name, 'class': cls, 'topic': topic})
            self._subscriptionListTopics.append(topic)
        else:
            self._verbose:print("Subscription cannot be added, com module not in config mode")


    def _decode(self,topic,message):                        #TODO: THIS NEEDS TO BE GENERALIZED -> class passing not working perfectly over MQTT yet
        for subscription in self._subscriptionList:
            if topic == subscription["topic"]:
                cls = subscription["class"]
                # RESOLVE ACTION AND CONTAINED PROPERTIES
                actionRaw = type(cls, (object,), json.loads(message.payload))
                pList = []
                for p in actionRaw._propertyList:
                    poi = Proptery()
                    poi.name = p["name"]
                    poi.value = p["value"]
                    poi.description = p["description"]
                    pList.append(poi)

                action = Action()
                action.ID = actionRaw._ID
                action.UUID = actionRaw._UUID
                action.description = actionRaw._description
                action.propertyList = pList

        return action   #type(cls, (object,), json.loads(message.payload))

    #def _SpinOnceFcn(self, args):
    #    if self._verbose: print("Spin once function not yet overloaded")




    def _EnterInitializationModeFcn(self):
        self._subscriber = MQTTInterface(name="MQTTSubscriber", subscriptions=self._subscriptionListTopics, VERBOSE=self._verbose)
        self._subscriber.reactiveInput = True
        self._subscriber._messageReceiveCallback = self.RaPSpin_once        #TODO: check if double method overloading works

    def _ExitInitializationModeFcn(self):
        self._subscriber._mqttClient = self._subscriber._mqtt_setup()
        self._subscriber.start()

    def _EnterConfigurationModeFcn(self):
        with open(self._cfg, 'r') as file:
            cfg = yaml.safe_load(file)
            for msg in cfg["effector"]["endpoints"]:
                p = msg["endpoint"]
                self._create_subscription(cls=p["class"], name=p["name"], topic=p["topic"], QoS=p["QoS"])
                self._endpointList.append([p["name"],p["topic"]])

