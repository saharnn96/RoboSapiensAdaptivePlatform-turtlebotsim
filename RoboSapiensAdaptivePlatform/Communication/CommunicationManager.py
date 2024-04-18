#**********************************************************************************
# * Copyright (C) 2024-present Bert Van Acker (B.MKR) <bert.vanacker@uantwerpen.be>
# *
# * This file is part of the roboarch R&D project.
# *
# * RAP R&D concepts can not be copied and/or distributed without the express
# * permission of Bert Van Acker
# **********************************************************************************
import yaml
import json
from RoboSapiensAdaptivePlatform.utils.constants import *
from RoboSapiensAdaptivePlatform.utils.nodes import Node
from RoboSapiensAdaptivePlatform.Communication.Messages.messages import *
from RoboSapiensAdaptivePlatform.utils.CommunicationInterfaces import MQTTInterface



class CommunicationManager(Node):

    def __init__(self,mode = comMode.MAPE,config="default",communicationProtocol="MQTT", logger = None, verbose=False):
        super().__init__(verbose=verbose)
        """Initialize the generic CommunicationManager component.

                Parameters
                ----------
                mode: enum
                    Communication mode (application|MAPE)
                
                config : string
                    Path to the YAML config file
                    
                communicationProtocol : enum
                    communicationProtocol used to interface
                    
                Logger : RaP logger
                    Logger component used within the RaP
                    
                verbose : bool
                    component verbose execution

        See Also
        --------
        ..

        Examples
        --------
        >> cm = CommunicationManager(communicationProtocol=communicationProtocol.MQTT,verbose=False)

        """

        self._name = "Communication Manager"
        self._description = "The communication manager handles the (cross-device) communication within the RoboSapiens Adaptive Platform."
        self._cfg = config
        self._communicationProtocol = communicationProtocol
        self._mode = mode
        self._verbose = verbose

        self._publisher = None
        self._subscriber = None
        self._subscriptionList = []
        self._subscriptionListTopics = []
        self._publishList = []

        # RaP logger component
        self._RaPlogger = logger


    #------------------------------------------------------------------------------------------------
    #-------------------------------------INTERFACE FUNCTIONS---------------------------------------
    #------------------------------------------------------------------------------------------------
    def create_subscription(self,cls=Proptery,name="",topic='topic',QoS=10):
        if self._state == genericNodeStates.CONFIGURATION_MODE:
            self._subscriptionList.append({'name': name, 'class': cls, 'topic': topic})
            self._subscriptionListTopics.append(topic)
        else:
            self._verbose:print("Subscription cannot be added, com module not in config mode")

    def create_publisher(self,cls=Proptery,name="",topic='topic',QoS=10):
        if self._state == genericNodeStates.CONFIGURATION_MODE:
            self._publishList.append({'name': name, 'class': cls, 'topic': topic})
        else:
            self._verbose:print("Publisher cannot be added, com module not in config mode")

    def publish(self,cls):
        for publisher in self._publishList:
            if cls.name == publisher["name"]:
                self._publisher.push(topic=publisher["topic"],value=json.dumps(cls, default=lambda o: o.__dict__, sort_keys=True, indent=4))

    def getQueuedMessages(self):
        messages = self._subscriber.fetch(blocking=False)
        if len(messages) != 0:
            return messages
        else:
            return -1
    # ------------------------------------------------------------------------------------------------
    # -------------------------------------INTERNAL FUNCTIONS----------------------------------------
    # ------------------------------------------------------------------------------------------------
    def _messageReceiveCallback(self):
        if self._verbose:print("TO BE IMPLEMENTED")

    def _encode(self,message):
        return json.dumps(message, default=lambda o: o.__dict__,sort_keys=True, indent=4)

    def _decode(self,topic,message):
        for subscription in self._subscriptionList:
            if topic == subscription["topic"]:
                cls = subscription["class"]
        return type(cls, (object,), json.loads(message.payload))


    def _EnterInitializationModeFcn(self):
        if self._communicationProtocol == communicationProtocol.MQTT:
            if self._RaPlogger is not None: self._RaPlogger.log(self.name+"["+self._mode+"] entered INITIALIZATION mode - MQTT")
            # Publisher
            self._publisher = MQTTInterface(name="MQTTPublisher", VERBOSE=self._verbose)
            # Subscriber
            self._subscriber = MQTTInterface(name="MQTTSubscriber", subscriptions=self._subscriptionListTopics, VERBOSE=self._verbose)
        else:
            if self._RaPlogger is not None: self._RaPlogger.log(self.name+"["+self._mode+"] entered INITIALIZATION mode - default(MQTT)")
            #Publisher
            self._publisher = MQTTInterface(name="MQTTPublisher", VERBOSE=self._verbose)
            #Subscriber
            self._subscriber = MQTTInterface(name="MQTTSubscriber", subscriptions=[],VERBOSE=self._verbose)

    def _ExitInitializationModeFcn(self):
        if self._communicationProtocol == communicationProtocol.MQTT:
            if self._RaPlogger is not None: self._RaPlogger.log(self.name+"["+self._mode+"] entered INITIALIZED mode - MQTT")
            self._publisher._mqttClient = self._publisher._mqtt_setup()
            self._publisher.start()
            self._subscriber._mqttClient = self._subscriber._mqtt_setup()
            self._subscriber.start()
        else:
            if self._RaPlogger is not None: self._RaPlogger.log(self.name+"["+self._mode+"] entered INITIALIZED mode - default(MQTT)")
            self._publisher._mqttClient = self._publisher._mqtt_setup()
            self._publisher.start()
            self._subscriber._mqttClient = self._subscriber._mqtt_setup()
            self._subscriber.start()

    def _EnterConfigurationModeFcn(self):
        if self._RaPlogger is not None:self._RaPlogger.log(self.name+"["+self._mode+"] entered CONFIGURATION mode")
        with open(self._cfg, 'r') as file:
            cfg = yaml.safe_load(file)
            for msg in cfg["com"]["messages"]:
                message = msg["message"]
                if message["destination"] == self._mode:
                    self.create_subscription(cls=message["class"],name=message["name"],topic=message["topic"],QoS=message["QoS"])
                elif message["source"]== self._mode:
                    self.create_publisher(cls=message["class"], name=message["name"], topic=message["topic"],QoS=message["QoS"])


