import paho.mqtt.client as mqtt
from queue import Queue
import time
import threading
import logging


class MQTTInterface():

    def __init__(self, name, host_name="localhost", host_port=1883, subscriptions=None,type="PUB",VERBOSE=False, DEBUG=True):

        self.Name = name
        self._type = type
        self.VERBOSE = VERBOSE
        self.DEBUG = DEBUG
        if subscriptions is None:
            self._subscriptions = []
        else:
            self._subscriptions = subscriptions

        # --- MQTT setup ---
        self.host_name = host_name
        self.host_port = host_port
        self._thread = None
        self._mqttClient = None
        self._mqttClient = self._mqtt_setup()

        # --- message queues ---
        self._publishQueue = Queue()
        self._subscriberQueue = Queue()

        # --- reactive input ---
        self._reactiveInput = False

    @property
    def reactiveInput(self):
        """The reactiveInput class (read-only)."""
        return self._reactiveInput

    @reactiveInput.setter
    def reactiveInput(self, cmp):
        """The reactiveInput class (write)."""
        self._reactiveInput = cmp

    @property
    def subscriberQueue(self):
        """The queue for the active subscription messages (read-only)."""
        return self._subscriberQueue

    def add_Subscription(self, subscription_for_adding):
        """Add a single subscription `subscription`

                        Parameters
                        ----------
                        subscription_for_adding : String
                            Subscription(topic) to be added


                        See Also
                        --------
                        ..

                        Examples
                        --------
                        >> todo:add example

                        """
        if self.VERBOSE:print("Subscription added - topic: "+ subscription_for_adding)
        if subscription_for_adding not in self._subscriptions:
            if subscription_for_adding is None:
                raise ValueError("Subscription(topic) cannot be added")
            else:
                self._subscriptions.append(subscription_for_adding)

    def _mqtt_setup(self):
        self._mqttClient = mqtt.Client()
        self._mqttClient.connect(self.host_name, self.host_port)
        self._mqttClient.on_connect = self._on_connect
        self._mqttClient.on_message = self._on_message
        # setup thread for publisher
        self._thread = threading.Thread(target=self._update)
        return self._mqttClient

    def _on_connect(self, client, userdata, flags, rc):
        if self.VERBOSE: print("Connected with result code " + str(rc))
        for subscription in self._subscriptions:
            self._mqttClient.subscribe(subscription)

    def _on_message(self,client, userdata, msg):
        if self.VERBOSE: print("MQTT message received")
        #add to subscriber queue
        self._subscriberQueue.put(msg)
        # TRIGGER ON RECEIVE FOR REACTIVE INPUT
        if self._reactiveInput: self._messageReceiveCallback(msg)
    def _update(self):
        while True:  # loop forever
            self._mqttClient.loop(0.1)

            if not self._publishQueue.empty():
                while not self._publishQueue.empty():
                    message = self._publishQueue.get()
                    if self.VERBOSE: print("Message in publisher queue")

                    #publish interpreted data
                    self._mqttClient.publish(message[0], message[1])

    def start(self):
        self._thread.start()

    def fetch(self,blocking=False):
        if self.VERBOSE and blocking: print("Fetch request - blocking")
        if self.VERBOSE and blocking: print("Fetch request - none-blocking")

        _messageList = []
        if blocking:
            while not self._subscriberQueue.empty():
                message = self._subscriberQueue.get()
                if message is None:
                    continue
                else:
                    _messageList.append(message)
        else:
            if not self._subscriberQueue.empty():
                while not self._subscriberQueue.empty():
                    message = self._subscriberQueue.get()
                    _messageList.append(message)

        return _messageList

    def push(self,topic,value):
        if self.VERBOSE:print("Push request")
        message = [topic,value]
        self._publishQueue.put(message)

    def _messageReceiveCallback(self,args):
        if self.VERBOSE:print("TO BE IMPLEMENTED")