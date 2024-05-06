import json
from paho.mqtt import client as mqtt_client
from RoboSapiensAdaptivePlatform.Communication.Messages.messages import *
from RoboSapiensAdaptivePlatform.utils.CommunicationInterfaces import MQTTInterface


poi = Property()
poi.name='target_speed'
poi.value = 1.0
poi.description = 'Slow down robot'

action2 = Action()
action2.ID = actionType.ADAPTATIONTYPE
action2.description = "SPEED ADAPTATION"
action2.propertyList = [poi]


publisher = MQTTInterface(name="MQTTPublisher")
publisher.start()

publisher.push(topic="/RaP_Effector_action",value=json.dumps(action2, default=lambda o: o.__dict__,sort_keys=True, indent=4))



