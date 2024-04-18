import json
from paho.mqtt import client as mqtt_client
from messages import property
from RoboSapiensAdaptivePlatform.utils.CommunicationInterfaces import MQTTInterface


poi = property()
poi.name = "Mass"
poi.description = "Design property mass"
poi.value = 150.0
poi.min = 0.0
poi.max = 200.0

#print(json.dumps(poi, default=lambda o: o.__dict__,sort_keys=True, indent=4))


publisher = MQTTInterface(name="MQTTPublisher")
publisher.start()

publisher.push(topic="/propertyX",value=json.dumps(poi, default=lambda o: o.__dict__,sort_keys=True, indent=4))



