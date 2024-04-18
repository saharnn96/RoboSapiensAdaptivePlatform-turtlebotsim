from messages import property
import json
from RoboSapiensAdaptivePlatform.utils.CommunicationInterfaces import MQTTInterface


def interpretIncoming(subscriber):
    messages = subscriber.fetch(blocking=False)
    if len(messages) != 0:
        for msg in messages:
            poi_res = type('property', (object,), json.loads(msg.payload))
            print(poi_res.name)


subscriber = MQTTInterface(name="MQTTSubscriber", subscriptions=["/propertyX"])
subscriber.start()



while True:
    interpretIncoming(subscriber)

