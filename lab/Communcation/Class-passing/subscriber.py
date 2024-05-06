from messages import property
from RoboSapiensAdaptivePlatform.Communication.Messages.messages import *
import json
from RoboSapiensAdaptivePlatform.utils.CommunicationInterfaces import MQTTInterface


def interpretIncoming(subscriber):
    messages = subscriber.fetch(blocking=False)
    if len(messages) != 0:
        for msg in messages:
            if msg.topic == "/RobotOdometry":
                odometryRAW = type('RobotPose', (object,), json.loads(msg.payload))
                odometry = RobotPose()
                odometry.instantiate(odometryRAW)

                print(odometry.name)
                print(odometry.position)


            if msg.topic == "/DetectedPersons":
                detectionsRAW = type('ObjectsStamped', (object,), json.loads(msg.payload))

                detections = ObjectsStamped()
                detections.instantiate(detectionsRAW)

                print(detections.name)
                print(detections.objectList)


subscriber = MQTTInterface(name="MQTTSubscriber", subscriptions=["/RobotOdometry","/DetectedPersons"])
subscriber.start()



while True:
    interpretIncoming(subscriber)

