import threading
import time

from adaptationLayer.M_comp import monitor
from adaptationLayer.A_comp import analysis
from adaptationLayer.P_comp import plan
from adaptationLayer.L_comp import legitimize
from adaptationLayer.E_comp import execute
from adaptationLayer.adaptationOrchestrator import adaptationOrchestrator
from RoboSapiensAdaptivePlatform.Communication.Telegraf import Telegraf
from RoboSapiensAdaptivePlatform.Runtime.Core import remoteCore
from RoboSapiensAdaptivePlatform.Knowledge.KnowledgeBase import KnowledgeBase
from RoboSapiensAdaptivePlatform.utils.constants import *


verbose = True

# --- RoboSapiens Adaptive Runtime ---
RaP = remoteCore(verbose=verbose)

# --- adaptation layer blocks ---
KB = KnowledgeBase()
M = monitor(logger = RaP.logger,knowledgeBase=KB,verbose=verbose)
A = analysis(logger=RaP.logger,knowledgeBase=KB,verbose=verbose)
P = plan(logger = RaP.logger,knowledgeBase=KB,verbose=verbose)
L = legitimize(logger=RaP.logger,knowledgeBase=KB,verbose=verbose)
E = execute(logger=RaP.logger,knowledgeBase=KB,adaptationManagement=RaP.adaptationManagement,verbose=verbose)

# --- adaptation orchestrator @ 5s ---
adaptationOrchestrator = adaptationOrchestrator(logger=RaP.logger,knowledgeBase=KB,dt=5,M=M,A=A,P=P,L=L,E=E,verbose=verbose)
RaP.adaptationOrchestrator = adaptationOrchestrator

RaP.knowledgeBase = KB
RaP.monitor = M
RaP.analysis = A
RaP.plan = P
RaP.legitimize = L
RaP.execute = E

# --- configure telegraf ---
telegraf = Telegraf(logger=RaP.logger,knowledgeManagement=RaP.knowledgeManagement,verbose=verbose)
telegraf.gatewayMatrix = [{"name" : "DetectedPersons","type":knowledgeType.DETECTEDOBJECTS},{"name" : "RobotOdometry","type":knowledgeType.ROBOTODOMETRY},{"name" : "LidarRange","type":knowledgeType.DETECTEDOBJECTS}]
RaP.telegraf = telegraf

# --- MQTT CONNTECTED MANAGED SYSTEM -> START Turtlebot_SafePerson.py first ---


# --- configure and initialize ---
RaP.RaPEnterConfigurationMode()
RaP.RaPExitConfigurationMode()
RaP.RaPEnterInitializationMode()
RaP.RaPExitInitializationMode()

#-----------------------------------------------------------------------------------------------------------------------
# ----------------------------- RoboSapiens Adaptive Platform execution ------------------------------------------------
#-----------------------------------------------------------------------------------------------------------------------
def start():
    RaP.adaptationOrchestrator._OrchestrationLoop()
    # START ORCHESTRATION
    RaP.adaptationOrchestrator.RaPStartOrchestration()



if __name__ == '__main__':
    start()




