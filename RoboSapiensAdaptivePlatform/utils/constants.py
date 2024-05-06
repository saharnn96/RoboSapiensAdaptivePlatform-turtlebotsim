

class adaptivityComponents():
    MONITOR = "monitor"
    ANALYSIS = "analysis"
    PLAN = "plan"
    LEGITIMIZE = "legitimize"
    EXECUTE = "execute"

class managedSystemComponents():
    EFFECTOR = "effector"

class monitorStatus():
    NORMAL = "normal"
    ANOMALY = "anomaly"

class analysisStatus():
    NORMAL = "normal"
    ANOMALY = "anomaly"

class planStatus():
    IDLE = "idle"
    PLANNING = "planning"
    PLANNED = "planned"

class executeStatus():
    IDLE = "idle"
    EXECUTION = "execution"

class legitimizeStatus():
    IDLE = "idle"
    VALID = "valid"
    INVALID = "invalid"

class effectorStatus():
    IDLE = "idle"
    DIAGNOSIS = "diagnosis"
    ADAPTATION = "adaptation"
    ERROR = "error" #THIS CAN BE EXTENDED WITH ID OF THE ERROR


class genericStates():
    STARTUP="startup"
    INIT = "init"

class genericNodeStates():
    INSTANTIATED = "startup"
    CONFIGURATION_MODE="configurationMode"
    INITIALIZATION_MODE = "initializationMode"
    INITIALIZED = "initialized"
    TERMINATED = "terminated"
    RUNNING = "running"
    IDLE = "idle"


class communicationProtocol():
    MQTT = "mqtt"
    SERIAL="serial"
    UPD = "udp"

class comMode():
    APPLICATION="application"
    MAPE = "MAPE"

class actionType():
    DIAGNOSISTYPE="diagnosistype"
    ADAPTATIONTYPE="adaptationtype"

class actionStatus():
    IDLE="idle"
    REGISTERED="registered"
    TRUSTED="trusted"
    EXECUTING="executing"
    FINISHED="finished"
    DROPPED="dropped"

class orchestrationStatus():
    IDLE="idle"
    MONITORING ="monitoring"
    ANALYZING="analyzing"
    PLANNING="planning"
    LEGITIMIZING="legitimizing"
    EXECUTING="executing"
    WAITING_FOR_ACTION_COMPLETE = "waiting_for_action_complete"
    ERROR = "error"

class trackingState():
    OFF="off"
    OK="ok"
    SEARCHING="searching"

class actionState():
    IDLE="idle"
    MOVING="moving"

class knowledgeType():
    PROPERTY="property"
    DETECTEDOBJECTS="ObjectsStamped"
    ROBOTODOMETRY = "RobotPose"

class communicationEndPointStatus():
    IDLE = "idle"
    RUNNING = "running"
    ERROR = "error" #THIS CAN BE EXTENDED WITH ID OF THE ERROR


