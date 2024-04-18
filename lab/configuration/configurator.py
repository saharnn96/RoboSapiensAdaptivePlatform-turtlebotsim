import yaml

configFile = "input/config.yaml"
with open(configFile, 'r') as file:
    cfg = yaml.safe_load(file)
    #print(cfg["com"]["application"])
    for prop in cfg["com"]["messages"]:
        print(prop["message"])
       # print(prop[""])