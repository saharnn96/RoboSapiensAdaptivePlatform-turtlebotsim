from textx import metamodel_from_file
mmCustomMessage = metamodel_from_file('C:/Users/Bert/UAntwerpen/Documenten/00_UA/02_Projects/01_RoboSAPIENS/99_RemoteRepositories/RoboSapiensAdaptivePlatform/lab/CustomMessages/metamodel/CustomMessageGrammar.tx')

#use meta-model to create models from it
customMessage = mmCustomMessage.model_from_file('C:/Users/Bert/UAntwerpen/Documenten/00_UA/02_Projects/01_RoboSAPIENS/99_RemoteRepositories/RoboSapiensAdaptivePlatform/lab/CustomMessages/examples/example1.msg')
x=12
for var in customMessage.elements:
    if isinstance(var, mmCustomMessage.namespaces['CustomMessageGrammar']['Variable']):
        print(var.type)
        print(var.name)
    if isinstance(var, mmCustomMessage.namespaces['CustomMessageGrammar']['Class']):
        print(var.type)
        print(var.name)
    if isinstance(var, mmCustomMessage.namespaces['CustomMessageGrammar']['List']):
        print(var.type)
        print(var.name)
        print(var.size)

