class Property():
    def __init__(self, name="tbd", description="tbd", value=0.0, min=0.0, max=0.0):
        self.name=name
        self.description=description
        self.value=value
        self.min=min
        self.max=max
class Action(object):
    def __init__(self,name="tbd",ID="tbd",description="tbd",propertyList=None):

        self.name = name
        self.ID= ID
        self.description=description
        self.propertyList = propertyList
