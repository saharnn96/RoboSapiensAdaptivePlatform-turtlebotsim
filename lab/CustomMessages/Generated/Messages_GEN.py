class example1(object):
    def __init__(self):

        self._Target_Speed= 0.0
        self._isTriggerd= False
        self._PersonList= None


    @property
    def Target_Speed(self):
        """The Target_Speed (read-only)."""
        return self._Target_Speed

    @Target_Speed.setter
    def Target_Speed(self, cmp):
        """The Target_Speed (write)."""
        self._Target_Speed = cmp

    @property
    def isTriggerd(self):
        """The isTriggerd (read-only)."""
        return self._isTriggerd

    @isTriggerd.setter
    def isTriggerd(self, cmp):
        """The isTriggerd (write)."""
        self._isTriggerd = cmp

    @property
    def PersonList(self):
        """The PersonList (read-only)."""
        return self._PersonList

    @PersonList.setter
    def PersonList(self, cmp):
        """The PersonList (write)."""
        self._PersonList = cmp

