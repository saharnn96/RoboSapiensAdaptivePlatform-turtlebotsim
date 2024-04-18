#**********************************************************************************
# * Copyright (C) 2024-present Bert Van Acker (B.MKR) <bert.vanacker@uantwerpen.be>
# *
# * This file is part of the roboarch R&D project.
# *
# * RAP R&D concepts can not be copied and/or distributed without the express
# * permission of Bert Van Acker
# **********************************************************************************

class hallway(object):

    def __init__(self):
        """Initialize the hallway with specified walls and obstacles

                Parameters
                ----------
                verbose : bool
                    component verbose execution

        See Also
        --------
        ..

        Examples
        --------
        >> robot = irobot_create_3(verbose=False)

        """

        self._map = None
        self.set_map()

    @property
    def map(self):
        """The map property (read-only)."""
        return self._map

    def set_map(self):
        ox, oy = [], []
        for i in range(-10, 60):
            ox.append(i)
            oy.append(-10.0)
        for i in range(-10, 60):
            ox.append(60.0)
            oy.append(i)
        for i in range(-10, 61):
            ox.append(i)
            oy.append(60.0)
        for i in range(-10, 61):
            ox.append(-10.0)
            oy.append(i)
        for i in range(-10, 40):
            ox.append(20.0)
            oy.append(i)
        for i in range(0, 40):
            ox.append(40.0)
            oy.append(60.0 - i)

        self._map = [ox,oy]
