# **********************************************************************************
# * Copyright (C) 2024-present Bert Van Acker (B.MKR) <bert.vanacker@uantwerpen.be>
# *
# * This file is part of the roboarch R&D project.
# *
# * RAP R&D concepts can not be copied and/or distributed without the express
# * permission of Bert Van Acker
# **********************************************************************************
from RoboSapiensAdaptivePlatform.utils.nodes import Node
from RoboSapiensAdaptivePlatform.utils.constants import *



class Probe(Node):

    def __init__(self, telegraf = None,verbose=False):
        super().__init__(verbose=verbose)
        self._telegraf = telegraf

    @property
    def telegraf(self):
        """The telegraf class (read-only)."""
        return self._telegraf

    @telegraf.setter
    def telegraf(self, cmp):
        """The telegraf class (write)."""
        self._telegraf = cmp

    # ------------------------------------------------------------------------------------------------
    # -------------------------------------EXTERNAL FUNCTIONS----------------------------------------
    # ------------------------------------------------------------------------------------------------
    def push(self,name,value):
        self._telegraf.digest(name, value)

    # ------------------------------------------------------------------------------------------------
    # -------------------------------------INTERNAL FUNCTIONS----------------------------------------
    # ------------------------------------------------------------------------------------------------

    def _EnterInitializationModeFcn(self):
        if self._verbose: print("Enter initializationModeFcn not implemented")

    def _ExitInitializationModeFcn(self):
        if self._verbose: print("Exit initializationModeFcn not implemented")

    def _EnterConfigurationModeFcn(self):
        if self._verbose: print("Enter configurationModeFcn not implemented")

