#**********************************************************************************
# * Copyright (C) 2024-present Bert Van Acker (B.MKR) <bert.vanacker@uantwerpen.be>
# *
# * This file is part of the roboarch R&D project.
# *
# * RAP R&D concepts can not be copied and/or distributed without the express
# * permission of Bert Van Acker
# **********************************************************************************

class DiagnosisExecutor(object):

    def __init__(self,verbose=False):
        """Initialize the generic DiagnosisExecutor component.

                Parameters
                ----------
                verbose : bool
                    component verbose execution

        See Also
        --------
        ..

        Examples
        --------
        >> cm = DiagnosisExecutor(verbose=False)

        """

        self._name = "Diagnosis Executor"
        self._description = "The diagnosis executor handles the execution of the determined diagnosis routine for the managed system, triggered from within the RoboSapiens Adaptive Platform."
        self._verbose = verbose

    @property
    def name(self):
        """The name property (read-only)."""
        return self._name

    @property
    def description(self):
        """The description property (read-only)."""
        return self._description


    #------------------------------------------------------------------------------------------------
    # -------------------------------------INTERFACE FUNCTIONS---------------------------------------
    # ------------------------------------------------------------------------------------------------