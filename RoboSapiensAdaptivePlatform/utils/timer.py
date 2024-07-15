#**********************************************************************************
# * Copyright (C) 2024-present Bert Van Acker (B.MKR) <bert.vanacker@uantwerpen.be>
# *
# * This file is part of the roboarch R&D project.
# *
# * RAP R&D concepts can not be copied and/or distributed without the express
# * permission of Bert Van Acker
# **********************************************************************************
from threading import Thread,Lock,Timer,Event
import traceback

class perpetualTimer():

   def __init__(self,t,hFunction):
      self.t=t
      self.hFunction = hFunction
      self.thread = Timer(self.t,self.handle_function)

   def handle_function(self):
      self.hFunction()
      self.thread = Timer(self.t,self.handle_function)
      self.thread.start()

   def start(self):
      self.thread.start()

   def cancel(self):
      self.thread.cancel()


class StoppableThread(Thread):
   def __init__(self):
      super().__init__()
      self._stop_event = Event()


   def stop(self):
      self._stop_event.set()

   def run(self):
      while not self._stop_event.is_set():
         try:
            self._runFcn()
         except:
            print("ERROR - function execution failed!")
            print(traceback.format_exc())
         # pass

   def _runFcn(self):
     print("RUNNING THREAD")