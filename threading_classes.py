'''
A few convenient classes for multithreading.
'''

from threading import Thread, Lock, Event
import threading
from multiprocessing import Process, Value
import multiprocessing as mp

class LockedVar:
    '''
    Minimal class to implement a locking variable. Contains two private attributes, a value and a lock, and a few methods for safetly reading writing value via the lock.
    '''

    def __init__(self, val):
        self._value = val
        self._lock = Lock()

    def locked_read(self):
        with self._lock:
            return self._value

    def locked_update(self, val):
        with self._lock:
            self._value = val

class StoppableThread(Thread):
    '''
    Thread class with a stop() method. The thread itself has to check
    regularly for the stopped() condition.
    '''

    def __init__(self,  *args, **kwargs):
        super(StoppableThread, self).__init__(*args, **kwargs)
        self._stop_event = Event()

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()

class LockedVal:
    '''
    Minimal class to implement thread and process safe variables. Differs from LockedVar in that it uses both a threading and multiprocess lock at the same time.
    '''

    def __init__(self, val):
        self._value = val
        self._mplock = mp.Lock()
        self._thlock = Lock()

    def locked_read(self):
        with self._thlock:
            with self._mplock:
                return self._value

    def locked_update(self, val):
        with self._thlock:
            with self._mplock:
                self._value = val


class StoppableProcess(Process):
    '''
    Process class with a stop() method. The process itself has to check regularly for the stopped() condition
    '''

    def __init__(self, *args, **kwargs):
        super(StoppableProcess, self).__init__(*args, **kwargs)
        self._stop_event = mp.Event()

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()
