#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from datetime import datetime

class TimerClass(object):
    def __init__(self, sleep_dur=0):
        self.__sleep_dur = sleep_dur
        self.__unit = 1000000000
        self.last_time = None # self.get_time()

    def __get_time(self, unit_sec=True):
        float_secs = time.time()     # time.time()      # time.perf_counter()
        self.__secs = int(float_secs)
        self.__nsecs = int((float_secs - self.__secs) * self.__unit)

        #print("\n-->Secs = {0},\tNSecs = {1}\n".format(self.secs, self.nsecs))

        """
        if unit_sec:
            return self.to_sec()

        else:
            return self.to_nsec()
        """

        return self.to_sec()

    def __remaining(self, curr_time):
        # detect time jumping backwards
        if self.last_time > curr_time:
            self.last_time = curr_time

        # calculate remaining time
        elapsed = curr_time - self.last_time
        return self.__sleep_dur - elapsed

    def sleep(self, secs=0):
        if secs != 0:
            self.__sleep_dur = secs
        #self.__sleep_dur = secs

        if self.last_time == None:
            self.last_time = self.__get_time()

        curr_time = self.__get_time()

        try:
            #remaining = self.__remaining(curr_time)
            time.sleep(self.__remaining(curr_time))
            #print("\nLast Time = {0}, Curr Time = {1}, Remaining = {2}\n".format(self.last_time, curr_time, remaining))

        except Exception as err:
            #print("Exception Err = {}".format(err))
            self.last_time = self.__get_time()
            return

        self.last_time += self.__sleep_dur
        #self.last_time = self.last_time + self.__sleep_dur

        # detect time jumping forwards, as well as loops that are
        # inherently too slow
        if curr_time - self.last_time > self.__sleep_dur * 2:   # * 2
            self.last_time = curr_time

        #self.last_time = curr_time

    @classmethod
    def from_sec(cls, float_secs):
        secs = int(float_secs)
        nsecs = int((float_secs - secs) * self.__unit)
        return cls(secs, nsecs)

    def to_sec(self):  # noqa: D200, D400, D401
        """
        :returns: time as float seconds (same as time.time() representation), ``float``
        """
        return float(self.__secs) + float(self.__nsecs) / 1e9

    def to_nsec(self):  # noqa: D200, D400, D401
        """
        :returns: time as nanoseconds, ``long``
        """
        return self.__secs * int(1e9) + self.__nsecs
