#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import genpy

class Rate(object):
    """
    Convenience class for sleeping in a loop at a specified rate
    """
    
    def __init__(self, hz, reset=False):
        """
        Constructor.
        @param hz: hz rate to determine sleeping
        @type  hz: float
        @param reset: if True, timer is reset when rostime moved backward. [default: False]
        @type  reset: bool
        """
        self.last_time = get_time()
        self.sleep_dur = Duration(0, int(1e9/hz))
        self._reset = reset

    def _remaining(self, curr_time):
        """
        Calculate the time remaining for rate to sleep.
        @param curr_time: current time
        @type  curr_time: L{Time}
        @return: time remaining
        @rtype: L{Time}
        """
        # detect time jumping backwards
        if self.last_time > curr_time:
            self.last_time = curr_time

        # calculate remaining time
        elapsed = curr_time - self.last_time
        return self.sleep_dur - elapsed

    def remaining(self):
        """
        Return the time remaining for rate to sleep.
        @return: time remaining
        @rtype: L{Time}
        """
        curr_time = get_time()
        return self._remaining(curr_time)

    def sleep(self):
        """
        Attempt sleep at the specified rate. sleep() takes into
        account the time elapsed since the last successful
        sleep().
        
        @raise ROSInterruptException: if ROS shutdown occurs before
        sleep completes
        @raise ROSTimeMovedBackwardsException: if ROS time is set
        backwards
        """
        curr_time = get_time()
        try:
            sleep(self._remaining(curr_time))
        except Exception as err:
            if not self._reset:
                raise
            self.last_time = get_time()
            return
        self.last_time = self.last_time + self.sleep_dur

        # detect time jumping forwards, as well as loops that are
        # inherently too slow
        if curr_time - self.last_time > self.sleep_dur * 2:
            self.last_time = curr_time


class Duration(genpy.Duration):
    """
    Duration represents the ROS 'duration' primitive type, which
    consists of two integers: seconds and nanoseconds. The Duration
    class allows you to add and subtract Duration instances, including
    adding and subtracting from L{Time} instances.

    Usage::
      five_seconds = Duration(5)
      five_nanoseconds = Duration(0, 5)

      print 'Fields are', five_seconds.secs, five_seconds.nsecs

      # Duration arithmetic
      ten_seconds = five_seconds + five_seconds
      five_secs_ago = rospy.Time.now() - five_seconds # Time minus Duration is a Time

      true_val = ten_second > five_seconds
    """
    __slots__ = []

    def __init__(self, secs=0, nsecs=0):
        """
        Create new Duration instance. secs and nsecs are integers and
        correspond to the ROS 'duration' primitive type.

        @param secs: seconds
        @type  secs: int
        @param nsecs: nanoseconds
        @type  nsecs: int
        """
        super(Duration, self).__init__(secs, nsecs)

    def __repr__(self):
        return 'rospy.Duration[%d]' % self.to_nsec()

class Time(genpy.Time):
    """
    Time represents the ROS 'time' primitive type, which consists of two
    integers: seconds since epoch and nanoseconds since seconds. Time
    instances are mutable.

    The L{Time.now()} factory method can initialize Time to the
    current ROS time and L{from_sec()} can be used to create a
    Time instance from the Python's time.time() float seconds
    representation.

    The Time class allows you to subtract Time instances to compute
    Durations, as well as add Durations to Time to create new Time
    instances.

    Usage::
      now = rospy.Time.now()
      zero_time = rospy.Time()

      print 'Fields are', now.secs, now.nsecs

      # Time arithmetic
      five_secs_ago = now - rospy.Duration(5) # Time minus Duration is a Time
      five_seconds  = now - five_secs_ago  # Time minus Time is a Duration
      true_val = now > five_secs_ago

      # NOTE: in general, you will want to avoid using time.time() in ROS code
      import time
      py_time = rospy.Time.from_sec(time.time())
    """
    __slots__ = []    

    def __init__(self, secs=0, nsecs=0):
        """
        Constructor: secs and nsecs are integers and correspond to the
        ROS 'time' primitive type. You may prefer to use the static
        L{from_sec()} and L{now()} factory methods instead.
        
        @param secs: seconds since epoch
        @type  secs: int
        @param nsecs: nanoseconds since seconds (since epoch)
        @type  nsecs: int
        """
        super(Time, self).__init__(secs, nsecs)
        
    def __repr__(self):
        return 'Time[%d]' % self.to_nsec()

    @staticmethod
    def now():
        """
        Create new L{Time} instance representing current time. This
        can either be wall-clock time or a simulated clock. It is
        strongly recommended that you use the now() factory to create
        current time representations instead of reading wall-clock
        time and create Time instances from it.
        
        @return: L{Time} instance for current time
        @rtype: L{Time}
        """
        return get_time()

    @classmethod
    def from_seconds(cls, float_secs):
        """
        Use Time.from_sec() instead. Retained for backwards compatibility.
        
        @param float_secs: time value in time.time() format
        @type  float_secs: float
        @return: Time instance for specified time
        @rtype: L{Time}
        """
        return cls.from_sec(float_secs)


def get_time():
    """
    Get the current time as a L{Time} object    
    @return: current time as a L{rospy.Time} object
    @rtype: L{Time}
    """

    """
    if not _rostime_initialized:
        raise rospy.exceptions.ROSInitException("time is not initialized. Have you called init_node()?")
    if _rostime_current is not None:
        # initialize with sim time
        return _rostime_current
    else:
        # initialize with wallclock
        float_secs = time.time()
        secs = int(float_secs)
        nsecs = int((float_secs - secs) * 1000000000)
        print("\n\nSecs = {0},\tNSecs = {1}".format(secs, nsecs))
        return Time(secs, nsecs)
    """

    float_secs = time.time()
    secs = int(float_secs)
    nsecs = int((float_secs - secs) * 1000000000)
    print("\n\nSecs = {0},\tNSecs = {1}".format(secs, nsecs))
    return Time(secs, nsecs)

def get_time():
    """
    Get the current time as float secs (time.time() format)
    @return: time in secs (time.time() format)    
    @rtype: float
    """
    return Time.now().to_sec()

if __name__ == '__main__':
    #try:
    rospy.init_node('temp_test', anonymous=True)
    x = 0
    rate = Rate(1)
    while x < 100:
        x += 1

        rate.sleep()

    """
    except Exception as err:
        print("Error = {}!".format(err))
    """
