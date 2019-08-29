#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from diagnostic_msgs.msg import *
from threading import Thread

print("ultrasound_monitor.py initialised", sys.version)

"""static variables"""
distance_limit = 0.5  # metres
message_buffer_length = 3  # can be used to dynamically change the size of the message buffer
max_possible_reading = 5  # meters, sets max limit. above this an error will be thrown


class UltrasoundData:

    def __init__(self, lower_limit=0.0, upper_limit=100.0, shield_left=-0.0, shield_right=-0.0,
                 side_left=-0.0, side_right=0):

        self.shield_left = shield_left
        self.shield_right = shield_right
        self.side_left = side_left
        self.side_right = side_right

        self.lower_limit = lower_limit
        self.upper_limit = upper_limit

    def filter_var(self, ultrasound_reading):

        if ultrasound_reading < self.lower_limit:
            ultrasound_reading = self.lower_limit
        if ultrasound_reading < self.upper_limit:
            pass
        else:
            rospy.logerr("buffer length is %s should be %s", len(self.safety_status_array), message_buffer_length)

        return ultrasound_reading

    def get_shield_left(self):
        """automatically filters, returns stored data"""
        return self.filter_var(self.shield_left)

    def get_shield_right(self):
        """automatically filters, returns stored data"""
        return self.filter_var(self.shield_right)

    def get_side_left(self):
        """automatically filters, returns stored data"""
        return self.filter_var(self.side_left)

    def get_side_right(self):
        """automatically filters, returns stored data"""
        return self.filter_var(self.side_right)


class Main:

    def __init__(self):

        """the class ultrasound_monitor listens to the diagnostics ROS topic and extracts the ultrasound information
        from the front and shield sensors. In this code, shield values are obtained but ignored. the obtained messages
         are first filtered to remove and 0.00 values, and then acted upon. Ultrasound readings below the distance_limit
         will cause a stop service to be called. then the program will continue listening for new messages. Once the
         low readings are no longer being received, a start service is called. message_buffer_size is used to determine
         how many messages are required to be received as safe before the start service is called. The buffering system
         also aims to reduce the number of service calls that are made"""

        """declaring variables"""

        self.movement_paused = False
        self.safety_status_array = [True] * message_buffer_length  # creates an array of True booleans
        """begin initialising code segments"""
        self.init_publisher()
        self.init_subscriber()

        self.ultrasound_data = UltrasoundData(distance_limit)

    def init_publisher(self):

        """"it is possible this should be turned into a srv once Adam has sorted this out"""
        rospy.Publisher('Ultrasound_monitor_halt_msg', String, queue_size=10)  # not used yet

    def init_subscriber(self):
        """listens to diagnostics to collect ultrasound data"""

        rospy.Subscriber("diagnostics", DiagnosticArray, self.diagnostic_callback)
        rospy.spin()

    def diagnostic_callback(self, array):
        """interprets the data from diagnostics to establish the two ultrasound readings and
         passes them to the processing method"""

        try:
            for s in array.status:
                if "LightCtrl: Ultrasonic" in s.name:
                    for v in s.values:
                        if "Front shield right" in v.key:
                            self.ultrasound_data.shield_right = float(v.value)  # type check original
                        if "Front shield left" in v.key:
                            self.ultrasound_data.shield_left = float(v.value)
                        if "Front side right" in v.key:
                            self.ultrasound_data.side_right = float(v.value)
                        if "Front side left" in v.key:
                            self.ultrasound_data.side_left = float(v.value)
        except rospy.ROSInterruptException:
            print("error while subscribing")
        self.diagnostic_processing()

    def diagnostic_processing(self):

        """this section processes the messages it receives and calls the start or stop service publisher as appropriate.
         Some logic handle is performed by the publishers to ensure that the situation is only updated upon a
         successful service message delivery. """

        if self.ultrasound_data.get_side_right() < distance_limit or\
                self.ultrasound_data.get_side_left() < distance_limit:
            self.safety_status_array.append(False)
        else:
            self.safety_status_array.append(True)
        del self.safety_status_array[0]  # gets rid of old messages, keeps buffer size the same

        if False in self.safety_status_array and self.movement_paused is False:
            # robot has ability to move but
            self.stop_service_publisher()

        if False not in self.safety_status_array and self.movement_paused is True:
            self.start_service_publisher()

    def stop_service_publisher(self):

        """send a stop service to the appropriate topic to stop the robot when called by the callback
        In addition it sets the movement paused bool to True once the service has been delivered"""
        print("stop message function called")
        print("side_right is ", self.side_right, "side_left is ", self.side_left, )
        print("safety status array is ", self.safety_status_array, "and movement_paused is ", self.movement_paused)

        # send function
        self.movement_paused = True

    def start_service_publisher(self):

        """send a start service to the appropriate topic to stop the robot when called by the callback
        In addition it sets the movement paused bool to False once the service has been delivered"""
        print("start message function called")

        # need to actually add the service call once implementation is understood

        print("side_right is ", self.side_right, "side_left is ", self.side_left,)
        print("safety status array is ", self.safety_status_array, "and movement_paused is ", self.movement_paused)

        # send start service  # get the mir to be allowed to move again

        self.movement_paused = False 


if __name__ == '__main__':
    rospy.init_node('ultrasound_monitor', anonymous=True)
    try:
        Main()

    except KeyboardInterrupt:
        rospy.loginfo("Node terminated.")
