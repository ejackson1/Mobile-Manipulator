#!/usr/bin/env python3

import rospy
from std_msgs.msg import int32, Bool, String
from base_package.msg import encoder_values
from base_package.srv import moveToElev, moveToElevResponse
from queue import Queue
import time



class baseServer:
    def __init__(self, init_node=False) -> None:
        if init_node:
            rospy.init_node("base_server", anonymous=True)
        
        # Elevator Service
        self.elev = rospy.Service('/elevation_request', moveToElev, self.move_elevators_linear)
        self.setpoint = rospy.Publisher("/base/elevator_setpoint", int32, queue_size=10)
        rospy.Subscriber("/base/elevator_encoders", encoder_values, self.encoder_callback)
        self.cache_length = 5
        self.elev_tolerance = 200
        self.timeout_duration = 45 # 45 seconds
        
        # Travel Service
        #   TODO


    
    def calculateOffset(self, values):
        # Returns the average offset of the jacks
        offset_indv = [None] * 3
        for count, jack_pos in enumerate(values):
                avg_encoder_offset = sum(values)/len(values) - jack_pos
                offset_indv[count] = avg_encoder_offset
        offset_avg = sum(offset_indv) / len(self.values)

        return offset_avg 
    
    def move_elevators_linear(self, height):
        # Publish height
        self.setpoint.publish(height.data)
        #reading_Cache = [None] * self.cache_length

        reading_Cache = []        
        
        bool = Bool()
        string = String()
        string.error = "" # no error

        last_offset_avg = self.calculateOffset(self.encoder_values)
        start_time = time.time()


        while True:
            avg_encoder = sum(self.encoder_values)/len(self.encoder_values)
            offset_avg = self.calculateOffset(self.encoder_values)
            
            if len(reading_Cache) < self.cache_length:
                reading_Cache.append(avg_encoder)
                
            avg_reading_cache = sum(reading_Cache) / len(reading_Cache)
                        
            if avg_reading_cache <= self.elev_tolerance/2 or avg_reading_cache >= self.elev_tolerance/2:
                # Elevator reached desired location
                bool.data = True
                return moveToElevResponse(bool, string)

            elif start_time + self.timeout_duration <= time.time():
                # Timeout, something went wrong
                bool.data = False
                string.error = "Timeout Exception."
                return moveToElevResponse(bool, string)

            elif offset_avg > last_offset_avg:
                # Jacks moved away from the goal position
                bool.data = False
                string.error = "Jacks moving away from goal, something is very wrong"
                return moveToElevResponse(bool, string)

            
            
            else:
                # Remove the first element and append the most current reading
                reading_Cache.pop(0)
                reading_Cache.append(avg_encoder)

            last_offset_avg = offset_avg # reinitalize offset avg

    
    def encoder_callback(self, msg):
        # Updates encoder values in function
        self.encoder_values = msg.Values
        


    if __name__ == "__main__":
        baseServer = baseServer()
        rospy.spin()
