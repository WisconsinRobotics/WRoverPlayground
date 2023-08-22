#!/usr/bin/env python

import rospy
from wrover_status_light.srv import DoubleService
from wrover_status_light.srv import DoubleServiceRequest
from wrover_status_light.srv import DoubleServiceResponse

def main():
    '''
    Creates a new node called "service_client"
    '''
    rospy.init_node("service_client")

    '''
    Creates a new callable object for a service named "double_service" that is defined by the 
    "DoubleService" class.
    '''
    double_service = rospy.ServiceProxy("double_service", DoubleService)

    '''
    Initializes various local variables. rate specifies the frequency we would want to run at, 
    and num is the value that is going to be doubled. 
    '''
    rate = rospy.Rate(10)
    num = 1

    '''
    This is blocking call that waits until a service called "double_service" is created and ready.
    '''
    rospy.wait_for_service("double_service")

    '''
    The funciton rospy.is_shutdown() checks if the module is still running or not. 
    '''
    print(num)
    while not rospy.is_shutdown():

        '''
        Here we use a try-except block to guard against any potential exceptions that might occur 
        when interacting with the service. 
        '''
        try:
            '''
            Calls the service using the previously created service object and saves the response in 
            a local variable. Similar to the server's handler function, here we simplify the code by 
            not explicitly creating a new DoubleServiceRequest instance since it only contains one
            field.
            '''
            response:DoubleServiceResponse = double_service(num)
            num = response.output
            print(num)
        except:
            '''
            If calling the service causes an exception, this code will run instead
            '''
            print("Integer overflow encountered")

        '''
        Waits for 100ms before resuming the program. This ensures that the loop runs at 10hz, 
        which was provided in the constructor.
        '''
        rate.sleep()

if __name__ == "__main__":
    main()