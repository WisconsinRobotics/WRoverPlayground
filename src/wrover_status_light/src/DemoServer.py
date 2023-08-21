#!/usr/bin/env python

import rospy
from wrover_status_light.srv import DoubleService
from wrover_status_light.srv import DoubleServiceRequest
from wrover_status_light.srv import DoubleServiceResponse


'''
The handler function of the service. It will be called whenever this server receives a new 
request message. Handler functions are responsible for carrying out the actual logics of the 
service and send a response back to the client. Handler functions in Python always take in a 
single argument of the request message type, and returns an instance of the response message type.
'''
def double_service_handler(request: DoubleServiceRequest):
    '''
    In this case, the handler function doubles the input integer and returns it. 
    '''
    return request.input * 2

    '''
    Another way to return a response is: 
    return DoubleServiceResponse(request.input * 2)
    However, in this case, returning a response can be simplified into what is used above 
    because it only contains a single field. Valid return types from handler functions are: 
    - None (which implies a failure)
    - Response type to a service
    - A tuple or list
    - A dictionary
    - The value of the field for response types with a single field
    '''

'''
Initializes the node
'''
def double_service_server():
    '''
    Creates a new node called "double_server". Without this the code cannot communicate with ROS, 
    and is usually declared at the beginning.
    '''
    rospy.init_node("double_server")

    '''
    Creates a new service called "double_service" that is defined by the "DoubleService" class, 
    and uses the function "double_service_handler" to handle any requests it receives.
    '''
    s = rospy.Service("double_service", DoubleService, double_service_handler)

    '''
    The rospy.spin() function and other versions of spin() keeps the code running by entering a 
    loop that listens for incoming requests.
    '''
    rospy.spin()

if __name__ == "__main__":
    double_service_server()