# About ROS Services
## What is a ROS Service?
Services are another form of communication between different software componenents, or more fomally known as nodes, that ROS offers. Common uses of services include sending requests for specific pieces of data, or notify another node to perform specific actions. 

## What is the Makeup of a ROS Service?

ROS services follow a client/server structure, where one node acts as a client that sends a request and another node acts as a server that receives the request and provides a response.

Here is a list of key components involved in services:
* Service definition: services are defined using a srv file. Similar to msg files for ROS topics, the srv file defines the name of a service, the data structure of the request message, and the data structure of the response message. 
* Client node: the client node in responsible for initiating the service request. It creates the request message, populates it with the required data, and sends it to the server node. The client then waits for the server to process the request and replies with a response.
* Server node: the server node receives and handles the service request calls from client nodes. It retrieves data from the request message, performs the necessary actions, and generates a response message which is sent back to the client. 
* Request and response messages: these messages are structured data containers that are defined by the service definition. The client and server use these messages to exchange information.

## (Optional) Why do we use Services when there's already Topics?

While both services and topics work to allow communication between nodes, they differ in terms of their data exchange patterns and the types of interactions they support. 

Unlike publishers and subscribers over topics, which operate asynchronously, service calls provide synchronous communication. The client node waits for the server node to process the request and receive a response before continuing its execution. This synchronous behavior is beneficial in cases where a response is required before proceeding with the next steps of a program.

In addition, topics are designed for data streaming and broadcasting continuous streams of information, such as sensor data or robot state updates. They are ideal when nodes require a constant flow of information and do not require immediate responses. On the other hand, services are typically used for accessing functionality that is provided by another node. They are well-suited for situations where nodes need to invoke specific actions or request information from another node, and a response is expected.

## Defining a Service
Services are defined by srv files, which are identical to msg files in terms of formatting, and should be located in the `/srv` folder in a package. In this example, the service takes in a `string` and an `int` as input and returns a `float` as output. 
```
#request fields
string input1
int32 input2
---
#response fields
float32 output
```
After defining a message, ROS would convert the srv file into three classes: service definition, request messages, and response messages. Suppose we have `ExampleService.srv` in an package called `ExamplePackage`, then the names of these packages would be:
* Service definition: `ExamplePackage.srv.ExampleService` for Python and `ExamplePackage::ExampleService` for C++
* Request messages: `ExamplePackage.srv.ExampleServiceRequest` for Python and `ExamplePackage::ExampleService::Request` for C++
* Response messages: `ExamplePackage.srv.ExampleServiceResponse` for Python and `ExamplePackage::ExampleService::Request` for C++

## Writing a Server Node
### Python
A server code can be initialized with
```
def example_service_server():
    rospy.init_node('example_service_server')
    s = rospy.Service('example_service', ExamplePackage.srv.ExampleService, example_service_execution)
    rospy.spin()

def example_service_execution(request):
    return ExamplePackage.srv.ExampleServiceResponse(0.0)

example_service_server()
```
To break the code down, the function `example_service_server()` initializes the node.
```
rospy.init_node('example_service_server')
```
Creates a new node with the name of `example_service_server`.
```
rospy.Service('example_service', ExamplePackage.srv.ExampleService, example_service_execution)
```
Creates a new service with the name of `example_service` that is defined by the `ExamplePackage.srv.ExampleService` class. When the service is called, the `example_service_execution` method will be called to handle the request. 
```
rospy.spin()
```
Keeps the code running by entering a loop that listens for incoming requests. 

The function `example_service_execution()` is the handler function for this service and is called whenever the node receives a new request message. Handler functions are responsible for carrying out the actual logics of the service and send a response back to the client. Handler functions always take in a single argument of the request message type, and returns an instance of the response message type.

```
return ExamplePackage.srv.ExampleServiceResponse(0.0)
```
In this example, the handler function always returns a hardcoded response of 0.0. If the handler function wants to use the request inputs, they can be accessed with `request.input1` or `request.input2`.

Returning response messages can also be streamlined by not having to manually create a new instance. In this case, simply 
```
return 0.0
``` 
would also work. 

### C++
A server node can be initialized with
```
bool ExampleServiceServer::callback(ExamplePackage::ExampleService::Request& request, ExamplePackage::ExampleService::Response& response) {
    return 0.0;
}

ExamplePackage::ExampleService exampleService;
ros::NodeHandle nh;
ros::ServiceServer service = nh.advertiseService("example_service", &ExampleServiceServer::callblack, &exampleService);
```
To break the code down, the function `ExampleServiceServer::callback()` is the handler function of the server, and is ran every time the server receives a new request. The handler function must take in two arguments, a request and a response.
```
ExmaplePackage::ExampleService exampleService;
ros::NodeHandle nh;
ros::ServiceServer service = nh.advertiseService("example_service", &ExampleServiceServer::callback, &exampleService);
```
The code sets up the service server using a `NodeHandle` instance. NodeHandles provide functions for creating, managing, and interacting using ROS nodes. Here the `NodeHandle` creates a new server with `example_service` as the server name, `ExampleServiceServer::callback` as the handler function for service calls, and `exampleService` defines the service type. 

## Writing a Client Node
### Python
Services can be called by 
```
rospy.wait_for_service('example_service')
example_service = rospy.ServiceProxy('example_service', ExamplePackage.srv.ExampleService)
response = example_service(ExamplePackage.srv.ExampleServiceRequest(..., ...))
```
To break the code down,
```
rospy.wait_for_service('exmaple_service')
```
This is a blocking operation that waits until a service with the name `example_service` is created. 
```
example_service = rospy.ServiceProxy('example_service', ExamplePackage.srv.ExampleService)
```
Creates a new callable proxy object for the service named `example_service` and defined by `ExmaplePackage.srv.ExampleService`.
```
response = example_service(ExamplePackage.srv.ExampleServiceRequest(..., ...))
```
Calls the service using an instance of the request message class and saves the response in a local variable. Similar to before, the service can also be called without explicitly making a new instance with
```
response = exmaple_service(..., ...)
```

### C++
Services can be called by
```
ros::NodeHandle nh;
ros::ServiceClient client = nh.serviceClient<Example_Package::Example_Service>("example_service");

Example_Package::Example_Service srv;
srv.request.input1 = ...
srv.request.input2 = ...
if (client.call(srv)) {
    ...
} else {
    ...
}
```
To break the code down
```
ros::NodeHandle nd;
ros::ServiceClient client = nh.serviceClient<Example_Package::Example_Service>("example_service");
```
The code first creates a new NodeHandle instance. Here the NodeHandle instance creates a new ServiceClient using the type `Example_Package::Example_Service` and service name `example_service`.
```
Example_Package::Example_Service srv;
srv.request.input1 = ...
srv.request.input2 = ...
```
The code then creates a new `Example_Service` instance and populates the request variables.
```
if (client.call(srv)) {
    ...
} else {
    ...
}
```
Lastly, the code calls the service using the `call()` function. If the service call is successful, then the response value can be accessed using `srv.response`. If it is unsuccessful, then the code will go into the else block.

## Notes
ROS's wiki page on services: http://wiki.ros.org/Services