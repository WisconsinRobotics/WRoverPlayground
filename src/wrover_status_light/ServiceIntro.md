# About ROS Services

After completing the Topics training, you should be familiar with the following:
- Writing and running ROS nodes
- Applications and implementations for publishers and subscribers
- Debug tools such as `rqt_graph` and `rqt_plot`

After completing the ROS Service training, you should be familiar with:
- What at ROS service is
- Purpose of ROS services
- Writing and running ROS server and clients
- Defining custom service types

## What is a ROS Service?
Services are another form of communication between different ROS nodes. Common uses of services include sending requests for specific pieces of data, or notify another node to perform specific actions.  A good way to conceptualize a service is as a remote function call.

## What is the Makeup of a ROS Service?

ROS services follow a client/server structure, where one node acts as a client that sends a request and another node acts as a server that receives the request and provides a response.

Here is a list of key components involved in services:
* Service definition: services are defined using a srv file. Similar to msg files for ROS topics, the srv file defines the name of a service, the data structure of the request message, and the data structure of the response message. 
* Client node: the client node in responsible for initiating the service request. It creates the request message, populates it with the required data, and sends it to the server node. The client then waits for the server to process the request and replies with a response.
* Server node: the server node receives and handles the service request calls from client nodes. It retrieves data from the request message, performs the necessary actions, and generates a response message which is sent back to the client. 
* Request and response messages: these messages are structured data containers that are defined by the service definition. The client and server use these messages to exchange information.

## Why do we use Services when there's already Topics?

While both services and topics work to allow communication between nodes, they differ in terms of their data exchange patterns and the types of interactions they support. 

Unlike topics, which operate asynchronously, service calls provide synchronous communication. The client node waits for the server node to process the request and receive a response before continuing its execution. This synchronous behavior is beneficial in cases where a response is required before proceeding with the next steps of a program.  It is not acceptable for a server to not exist or to never return a response; these will generate exceptions for the service client to handle.

In addition, topics are designed for data streaming and broadcasting continuous streams of information, such as sensor data or robot state updates. They are ideal when nodes require a constant flow of information and do not require immediate responses. On the other hand, services are typically used for accessing functionality that is provided by another node. They are well-suited for situations where nodes need to invoke specific actions or request information from another node, and a response is expected.

## Defining a Service
Services are defined by srv files, which are similar to msg files in terms of formatting except they contain two parts:
```
#request fields
insert your fields for request messages here as follows:
[data type] [field name]
[data type] [field name]
...
---
#response fields
insert your fields for response messages here as follows:
[data type] [field name]
[data type] [field name]
...
```
Fields are listed in the srv file each on its individual line, with a `---` separating the request fields from the response fields. Both requests and responses can have a single field, multiple fields, or no fields at all. All srv files for a module should be located in the `/srv` folder for proper complication. 

After defining a message and building this project, ROS would convert a srv file into three classes: service definition, request messages, and response messages. Suppose we have `ExampleService.srv` in an package called `ExamplePackage`, then the names of these packages would be:
|                   | C++                                     | Python                                    |
|---                 | ---                                     | ---   |
| Service definition |`ExamplePackage::ExampleService`          |`ExamplePackage.srv.ExampleService`        |
| Request message    |`ExamplePackage::ExampleService::Request` |`ExamplePackage.srv.ExampleServiceRequest` |
| Response message   |`ExamplePackage::ExampleService::Response`|`ExamplePackage.srv.ExampleServiceResponse`|

## Writing Server and Client nodes
Inside this module are two demo code files in the `src` folder, one in Python and one in C++. The server code creates a service server node that takes in an interger and doubles it. The client code creates a service client node that uses the server to double a number on repeat. Note that since the service uses `int64` (a integer with 64 bits), running the client code will eventually reach an integer overflow. 

Read through the demo files to understand how to write server and client nodes. Come back to this file and continue when you are ready.  

### C++ Server
A basic server node can be initialized in C++ with:
``` C++
ros::init(argc, argv, "double_server");
ros::NodeHandle nh;
ros::ServiceServer service = nh.advertiseService("double_service", doubleServiceCallback);
ros::spin();

bool doubleServiceCallback(wrover_status_light::DoubleServiceRequest& req, wrover_status_light::DoubleServiceResponse& res) {
    ...
}
```
To break the code down:
``` C++
ros::init(argc, argv, "double_server");
ros::NodeHandle nh;
```
This creates a new node called `double_server` and a new NodeHandle reference. NodeHandles provide functions for creating, managing, and interacting with ROS nodes.
``` C++
ros::ServiceServer service = nh.advertiseService("double_service", doubleServiceCallback);
```
This creates a new service with the name `double_service` and callback function `doubleServiceCallback`.
``` C++
ros::spin();
```
This ensures that the code will run continuously and the service will be available unless the code crashed or is exited. 
```C++
bool doubleServiceCallback(wrover_status_light::DoubleServiceRequest& req, wrover_status_light::DoubleServiceResponse& res) {
    ...
}
```
This defines the handler function of the server, which will be called every time the server receives a new request message. Handler functions for services in C++ always takes in two arguments, one for the request message and one for the response message, and returns a boolean to indicate if the operation is successful or has failed. 

### C++ Client
A basic client node can be initialized in C++ with:
``` C++
ros::init(argc, argv, "double_client");
ros::NodeHandle nh;
ros::ServiceClient client = nh.serviceClient<wrover_status_light::DoubleService>("double_service");
ros::service::waitForService("double_service");
wrover_status_light::DoubleService srv;
client.call(...)
```
To break the code down:
``` C++
ros::init(argc, argv, "double_client");
ros::NodeHandle nh;
```
This creates a new node called `double_client` and a new NodeHandle reference. NodeHandles provide functions for creating, managing, and interacting with ROS nodes.
``` C++
ros::ServiceClient client = nh.serviceClient<wrover_status_light::DoubleService>("double_service");
```
This creates a new client to the service with the name `double_service` that is defined by the service class `wrover_status_light::DoubleService`. 
``` C++
ros::service::waitForService("double_service");
```
This waits until the service is created and ready. 
``` C++
wrover_status_light::DoubleService srv;
client.call(...)
```
This creates a new object for the service and uses the previously made client object to call the service. 

### Python Server
A basic server node can be initialized in Python with: 
``` Python
rospy.init_node("double_server")
s = rospy.Service("double_service", DoubleService, double_service_handler)
rospy.spin()

def double_service_handler(request: DoubleServiceRequest):
    ...
```
To break the code down: 
``` Python
rospy.init_node("double_server")
```
This creates a new node called `double_server`.
``` Python
s = rospy.Service("double_service", DoubleService, double_service_handler)
```
This declares a new service called `double_service` with the `DoubleService` service type, and uses the `double_service_handler` function to process all incoming request messages.
``` Python
rospy.spin()
```
This ensures that the code will run continuously and the service will be available unless the code crashed or is exited. 
``` Python
def double_service_handler(request: DoubleServiceRequest):
    ...
```
This defines the handler function of the server, which will be called every time the server receives a new request message. Handler functions for services in Python always takes in the request message as an argument and returns the response message.

### Python Client
A basic client node can be initialized in Python with: 
``` Python
rospy.init_node("service_client")
double_service = rospy.ServiceProxy("double_service", DoubleService)
rospy.wait_for_service("double_service")
response:DoubleServiceResponse = double_service(num)
```
To break the code down: 
``` Python
rospy.init_node("service_client")
```
This creates a new node called `service_client`.
``` Python
double_service = rospy.ServiceProxy("double_service", DoubleService)
```
This creates a handle to call the service. Here we specify the name of the service, `double_service`, and the type of the service, `DoubleService`. 
``` Python
rospy.wait_for_service("double_service")
```
This waits until the service is created and ready. 
``` Python
response:DoubleServiceResponse = double_service(...)
```
This calls the service with a request message as input, and saves the returned response message. 

## Running Server and Client nodes
### C++
To run the C++ code:
1. Navigate to this repository using `cd {your file path}/WRoverPlayground`.
2. Run `catkin_make` to build the project if you haven't yet.
3. Run `source devel/setup.bash` to source the project if you haven't yet. Also keep in mind that you have to resource every time you open a new terminal. 
4. Run `roscore` to start ROS master.
5. To run the server code, in a new terminal, run `rosrun wrover_status_light DemoServer`.
6. To see a list of all services, in a new terminal, run `rosservice list`. You should see `/double_service` listed. Now would also be a good opportunity to run `rosservice help` and familiarize yourself with all the `rosservice` commands.
7. You can run the service manually from the command line using `rosservice call`. In this case, try running `rosservice call /double_service "input: [insert your number here]"`. You should see that the inputted number is doubled. 
8. To run the client code, run `rosrun wrover_status_light DemoClient`. You should see powers of 2 being printed. Eventually the program will print `Integer overflow encountered`. This is because the response/request messages use a 64-bit integer, and as the number is doubled eventually we will reach integer overflow.
9. Use `Ctrl-C` to exit out of any programs

### Python
To run the Python code:
1. Navigate to this repository using `cd {your file path}/WRoverPlayground`.
2. Run `catkin_make` to build the project if you haven't yet.
3. Run `source devel/setup.bash` to source the project if you haven't yet. Also keep in mind that you have to resource every time you open a new terminal. 
4. Run `roscore` to start ROS master.
5. To run the server code, in a new terminal, run `rosrun wrover_status_light DemoServer.py`.
6. To see a list of all services, in a new terminal, run `rosservice list`. You should see `/double_service` listed. Now would also be a good opportunity to run `rosservice help` and familiarize yourself with all the `rosservice` commands.
7. You can run the service manually from the command line using `rosservice call`. In this case, try running `rosservice call /double_service "input: [insert your number here]"`. You should see that the inputted number is doubled. 
8. To run the client code, run `rosrun wrover_status_light DemoClient.py`. You should see powers of 2 being printed. Eventually the program will print `Integer overflow encountered`. This is because the response/request messages use a 64-bit integer, and as the number is doubled eventually we will reach integer overflow.
9. Use `Ctrl-C` to exit out of any programs

## Challenge
**TODO**

## Notes
ROS's wiki page on services: http://wiki.ros.org/Services
ROS tutorial on writing a service and client in C++: http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29
ROS tutorial on writing a service and client in Python: http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29