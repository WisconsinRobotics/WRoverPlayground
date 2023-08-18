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
Services are defined by srv files, which are identical to msg files in terms of formatting, and should be located in the `/srv` folder in a package. In this example, the service takes in a `string` and an `int` as input and returns a `float` as output. You can find an example service file at `srv/DoubleService.srv`.
```
#request fields
string input1
int32 input2
---
#response fields
float32 output
```
After defining a message and building this project, ROS would convert a srv file into three classes: service definition, request messages, and response messages. Suppose we have `ExampleService.srv` in an package called `ExamplePackage`, then the names of these packages would be:
|                   | C++                                     | Python                                    |
|---                 | ---                                     | ---   |
| Service definition |`ExamplePackage::ExampleService`          |`ExamplePackage.srv.ExampleService`        |
| Request message    |`ExamplePackage::ExampleService::Request` |`ExamplePackage.srv.ExampleServiceRequest` |
| Response message   |`ExamplePackage::ExampleService::Response`|`ExamplePackage.srv.ExampleServiceResponse`|

## Writing Server and Client nodes
On how to write server and client nodes, see the demo files in the `src` folder for example code and explanations. Come back to this file and continue when you are done going through them. 

## Running Server and Client nodes
### Python

### C++

## Challenge
**TODO**

## Notes
ROS's wiki page on services: http://wiki.ros.org/Services
ROS tutorial on writing a service and client in C++: http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29
ROS tutorial on writing a service and client in Python: http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29