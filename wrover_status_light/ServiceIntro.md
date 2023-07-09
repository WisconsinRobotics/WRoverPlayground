# About ROS Services
## What is a ROS Service?
Services are another form of communication between different software componenents, or more fomally known as nodes, that ROS offers. Common uses of services include sending requests for specific pieces of data, or notify another node to perform specific actions. 

## What is the Makeup of a ROS Service?

ROS services follow a client/server structure, where one node acts as a client that sends a request and another node acts as a server that receives the request and provides a response.

Here is a list of key components involved in services:

* Service definition: services are defined using a .srv file. Similar to .msg files for ROS topics, the .srv file defines the name of a service, the data structure of the request message, and the data structure of the response message. 
* Client node: the client node in responsible for initiating the service request. It creates the request message, populates it with the required data, and sends it to the server node. the client then waits for the server to process the request and replies with a response.
* Server node: the server node receives and handles the service request calls from client nodes. It retrieves data from the request message, performs the necessary actions, and generates a response message which is sent back to the client. 
* Request and response messages: these messages are structured data containers that are defined by the service definition. The client and server use these messages to exchange information.

## (Optional) Why do we use Services when there's already Topics?

While both services and topics work to allow communication between nodes, they differ in terms of their data exchange patterns and the types of interactions they support. 

Unlike publishers and subscribers over topics, which operate asynchronously, service calls provide synchronous communication. The client node waits for the server node to process the request and receive a response before continuing its execution. This synchronous behavior is beneficial in cases where a response is required before proceeding with the next steps of a program.

In addition, topics are designed for data streaming and broadcasting continuous streams of information, such as sensor data or robot state updates. They are ideal when nodes require a constant flow of information and do not require immediate responses. On the other hand, services are typically used for accessing functionality that is provided by another node. They are well-suited for situations where nodes need to invoke specific actions or request information from another node, and a response is expected.

## Defining a Service

## Writing a Server Node

## Writing a Client Node

ROS's wiki page on services: http://wiki.ros.org/Services