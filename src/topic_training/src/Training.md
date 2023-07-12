# WRoverPlayground - Topic Training
Now that you have completed the Setup Training, you are ready to learn about ROS topics. From the Setup Training, you should be familiar with:
(*list out what newcomers should already know*)
- ROS Nodes
- etc.
- these ROS commands:
    - .
Once you complete this training, you should be familiar with:
(*list out what newcomers should learn*)
- topics
- pub/sub relationship
- these ROS commands:
    - .

## The Purpose of Topics
The word "topic" sounds very general, but it is actually a specific concept that allows ROS nodes to communicate with each other. 

## Publisher/Subscriber Relationship
The Publisher and Subscriber are both nodes that are connected by a topic, allowing them to send messages between themselves. The Publisher node publishes messages on the topic, while the Subscriber node subscribes to the same topic to receive the Publisher's messages. 

Publishers and Subscribers do not need to be in separate files. In fact, if the Publisher and Subscriber rely on each other's data, it may be better to keep them in the same file. An example of the Publisher and Subscriber residing in the same file will be provided in the next section.

## Setting up a Publisher and Subscriber
Within this directory, you will find two files called PubSubDemo. These files will show a basic class containing both a Publisher and Subscriber. There will be comments throughout the files that explain the purpose of specific sections of the code.

One file will be written in C++, and the other will be in Python. I recommend discussing with one of the Software Leads to determine which part of the rover you would like to be assigned to. This will help you decide which language to focus on for this training. However, being familiar with both languages will not hurt.

Go through one of the files, making sure you understand all the code and comments. Come back to this file once you feel ready to move on.

## Defining a Custom Topic Type

## Challenge
