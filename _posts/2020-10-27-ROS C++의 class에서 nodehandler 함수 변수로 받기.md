---
layout: post
title: ROS C++의 class에서 nodehandler 함수 변수로 받기
subtitle: 포인터를 통한 nodehandler 공유
tags: [ROS, C++, Class]
comments: true
description: ROS C++ 클래스에서 ros::NodeHandle을 여러 모듈이 공유해야 할 때, 생성자에 NodeHandle 포인터를 넘기고 멤버 참조로 받는 패턴을 코드 예시와 함께 정리한다.
permalink: /2020/10/27/ros-cpp-class-nodehandle-pointer/
redirect_from:
  - '/2020-10-27-ROS C++의 class에서 nodehandler 함수 변수로 받기/'
  - '/2020-10-27-ROS-C++의-class에서-nodehandler-함수-변수로-받기/'
---

ROS 개발을 하다보면 하나의 ros::NodeHandle을 여러 군데에서 공유하고 싶을 때가 있다.
이럴 때는 아래에서처럼 NodeHandle의 주솟값을 함수의 입력으로 넘겨주면 된다.

아래 링크에서 사용 예제 확인 가능

https://github.com/wsnewman/ros_class/blob/master/example_ros_class/src/example_ros_class.cpp

```cpp
int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "exampleRosClass"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type ExampleRosClass");
    ExampleRosClass exampleRosClass(&nh);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("main: going into spin; let the callbacks do all the work");
    ros::spin();
    return 0;
} 
```
```cpp
//CONSTRUCTOR:  this will get called whenever an instance of this class is created
// want to put all dirty work of initializations here
// odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
ExampleRosClass::ExampleRosClass(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("in class constructor of ExampleRosClass");
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();
    initializeServices();
    
    //initialize variables here, as needed
    val_to_remember_=0.0; 
    
    // can also do tests/waits to make sure all required services, topics, etc are alive
}
```
