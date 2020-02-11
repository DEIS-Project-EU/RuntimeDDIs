#pragma once

#include <map>
#include <string>

#include "ros/ros.h"
#include "ros/timer.h"

// Subscriber storage
std::map<std::string, ros::Subscriber>* subscribersMapPtr = nullptr;

// Timer storage
std::map<std::string, ros::Timer>* timersMapPtr = nullptr;

// Node globals
ros::Rate* loopRatePtr = nullptr;
ros::NodeHandle* nodePtr = nullptr;

// Locking for configuration changes during operation
std::mutex configMutex;

// Publisher storage
std::map<std::string, ros::Publisher>* publishersPtr = nullptr;

// Play/pause simulation
bool simulationIsRunning = true;

// Qt Visualizer
bool showQtVisualizer = true;

unsigned qtUIUpdateDelayMS = 0;

class QtVisualizer;
QtVisualizer* qtVisualizerPtr = nullptr;

class QTimer;
QTimer* updateVarTimerPtr = nullptr;
QTimer* rosQtTimerPtr = nullptr;