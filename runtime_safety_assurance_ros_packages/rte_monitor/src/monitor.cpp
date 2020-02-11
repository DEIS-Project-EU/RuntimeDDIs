/*
MIT License

Copyright (c) 2019 DEIS Project

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

// TODO:
// 1. Refactor global methods and variables into Buffer class

#include "ros/ros.h"
#include "ros/timer.h"
#include "rte_monitor/ConfidenceInputEvent.h"
#include "rte_monitor/RtEList.h"
#include "rte_monitor/Configuration.h"
#include "rte_monitor/ConfidenceInputEventWEnc.h"
#include "rte_list_monitor/ChangedSimulationState.h"

#include <string>
#include <sstream>
#include <mutex>
#include <array>
#include <algorithm>
#include <map>
#include <iostream>
#include <fstream>

#include "smile/smile.h"
#include "smile/smile_license/smile_license.h"

#include "cryptopp/integer.h"
#include "cryptopp/cryptlib.h"
#include "cryptopp/aes.h"
#include "cryptopp/modes.h"
#include "cryptopp/filters.h"
#include "cryptopp/hex.h"

#include "cryptopp/hmac.h"
#include "cryptopp/sha.h"

#include <QApplication>
#include <QLabel>
#include <QWidget>
#include <QTimer>
#include <QThread>

#include "constants.hpp"
// Dangerous: Global import only works if preceding buffer import
// TODO: Eliminate globals
#include "globals.hpp"
#include "buffer.hpp"

#include "qtvisualizer.hpp"

/*
#ifdef _DEBUG
	#define DEBUG_OUT(x) ROS_INFO(x)
#else
	#define DEBUG_OUT(x)
#endif
*/

// TODO: Eliminate 'global' monitor->buffer map
// Map of monitored variables to buffers
std::map<std::string, Buffer> monitorsToBuffers;

/****************************************************************************************************************************/
// Configuration commands
/****************************************************************************************************************************/
void addPublisherTopic(ros::NodeHandle& n, const std::string& topicName) {

	if (publishersPtr->find(topicName) != publishersPtr->end()) {
		if (DEBUG_FLAG)
			ROS_INFO("Topic [%s] already being published to", topicName.c_str());
		return;
	}

	(*publishersPtr)[topicName] = n.advertise<rte_monitor::RtEList>(topicName.c_str(), OUTPUT_QUEUE_SIZE);
}

void registerMonitoredVariable(ros::NodeHandle& n, 
	const std::string& topicName, 
	const std::string& variableName, 
	const std::string& id,
	bool cryptoMode = false) {

	// If variable already registered, no op
	if (monitorsToBuffers.find(variableName) != monitorsToBuffers.end()) {
		if (DEBUG_FLAG)
			ROS_INFO("Variable [%s] already registered", variableName.c_str());
		return;
	}

	monitorsToBuffers[variableName];
	monitorsToBuffers[variableName].topicName = topicName;
	monitorsToBuffers[variableName].variableID = id;
	monitorsToBuffers[variableName].variableName = variableName;
	monitorsToBuffers[variableName].cryptoMode = cryptoMode;

	if (!cryptoMode)
		(*subscribersMapPtr)[variableName] = n.subscribe(topicName.c_str(), INPUT_QUEUE_SIZE, &Buffer::inputCallback, &monitorsToBuffers[variableName]);
	else
		(*subscribersMapPtr)[variableName] = n.subscribe(topicName.c_str(), INPUT_QUEUE_SIZE, &Buffer::cryptoInputCallback, &monitorsToBuffers[variableName]);

	(*timersMapPtr)[variableName] = n.createTimer(ros::Duration(OMISSION_TIMER_PERIOD), &Buffer::timerCallback, &monitorsToBuffers[variableName]);
}

void setVariableThreshold(const std::string& variableName, float threshold) {
	// If no such variable registered, no op
	if (monitorsToBuffers.find(variableName) == monitorsToBuffers.end()) {
		if (DEBUG_FLAG)
			ROS_INFO("Variable with name [%s] not monitored", variableName.c_str());
		return;
	}

	// Lock buffer
	std::lock_guard<std::mutex> lock(monitorsToBuffers[variableName].bufferMutex);
	// Change monitored type to 'float threshold', update threshold
	monitorsToBuffers[variableName].setThreshold(threshold);	
    // std::cout << "-----\nBufferTag FLOAT: " << monitorsToBuffers[variableName].bufferContent[0].tag << "\n-----";
}

void setVariableBounds(const std::string& variableName, float lowerBound, float upperBound) {
	// If no such variable registered, no op
	if (monitorsToBuffers.find(variableName) == monitorsToBuffers.end()) {
		if (DEBUG_FLAG)
			ROS_INFO("Variable with name [%s] not monitored", variableName.c_str());
		return;
	}

	// Lock buffer
	std::lock_guard<std::mutex> lock(monitorsToBuffers[variableName].bufferMutex);
	monitorsToBuffers[variableName].setBounds(lowerBound, upperBound);
}

void setVariableTrueFalseConditions(const std::string& variableName, const std::string& trueCondition, const std::string& falseCondition) {
	// If no such variable registered, no op
	if (monitorsToBuffers.find(variableName) == monitorsToBuffers.end()) {
		if (DEBUG_FLAG)
			ROS_INFO("Variable with name [%s] not monitored", variableName.c_str());
		return;
	}

	// Lock buffer
	std::lock_guard<std::mutex> lock(monitorsToBuffers[variableName].bufferMutex);
	// Change monitored type to 'bool', update true/false conditions
	monitorsToBuffers[variableName].setTrueFalseConditions(trueCondition, falseCondition);
    // std::cout << "-----\nBufferTag BOOL: " << monitorsToBuffers[variableName].bufferContent[0].tag << "\n-----";
}

void setOmissionPeriod(const std::string& variableName, float omissionTimerPeriod) {
	// If no such variable registered, no op
	if (monitorsToBuffers.find(variableName) == monitorsToBuffers.end()) {
		if (DEBUG_FLAG)
			ROS_INFO("Variable with name [%s] not monitored", variableName.c_str());
		return;
	}

	// Lock buffer
	std::lock_guard<std::mutex> lock(monitorsToBuffers[variableName].bufferMutex);
	// Update omission time period, reset timer
	monitorsToBuffers[variableName].omissionTimerPeriod = omissionTimerPeriod;
	monitorsToBuffers[variableName].resetTimer();
}

void setMonitoringWindow(const std::string& variableName, int monitoringWindow) {
	// If no such variable registered, no op
	if (monitorsToBuffers.find(variableName) == monitorsToBuffers.end()) {
		if (DEBUG_FLAG)
			ROS_INFO("Variable with name [%s] not monitored", variableName.c_str());
		return;
	}

	// Lock buffer
	std::lock_guard<std::mutex> lock(monitorsToBuffers[variableName].bufferMutex);
	// Update monitoring window
	monitorsToBuffers[variableName].bufferWindow = monitoringWindow;
}

void setNodeRateNoQt(float rate) { *loopRatePtr = ros::Rate(rate); }
void setNodeRate(int rateMs) { updateVarTimerPtr->setInterval(rateMs); }
void setUIRate(int rateMs) { rosQtTimerPtr->setInterval(rateMs); }
void setUIDelay(unsigned delayMs) { qtUIUpdateDelayMS = delayMs; }

void processConfigCmd(const std::vector<std::string>& configCmd);
void parseConfigStream(const std::vector<std::string>& configStream);

void processConfigFile(const std::string& fileName) {
	if (DEBUG_FLAG)
		ROS_INFO("Parsing [%s]", fileName.c_str());

	std::ifstream configFile(fileName.c_str(), std::ifstream::in);
	std::vector<std::string> configStream;
	std::string next;
	while (configFile >> next)
		configStream.push_back(next);
	parseConfigStream(configStream);
	configFile.close();
}

void processConfigCmd(const std::vector<std::string>& configCmd) {

	const auto& command = configCmd[0];
	const auto& parameter1 = configCmd.size() >= 2 ? configCmd[1] : "";
	const auto& parameter2 = configCmd.size() >= 3 ? configCmd[2] : "";
	const auto& parameter3 = configCmd.size() >= 4 ? configCmd[3] : "";
	const auto& parameter4 = configCmd.size() >= 5 ? configCmd[4] : "";

	/* std::cout << "-----------" << std::endl;
	std::cout << command << std::endl;
	std::cout << parameter1 << std::endl;
	std::cout << parameter2 << std::endl;
	std::cout << parameter3 << std::endl; */

	if (DEBUG_FLAG)
		ROS_INFO("Processing Command [%s] with params [%s], [%s], [%s], [%s]", 
			command.c_str(), parameter1.c_str(), parameter2.c_str(), parameter3.c_str(), parameter4.c_str());

	if (command == ADD_PUBLISHER_TOPIC_CMD)
		addPublisherTopic(*nodePtr, parameter1);
	else if (command == SET_OMISSION_PERIOD_CMD) {
		float omissionPeriod = std::stof(parameter2);
		setOmissionPeriod(parameter1, omissionPeriod);
	}
	else if (command == REG_VARIABLE_CMD) {
		if (parameter4 == CRYPTO_MODE)
			registerMonitoredVariable(*nodePtr, parameter1, parameter2, parameter3, true);
		else
			registerMonitoredVariable(*nodePtr, parameter1, parameter2, parameter3);
	}
	else if (command == SET_NODE_RATE_WITHOUT_QT_CMD) {
		int rate = std::stof(parameter1);
		setNodeRateNoQt(rate);
	}
	else if (command == HIDE_QT_VISUALIZER_CMD) {
		showQtVisualizer = false;
	}
	else if (command == SET_NODE_RATE_CMD) {
		int nodeRate = std::stoi(parameter1);
		setNodeRate(nodeRate);
	}
	else if (command == SET_UI_RATE_CMD) {
		int uiRate = std::stoi(parameter1);
		setUIRate(uiRate);
	}
	else if (command == SET_UI_BUFFER_COUNT_CMD) {
		unsigned bufferCount = std::stoul(parameter1);
		qtVisualizerPtr->SetBufferElements(bufferCount);
	}
	else if (command == SET_QT_DELAY) {
		unsigned uiDelay = std::stoul(parameter1);
		setUIDelay(uiDelay);
	}
	else if (command == SET_VAR_CONDITIONS_CMD)
		setVariableTrueFalseConditions(parameter1, parameter2, parameter3);
	else if (command == SET_VAR_THRESHOLD_CMD) {
		if (parameter3 == "") { // simple threshold check
		    // std::cout << "------------------------------\nset as threshold\n------------------------------" << std::endl;
			float threshold = std::stof(parameter2);
			setVariableThreshold(parameter1, threshold);
		}
		else { // lower-upper bounds check
		    // std::cout << "------------------------------\nset as bounds\n------------------------------" << std::endl;
			float lowerBound = std::stof(parameter2);
			float upperBound = std::stof(parameter3);
			setVariableBounds(parameter1, lowerBound, upperBound);
		}
	}
	else if (command == SET_WINDOW_CMD) {
		int window = std::stoi(parameter2);
		setMonitoringWindow(parameter1, window);
	}
	else if (command == TOGGLE_DEBUG_CMD) {
		DEBUG_FLAG = !DEBUG_FLAG;
	}
	else if (command == CONFIG_FILE_CMD)
		processConfigFile(parameter1);
}

void configurationCallback(const rte_monitor::Configuration::ConstPtr& msg) {

	if (DEBUG_FLAG)
		ROS_INFO("Received configuration command : [%s]", msg->command.c_str());

	std::vector<std::string> configCmd;
	configCmd.push_back(msg->command);
	configCmd.push_back(msg->parameter1);
	configCmd.push_back(msg->parameter2);
	configCmd.push_back(msg->parameter3);
	configCmd.push_back(msg->parameter4);

	std::lock_guard<std::mutex> lock(configMutex);
	processConfigCmd(configCmd);
}

void parseConfigStream(const std::vector<std::string>& configStream) {

	std::vector<std::string> currentConfig;
	for (const auto& input : configStream) {

		if (input[0] == '-' && (currentConfig.size() > 0)) {
			processConfigCmd(currentConfig);
			currentConfig.clear();
		}
		currentConfig.push_back(input);
	}

	if (currentConfig.size())
		processConfigCmd(currentConfig);
}

void pauseSimulationCallback(const rte_list_monitor::ChangedSimulationState::ConstPtr& msg) {

	simulationIsRunning = msg->simulationIsRunning;

	// Restore subscriptions
	if (simulationIsRunning)
		for(auto bufferIt = monitorsToBuffers.begin(); bufferIt != monitorsToBuffers.end(); ++bufferIt) {

			const auto& varName = bufferIt->first;
			const auto& topicName = bufferIt->second.topicName;

			if (monitorsToBuffers[varName].cryptoMode)
				(*subscribersMapPtr)[varName] = nodePtr->subscribe(topicName.c_str(), INPUT_QUEUE_SIZE, &Buffer::cryptoInputCallback, &monitorsToBuffers[varName]);
			else
				(*subscribersMapPtr)[varName] = nodePtr->subscribe(topicName.c_str(), INPUT_QUEUE_SIZE, &Buffer::inputCallback, &monitorsToBuffers[varName]);
			(*timersMapPtr)[varName].start();
		}
	// Unsubscribe all variables, stop omission timers
	else
		for(auto subscriberIt = subscribersMapPtr->begin(); subscriberIt != subscribersMapPtr->end(); ++subscriberIt) {

			subscriberIt->second.shutdown();
			(*timersMapPtr)[subscriberIt->first].stop();
		}
}

/****************************************************************************************************************************/
// End of Configuration commands
/****************************************************************************************************************************/

#include <algorithm>
#include <iterator>

void RosSpinLoop() {

	// Update callback queues
	ros::spinOnce();

	if (simulationIsRunning) {

		// Update each monitor, collect statuses in list
		rte_monitor::RtEList outputList;

		for (auto monitorIt = monitorsToBuffers.begin(); monitorIt != monitorsToBuffers.end(); ++monitorIt)
			monitorIt->second.update(outputList);

		qtVisualizerPtr->UpdateRtEList(outputList);

		// Publish list
		for (auto publisherIt = publishersPtr->begin(); publisherIt != publishersPtr->end(); ++publisherIt)
			publisherIt->second.publish(outputList);
	
		// Update visualizer variable list
		std::vector<std::string> visualizerVariables = qtVisualizerPtr->GetVariableList();
		std::vector<std::string> monitoredVariables;
		for (auto monitorIt = monitorsToBuffers.begin(); monitorIt != monitorsToBuffers.end(); ++monitorIt)
			monitoredVariables.push_back(monitorIt->first);

		std::vector<std::string> toRemove, toAdd;

		std::set_difference(monitoredVariables.begin(), monitoredVariables.end(),
			visualizerVariables.begin(), visualizerVariables.end(),
			std::inserter(toAdd, toAdd.begin()));

		std::set_difference(visualizerVariables.begin(), visualizerVariables.end(),
			monitoredVariables.begin(), monitoredVariables.end(),
			std::inserter(toRemove, toRemove.begin()));

		for (const auto& varToRemove : toRemove)
			qtVisualizerPtr->OnRemoveVariable(varToRemove);

		for (const auto& varToAdd : toAdd)
			qtVisualizerPtr->OnAddVariable(varToAdd);
	}
}

void QtVisualizer::OnSelectedVariable(QListWidgetItem* current) {
	if (!current)
		return;

	OnUpdateSelectedVariable();
}

void QtVisualizer::OnUpdateSelectedVariable() {

	if (list->currentItem() == nullptr)
		return;

	auto name = list->currentItem()->text().toStdString();

	// Look for variable name on RtE output list
	for (const auto& rte : rteList.runtimeEvidences) {
		if (rte.name == name) {
			std::string valueTag;
			float value = 0.f;
			// ASSERTION: At least two RtE values
			if (rte.values[0].probability > rte.values[1].probability) {
				valueTag = rte.values[0].tag;
				value = rte.values[0].probability;
			}
			else {
				valueTag = rte.values[1].tag;
				value = rte.values[1].probability;
			}
			auto statusText = QString(
				"<center>Variable status</center><br>"
				"Variable name: %1<br>"
				"Variable type: %2<br>"
				"Variable output: %3, %4")
					.arg(rte.name.c_str())
					.arg(rte.type.c_str())
					.arg(valueTag.c_str())
					.arg(value);
			statusLabel->setText(statusText);

			if (rte.type == UNKNOWN_TAG)
				statusLabel->setStyleSheet("background-color: grey; border: 1px solid black");
			else if (rte.type == OMISSION_TAG)
				statusLabel->setStyleSheet("background-color: yellow; border: 1px solid black");
			else if (rte.type == NORMAL_TAG) {
				if (valueTag == TRUE_TAG)
					statusLabel->setStyleSheet("background-color: green; border: 1px solid black");
				else
					statusLabel->setStyleSheet("background-color: red; border: 1px solid black");
			}

			break;
		}
	}

	//QThread::msleep(qtUIUpdateDelayMS);

	// Update buffers
	if (monitorsToBuffers.find(name) == monitorsToBuffers.end())
		return;

	const auto& selectedBuffer = monitorsToBuffers[name];
	for (int windowIndex = 0; windowIndex < selectedBuffer.bufferWindow; ++windowIndex) {
		int windowPosition = (selectedBuffer.bufferIndex - 1 - windowIndex) % selectedBuffer.bufferContent.size();
		const auto& bufferElement = selectedBuffer.bufferContent[windowPosition];
		auto color = "grey";
		if (bufferElement.tag == OMISSION_TAG)
			color = "yellow";
		else
		switch (selectedBuffer.monitorVariableType) {
			case Buffer::MONITOR_VARIABLE_FLOAT_THRESHOLD:
			if (bufferElement.floatValue <= selectedBuffer.threshold)
				color = "green";
			else
				color = "red";
			break;
			case Buffer::MONITOR_VARIABLE_FLOAT_BOUNDS:
			if (selectedBuffer.lowerBound <= bufferElement.floatValue && bufferElement.floatValue <= selectedBuffer.upperBound)
				color = "green";
			else
				color = "red";
			break;
			case Buffer::MONITOR_VARIABLE_BOOL:
			if (bufferElement.boolValue)
				color = "green";
			else
				color = "red";
			break;
			default:
				color = "grey";
			break;
		}
		qtVisualizerPtr->UpdateBufferVisualisation(windowIndex, bufferElement.tag, color, bufferElement.floatValue, bufferElement.boolValue);
		//QThread::msleep(qtUIUpdateDelayMS);
	}
}

int main(int argc, char** argv) {

	/*
	// TODO:Remove
	// SMILE build test
	DSL_network net;

	std::string filename = "ConCertConcept.xdsl";
	int bnFileReadOK = net.ReadFile(filename.c_str());
	if (bnFileReadOK != DSL_OKAY) {
		if (DEBUG_FLAG) ROS_INFO("Could not read BN file [%s]", filename.c_str());
		std::cout << "Not ok reading " << filename << std::endl;
		return -1;
	}
	// Get Handle to Top Event
	std::string topEventName = "SG1";
	int topHandle = net.FindNode(topEventName.c_str());
	if (topHandle < 0) {
		if (DEBUG_FLAG) ROS_INFO("Could not find handle for top event [%s]", topEventName.c_str());
		std::cout << "Not found handle for top event " << topEventName << std::endl;
		return -1;
	}
	// Get Handle to Evidence
	std::string evidenceName = "RtE1";
	int evHandle1 = net.FindNode(evidenceName.c_str());
	if (evHandle1 < 0) {
		if (DEBUG_FLAG) ROS_INFO("Could not find handle for evidence [%s]", evidenceName.c_str());
		std::cout << "Not found evidence " << evidenceName << std::endl;
		return -1;
	}
	// Get evidence node
	DSL_node *evNode1 = net.GetNode(evHandle1);
	if (!evNode1) {
		if (DEBUG_FLAG) ROS_INFO("Could not find node for evidence [%s]", evidenceName.c_str());
		return -1;
	}
	// On Monitor Update
	// Set Evidence
	// Find desired state of evidence index
	std::string evStateName = "True";
	int evStateIdx = evNode1->Definition()->GetOutcomesNames()->FindPosition(evStateName.c_str());
	if (evStateIdx < 0) {
		if (DEBUG_FLAG) ROS_INFO("Could not find evidence state [%s] for node [%s]", evStateName.c_str(), evidenceName.c_str());
		std::cout << "Not found evidence state " << evStateName << std::endl;
		return -1;
	}

	evNode1->Value()->SetEvidence(evStateIdx);

	net.UpdateBeliefs();

	// Retrieve Top Event state(s)
	std::cout << "Ready to get top event states" << std::endl;
	///////////////////////////////
	*/

	// Parse cmdline input to configure node
	std::vector<std::string> args;
	for (unsigned argIdx = 1; argIdx < argc; ++argIdx)
		args.push_back(std::string(argv[argIdx]));

	// Setup ROS node
	ros::init(argc, argv, NODE_NAME);

	ros::NodeHandle n;

	// Setup local storage pointed to by global pointers
	// Note: while having the vectors global is possible, it causes a destructor error on the mutex
	// when terminating on Ctrl+C
	// Workaround: have the subscribers be destroyed locally, use global pointers to them for configuration functions

	nodePtr = &n;

	auto loopRate = ros::Rate(DEFAULT_NODE_FREQUENCY);
	loopRatePtr = &loopRate;

	std::map<std::string, ros::Subscriber> subscribersMap;
	subscribersMapPtr = &subscribersMap;

	std::map<std::string, ros::Timer> timersMap;
	timersMapPtr = &timersMap;

	std::map<std::string, ros::Publisher> topicsToPublishers;
	publishersPtr = &topicsToPublishers;

	QApplication app(argc, argv);
	qtVisualizerPtr = new QtVisualizer();
	rosQtTimerPtr = new QTimer(qtVisualizerPtr);
	updateVarTimerPtr = new QTimer(qtVisualizerPtr);

	// Process parsed commands
	parseConfigStream(args);

	// Configuration subscriber
	auto configSubscriber = n.subscribe("Monitor_Config", 200, &configurationCallback);

	// Play/Stop subscriber
	auto playStopSubscriber = n.subscribe("ChangedSimulationState", 200, &pauseSimulationCallback);

	if (showQtVisualizer) {

		qtVisualizerPtr->show();
	
		qtVisualizerPtr->connect(rosQtTimerPtr, &QTimer::timeout, qtVisualizerPtr, &RosSpinLoop);
		if (!rosQtTimerPtr->interval())
			rosQtTimerPtr->setInterval(DEFAULT_UI_RATE);
		rosQtTimerPtr->start();
		
		qtVisualizerPtr->connect(updateVarTimerPtr, SIGNAL(timeout()), qtVisualizerPtr, SLOT(OnUpdateSelectedVariable()));
		if (!updateVarTimerPtr->interval())
			updateVarTimerPtr->setInterval(DEFAULT_NODE_RATE);
		updateVarTimerPtr->start();

		return app.exec();
	}
	
	// Run ROS event loop instead
	while (ros::ok()) {

		if (simulationIsRunning) {

			// Update each monitor, collect statuses in list
			rte_monitor::RtEList outputList;

			for (auto monitorIt = monitorsToBuffers.begin(); monitorIt != monitorsToBuffers.end(); ++monitorIt)
				monitorIt->second.update(outputList);

			// Publish list
			for (auto publisherIt = publishersPtr->begin(); publisherIt != publishersPtr->end(); ++publisherIt)
				publisherIt->second.publish(outputList);
		}

		ros::spinOnce();

		loopRate.sleep();
	}
}
