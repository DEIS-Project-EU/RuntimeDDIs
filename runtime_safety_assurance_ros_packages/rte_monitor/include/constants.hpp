#pragma once

bool DEBUG_FLAG = false;

// Node-specific variables
const std::string NODE_NAME = "proto_monitor"; // Node name
const double DEFAULT_NODE_FREQUENCY = 2; // Frequency of node update without Qt in Hz
const unsigned DEFAULT_NODE_RATE = 500; // Frequency of node update in ms

// Monitor-specific variables
const int MAX_BUFFER_SIZE = 100; // Maximum number of elements in buffer
const int BUFFER_WINDOW = 3; // (Largest) number of buffer elements stored that are monitored
const double OMISSION_TIMER_PERIOD = 2.0;

// Symmetric encryption key
const std::string SYMMETRIC_KEY = "J@NcQfTjWnZr4u7x!A%D*GKaPdSgUkXM";

// Topic queue sizes
const int INPUT_QUEUE_SIZE = 1000;
const int OUTPUT_QUEUE_SIZE = 1000;

// Tags in messages
const std::string NONE_TAG = "none";
const std::string FLOAT_TAG = "float";
const std::string BOOL_TAG = "boolean";
const std::string NORMAL_TAG = "normal";
const std::string OMISSION_TAG = "omission";
const std::string UNKNOWN_TAG = "unknown";
const std::string TRUE_TAG = "True";
const std::string FALSE_TAG = "False";

// UI
const unsigned DEFAULT_UI_RATE = 100; // Frequency of Qt UI update in ms 
const unsigned DEFAULT_UI_BUFFER_COUNT = 3; // Number of buffer elements to show in Qt UI

// Parameter handling
const std::string USAGE_MSG =
	"Command Options: \n"
	"-h : shows the command options \n"
	"-d : toggle debug messages; default is off \n"
	"-f <config file path>: setup node using configuration file \n"
	"-o <topic name> : adds a publisher topic \n"
	"-t <variable name> <omission period in s (float)> : sets the variable's omission timer period \n"
	"-r <topic name> <variable name> <variable id> <crypto mode> : registers a variable to be monitored on the selected topic "
		"\t use 'crypto' without quotes to activate encryption for the variable \n"
	"-rate <node update rate in Hz <float>: sets the ROS node update frequency in Hz - only effective if -hide_qt is also set\n"
	"-node_t <node interval in ms (int)> : sets the ROS node update thread interval i.e. time between updates \n"
	"-ui_t <Qt UI update interval in ms (int)> : sets the Qt UI update thread interval \n"
	"-ui_delay <Qt UI update delay in ms (unsigned)> : sets the Qt UI update delay \n"
	"-ui_buffer_count <number of buffer elements to show (unsigned)> : sets the number of buffer elements to show in the Qt UI, each buffer element shows the state of the buffer one position further back\n"
	"-vb <variable name> <true condition description> <false condition description> : sets the monitored variable "
		" to a boolean condition check, outputs true/false condition description \n"
	"-vf <variable name> <variable threshold (float)> [variable upper bound (float)]: sets the monitored variable to a floating-point threshold comparison, "
		" if optional upper bound set, previous parameter is lower bound \n"
	"-w <variable name> <monitoring window (int)> : sets the variable's monitoring window \n"
	"-hide_qt : hide Qt Visualizer; only available from offline configuration for now \n"
	"start : starts node processing, no further commands readable from cmdline \n"
;

const std::string SHOW_HELP_CMD = "-h";
const std::string ADD_PUBLISHER_TOPIC_CMD = "-o";
const std::string SET_OMISSION_PERIOD_CMD = "-t";
const std::string REG_VARIABLE_CMD = "-r";
const std::string SET_NODE_RATE_WITHOUT_QT_CMD = "-rate";
const std::string SET_NODE_RATE_CMD = "-node_t";
const std::string SET_VAR_CONDITIONS_CMD = "-vb";
const std::string SET_VAR_THRESHOLD_CMD = "-vf";
const std::string SET_WINDOW_CMD = "-w";
const std::string TOGGLE_DEBUG_CMD = "-d";
const std::string CONFIG_FILE_CMD = "-f";
const std::string CRYPTO_MODE = "crypto";
const std::string SET_UI_RATE_CMD = "-ui_t";
const std::string HIDE_QT_VISUALIZER_CMD = "-hide_qt";
const std::string SET_QT_DELAY = "-ui_delay";
const std::string SET_UI_BUFFER_COUNT_CMD = "-ui_buffer_count";