#pragma once

#include "constants.hpp"

// Each buffer element replicates incoming message contents
// Overhead on memory and copying for avoiding to handle ROS message types
struct BufferElement {
	std::string tag = NONE_TAG;
	float floatValue = 0.0f;
	bool boolValue = false;
};

struct Buffer {

	enum MonitoredVariableType {
		MONITOR_VARIABLE_FLOAT_THRESHOLD,
		MONITOR_VARIABLE_FLOAT_BOUNDS,
		MONITOR_VARIABLE_BOOL
	} monitorVariableType = MONITOR_VARIABLE_FLOAT_THRESHOLD;

	bool cryptoMode = false;

	int bufferIndex = 0;
	int bufferWindow = BUFFER_WINDOW;

	float threshold = 0.f;
	float lowerBound = 0.f, upperBound = 0.f;
	float omissionTimerPeriod = OMISSION_TIMER_PERIOD;

	std::string bufferTag = "";
	std::string topicName = "";
	std::string variableID = "";
	std::string variableName = "";
	std::string trueCondition = "";
	std::string falseCondition = "";

	std::mutex bufferMutex;
	std::array<BufferElement, MAX_BUFFER_SIZE> bufferContent;

	void copyMsgToBuffer(const rte_monitor::ConfidenceInputEvent::ConstPtr& msg) {
		copyMsgToBuffer(*msg);
	}

	void copyMsgToBuffer(const rte_monitor::ConfidenceInputEvent& msg) {
		this->bufferContent[this->bufferIndex].tag = 
			(this->monitorVariableType == MONITOR_VARIABLE_FLOAT_THRESHOLD) || 
				(this->monitorVariableType == MONITOR_VARIABLE_FLOAT_BOUNDS) ? 
					FLOAT_TAG : BOOL_TAG;
		this->bufferContent[this->bufferIndex].floatValue = msg.floatValue;
		this->bufferContent[this->bufferIndex].boolValue = msg.booleanValue;
	}

	void insertOmission() { this->bufferContent[bufferIndex].tag = OMISSION_TAG; }
	void incrementPosition() { ++this->bufferIndex %= this->bufferContent.size(); }

	void resetTimer() {
		(*timersMapPtr)[this->variableName].setPeriod(ros::Duration(this->omissionTimerPeriod), true);
	}

	void setThreshold(float threshold) {
		this->monitorVariableType = MONITOR_VARIABLE_FLOAT_THRESHOLD;
		this->bufferTag = FLOAT_TAG;
		this->threshold = threshold;
	}

	void setBounds(float lowerBound, float upperBound) {
		this->monitorVariableType = MONITOR_VARIABLE_FLOAT_BOUNDS;
		this->bufferTag = FLOAT_TAG;
		this->lowerBound = lowerBound;
		this->upperBound = upperBound;
	}

	void setTrueFalseConditions(const std::string& trueCondition, const std::string& falseCondition) {
		this->monitorVariableType = MONITOR_VARIABLE_BOOL;
		this->bufferTag = BOOL_TAG;
		this->trueCondition = trueCondition;
		this->falseCondition = falseCondition;
	}

	void inputCallback(const rte_monitor::ConfidenceInputEvent::Ptr& msg) {
		if (DEBUG_FLAG)
			ROS_INFO("Received: [%s] message", msg->tag.c_str());

		// Inspect message tag, do not update buffer if mismatch
		if (msg->tag != this->variableName)
			return;

		// Lock buffer
		std::lock_guard<std::mutex> lock(this->bufferMutex);
		// Copy message contents to buffer
		this->copyMsgToBuffer(msg);
		// Move to next position in buffer
		this->incrementPosition();
		// Reset omission timer
		this->resetTimer();
	}

	void cryptoInputCallback(const rte_monitor::ConfidenceInputEventWEnc::Ptr& msg) {
		if (DEBUG_FLAG)
			ROS_INFO("Received encrypted message");

		using CryptoPP::AES;
		using CryptoPP::ECB_Mode;
		using CryptoPP::StringSink;
		using CryptoPP::StringSource;
		using CryptoPP::StreamTransformationFilter;
		using CryptoPP::HashVerificationFilter;
		using CryptoPP::HashFilter;
		using CryptoPP::HexEncoder;
		using CryptoPP::HexDecoder;
		using CryptoPP::HMAC;
		using CryptoPP::SHA256;

		byte converted_key[32];
		std::copy(SYMMETRIC_KEY.begin(), SYMMETRIC_KEY.end(), converted_key);

		try {
			// Confirm HMAC
			HMAC<SHA256> hmac(converted_key, sizeof(converted_key));
			const int flags =
				HashVerificationFilter::THROW_EXCEPTION |
				HashVerificationFilter::HASH_AT_END;

			std::string decoded_mac;
			StringSource(msg->mac, true,
				new HexDecoder(
					new StringSink(decoded_mac)
				)
			);
			StringSource(msg->content + decoded_mac, true,
				new HashVerificationFilter(hmac, NULL, flags)
			);
		} catch (const CryptoPP::Exception& e) {
			if (DEBUG_FLAG)
				ROS_INFO("HMAC Verification Failed: %s", e.what());
			return;
		}

		std::string decryptedMessage;

		try {
			// Decrypt message
			ECB_Mode<AES>::Decryption decrypt;
			decrypt.SetKey(converted_key, sizeof(converted_key));

			std::string decoded_content;
			StringSource(msg->content, true,
				new HexDecoder(
					new StringSink(decoded_content)
				)
			);

			StringSource s(decoded_content, true, 
				new StreamTransformationFilter(decrypt,
					new StringSink(decryptedMessage)));
		} catch (const CryptoPP::Exception& e) {
			if (DEBUG_FLAG)
				ROS_INFO("AES Decryption Failed: %s", e.what());
			return;
		}

		// De-serialize message contents
		rte_monitor::ConfidenceInputEvent deserializedMessage;
		unsigned tagLength = std::stoul(decryptedMessage.substr(0, 3));
		deserializedMessage.tag = decryptedMessage.substr(3, tagLength);

		// Inspect message tag, do not update buffer if mismatch
		if (deserializedMessage.tag != this->variableName)
			return;

		unsigned floatLength = std::stoul(decryptedMessage.substr(tagLength + 3, 3));
		deserializedMessage.floatValue =
			std::stof(decryptedMessage.substr(tagLength + 6, floatLength));
		deserializedMessage.booleanValue =
			(bool)std::stoul(decryptedMessage.substr(tagLength + 6 + floatLength, 1));

		// Lock buffer
		std::lock_guard<std::mutex> lock(this->bufferMutex);
		// Copy decrypted message contents to buffer
		this->copyMsgToBuffer(deserializedMessage);
		// Move to next position in buffer
		this->incrementPosition();
		// Reset omission timer
		this->resetTimer();
	}

	void timerCallback(const ros::TimerEvent& evt) {
		if (DEBUG_FLAG)
			ROS_INFO("Omission callback triggered");

		// Lock and update buffer
		std::lock_guard<std::mutex> lock(this->bufferMutex);
		// Insert omission entry in buffer
		this->insertOmission();
		// Move to next position in buffer
		this->incrementPosition();
	}

private:
	static void SetEvidence(rte_monitor::RuntimeEvidence& evidence,
		const std::string& varName,
		const std::string& typeTag,
		bool trueCondition,
		const std::string& dbgMsg) {
			if (DEBUG_FLAG)
				ROS_INFO("%s", dbgMsg.c_str());

			evidence.name = varName;
			evidence.type = typeTag;

			rte_monitor::RtEValue value;
			value.tag = TRUE_TAG;
			trueCondition ? value.probability = 1.f : value.probability = 0.f;
			evidence.values.push_back(value);

			value.tag = FALSE_TAG;
			trueCondition ? value.probability = 0.f : value.probability = 1.f;
			evidence.values.push_back(value);
		}

public:
	void update(rte_monitor::RtEList& outputList) {

		rte_monitor::RuntimeEvidence output;
		output.id = this->variableID;

		int unknownCount = 0;
		int omissionCount = 0;
		int	leBoundCount = 0;
		int inBoundCount = 0;
		int gBoundCount = 0;
		int trueCount = 0;
		int falseCount = 0;

		for (unsigned windowIndex = 0; windowIndex < this->bufferWindow; ++windowIndex) {

			int windowPosition = (this->bufferIndex - 1 - windowIndex) % this->bufferContent.size();
			const auto& bufferElement = this->bufferContent[windowPosition];

			if (bufferElement.tag == BOOL_TAG)
				bufferElement.boolValue ? ++trueCount : ++falseCount;
			else if (bufferElement.tag == FLOAT_TAG) {
				if (this->monitorVariableType == MONITOR_VARIABLE_FLOAT_THRESHOLD)
					(bufferElement.floatValue <= this->threshold) ? ++leBoundCount : ++gBoundCount;
				else { // if (this->monitorVariableType == MONITOR_VARIABLE_FLOAT_BOUNDS)
					if (bufferElement.floatValue < this->lowerBound) ++leBoundCount;
					else if (bufferElement.floatValue > this->upperBound) ++gBoundCount;
					else ++inBoundCount;
				}
			}
			else if (bufferElement.tag == OMISSION_TAG)
				++omissionCount;
			else
				++unknownCount;
		}

		int voteSum = unknownCount + omissionCount + leBoundCount +
						inBoundCount + gBoundCount + trueCount + falseCount;

		if (DEBUG_FLAG)
			ROS_INFO("Vote Sum:%1d, Unknowns:%1d, LEs:%1d, Ins:%1d, Gs:%1d, Omissions:%1d, TrueBools:%1d, FalseBools:%1d",
					 voteSum, unknownCount, leBoundCount, inBoundCount, gBoundCount, omissionCount, trueCount, falseCount);

		if (leBoundCount > voteSum - leBoundCount) {
			if (this->monitorVariableType == MONITOR_VARIABLE_FLOAT_THRESHOLD)
				SetEvidence(output, this->variableName, NORMAL_TAG, true, "Will output <=");
			else
				// Below lower bound is not normal, so set as False -> 1.0
				SetEvidence(output, this->variableName, NORMAL_TAG, false, "Will output not <=");
		}
		else if (gBoundCount > voteSum - gBoundCount) {
			// It's the same in both threshold and upper bound checking
			SetEvidence(output, this->variableName, NORMAL_TAG, false, "Will output >");
		}
		else if (inBoundCount > voteSum - inBoundCount) {
			SetEvidence(output, this->variableName, NORMAL_TAG, true, "Will output in bounds");
		}
		else if (trueCount > voteSum - trueCount) {
			SetEvidence(output, this->variableName, NORMAL_TAG, true, "Will output true condition");
		}
		else if (falseCount > voteSum - falseCount) {
			SetEvidence(output, this->variableName, NORMAL_TAG, false, "Will output false condition");
		}
		else if (omissionCount > voteSum - omissionCount) {
			SetEvidence(output, this->variableName, OMISSION_TAG, true, "Will output omission");
		}
		else /* no majority found or unknown is the majority */ {
			SetEvidence(output, this->variableName, UNKNOWN_TAG, true, "Will output no majority/unknown");
		}

		outputList.runtimeEvidences.push_back(output);
	}
};
