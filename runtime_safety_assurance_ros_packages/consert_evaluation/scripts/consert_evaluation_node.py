'''
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
'''

#!/usr/bin/env python
import os

import rospy
from rte_monitor.msg import RtEList
from consert_evaluation.msg import ActiveGuarantee
from thrift.protocol import TBinaryProtocol
from thrift.transport import TSocket

from generatedThriftFiles.ConSertEvaluationService import *
from generatedThriftFiles.constants import *

debug_mode = False
class ConSertsEvaluationNode(object):

    def __init__(self):
        self.__file_name = None
        self.__model_configurations = None

        # ROS specific
        self.__active_guarantee_publisher = None

        # Thrift communication
        self.__thrift_transport = None
        self.__thrift_client = None

        # Initialization
        self.__set_this_python_file_name()
        self.__set_model_configurations()

        rospy.init_node('consert_evaluator_wrapper_node', anonymous=True)
        self.__initialize_ros_subscriber()
        self.__initialize_ros_publisher()

        self.__initialize_thrift_communication()

        rospy.spin()

    def __initialize_ros_subscriber(self):
        rospy.Subscriber('UpdatedRtEList', RtEList, self.__on_changed_rte_list)

    def __initialize_ros_publisher(self):
        self.__active_guarantee_publisher = rospy.Publisher(
            'ActiveGuarantee', ActiveGuarantee, queue_size=10
        )

    def __set_this_python_file_name(self):
        # get filename of this python script
        file_base_name = os.path.basename(__file__)
        file_name = os.path.splitext(file_base_name)[0]
        self.__file_name = file_name

    def __set_model_configurations(self):
        """This method reads the model configuration from the configured YAML file and
           returns a the Thrift representation of it"""

        model_file_information_list = []

        # read from yaml config file
        try:
            model_configurations_from_yaml = rospy.get_param("/" + self.__file_name + "/model_config")
        except:
            rospy.logwarn("Configuration YAML file could not be found.")
            exit(1)

        if len(model_configurations_from_yaml) == 0:
            rospy.logwarn('Configuation file is not defined for models that shall be used.')
            exit(1)
        else:
            # for each model config specified in the YAML file, create a Thrift object for it
            for config in model_configurations_from_yaml:
                model_file_info = TModelFileInformation()
                model_file_info.ModelFileName = config["model_name"]
                model_file_info.SystemId = config["system_id"]
                model_file_info.SystemProvidesAppservice = config["provides_app_service"]
                model_file_information_list.append(model_file_info)

        self.__model_configurations = model_file_information_list

    def __on_changed_rte_list(self, updated_rte_list):
        """ This method is called when the node receives a ROS message with the topic 'UpdatedRtEList'
            1. The ConSerts evaluation configuration is initialized with the new RtE list
            2. Thrift client is initialized
            3. Remote service is invoked with thrift client and the configuration as parameter"""

        if not self.__check_if_all_model_files_exist():
            rospy.logwarn("ConSerts evaluation cannot be started as model files do not exist yet. Consider providing ConSerts models to the RoS architecture")
            return

        eval_config = self.__get_consert_evaluation_configuration(updated_rte_list.runtimeEvidences)

        # Connect to ConSerts evaluation software component
        self.__thrift_transport.open()

        if debug_mode:
            rospy.loginfo("Invoke Thrift service for ConSerts Evaluation")

        # Invoke remote service
        result = self.__thrift_client.EvaluateConSerts(eval_config)

        # Close connection to ConSerts evaluation software component.
        self.__thrift_transport.close()

        safety_guarantee_id = ""

        if len(result.ChoosenSystemConfigurationsByGuaranteedProvidedService) > 0:
            safety_guarantee_id = result.ChoosenSystemConfigurationsByGuaranteedProvidedService[0].GuaranteeId

        if debug_mode:
            rospy.loginfo("New Active Safety guarantee = " + safety_guarantee_id)

        # publish active guarantee id
        active_guarantee = ActiveGuarantee()
        active_guarantee.value = safety_guarantee_id
        self.__active_guarantee_publisher.publish(active_guarantee)

    def __get_consert_evaluation_configuration(self, updated_rtes):
        eval_config = TConSertEvaluationConfiguration()

        eval_config.ValidRuntimeEvidenceIds = self.__get_valid_thrift_runtime_evidences(updated_rtes)
        eval_config.ContributingModelFileInformation = self.__model_configurations

        return eval_config

    def __get_valid_thrift_runtime_evidences(self, updated_rtes):
        """
        This method gets the ids of all the received runtime evidences,
            that are of type 'normal' (not 'unknown' or 'omission') AND
            that have a higher probability for the value 'True' than for the value 'False'
        """
        ids_of_valid_rtes = []

        # processable_rtes: list of rtes of type 'normal' that have at least 2 values, tagged as 'true' and 'false'
        processable_rtes = filter(lambda runtime_evidence:
                                  str.lower(runtime_evidence.type) == 'normal' and
                                  any(str.lower(rte_val.tag) == 'true' for rte_val in runtime_evidence.values) and
                                  any(str.lower(rte_val.tag) == 'false' for rte_val in runtime_evidence.values), updated_rtes)

        for rte in processable_rtes:
            true_value_probability = next(rte_val for rte_val in rte.values if str.lower(rte_val.tag) == 'true').probability
            false_value_probability = next(rte_val for rte_val in rte.values if str.lower(rte_val.tag) == 'false').probability

            if true_value_probability > false_value_probability:
                ids_of_valid_rtes.append(rte.id)

        if debug_mode:
            rospy.loginfo('valid rte ids')
            rospy.loginfo(ids_of_valid_rtes)

        return ids_of_valid_rtes

    def __initialize_thrift_communication(self):
        thrift_server_port = rospy.get_param("/" + self.__file_name + "/thrift_server_port")

        self.__thrift_transport = TSocket.TSocket('localhost', thrift_server_port)
        protocol = TBinaryProtocol.TBinaryProtocol(self.__thrift_transport)
        self.__thrift_client = Client(protocol)

    def __check_if_all_model_files_exist(self):
        model_storage_directory = rospy.get_param("model_storage_directory", "")
        if model_storage_directory == "":
            rospy.logwarn("Model storage directory could not be loaded from parameter server. Please make sure, you pass"
                          " the directory as a ros parameter within the ConSerts evaluation package launch file.")
            return False

        models_exist = True
        for model_info in self.__model_configurations:
            model_path = os.path.join(model_storage_directory, model_info.ModelFileName)
            if not os.path.isfile(model_path):
                rospy.logwarn("Model file with path \"" + model_path + "\" does not exist.")
                models_exist = False

        return models_exist


def main():
    ConSertsEvaluationNode()


if __name__ == '__main__':
    main()
