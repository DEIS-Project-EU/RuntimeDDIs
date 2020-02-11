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

import rospy
from time import sleep
from rte_list_monitor.msg import ChangedSimulationState
from rte_monitor.msg import RtEList

debug_mode = False
class RuntimeEvidenceListMonitor:

    def __init__(self):
        self.__current_runtime_evidences = []
        self.__updated_runtime_evidence_list_publisher = None
        self.__change_simulation_state_publisher = None

        # ROS Node Initialization
        rospy.init_node('rte_list_monitor', anonymous=True)
        self.__initialize_ros_subscribers()
        self.__initialize_ros_publishers()

        rospy.spin()

    # Private methods
    def __initialize_ros_subscribers(self):
        rospy.Subscriber('RtEList', RtEList, self.__delegate_rte_list_received)

    def __initialize_ros_publishers(self):
        self.__change_simulation_state_publisher = rospy.Publisher(
            'ChangedSimulationState', ChangedSimulationState, queue_size=10
        )

        self.__updated_runtime_evidence_list_publisher = rospy.Publisher(
            'UpdatedRtEList', RtEList, queue_size=10
        )

    def __delegate_rte_list_received(self, rte_list):
        """
        Handles the event that a new rte list has been published. If there is a rte with type 'unknown', further processing
        is canceled as the rte monitor does not deliver usable values yet
        """

        received_rte_list = list(rte_list.runtimeEvidences)

        # rte monitor cannot provide usable values, yet.
        if any((rte.type == 'unknown' or rte.type == 'omission') for rte in received_rte_list):
            if debug_mode:
                rospy.loginfo('There is a rte with an unknown type.')
            return

        runtime_evidences_are_equal = self.received_rte_are_equal(received_rte_list)
        if not runtime_evidences_are_equal:
            # update current rte list
            self.__current_runtime_evidences = received_rte_list

            # delay message publishing 
            # --> assure that the vehicle icons are updated in the CARLA client before the simulation is paused
            # (without also updating the timegap setpoint before the Consert Tree in the unity component is evaluated)
            # --> assure that the unity component animation is delayed, so that the animation starts at the same time the simulation gets paused 
            sleep(0.25)

            # publish topic to pause simulation
            #rospy.loginfo('ChangedSimulationState is set to false')
            changed_simulation_state = ChangedSimulationState()
            changed_simulation_state.simulationIsRunning = False
            self.__change_simulation_state_publisher.publish(changed_simulation_state)

            # publish topic with updated rte list
            updated_rte_list = RtEList()
            #rospy.loginfo('Updated rte list is published')
            updated_rte_list.runtimeEvidences = tuple(self.__current_runtime_evidences)
            self.__updated_runtime_evidence_list_publisher.publish(updated_rte_list)

    def received_rte_are_equal(self, received_rte_list):
        for received_rte in received_rte_list:
            # get same rte from already saved rte list
            already_saved_rte = next((existing_rte for existing_rte in self.__current_runtime_evidences if
                                      existing_rte.id == received_rte.id and
                                      existing_rte.name == received_rte.name and
                                      existing_rte.type == received_rte.type), None)
            # if a rte with the same id, name and type (normal, Omission etc.) exists, then compare if the values are
            # still the same.
            # If same values, received rte is already cached -> do nothing and continue comparing with next rte
            # Else: received rte differs with already cached ones --> return False
            if already_saved_rte is not None:
                for cached_value in already_saved_rte.values:
                    if not any(received_rte_val.tag == cached_value.tag and
                               received_rte_val.probability == cached_value.probability
                               for received_rte_val in received_rte.values):
                        return False

            # if there does not exist a cached rte with the same id, name and type --> return False
            else:
                return False

        return True


def main():
    rte_list_monitor = RuntimeEvidenceListMonitor()


if __name__ == '__main__':
    main()
