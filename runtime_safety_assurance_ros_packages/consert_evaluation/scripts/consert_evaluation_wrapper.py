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
import subprocess

import rospy


def start_consert_evaluator():
    rospy.init_node('consert_evaluation_wrapper', anonymous=True)

    # get relevant ros parameters that shall be defined in the launch file that starts this node.
    model_storage_directory = rospy.get_param("model_storage_directory", "")
    epsilon_script_directory = rospy.get_param("~epsilon_script_directory")
    meta_model_file_path = rospy.get_param("~metamodel_file")
    conserts_evaluator_path = rospy.get_param("~conserts_evaluator_path")
    thrift_server_port = str(rospy.get_param("~thrift_server_port"))

    print("server port = " + thrift_server_port)

    print('Start ConSerts evaluation component.')
    # start ConSert evaluation component in an additional subprocess
    subprocess.call(['java',
                     '-jar', conserts_evaluator_path,
                     '-server-port', thrift_server_port,
                     '-modelStorageDirectory', model_storage_directory,
                     '-epsilonScriptDirectory', epsilon_script_directory,
                     '-metaModelFile', meta_model_file_path])
    rospy.spin()


if __name__ == '__main__':
    start_consert_evaluator()
