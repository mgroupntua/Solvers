import sys
import traceback
import json
import time

import numpy as np

from typing import Type
from contextlib import redirect_stdout

from src.my_utilities.interop_paths import InteropPaths


#Use "generics" with TypeVar

def call_python_func(function, input_class: Type, output_class: Type):
    try:
        start = time.time_ns()
        path_work_directory = sys.argv[1]
        # path_log_performance = sys.argv[2]
        # path_log_errors = sys.argv[3]
        durations = {"IO": 0, "Setup": 0}
        paths = InteropPaths(path_work_directory)

        with open(paths.log_errors, 'w') as file_log_errors:
            file_log_errors.write("")
            with redirect_stdout(file_log_errors):
                # Read input
                inputs = input_class()
                read_input_from_files(inputs, paths)
                file_log_errors.write("Read inputs successfully\n")
                durations["IO"] += (time.time_ns() - start) // 1000000  # in ms

                # Call function that does the actual work
                outputs = output_class()
                setup_duration = function(inputs, outputs)
                if setup_duration is not None:
                    durations["Setup"] = setup_duration
                file_log_errors.write("Called function successfully\n")

                # Write output
                start = time.time_ns()
                write_output_to_files(outputs, paths)
                durations["IO"] += (time.time_ns() - start) // 1000000  # in ms
        with open(paths.log_performance, 'w') as file_results:
            json.dump(durations, file_results)
    except:  # Exception by that actual function or during IO. This catches every, unlike 'except Exception as ex'
        with open(paths.log_errors, 'w') as file_log_errors:
            traceback.print_exc(file=file_log_errors)
        sys.exit(100)
    else: # Everything run successfully
        sys.exit(0)


def read_input_from_files(inputs: any, paths: InteropPaths):
    inputs_serialized = dict[str, any]()
    with open(paths.inputs_serialized) as file_inputs_serialized:
        inputs_serialized = json.load(file_inputs_serialized)
    for attr_name in dir(inputs):
        if attr_name[0] == '_':
            continue
        attr_value_default = getattr(inputs, attr_name)  # TODO: avoid this since it depends on the user to initialize the attributes appropriately
        if isinstance(attr_value_default, np.ndarray):
            path_array = paths.make_path_for_input_array(attr_name)
            attr_value_actual = np.load(path_array)
            setattr(inputs, attr_name, attr_value_actual)
        else:
            # setattr(json_dto, attr_name, attr_value)
            attr_value_actual = inputs_serialized[attr_name]
            setattr(inputs, attr_name, attr_value_actual)


def write_output_to_files(outputs: any, paths: InteropPaths):
    json_output = dict[str, any]()
    for attr_name in dir(outputs):
        if attr_name[0] == '_':
            continue
        attr_value = getattr(outputs, attr_name)
        if isinstance(attr_value, np.ndarray):
            path_array =  paths.make_path_for_output_array(attr_name)
            np.save(path_array, attr_value)
        else:
            # setattr(json_dto, attr_name, attr_value)
            json_output[attr_name] = attr_value
    with open(paths.outputs_serialized, 'w') as file:
        json.dump(json_output, file)
