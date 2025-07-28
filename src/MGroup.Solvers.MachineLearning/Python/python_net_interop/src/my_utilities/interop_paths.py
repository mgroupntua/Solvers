class InteropPaths:
    def __init__(self, work_directory_path: str):
        self.work_directory = work_directory_path
        self.inputs_serialized = work_directory_path + '/inputs_serialized.json'
        self.outputs_serialized = work_directory_path + '/outputs_serialized.json'
        self.log_errors = work_directory_path + '/log_errors.json'
        self.log_performance = work_directory_path + '/log_performance.json'

    def make_path_for_input_array(self, array_name: str) -> str:
        return self.work_directory + '/input-' + array_name + '.npy'

    def make_path_for_output_array(self, array_name: str) -> str:
        return self.work_directory + '/output-' + array_name + '.npy'