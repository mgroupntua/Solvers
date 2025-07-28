import numpy as np
from src.my_utilities import csharp_interop


class InputParams:
    def __init__(self):
        self.VectorX = np.zeros(1)
        self.CoeffX = 0.0
        self.VectorY = np.zeros(1)
        self.CoeffY = 0.0

class OutputParams:
    def __init__(self):
        self.Combo = np.zeros(1)
        self.DotProduct = 0.0


def linear_combo_and_dot_product(inputs: InputParams, outputs: OutputParams) -> int:
    x = inputs.VectorX
    y = inputs.VectorY
    a = inputs.CoeffX
    b = inputs.CoeffY
    z = a * x + b * y
    outputs.Combo = z
    outputs.DotProduct = np.dot(x, y)
    return 0


if __name__ == '__main__':
	# To debug, run this file by itself (inside Pycharm) after uncommenting the next 2 lines:
	#import sys
	#sys.argv[1] = "C:\\Users\\Serafeim\\Documents\\Coding\\AISolve\\temp" # e.g. for temporary work directory
    csharp_interop.call_python_func(linear_combo_and_dot_product, InputParams, OutputParams)
