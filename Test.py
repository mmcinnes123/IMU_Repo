import opensim as osim
import numpy as np

model_file_path = r"das3.osim"

my_model = osim.Model(model_file_path)

min_value = - np.pi
max_value = np.pi

my_model.getCoordinateSet().get('GH_y').set_range(0, min_value)
my_model.getCoordinateSet().get('GH_y').set_range(1, max_value)
my_model.getCoordinateSet().get('GH_z').set_range(0, min_value)
my_model.getCoordinateSet().get('GH_z').set_range(1, max_value)
my_model.getCoordinateSet().get('GH_x').set_range(0, min_value)
my_model.getCoordinateSet().get('GH_x').set_range(1, max_value)

my_model.printToXML(model_file_path)