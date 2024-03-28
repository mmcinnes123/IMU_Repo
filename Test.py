
import opensim as osim
import os
from scipy.spatial.transform import Rotation as R
import numpy as np

parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection\P3'
analysis_template = os.path.join(parent_dir, 'Analysis_template.xml')
model_file_path = os.path.join(parent_dir, 'Calibrated_das3.osim')

mot_file = 'IMU_IK_results.mot'
mot_file_path = os.path.join(parent_dir, mot_file)

start_time = 0
end_time = 5

# def run_analyze_tool(results_dir, model_file_path, mot_file_path, start_time, end_time):
#
#     analyze_Tool = osim.AnalyzeTool('Analyze_Settings.xml')
#     analyze_Tool.updAnalysisSet().cloneAndAppend(osim.BodyKinematics())
#     analyze_Tool.setModelFilename(model_file_path)
#     analyze_Tool.setName("analyze")
#     analyze_Tool.setCoordinatesFileName(mot_file_path)
#     analyze_Tool.setStartTime(start_time)
#     analyze_Tool.setFinalTime(end_time)
#     analyze_Tool.setResultsDir(results_dir)
#     analyze_Tool.run()
#
# run_analyze_tool(parent_dir, model_file_path, mot_file_path, start_time, end_time)
#

#
analysis_sto = 'analyze_BodyKinematics_pos_global.sto'
analysis_sto_path = os.path.join(parent_dir, analysis_sto)



thorax_OMC, humerus_OMC, radius_OMC = get_body_quats_from_analysis_sto(analysis_sto_path)


