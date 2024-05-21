# Functions used to run analysis.py

import opensim as osim

def run_analyze_tool(analyze_settings_template_file, results_dir, model_file_path, mot_file_path, start_time, end_time):

    analyze_Tool = osim.AnalyzeTool(analyze_settings_template_file)
    analyze_Tool.updAnalysisSet().cloneAndAppend(osim.BodyKinematics())
    analyze_Tool.setModelFilename(model_file_path)
    analyze_Tool.setName("analyze")
    analyze_Tool.setCoordinatesFileName(mot_file_path)
    analyze_Tool.setStartTime(start_time)
    analyze_Tool.setFinalTime(end_time)
    analyze_Tool.setResultsDir(results_dir)
    print('Running Analyze Tool...')
    analyze_Tool.run()
    print('Analyze Tool run finished.')
