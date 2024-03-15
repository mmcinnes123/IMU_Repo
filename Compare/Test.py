import opensim as osim

model_file_path = r"C:\Users\r03mm22\Documents\Protocol_Testing\OMC_Repo\das3.osim"

my_model = osim.Model(model_file_path)

thorax = my_model.getBodySet().get('thorax')

print(thorax.getOutputNames())

print(thorax.dump())