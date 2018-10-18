import numpy
import opensim
import matplotlib.pyplot as plt
import os



model = opensim.Model(location_models + model_name)
model.updateMarkerSet(markerset)
model.initSystem()
state = model.initSystem()


#Scale Tool 
scale_tool = opensim.ScaleTool(os.path.join(location_XMLs, scale_file))
scale_tool.setSubjectMass(subject_mass)
scale_tool.setPathToSubject('')

scale_tool.getGenericModelMaker().setModelFileName(os.path.join(location_models, model_name))
scale_tool.getGenericModelMaker().setMarkerSetFileName(os.path.join(location_markersets, markerset_name))
scale_tool.getGenericModelMaker().processModel()

scale_tool.getModelScaler().setMarkerFileName(marker_file)
# scale_tool.getModelScaler().setOutputModelFileName(os.path.join(data_folder,scaled_model_filename))
scale_tool.getModelScaler().processModel(model, '', subject_mass)

scale_tool.getMarkerPlacer().setMarkerFileName(marker_file)
scale_tool.getMarkerPlacer().processModel(model)

scale_tool.run()

#Save Model 
model.printToXML(os.path.join(location_motion_data,scaled_model_filename))

#Perform IK 
ik_tool = opensim.InverseKinematicsTool(os.path.join(location_XMLs, ik_file))
ik_tool.setModel(model)
ik_tool.setName(marker_data_filenames)
ik_tool.setMarkerDataFileName(os.path.join(location_motion_data, marker_data_filenames))
ik_tool.setStartTime(ik_start_time)
ik_tool.setEndTime(ik_end_time)
ik_tool.setOutputMotionFileName(os.path.join(location_motion_data, save_ik_name))
ik_tool.run()

#Build external loads file and save to disk - this will be loaded in for IK and SO 
external_loads = opensim.ExternalLoads(os.path.join(location_XMLs, external_loads_xml), True)
external_loads.setLowpassCutoffFrequencyForLoadKinematics(kinematic_filter_frequency)
external_loads.setDataFileName(os.path.join(location_motion_data, pedal_data_filenames))
external_loads.setName('both_pedal_forces')
external_loads.printToXML(os.path.join(location_motion_data, save_external_loads_name))

#perform ID 
id_tool = opensim.InverseDynamicsTool()
id_tool.setModel(model)
# id_tool.setModelFileName()
id_tool.setCoordinatesFileName(os.path.join(location_motion_data, save_ik_name))
id_tool.setLowpassCutoffFrequency(kinematic_filter_frequency)
id_tool.setResultsDir(location_motion_data)
id_tool.setOutputGenForceFileName(save_id_name)
id_tool.setStartTime(id_start_time)
id_tool.setEndTime(id_end_time)

excludedForces = opensim.ArrayStr()
excludedForces.append('Muscles')
id_tool.setExcludedForces(excludedForces)
id_tool.setExternalLoadsFileName(os.path.join(location_motion_data, save_external_loads_name))
id_tool.printToXML(os.path.join(location_motion_data, save_id_settings_name))
id_tool.run()


#Re-Load Model 
#This was added to re-initiate the model so that it wouldnt have multiple sets of external loads associated with it. 

model = opensim.Model(os.path.join(location_motion_data, scaled_model_filename))
model.updateMarkerSet(markerset)
model.initSystem()
state = model.initSystem()

#Functions to create torque and point actuators, to be attached to model. 
#These are to add for eating up forces during static optimization

def createTorqueActuator(bodyA, bodyB, axis, name, optimal_force=10, 
                         torque_global=True, min_control=-float('inf'), 
                         max_control=float('inf')):
    torque_actuator = opensim.TorqueActuator()
    torque_actuator.setBodyA(bodyA)
    torque_actuator.setBodyB(bodyB)
    torque_actuator.setAxis(opensim.Vec3(axis[0], axis[1], axis[2]))
    torque_actuator.setOptimalForce(optimal_force)
    torque_actuator.setTorqueIsGlobal(torque_global)
    torque_actuator.setMinControl(min_control)
    torque_actuator.setMaxControl(max_control)
    torque_actuator.setName(name)
    return(torque_actuator)

def createPointActuator(body, point, direction, name, optimal_force=10, 
                        min_control=-float('inf'), max_control=float('inf'), 
                        point_is_global=False, force_is_global=True):
    point_actuator = opensim.PointActuator()
    point_actuator.set_body(body)
    point_actuator.set_point(opensim.Vec3(point[0], point[1], point[2]))
    point_actuator.set_direction(opensim.Vec3(direction[0], direction[1], direction[2]))
    point_actuator.setOptimalForce(optimal_force)
    point_actuator.setMinControl(min_control)
    point_actuator.setMaxControl(max_control)
    point_actuator.setName(name)
    point_actuator.set_point_is_global(point_is_global)
    point_actuator.set_force_is_global(force_is_global)
    return(point_actuator)

# Add torque actuators
pelvis_body = model.getBodySet().get('pelvis')
ground_body = model.getGround()
Mx_axis = (1,0,0)
Mx_name = 'pelvis_Mx'
My_axis = (0,1,0)
My_name = 'pelvis_My'
Mz_axis = (0,0,1)
Mz_name = 'pelvis_Mz'
pelvis_Mx_torque_actuator = createTorqueActuator(pelvis_body, ground_body, Mx_axis, Mx_name)
pelvis_My_torque_actuator = createTorqueActuator(pelvis_body, ground_body, My_axis, My_name)
pelvis_Mz_torque_actuator = createTorqueActuator(pelvis_body, ground_body, Mz_axis, Mz_name)
model.getForceSet().append(pelvis_Mx_torque_actuator)
model.getForceSet().append(pelvis_My_torque_actuator)
model.getForceSet().append(pelvis_Mz_torque_actuator)

# Add point actuators
pelvis_Fx_point_actuator = createPointActuator('pelvis', (-0.034,0.0,0), (1,0,0), 'pelvis_FX')
pelvis_Fy_point_actuator = createPointActuator('pelvis', (-0.034,0.0,0), (0,1,0), 'pelvis_Fy')
pelvis_Fz_point_actuator = createPointActuator('pelvis', (-0.034,0.0,0), (0,0,1), 'pelvis_Fz')

model.getForceSet().append(pelvis_Fx_point_actuator)
model.getForceSet().append(pelvis_Fy_point_actuator)
model.getForceSet().append(pelvis_Fz_point_actuator)


#Analyze / SO tool 
analyze_tool = opensim.AnalyzeTool(os.path.join(location_XMLs, so_analyze_file), False)
analyze_tool.setModel(model)
# analyzeTool.setModelFilename(osimModel.getDocumentFileName());
analyze_tool.setInitialTime(so_start_time)
analyze_tool.setFinalTime(so_end_time)
analyze_tool.setLowpassCutoffFrequency(kinematic_filter_frequency)
analyze_tool.setCoordinatesFileName(os.path.join(location_motion_data, save_ik_name))
analyze_tool.setExternalLoadsFileName(os.path.join(location_motion_data, save_external_loads_name))
analyze_tool.setLoadModelAndInput(True)
analyze_tool.setResultsDir(location_motion_data)


# These are parameters that can be used to programatically alter the optimization parameters
# analyze_tool.setReplaceForceSet(False)
# analyze_tool.setOutputPrecision(6)
# analyze_tool.setSolveForEquilibrium(False)
# analyze_tool.setMaximumNumberOfSteps(20000)
# analyze_tool.setMaxDT(1)
# analyze_tool.setMinDT(1e-008)
# analyze_tool.setErrorTolerance(1e-005)

# analyze_tool.getAnalysisSet().get(0).setStartTime(so_start_time)
# analyze_tool.getAnalysisSet().get(0).setEndTime(so_end_time)

analyze_tool.printToXML(os.path.join(location_motion_data, save_analyze_tool_setting_name))
analyze_tool.run()
