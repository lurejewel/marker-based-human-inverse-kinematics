# marker-based human inverse kinematics

Calculate joint angles according to kinematic tracking data defined by markers.
Run *model_inverseKinematics_OpenSimMethod.m* or *model_inverseKinematics_CurrentMethod.m* to proceed.

## input, output, notes

**input**: both can be processed properly:
- A: marker data in .trc, format should be compatible with OpenSim
- B: marker data in .csv, directly exported from Vicon Nexus

**output**
- animation video in .avi format
- jonit angle record in .csv format

**notes**
- you should have *OpenSim API* installed on MATLAB. check this website for more detail: https://opensimconfluence.atlassian.net/wiki/spaces/OpenSim/pages/53089380/Scripting+with+Matlab
- all codes tested on OpenSim 4.5 and MATLAB R2024b. it should work well on OpenSim 4.* and MATLAB version after R2016a, but not guaranteed.

## model information

### Axis definition

X: forward
Y: upward
Z: rightward

### Joint and DOF definition

| Name | sagittal rotation | frontal rotation | transversal rotation | translational DOF |
| --- | --- | --- | --- | --- |
| ground-pelvis | pelvis_tilt | pelvis_list | pelvis_rotation | pelvis_tx/ty/tz |
| hip_r | hip_flexion_r | hip_adduction_r | hip_rotation_r | |
| hip_l | hip_flexion_l | hip_adduction_l | hip_rotation_l | |
| knee_r | knee_flexion_r | knee_adduction_r | knee_rotation_r | |
| knee_l | knee_flexion_l | knee_adduction_l | knee_rotation_l | |
| ankle_r | ankle_dorsiflexion_r | ankle_adduction_r | ankle_rotation_r | |
| ankle_l | ankle_dorsiflexion_l | ankle_adduction_l | ankle_rotation_l | |
| back | lumbar_extension | lumbar_bending | lumbar_rotation | |
| neck | neck_extension | neck_bending | neck_rotation | |
| shoulder_r | arm_flex_r | arm_add_r | arm_rot_r | |
| shoulder_l | arm_flex_l | arm_add_l | arm_rot_l | |
| elbow_r | elbow_flex_r | elbow_rot_r | | |
| elbow_l | elbow_flex_l | elbow_rot_l | | |
| wrist_r | wrist_flex_r | wrist_dev_r | | |
| wrist_l | wrist_flex_l | wrist_dev_l | | |

### Body (skeleton) definition

| Name | Mass | COM | Inertia |
| --- | --- | --- | --- |
| ground | 0 | [0 0 0] | [0 0 0 0 0 0] |
| pelvis | 11.777 | [-0.0707 0 0] | [0.1028 0.0871 0.0579 0 0 0] |
| femur_r | 9.3014 | [0 -0.17 0] | [0.1339 0.0351 0.1412 0 0 0] |
| femur_l | 9.3014 | [0 -0.17 0] | [0.1339 0.0351 0.1412 0 0 0] |
| tibia_r | 3.7075 | [0 -0.1867 0] | [0.0504 0.0051 0.0511 0 0 0] |
| tibia_l | 3.7075 | [0 -0.1867 0] | [0.0504 0.0051 0.0511 0 0 0] |
| foot_r | 1.25 | [0.1 0.03 0] | [0.014 0.039 0.041 0 0 0] |
| foot_l | 1.25 | [0.1 0.03 0] | [0.014 0.039 0.041 0 0 0] |
| torso | 20.8266 | [-0.03 0.32 0] | [1.4745 0.7555 1.4314 0 0 0] |
| head | 6 | [0.0173 0.063 0] | [0.0173 0.015 0.022605 0 0 0] |
| arm_r | 2.0325 | [0 -0.164502 0] | [0.011946 0.004121 0.013409 0 0 0] |
| arm_l | 2.0325 | [0 -0.164502 0] | [0.011946 0.004121 0.013409 0 0 0] |
| forearm_r | 1.215 | [0 -0.120525 0] | [0.002962 0.000618 0.003213 0 0 0] |
| forearm_l | 1.215 | [0 -0.120525 0] | [0.002962 0.000618 0.003213 0 0 0] |
| hand_r | 0.4575 | [0 -0.068095 0] | [0.000892 0.000547 0.00134 0 0 0] |
| hand_l | 0.4575 | [0 -0.068095 0] | [0.000892 0.000547 0.00134 0 0 0] |

> Data derived from the official OpenSim model.

### Marker definition

| Name | attached body | deviation |
| --- | --- | --- |
| RFHD | head | [0.0858 0.191 0.037] |
| LFHD | head | [0.0858 0.191 -0.037] |
| RBHD | head | [-0.079 0.191 0.037] |
| LBHD | head | [-0.079 0.191 -0.037] |
| Sternum | torso | [0.07 0.3 0] |
| RAcromium | torso | [-0.03 0.44 0.15] |
| LAcromium | torso | [-0.03 0.44 -0.15] |
| C7 | torso | [-0.07 0.44 0] |
| T10 | torso | [-0.1 0.22 0] |
| Clavicle | torso | [0.04 0.38 0] |
| RASIS | pelvis | [0.02 0.03 0.128] |
| LASIS | pelvis | [0.02 0.03 -0.128] |
| RPSIS | pelvis | [-0.155 0.035 0.045] |
| LPSIS | pelvis | [-0.155 0.035 -0.045] |
| RTIB | tibia_r | [0.00971752 -0.225862 0.05229675] |
| LTIB | tibia_l | [0.00971752 -0.225862 -0.05229675] |
| RBicep | arm_r | [-0.04 -0.15 0] |
| LBicep | arm_l | [-0.04 -0.15 0] |
| RElbow | arm_r | [0.00729 -0.28128 0.06786] |
| LElbow | arm_l | [0.00729 -0.28128 -0.06786] |
| RFAsuperior | forearm_r | [-0.02 -0.15 0.03] |
| LFAsuperior | forearm_l | [-0.02 -0.15 -0.03] |
| RThighUpper | femur_r | [0.018 -0.2 0.064] |
| RThighFront | femur_r | [0.08 -0.25 0.0047] |
| RThighRear | femur_r | [0.01 -0.3 0.06] |
| RKneeLat | femur_r | [0 -0.404 0.05] |
| RKneeMed | femur_r | [0 -0.404 -0.05] |
| RShankUpper | tibia_r | [0.005 -0.065 0.05] |
| RShankFront | tibia_r | [0.005 -0.08 0] |
| RShankRear | tibia_r | [-0.02 -0.13 0.05] |
| RAnkleLat | tibia_r | [-0.005 -0.41 0.053] |
| RAnkleMed | tibia_r | [0.006 -0.3888 -0.038] |
| RHeel | foot_r | [-0.02 0.02 0] |
| RToeLat | foot_r | [0.1412 -0.042 0.0571] |
| RToeTip | foot_r | [0.2112 -0.037 0.0079] |
| RWristMed | forearm_r | [-0.01531 -0.23272 -0.01839] |
| RWristLat | forearm_r | [-0.00316 -0.22312 0.04988] |
| LThighUpper | femur_l | [0.018 -0.2 -0.064] |
| LThighFront | femur_l | [0.08 -0.25 -0.0047] |
| LThighRear | femur_l | [0.01 -0.3 -0.06] |
| LKneeLat | femur_l | [0 -0.404 -0.05] |
| LKneeMed | femur_l | [0 -0.404 0.05] |
| LShankUpper | tibia_l | [0.005 -0.065 -0.05] |
| LShankFront | tibia_l | [0.005 -0.08 0] |
| LShankRear | tibia_l | [-0.02 -0.13 -0.05] |
| LAnkleLat | tibia_l | [-0.005 -0.41 -0.053] |
| LAnkleMed | tibia_l | [0.006 -0.3888 0.038] |
| LHeel | foot_l | [-0.02 0.02 0] |
| LToeLat | foot_l | [0.1412 -0.042 -0.0571] |
| LToeTip | foot_l | [0.2112 -0.037 -0.0079] |
| LWristMed | forearm_l | [-0.01531 -0.23272 0.01839] |
| LWristLat | forearm_l | [-0.00316 -0.22312 -0.04988] |

> Part of the deviation data derived from the official OpenSim model.

# muscle length validation

Calculate the length of muscles, and compare them with OpenSim results.
Run *muscle_length_validation* to proceed.

## input, output, notes

**input**:
- OpenSim file path (.osim): extract muscles from the model.
- OpenSim muscle length references (.csv): can be displayed and exported from OpenSim Tools -> Plot.. function.

**output**:
- muscles: An array of muscle classes that obtain informations of all muscles (extracted from .osim model)
- muscleMap: stores the muscle name - index mapping relationship.
- plots of length comparison for each muscle of each dof (data stroed in ).

## muscle

Muscle info extracted from the OpenSim Gait2392 model.
More muscles (espacially upper limb muscles) will be added in the future.

basic information of the muscle class:
- name
- nPathPoint (#path points)
- pathPoints (path point class)
- mvc (max voluntary contraction / max isometric force)
- optFiberLen (optimal fiber length)
- tendonSlackLen (tendon slack length)
- penAngleAtOpt (pennation angle (angle between tendon and fibers) at optimal fiber length (in rad))
- FmaxTendonStrain (tendon strain at mvc)
- FmaxMuscleStrain (passive muscle strain at mvc)
- KshapeActive (shape factor for Gautssian active muscle force-length relationship)
- KshapePassive (exponential shape factor for passive force-length relationship)
- Af (force-velocity shape factor)
- Flen (max normalized lengthening force)
- actTime (activation time constant (in sec))
- deactTime (deactivation time constant (in sec))
- muscleLength (muscle length (fiber + tendon))

## muscle groups

Muscles are grouped according to the dofs of joints they participate:

### hip-related muscle groups

- **R_hip_abd**: glut_max1_r, glut_med1_r, glut_med2_r, glut_med3_r, glut_min1_r, glut_min2_r, glut_min3_r, peri_r, sar_r, tfl_r
- **R_hip_add**: add_brev_r, add_long_r, add_mag1_r, add_mag2_r, add_mag3_r bifemlh_r, grac_r, pect_r, semimem_r, semiten_r
- **R_hip_flex**: add_brev_r, add_long_r, glut_med1_r, glut_min1_r, grac_r, iliacus_r, pect_r, psoas_r, rect_fem_r, sar_r, tfl_r
- **R_hip_ext**: add_long_r, add_mag1_r, add_mag2_r, add_mag3_r, bifemlh_r, glut_max1_r, glut_max2_r, glut_max3_r, glut_med3_r, glut_min3_r, semimem_r, semiten_r
- **R_hip_inrot**: glut_med1_r, glut_min1_r, iliacus_r, psoas_r, tfl_r
- **R_hip_exrot**: gem_r, glut_med3_r, glut_min3_r, peri_r, quad_fem_r

> left side muscles are similarly grouped.

### knee-related muscle groups

- **R_knee_bend**: bifemlh_r, bifemsh_r, grac_r, lat_gas_r, med_gas_r, sar_r, semimem_r, semiten_r
- **R_knee_ext**: rect_fem_r, vas_int_r, vas_lat_r, vas_med_r

> left side muscles are similarly grouped.

### ankle-related muscle groups

- **R_ankle_pf**: flex_dig_r, flex_hal_r, lat_gas_r, med_gas_r, per_brev_r, per_long_r, soleus_r, tib_post_r
- **R_ankle_df**: ext_dig_r, ext_hal_r, per_tert_r, tib_ant_r
- **R_inverter**: ext_hal_r, flex_dig_r, flex_hal_r, tib_ant_r, tib_post_r
- **R_everter**: ext_dig_r, per_brev_r, per_long_r, per_tert_r

> left side muscles are similarly grouped.

### back-related muscle groups

- **back_flex**: extobl_l, extobl_r, intobl_l, intobl_r
- **back_ext**: ercspn_l, ercspn_r
- **back_rlb**: ercspn_r, extobl_r, intobl_r
- **back_llb**: ercspn_l, extobl_l, intobl_l
- **back_introt**: ercspn_r, extolb_l, intobl_r
- **back_extrot**: ercspn_l, extolb_r, intobl_l