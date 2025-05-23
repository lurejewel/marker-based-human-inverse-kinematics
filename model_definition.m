% ----------
% TODO:
% 1. 静止站立时的scale factor的计算有缺失：、head、arm、forearm。
% ----------

close all
import org.opensim.modeling.*

BodyMass = 72;
BodyHeight = 180;
scaleFactor.height = BodyHeight / 168.85;

%% Define joints of the model

dofs = containers.Map();

% global DOF
dofs('pelvis_tx') = struct('axis', [1 0 0 1]', 'range', [-5 5]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('pelvis_ty') = struct('axis', [0 1 0 1]', 'range', [-1 2]', 'defaultValue', 0.95, 'defaultSpeed', 0);
dofs('pelvis_tz') = struct('axis', [0 0 1 1]', 'range', [-3 3]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('pelvis_tilt') = struct('axis', [0 0 1 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('pelvis_list') = struct('axis', [1 0 0 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('pelvis_rotation') = struct('axis', [0 1 0 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
ground_pelvis = JointDef('translation_parent', [0 0 0]', ...
    'Ndof', 6, ...
    'axis', [dofs('pelvis_tx').axis, dofs('pelvis_ty').axis, dofs('pelvis_tz').axis, dofs('pelvis_tilt').axis, dofs('pelvis_list').axis, dofs('pelvis_rotation').axis], ...
    'range', [dofs('pelvis_tx').range, dofs('pelvis_ty').range, dofs('pelvis_tz').range, dofs('pelvis_tilt').range, dofs('pelvis_list').range, dofs('pelvis_rotation').range], ...
    'defaultValue', [dofs('pelvis_tx').defaultValue, dofs('pelvis_ty').defaultValue, dofs('pelvis_tz').defaultValue, dofs('pelvis_tilt').defaultValue, dofs('pelvis_list').defaultValue, dofs('pelvis_rotation').defaultValue], ...
    'defaultSpeed', [dofs('pelvis_tx').defaultSpeed, dofs('pelvis_ty').defaultSpeed, dofs('pelvis_tz').defaultSpeed, dofs('pelvis_tilt').defaultSpeed, dofs('pelvis_list').defaultSpeed, dofs('pelvis_rotation').defaultSpeed]);

% right hip
dofs('hip_flexion_r') = struct('axis', [0 0 1 0]', 'range', deg2rad(120)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('hip_adduction_r') = struct('axis', [1 0 0 0]', 'range', deg2rad(120)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('hip_rotation_r') = struct('axis', [0 1 0 0]', 'range', deg2rad(120)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
hip_r = JointDef('translation_parent', [-0.0707 -0.0661 0.0835]', ...
    'Ndof', 3, ...
    'axis', [dofs('hip_flexion_r').axis, dofs('hip_adduction_r').axis, dofs('hip_rotation_r').axis], ...
    'range', [dofs('hip_flexion_r').range, dofs('hip_adduction_r').range, dofs('hip_rotation_r').range], ...
    'defaultValue', [dofs('hip_flexion_r').defaultValue, dofs('hip_adduction_r').defaultValue, dofs('hip_rotation_r').defaultValue], ...
    'defaultSpeed', [dofs('hip_flexion_r').defaultSpeed, dofs('hip_adduction_r').defaultSpeed, dofs('hip_rotation_r').defaultSpeed]);

% left hip
dofs('hip_flexion_l') = struct('axis', [0 0 1 0]', 'range', deg2rad(120)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('hip_adduction_l') = struct('axis', [-1 0 0 0]', 'range', deg2rad(120)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('hip_rotation_l') = struct('axis', [0 -1 0 0]', 'range', deg2rad(120)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
hip_l = JointDef('translation_parent', [-0.0707 -0.0661 -0.0835]', ...
    'Ndof', 3, ...
    'axis', [dofs('hip_flexion_l').axis, dofs('hip_adduction_l').axis, dofs('hip_rotation_l').axis], ...
    'range', [dofs('hip_flexion_l').range, dofs('hip_adduction_l').range, dofs('hip_rotation_l').range], ...
    'defaultValue', [dofs('hip_flexion_l').defaultValue, dofs('hip_adduction_l').defaultValue, dofs('hip_rotation_l').defaultValue], ...
    'defaultSpeed', [dofs('hip_flexion_l').defaultSpeed, dofs('hip_adduction_l').defaultSpeed, dofs('hip_rotation_l').defaultSpeed]);

% right knee
dofs('knee_flexion_r') = struct('axis', [0 0 1 0]', 'range', deg2rad(10)*[-12 1]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('knee_adduction_r') = struct('axis', [1 0 0 0]', 'range', deg2rad(10)*[-3 3]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('knee_rotation_r') = struct('axis', [0 1 0 0]', 'range', deg2rad(10)*[-3 3]', 'defaultValue', 0, 'defaultSpeed', 0);
knee_r = JointDef('translation_parent', [0 0 0]', ...
    'Ndof', 3, ...
    'axis', [dofs('knee_flexion_r').axis, dofs('knee_adduction_r').axis, dofs('knee_rotation_r').axis], ...
    'range', [dofs('knee_flexion_r').range, dofs('knee_adduction_r').range, dofs('knee_rotation_r').range], ...
    'defaultValue', [dofs('knee_flexion_r').defaultValue, dofs('knee_adduction_r').defaultValue, dofs('knee_rotation_r').defaultValue], ...
    'defaultSpeed', [dofs('knee_flexion_r').defaultSpeed, dofs('knee_adduction_r').defaultSpeed, dofs('knee_rotation_r').defaultSpeed]);

% left knee
dofs('knee_flexion_l') = struct('axis', [0 0 1 0]', 'range', deg2rad(10)*[-12 1]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('knee_adduction_l') = struct('axis', [-1 0 0 0]', 'range', deg2rad(10)*[-3 3]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('knee_rotation_l') = struct('axis', [0 -1 0 0]', 'range', deg2rad(10)*[-3 3]', 'defaultValue', 0, 'defaultSpeed', 0);
knee_l = JointDef('translation_parent', [0 0 0]', ...
    'Ndof', 3, ...
    'axis', [dofs('knee_flexion_l').axis, dofs('knee_adduction_l').axis, dofs('knee_rotation_l').axis], ...
    'range', [dofs('knee_flexion_l').range, dofs('knee_adduction_l').range, dofs('knee_rotation_l').range], ...
    'defaultValue', [dofs('knee_flexion_l').defaultValue, dofs('knee_adduction_l').defaultValue, dofs('knee_rotation_l').defaultValue], ...
    'defaultSpeed', [dofs('knee_flexion_l').defaultSpeed, dofs('knee_adduction_l').defaultSpeed, dofs('knee_rotation_l').defaultSpeed]);

% knee roll
angle = [-2.0944 -1.74533 -1.39626 -1.0472 -0.698132 -0.349066 -0.174533 0.197344 0.337395 0.490178 1.52146 2.0944];
x = [-0.0032 0.00179 0.00411 0.0041 0.00212 -0.001 -0.0031 -0.005227 -0.005435 -0.005574 -0.005435 -0.00525];
xSpline = spline(angle, x);
angle = [-2.0944 -1.22173 -0.523599 -0.349066 -0.174533 0.159149 2.0944];
y = [-0.4226 -0.4082 -0.399 -0.3976 -0.3966 -0.395264 -0.396];
ySpline = spline(angle, y);

% right ankle
dofs('ankle_dorsiflexion_r') = struct('axis', [-0.10501355 -0.17402245 0.97912632 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('ankle_adduction_r') = struct('axis', [0.78717961 0.60474746 -0.12094949 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
ankle_r = JointDef('translation_parent', [0 -0.43 0]', ...
    'Ndof', 2, ...
    'axis', [dofs('ankle_dorsiflexion_r').axis, dofs('ankle_adduction_r').axis], ...
    'range', [dofs('ankle_dorsiflexion_r').range, dofs('ankle_adduction_r').range], ...
    'defaultValue', [dofs('ankle_dorsiflexion_r').defaultValue, dofs('ankle_adduction_r').defaultValue], ...
    'defaultSpeed', [dofs('ankle_dorsiflexion_r').defaultSpeed, dofs('ankle_adduction_r').defaultSpeed]);

% left ankle
dofs('ankle_dorsiflexion_l') = struct('axis', [0.10501355 0.17402245 0.97912632 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('ankle_adduction_l') = struct('axis', [-0.78717961 -0.60474746 -0.12094949 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
ankle_l = JointDef('translation_parent', [0 -0.43 0]', ...
    'Ndof', 2, ...
    'axis', [dofs('ankle_dorsiflexion_l').axis, dofs('ankle_adduction_l').axis], ...
    'range', [dofs('ankle_dorsiflexion_l').range, dofs('ankle_adduction_l').range], ...
    'defaultValue', [dofs('ankle_dorsiflexion_l').defaultValue, dofs('ankle_adduction_l').defaultValue], ...
    'defaultSpeed', [dofs('ankle_dorsiflexion_l').defaultSpeed, dofs('ankle_adduction_l').defaultSpeed]);

% back (lumbarsacral joint)
dofs('lumbar_extension') = struct('axis', [0 0 1 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('lumbar_bending') = struct('axis', [1 0 0 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('lumbar_rotation') = struct('axis', [0 1 0 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
back = JointDef('translation_parent', [-0.1007 0.0815 0]', ...
    'Ndof', 3, ...
    'axis', [dofs('lumbar_extension').axis, dofs('lumbar_bending').axis, dofs('lumbar_rotation').axis], ...
    'range', [dofs('lumbar_extension').range, dofs('lumbar_bending').range, dofs('lumbar_rotation').range], ...
    'defaultValue', [dofs('lumbar_extension').defaultValue, dofs('lumbar_bending').defaultValue, dofs('lumbar_rotation').defaultValue], ...
    'defaultSpeed', [dofs('lumbar_extension').defaultSpeed, dofs('lumbar_bending').defaultSpeed, dofs('lumbar_rotation').defaultSpeed]);

% t10 (thoracolumbar joint)
dofs('t10_extension') = struct('axis', [0 0 1 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('t10_bending') = struct('axis', [1 0 0 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('t10_rotation') = struct('axis', [0 1 0 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
t10 = JointDef('translation_parent', [-0.03 0.2 0]', ...
    'Ndof', 3, ...
    'axis', [dofs('t10_extension').axis, dofs('t10_bending').axis, dofs('t10_rotation').axis], ...
    'range', [dofs('t10_extension').range, dofs('t10_bending').range, dofs('t10_rotation').range], ...
    'defaultValue', [dofs('t10_extension').defaultValue, dofs('t10_bending').defaultValue, dofs('t10_rotation').defaultValue], ...
    'defaultSpeed', [dofs('t10_extension').defaultSpeed, dofs('t10_bending').defaultSpeed, dofs('t10_rotation').defaultSpeed]);

% neck
dofs('neck_extension') = struct('axis', [0 0 1 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('neck_bending') = struct('axis', [1 0 0 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('neck_rotation') = struct('axis', [0 1 0 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
neck = JointDef('translation_parent', [0.03 0.2 0]', ...
    'Ndof', 3, ...
    'axis', [dofs('neck_extension').axis, dofs('neck_bending').axis, dofs('neck_rotation').axis], ...
    'range', [dofs('neck_extension').range, dofs('neck_bending').range, dofs('neck_rotation').range], ...
    'defaultValue', [dofs('neck_extension').defaultValue, dofs('neck_bending').defaultValue, dofs('neck_rotation').defaultValue], ...
    'defaultSpeed', [dofs('neck_extension').defaultSpeed, dofs('neck_bending').defaultSpeed, dofs('neck_rotation').defaultSpeed]);

% right shoulder
dofs('arm_flex_r') = struct('axis', [0 0 1 0]', 'range', deg2rad(180)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('arm_add_r') = struct('axis', [1 0 0 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('arm_rot_r') = struct('axis', [0 1 0 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
shoulder_r = JointDef('translation_parent', [0.033155 0.1715 0.17]', ...
    'Ndof', 3, ...
    'axis', [dofs('arm_flex_r').axis, dofs('arm_add_r').axis, dofs('arm_rot_r').axis], ...
    'range', [dofs('arm_flex_r').range, dofs('arm_add_r').range, dofs('arm_rot_r').range], ...
    'defaultValue', [dofs('arm_flex_r').defaultValue, dofs('arm_add_r').defaultValue, dofs('arm_rot_r').defaultValue], ...
    'defaultSpeed', [dofs('arm_flex_r').defaultSpeed, dofs('arm_add_r').defaultSpeed, dofs('arm_rot_r').defaultSpeed]);

% left shoulder
dofs('arm_flex_l') = struct('axis', [0 0 1 0]', 'range', deg2rad(180)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('arm_add_l') = struct('axis', [-1 0 0 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('arm_rot_l') = struct('axis', [0 -1 0 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
shoulder_l = JointDef('translation_parent', [0.033155 0.1715 -0.17]', ...
    'Ndof', 3, ...
    'axis', [dofs('arm_flex_l').axis, dofs('arm_add_l').axis, dofs('arm_rot_l').axis], ...
    'range', [dofs('arm_flex_l').range, dofs('arm_add_l').range, dofs('arm_rot_l').range], ...
    'defaultValue', [dofs('arm_flex_l').defaultValue, dofs('arm_add_l').defaultValue, dofs('arm_rot_l').defaultValue], ...
    'defaultSpeed', [dofs('arm_flex_l').defaultSpeed, dofs('arm_add_l').defaultSpeed, dofs('arm_rot_l').defaultSpeed]);

% right elbow
dofs('elbow_flex_r') = struct('axis', [0 0 1 0]', 'range', deg2rad(150)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('elbow_rot_r') = struct('axis', [0 1 0 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
elbow_r = JointDef('translation_parent', [0.013144 -0.286273 -0.009595]', ...
    'Ndof', 2, ...
    'axis', [dofs('elbow_flex_r').axis, dofs('elbow_rot_r').axis], ...
    'range', [dofs('elbow_flex_r').range, dofs('elbow_rot_r').range], ...
    'defaultValue', [dofs('elbow_flex_r').defaultValue, dofs('elbow_rot_r').defaultValue], ...
    'defaultSpeed', [dofs('elbow_flex_r').defaultSpeed, dofs('elbow_rot_r').defaultSpeed]);

% left elbow
dofs('elbow_flex_l') = struct('axis', [0 0 1 0]', 'range', deg2rad(150)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('elbow_rot_l') = struct('axis', [0 -1 0 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
elbow_l = JointDef('translation_parent', [0.013144 -0.286273 0.009595]', ...
    'Ndof', 2, ...
    'axis', [dofs('elbow_flex_l').axis, dofs('elbow_rot_l').axis], ...
    'range', [dofs('elbow_flex_l').range, dofs('elbow_rot_l').range], ...
    'defaultValue', [dofs('elbow_flex_l').defaultValue, dofs('elbow_rot_l').defaultValue], ...
    'defaultSpeed', [dofs('elbow_flex_l').defaultSpeed, dofs('elbow_rot_l').defaultSpeed]);

% right wrist
dofs('wrist_flex_r') = struct('axis', [0 0 1 0]', 'range', deg2rad(70)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
wrist_r = JointDef('translation_parent', [-0.008797 -0.235841 0.01361]', ...
    'Ndof', 1, ...
    'axis', dofs('wrist_flex_r').axis, ...
    'range', dofs('wrist_flex_r').range, ...
    'defaultValue', dofs('wrist_flex_r').defaultValue, ...
    'defaultSpeed', dofs('wrist_flex_r').defaultSpeed);

% left wrist
dofs('wrist_flex_l') = struct('axis', [0 0 1 0]', 'range', deg2rad(70)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
wrist_l = JointDef('translation_parent', [-0.008797 -0.235841 -0.01361]', ...
    'Ndof', 1, ...
    'axis', dofs('wrist_flex_l').axis, ...
    'range', dofs('wrist_flex_l').range, ...
    'defaultValue', dofs('wrist_flex_l').defaultValue, ...
    'defaultSpeed', dofs('wrist_flex_l').defaultSpeed);
clear dofs
save joints.mat ground_pelvis hip_r hip_l knee_r knee_l ankle_r ankle_l back t10 neck shoulder_r shoulder_l elbow_r elbow_l wrist_r wrist_l xSpline ySpline

dofMap = containers.Map({'pelvis_tx', 'pelvis_ty', 'pelvis_tz', ...
    'pelvis_tilt', 'pelvis_list', 'pelvis_rotation', ...
    'hip_flexion_r', 'hip_adduction_r', 'hip_rotation_r', ...
    'hip_flexion_l', 'hip_adduction_l', 'hip_rotation_l', ...
    'knee_flexion_r', 'knee_adduction_r', 'knee_rotation_r', ...
    'knee_flexion_l', 'knee_adduction_l', 'knee_rotation_l', ...
    'ankle_dorsiflexion_r', 'ankle_adduction_r', ...
    'ankle_dorsiflexion_l', 'ankle_adduction_l', ...
    'lumbar_extension', 'lumbar_bending', 'lumbar_rotation', ...
    't10_extension', 't10_bending', 't10_rotation', ...
    'neck_extension', 'neck_bending', 'neck_rotation', ...
    'arm_flex_r', 'arm_add_r', 'arm_rot_r', ...
    'arm_flex_l', 'arm_add_l', 'arm_rot_l', ...
    'elbow_flex_r', 'elbow_rot_r', ...
    'elbow_flex_l', 'elbow_rot_l', ...
    'wrist_flex_r', 'wrist_flex_l'}, num2cell(1:43));

save dofMap.mat dofMap

%% Define bodies of the model

ground = BodyDef('mass', 0, 'COM', [0 0 0]', 'inertia', [0 0 0 0 0 0]');
pelvis = BodyDef('mass', 11.777, 'COM', [-0.0707 0 0]', 'inertia', [0.1028 0.0871 0.0579 0 0 0]');
femur_r = BodyDef('mass', 9.3014, 'COM', [0 -0.17 0]', 'inertia', [0.1339 0.0351 0.1412 0 0 0]');
femur_l = BodyDef('mass', 9.3014, 'COM', [0 -0.17 0]', 'inertia', [0.1339 0.0351 0.1412 0 0 0]');
tibia_r = BodyDef('mass', 3.7075, 'COM', [0 -0.1867 0]', 'inertia', [0.0504 0.0051 0.0511 0 0 0]');
tibia_l = BodyDef('mass', 3.7075, 'COM', [0 -0.1867 0]', 'inertia', [0.0504 0.0051 0.0511 0 0 0]');
foot_r = BodyDef('mass', 1.25, 'COM', [0.1 0.03 0]', 'inertia', [0.014 0.039 0.041 0 0 0]');
foot_l = BodyDef('mass', 1.25, 'COM', [0.1 0.03 0]', 'inertia', [0.014 0.039 0.041 0 0 0]');
lumbar = BodyDef('mass', 8.3266, 'COM', [0.01 0.1 0]', 'inertia', [0.042 0.151 0.054 0 0 0]');
thorax = BodyDef('mass', 12.5, 'COM', [0.01 0.1 0]', 'inertia', [0.156 0.284 0.219 0 0 0]');
head = BodyDef('mass', 6, 'COM', [0.0173 0.063 0]', 'inertia', [0.0173 0.015 0.022605 0 0 0]');
arm_r = BodyDef('mass', 2.0325, 'COM', [0 -0.164502 0]', 'inertia', [0.011946 0.004121 0.013409 0 0 0]');
arm_l = BodyDef('mass', 2.0325, 'COM', [0 -0.164502 0]', 'inertia', [0.011946 0.004121 0.013409 0 0 0]');
forearm_r = BodyDef('mass', 1.215, 'COM', [0 -0.120525 0]', 'inertia', [0.002962 0.000618 0.003213 0 0 0]');
forearm_l = BodyDef('mass', 1.215, 'COM', [0 -0.120525 0]', 'inertia', [0.002962 0.000618 0.003213 0 0 0]');
hand_r = BodyDef('mass', 0.4575, 'COM', [0 -0.068095 0]', 'inertia', [0.000892 0.000547 0.00134 0 0 0]');
hand_l = BodyDef('mass', 0.4575, 'COM', [0 -0.068095 0]', 'inertia', [0.000892 0.000547 0.00134 0 0 0]');

%% Scale mass of the bodies

TotalBodyMass = ground.mass + pelvis.mass + femur_r.mass + femur_l.mass + ...
    tibia_r.mass + tibia_l.mass + foot_r.mass + foot_l.mass + thorax.mass + ...
    lumbar.mass + head.mass + arm_r.mass + arm_l.mass + forearm_r.mass + ...
    forearm_l.mass + hand_r.mass + hand_l.mass;

scaleFactor.mass = BodyMass / TotalBodyMass;

pelvis.mass = pelvis.mass * scaleFactor.mass;
femur_r.mass = femur_r.mass * scaleFactor.mass;
femur_l.mass = femur_l.mass * scaleFactor.mass;
tibia_r.mass = tibia_r.mass * scaleFactor.mass;
tibia_l.mass = tibia_l.mass * scaleFactor.mass;
foot_r.mass = foot_r.mass * scaleFactor.mass;
foot_l.mass = foot_l.mass * scaleFactor.mass;
lumbar.mass = lumbar.mass * scaleFactor.mass;
thorax.mass = thorax.mass * scaleFactor.mass; 
head.mass = head.mass * scaleFactor.mass;
arm_r.mass = arm_r.mass * scaleFactor.mass;
arm_l.mass = arm_l.mass * scaleFactor.mass;
forearm_r.mass = forearm_r.mass * scaleFactor.mass;
forearm_l.mass = forearm_l.mass * scaleFactor.mass;
hand_r.mass = hand_r.mass * scaleFactor.mass;
hand_l.mass = hand_l.mass * scaleFactor.mass;

pelvis.inertia = pelvis.inertia * scaleFactor.mass;
femur_r.inertia = femur_r.inertia * scaleFactor.mass;
femur_l.inertia = femur_l.inertia * scaleFactor.mass;
tibia_r.inertia = tibia_r.inertia * scaleFactor.mass;
tibia_l.inertia = tibia_l.inertia * scaleFactor.mass;
foot_r.inertia = foot_r.inertia * scaleFactor.mass;
foot_l.inertia = foot_l.inertia * scaleFactor.mass;
lumbar.inertia = lumbar.inertia * scaleFactor.mass;
thorax.inertia = thorax.inertia * scaleFactor.mass; 
head.inertia = head.inertia * scaleFactor.mass;
arm_r.inertia = arm_r.inertia * scaleFactor.mass;
arm_l.inertia = arm_l.inertia * scaleFactor.mass;
forearm_r.inertia = forearm_r.inertia * scaleFactor.mass;
forearm_l.inertia = forearm_l.inertia * scaleFactor.mass;
hand_r.inertia = hand_r.inertia * scaleFactor.mass;
hand_l.inertia = hand_l.inertia * scaleFactor.mass;
% note: the inertia will also change in the model scaling process

clear BodyHeight BodyMass TotalBodyMass
save bodies.mat ground pelvis femur_r femur_l tibia_r tibia_l foot_r foot_l lumbar thorax head arm_r arm_l forearm_r forearm_l hand_r hand_l

%% Define muscles

[muscles, muscleMap] = read_muscle_info_from_osim('model_display.osim');
save muscles.mat muscles
save muscleMap.mat muscleMap

%% Define markers

RFHD = MarkerDef('attachedBody', 'head', 'deviation', [0.0858 0.191 0.037]'); % 右前额
LFHD = MarkerDef('attachedBody', 'head', 'deviation', [0.0858 0.191 -0.037]'); % 左前额
RBHD = MarkerDef('attachedBody', 'head', 'deviation', [-0.079 0.191 0.037]'); % 右后额
LBHD = MarkerDef('attachedBody', 'head', 'deviation', [-0.079 0.191 -0.037]'); % 左后额
Sternum = MarkerDef('attachedBody', 'thorax', 'deviation', [0.1 0.1 0]'); % 胸骨体中心
RAcromium = MarkerDef('attachedBody', 'thorax', 'deviation', [0 0.24 0.15]'); % 右肩
LAcromium = MarkerDef('attachedBody', 'thorax', 'deviation', [0 0.24 -0.15]'); % 左肩
C7 = MarkerDef('attachedBody', 'thorax', 'deviation', [-0.04 0.24 0]'); % 颈椎
T10 = MarkerDef('attachedBody', 'thorax', 'deviation', [-0.07 0.02 0]'); % 胸椎
Clavicle = MarkerDef('attachedBody', 'thorax', 'deviation', [0.07 0.18 0]'); % 胸骨柄
RASIS = MarkerDef('attachedBody', 'pelvis', 'deviation', [0.02 0.03 0.128]'); % 右髂前上棘
LASIS = MarkerDef('attachedBody', 'pelvis', 'deviation', [0.02 0.03 -0.128]'); % 左髂前上棘
RPSIS = MarkerDef('attachedBody', 'pelvis', 'deviation', [-0.155 0.035 0.045]'); % 右髂后上棘
LPSIS = MarkerDef('attachedBody', 'pelvis', 'deviation', [-0.155 0.035 -0.045]'); % 左髂后上棘
RTIB = MarkerDef('attachedBody', 'tibia_r', 'deviation', [0.00971752 -0.225862 0.05229675]'); % 右小腿外侧中部
LTIB = MarkerDef('attachedBody', 'tibia_l', 'deviation', [0.00971752 -0.225862 -0.05229675]'); % 左小腿外侧中部
RBicep = MarkerDef('attachedBody', 'arm_r', 'deviation', [-0.04 -0.15 0]'); % 右侧肱三头肌中部
LBicep = MarkerDef('attachedBody', 'arm_l', 'deviation', [-0.04 -0.15 0]'); % 左侧肱三头肌中部
RElbow = MarkerDef('attachedBody', 'arm_r', 'deviation', [0.00729 -0.28128 0.06786]'); % 右侧肱骨外上髁
LElbow = MarkerDef('attachedBody', 'arm_l', 'deviation', [0.00729 -0.28128 -0.06786]'); % 左侧肱骨外上髁
RFAsuperior = MarkerDef('attachedBody', 'forearm_r', 'deviation', [-0.02 -0.15 0.03]'); % 右侧小臂外侧中间
LFAsuperior = MarkerDef('attachedBody', 'forearm_l', 'deviation', [-0.02 -0.15 -0.03]'); % 左侧小臂外侧中间
RThighUpper = MarkerDef('attachedBody', 'femur_r', 'deviation', [0.018 -0.2 0.064]'); % 右大腿外侧中部偏上
RThighFront = MarkerDef('attachedBody', 'femur_r', 'deviation', [0.08 -0.25 0.0047]'); % 右大腿前侧中间
RThighRear = MarkerDef('attachedBody', 'femur_r', 'deviation', [0.01 -0.3 0.06]'); % 右大腿外侧中间偏下
RKneeLat = MarkerDef('attachedBody', 'femur_r', 'deviation', [0 -0.404 0.05]'); % 右侧股骨外上髁
RKneeMed = MarkerDef('attachedBody', 'femur_r', 'deviation', [0 -0.404 -0.05]'); % 右侧股骨内上髁
RShankUpper = MarkerDef('attachedBody', 'tibia_r', 'deviation', [0.005 -0.065 0.05]'); % 右侧胫骨外侧髁
RShankFront = MarkerDef('attachedBody', 'tibia_r', 'deviation', [0.005 -0.08 0]'); % 右侧胫骨粗隆靠下
RShankRear = MarkerDef('attachedBody', 'tibia_r', 'deviation', [-0.02 -0.13 0.05]'); % 右侧胫骨外侧髁偏下
RAnkleLat = MarkerDef('attachedBody', 'tibia_r', 'deviation', [-0.005 -0.41 0.053]'); % 右侧外踝
RAnkleMed = MarkerDef('attachedBody', 'tibia_r', 'deviation', [0.006 -0.3888 -0.038]'); % 右侧内踝
RHeel = MarkerDef('attachedBody', 'foot_r', 'deviation', [-0.02 0.02 0]'); % 右脚跟
RToeLat = MarkerDef('attachedBody', 'foot_r', 'deviation', [0.1412 -0.042 0.0571]'); % 右侧小脚趾尖
RToeTip = MarkerDef('attachedBody', 'foot_r', 'deviation', [0.2112 -0.037 0.0079]'); % 右侧大脚趾尖
RWristMed = MarkerDef('attachedBody', 'forearm_r', 'deviation', [-0.01531 -0.23272 -0.01839]'); % 右侧桡骨粗隆
RWristLat = MarkerDef('attachedBody', 'forearm_r', 'deviation', [-0.00316 -0.22312 0.04988]'); % 右侧尺骨粗隆
LThighUpper = MarkerDef('attachedBody', 'femur_l', 'deviation', [0.018 -0.2 -0.064]'); % 左大腿外侧中部偏上
LThighFront = MarkerDef('attachedBody', 'femur_l', 'deviation', [0.08 -0.25 -0.0047]'); % 左大腿前侧中间
LThighRear = MarkerDef('attachedBody', 'femur_l', 'deviation', [0.01 -0.3 -0.06]'); % 左大腿外侧中间偏下
LKneeLat = MarkerDef('attachedBody', 'femur_l', 'deviation', [0 -0.404 -0.05]'); % 左侧股骨外上髁
LKneeMed = MarkerDef('attachedBody', 'femur_l', 'deviation', [0 -0.404 0.05]'); % 左侧股骨内上髁
LShankUpper = MarkerDef('attachedBody', 'tibia_l', 'deviation', [0.005 -0.065 -0.05]'); % 左侧胫骨外侧髁
LShankFront = MarkerDef('attachedBody', 'tibia_l', 'deviation', [0.005 -0.08 0]'); % 左侧胫骨粗隆靠下
LShankRear = MarkerDef('attachedBody', 'tibia_l', 'deviation', [-0.02 -0.13 -0.05]'); % 左侧胫骨外侧髁偏下
LAnkleLat = MarkerDef('attachedBody', 'tibia_l', 'deviation', [-0.005 -0.41 -0.053]'); % 左侧外踝
LAnkleMed = MarkerDef('attachedBody', 'tibia_l', 'deviation', [0.006 -0.3888 0.038]'); % 左侧内踝
LHeel = MarkerDef('attachedBody', 'foot_l', 'deviation', [-0.02 0.02 0]'); % 左脚跟
LToeLat = MarkerDef('attachedBody', 'foot_l', 'deviation', [0.1412 -0.042 -0.0571]'); % 左侧小脚趾尖
LToeTip = MarkerDef('attachedBody', 'foot_l', 'deviation', [0.2112 -0.037 -0.0079]'); % 左侧大脚趾尖
LWristMed = MarkerDef('attachedBody', 'forearm_l', 'deviation', [-0.01531 -0.23272 0.01839]'); % 左侧桡骨粗隆
LWristLat = MarkerDef('attachedBody', 'forearm_l', 'deviation', [-0.00316 -0.22312 -0.04988]'); % 左侧尺骨粗隆

save markers.mat RFHD LFHD RBHD LBHD Sternum RAcromium LAcromium C7 T10 Clavicle RASIS LASIS RPSIS LPSIS RTIB LTIB RBicep LBicep RElbow LElbow RFAsuperior LFAsuperior RThighUpper RThighFront RThighRear RKneeLat RKneeMed RShankUpper RShankFront RShankRear RAnkleLat RAnkleMed RHeel RToeLat RToeTip RWristMed RWristLat LThighUpper LThighFront LThighRear LKneeLat LKneeMed LShankUpper LShankFront LShankRear LAnkleLat LAnkleMed LHeel LToeLat LToeTip LWristMed LWristLat