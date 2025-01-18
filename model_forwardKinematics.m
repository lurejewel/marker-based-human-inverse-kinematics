% 流程：
% 1. 定义所有关节在模型站立状态下的相对位移、旋转轴（已完成）
% 2. 定义关节位置在考虑所有关节自由度情况下的计算方法
% 3. 根据验证数据，计算行走状态下的人体模型，并显示
% 4. 定义marker点相对临近关节的位置
% 5. scale
% 6. IK
close all

BodyMass = 72;
BodyHeight = 180;
scaleFactor.height = BodyHeight / 168.85;

%% Define Joints of the model

dofs = containers.Map();

% global DOF
dofs('pelvis_tx') = struct('axis', [1 0 0 1]', 'range', [-5 5]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('pelvis_ty') = struct('axis', [0 1 0 1]', 'range', [-1 2]', 'defaultValue', 0.95, 'defaultSpeed', 0);
dofs('pelvis_tz') = struct('axis', [0 0 1 1]', 'range', [-3 3]', 'defaultValue', 0, 'defaultSpeed', 0);
ground_pelvis = JointDef('translation_parent', [0 0 0]', ...
    'Ndof', 3, ...
    'axis', [dofs('pelvis_tx').axis, dofs('pelvis_ty').axis, dofs('pelvis_tz').axis], ...
    'range', [dofs('pelvis_tx').range, dofs('pelvis_ty').range, dofs('pelvis_tz').range], ...
    'defaultValue', [dofs('pelvis_tx').defaultValue, dofs('pelvis_ty').defaultValue, dofs('pelvis_tz').defaultValue], ...
    'defaultSpeed', [dofs('pelvis_tx').defaultSpeed, dofs('pelvis_ty').defaultSpeed, dofs('pelvis_tz').defaultSpeed]);

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
knee_r = JointDef('translation_parent', [0 -0.396 0]', ...
    'Ndof', 1, ...
    'axis', dofs('knee_flexion_r').axis, ...
    'range', dofs('knee_flexion_r').range, ...
    'defaultValue', dofs('knee_flexion_r').defaultValue, ...
    'defaultSpeed', dofs('knee_flexion_r').defaultSpeed);

% left knee
dofs('knee_flexion_l') = struct('axis', [0 0 1 0]', 'range', deg2rad(10)*[-12 1]', 'defaultValue', 0, 'defaultSpeed', 0);
knee_l = JointDef('translation_parent', [0 -0.396 0]', ...
    'Ndof', 1, ...
    'axis', dofs('knee_flexion_l').axis, ...
    'range', dofs('knee_flexion_l').range, ...
    'defaultValue', dofs('knee_flexion_l').defaultValue, ...
    'defaultSpeed', dofs('knee_flexion_l').defaultSpeed);

% right ankle
dofs('ankle_dorsiflexion_r') = struct('axis', [0 0 1 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('ankle_adduction_r') = struct('axis', [0 1 0 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
ankle_r = JointDef('translation_parent', [0 -0.43 0]', ...
    'Ndof', 2, ...
    'axis', [dofs('ankle_dorsiflexion_r').axis, dofs('ankle_adduction_r').axis], ...
    'range', [dofs('ankle_dorsiflexion_r').range, dofs('ankle_adduction_r').range], ...
    'defaultValue', [dofs('ankle_dorsiflexion_r').defaultValue, dofs('ankle_adduction_r').defaultValue], ...
    'defaultSpeed', [dofs('ankle_dorsiflexion_r').defaultSpeed, dofs('ankle_adduction_r').defaultSpeed]);

% left ankle
dofs('ankle_dorsiflexion_l') = struct('axis', [0 0 1 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('ankle_adduction_l') = struct('axis', [0 1 0 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
ankle_l = JointDef('translation_parent', [0 -0.43 0]', ...
    'Ndof', 2, ...
    'axis', [dofs('ankle_dorsiflexion_l').axis, dofs('ankle_adduction_l').axis], ...
    'range', [dofs('ankle_dorsiflexion_l').range, dofs('ankle_adduction_l').range], ...
    'defaultValue', [dofs('ankle_dorsiflexion_l').defaultValue, dofs('ankle_adduction_l').defaultValue], ...
    'defaultSpeed', [dofs('ankle_dorsiflexion_l').defaultSpeed, dofs('ankle_adduction_l').defaultSpeed]);

% back
dofs('lumbar_extension') = struct('axis', [0 0 1 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('lumbar_bending') = struct('axis', [1 0 0 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('lumbar_rotation') = struct('axis', [0 1 0 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
back = JointDef('translation_parent', [-0.1007 0.0815 0]', ...
    'Ndof', 3, ...
    'axis', [dofs('lumbar_extension').axis, dofs('lumbar_bending').axis, dofs('lumbar_rotation').axis], ...
    'range', [dofs('lumbar_extension').range, dofs('lumbar_bending').range, dofs('lumbar_rotation').range], ...
    'defaultValue', [dofs('lumbar_extension').defaultValue, dofs('lumbar_bending').defaultValue, dofs('lumbar_rotation').defaultValue], ...
    'defaultSpeed', [dofs('lumbar_extension').defaultSpeed, dofs('lumbar_bending').defaultSpeed, dofs('lumbar_rotation').defaultSpeed]);

% neck
dofs('neck_extension') = struct('axis', [0 0 1 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('neck_bending') = struct('axis', [1 0 0 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('neck_rotation') = struct('axis', [0 1 0 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
neck = JointDef('translation_parent', [0 0.4 0]', ...
    'Ndof', 3, ...
    'axis', [dofs('neck_extension').axis, dofs('neck_bending').axis, dofs('neck_rotation').axis], ...
    'range', [dofs('neck_extension').range, dofs('neck_bending').range, dofs('neck_rotation').range], ...
    'defaultValue', [dofs('neck_extension').defaultValue, dofs('neck_bending').defaultValue, dofs('neck_rotation').defaultValue], ...
    'defaultSpeed', [dofs('neck_extension').defaultSpeed, dofs('neck_bending').defaultSpeed, dofs('neck_rotation').defaultSpeed]);

% right shoulder
dofs('arm_flex_r') = struct('axis', [0 0 1 0]', 'range', deg2rad(180)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('arm_add_r') = struct('axis', [1 0 0 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('arm_rot_r') = struct('axis', [0 1 0 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
shoulder_r = JointDef('translation_parent', [0.003155 0.3715 0.17]', ...
    'Ndof', 3, ...
    'axis', [dofs('arm_flex_r').axis, dofs('arm_add_r').axis, dofs('arm_rot_r').axis], ...
    'range', [dofs('arm_flex_r').range, dofs('arm_add_r').range, dofs('arm_rot_r').range], ...
    'defaultValue', [dofs('arm_flex_r').defaultValue, dofs('arm_add_r').defaultValue, dofs('arm_rot_r').defaultValue], ...
    'defaultSpeed', [dofs('arm_flex_r').defaultSpeed, dofs('arm_add_r').defaultSpeed, dofs('arm_rot_r').defaultSpeed]);

% left shoulder
dofs('arm_flex_l') = struct('axis', [0 0 1 0]', 'range', deg2rad(180)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('arm_add_l') = struct('axis', [-1 0 0 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
dofs('arm_rot_l') = struct('axis', [0 -1 0 0]', 'range', deg2rad(90)*[-1 1]', 'defaultValue', 0, 'defaultSpeed', 0);
shoulder_l = JointDef('translation_parent', [0.003155 0.3715 -0.17]', ...
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
save joints.mat ground_pelvis hip_r hip_l knee_r knee_l ankle_r ankle_l back neck shoulder_r shoulder_l elbow_r elbow_l wrist_r wrist_l

dofMap = containers.Map({'pelvis_tx', 'pelvis_ty', 'pelvis_tz', ...
    'hip_flexion_r', 'hip_adduction_r', 'hip_rotation_r', ...
    'hip_flexion_l', 'hip_adduction_l', 'hip_rotation_l', ...
    'knee_flexion_r', 'knee_flexion_l', ...
    'ankle_dorsiflexion_r', 'ankle_adduction_r', ...
    'ankle_dorsiflexion_l', 'ankle_adduction_l', ...
    'lumbar_extension', 'lumbar_bending', 'lumbar_rotation', ...
    'neck_extension', 'neck_bending', 'neck_rotation', ...
    'arm_flex_r', 'arm_add_r', 'arm_rot_r', ...
    'arm_flex_l', 'arm_add_l', 'arm_rot_l', ...
    'elbow_flex_r', 'elbow_rot_r', ...
    'elbow_flex_l', 'elbow_rot_l', ...
    'wrist_flex_r', 'wrist_flex_l'}, num2cell(1:33));

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
torso = BodyDef('mass', 20.8266, 'COM', [-0.03 0.32 0]', 'inertia', [1.4745 0.7555 1.4314 0 0 0]');
head = BodyDef('mass', 6, 'COM', [0.0173 0.063 0]', 'inertia', [0.0173 0.015 0.022605 0 0 0]');
arm_r = BodyDef('mass', 2.0325, 'COM', [0 -0.164502 0]', 'inertia', [0.011946 0.004121 0.013409 0 0 0]');
arm_l = BodyDef('mass', 2.0325, 'COM', [0 -0.164502 0]', 'inertia', [0.011946 0.004121 0.013409 0 0 0]');
forearm_r = BodyDef('mass', 1.215, 'COM', [0 -0.120525 0]', 'inertia', [0.002962 0.000618 0.003213 0 0 0]');
forearm_l = BodyDef('mass', 1.215, 'COM', [0 -0.120525 0]', 'inertia', [0.002962 0.000618 0.003213 0 0 0]');
hand_r = BodyDef('mass', 0.4575, 'COM', [0 -0.068095 0]', 'inertia', [0.000892 0.000547 0.00134 0 0 0]');
hand_l = BodyDef('mass', 0.4575, 'COM', [0 -0.068095 0]', 'inertia', [0.000892 0.000547 0.00134 0 0 0]');

%% Scale mass of the bodies

TotalBodyMass = ground.mass + pelvis.mass + femur_r.mass + femur_l.mass + ...
    tibia_r.mass + tibia_l.mass + foot_r.mass + foot_l.mass + ...
    torso.mass + head.mass + arm_r.mass + arm_l.mass + forearm_r.mass + ...
    forearm_l.mass + hand_r.mass + hand_l.mass;

scaleFactor.mass = BodyMass / TotalBodyMass;

pelvis.mass = pelvis.mass * scaleFactor.mass;
femur_r.mass = femur_r.mass * scaleFactor.mass;
femur_l.mass = femur_l.mass * scaleFactor.mass;
tibia_r.mass = tibia_r.mass * scaleFactor.mass;
tibia_l.mass = tibia_l.mass * scaleFactor.mass;
foot_r.mass = foot_r.mass * scaleFactor.mass;
foot_l.mass = foot_l.mass * scaleFactor.mass;
torso.mass = torso.mass * scaleFactor.mass;
head.mass = head.mass * scaleFactor.mass;
arm_r.mass = arm_r.mass * scaleFactor.mass;
arm_l.mass = arm_l.mass * scaleFactor.mass;
forearm_r.mass = forearm_r.mass * scaleFactor.mass;
forearm_l.mass = forearm_l.mass * scaleFactor.mass;
hand_r.mass = hand_r.mass * scaleFactor.mass;
hand_l.mass = hand_l.mass * scaleFactor.mass;

clear BodyHeight BodyMass TotalBodyMass
save bodies.mat ground pelvis femur_r femur_l tibia_r tibia_l foot_r foot_l torso head arm_r arm_l forearm_r forearm_l hand_r hand_l

%% Define Markers
Sternum = MarkerDef('attachedBody', 'torso', 'deviation', [0.07 0.3 0]');
LAcromium = MarkerDef('attachedBody', 'torso', 'deviation', [-0.03 0.44 -0.15]');
TopHead = MarkerDef('attachedBody', 'torso', 'deviation', [0.00084 0.657 0]');
RASIS = MarkerDef('attachedBody', 'pelvis', 'deviation', [0.02 0.03 0.128]');
LASIS = MarkerDef('attachedBody', 'pelvis', 'deviation', [0.02 0.03 -0.128]');
VSacral = MarkerDef('attachedBody', 'pelvis', 'deviation', [-0.16 0.04 0]');
RThighUpper = MarkerDef('attachedBody', 'femur_r', 'deviation', [0.018 -0.2 0.064]');
RThighFront = MarkerDef('attachedBody', 'femur_r', 'deviation', [0.08 -0.25 0.0047]');
RThighRear = MarkerDef('attachedBody', 'femur_r', 'deviation', [0.01 -0.3 0.06]');
RKneeLat = MarkerDef('attachedBody', 'femur_r', 'deviation', [0 -0.404 0.05]');
RKneeMed = MarkerDef('attachedBody', 'femur_r', 'deviation', [0 -0.404 -0.05]');
RShankUpper = MarkerDef('attachedBody', 'tibia_r', 'deviation', [0.005 -0.065 0.05]');
RShankFront = MarkerDef('attachedBody', 'tibia_r', 'deviation', [0.005 -0.08 0]');
RShankRear = MarkerDef('attachedBody', 'tibia_r', 'deviation', [-0.02 -0.13 0.05]');
RAnkleLat = MarkerDef('attachedBody', 'tibia_r', 'deviation', [-0.005 -0.41 0.053]');
RAnkleMed = MarkerDef('attachedBody', 'tibia_r', 'deviation', [0.006 -0.3888 -0.038]');
RHeel = MarkerDef('attachedBody', 'foot_r', 'deviation', [-0.02 0.02 0]');
RMidfootSup = MarkerDef('attachedBody', 'foot_r', 'deviation', [0.13 0.03 -0.03]');
RMidfootLat = MarkerDef('attachedBody', 'foot_r', 'deviation', [0.1 0.02 0.04]');
RMidToeLat = MarkerDef('attachedBody', 'foot_r', 'deviation', [0.19 0 0.065]');
RMidToeMed = MarkerDef('attachedBody', 'foot_r', 'deviation', [0.19 0.005 -0.04]');
RMidToeTip = MarkerDef('attachedBody', 'foot_r', 'deviation', [0.26 0.005 0]');
LThighUpper = MarkerDef('attachedBody', 'femur_l', 'deviation', [0.018 -0.2 -0.064]');
LThighFront = MarkerDef('attachedBody', 'femur_l', 'deviation', [0.08 -0.25 -0.0047]');
LThighRear = MarkerDef('attachedBody', 'femur_l', 'deviation', [0.01 -0.3 -0.06]');
LKneeLat = MarkerDef('attachedBody', 'femur_l', 'deviation', [0 -0.404 -0.05]');
LKneeMed = MarkerDef('attachedBody', 'femur_l', 'deviation', [0 -0.404 0.05]');
LShankUpper = MarkerDef('attachedBody', 'tibia_l', 'deviation', [0.005 -0.065 -0.05]');
LShankFront = MarkerDef('attachedBody', 'tibia_l', 'deviation', [0.005 -0.08 0]');
LShankRear = MarkerDef('attachedBody', 'tibia_l', 'deviation', [-0.02 -0.13 -0.05]');
LAnkleLat = MarkerDef('attachedBody', 'tibia_l', 'deviation', [-0.005 -0.41 -0.053]');
LAnkleMed = MarkerDef('attachedBody', 'tibia_l', 'deviation', [0.006 -0.3888 0.038]');
LHeel = MarkerDef('attachedBody', 'foot_l', 'deviation', [-0.02 0.02 0]');
LMidfootSup = MarkerDef('attachedBody', 'foot_l', 'deviation', [0.13 0.03 0.03]');
LMidfootLat = MarkerDef('attachedBody', 'foot_l', 'deviation', [0.1 0.02 -0.04]');
LMidToeLat = MarkerDef('attachedBody', 'foot_l', 'deviation', [0.19 0 -0.065]');
LMidToeMed = MarkerDef('attachedBody', 'foot_l', 'deviation', [0.19 0.005 0.04]');
LMidToeTip = MarkerDef('attachedBody', 'foot_l', 'deviation', [0.26 0.005 0]');

save markers.mat Sternum LAcromium TopHead RASIS LASIS VSacral RThighUpper RThighFront RThighRear RKneeLat RKneeMed RShankUpper RShankFront RShankRear RAnkleLat RAnkleMed RHeel RMidfootSup RMidfootLat RMidToeLat RMidToeMed RMidToeTip LThighUpper LThighFront LThighRear LKneeLat LKneeMed LShankUpper LShankFront LShankRear LAnkleLat LAnkleMed LHeel LMidfootSup LMidfootLat LMidToeLat LMidToeMed LMidToeTip

%% Read static TRC input (markers)

trcFile = MarkerData('.\marker_input_static.trc');
% rate = trcFile.getCameraRate;
markerNum = trcFile.getNumMarkers;

markerPos_input = zeros(markerNum, 3);
for i = 0 : markerNum-1 % for every marker
    currentMarkerPos = zeros(trcFile.getNumFrames, 3); % temporary buffer for marker position in STATIC position
    for frame = 0 : trcFile.getNumFrames-1 % for every frame
        tempPos = trcFile.getFrame(frame).getMarker(i);
        currentMarkerPos(frame+1, :) = [tempPos.get(0), tempPos.get(1), tempPos.get(2)] / 1000;
    end
    markerPos_input(i+1, :) = mean(currentMarkerPos);
end

markerInputMap = containers.Map({'RASIS', 'LASIS', 'VSacral', 'RThighUpper', ...
    'RThighFront', 'RThighRear', 'LThighUpper', 'LThighFront', ...
    'LThighRear', 'RKneeLat', 'RKneeMed', 'LKneeLat', 'LKneeMed', ...
    'RShankUpper', 'RShankFront', 'RShankRear', 'LShankUpper', ...
    'LShankFront', 'LShankRear', 'RAnkleLat', 'RAnkleMed', ...
    'LAnkleLat', 'LAnkleMed', 'RHeel', 'RMidfootSup', 'RMidfootLat', ...
    'RToeTip', 'LHeel', 'LMidfootSup', 'LMidfootLat', 'LToeTip', ...
    'Sternum', 'RAcromium', 'LAcromium', 'RBicep', 'LBicep', 'RElbow', ...
    'LElbow', 'RWristMed', 'RWristLat', 'LWristMed', 'LWristLat', ...
    'RToeLat', 'RToeMed', 'LToeLat', 'LToeMed', 'RTemple', ...
    'LTemple', 'TopHead'}, num2cell(1:markerNum));
clear tempPos markerNum currentMarkerPos i frame
save markerInputMap.mat markerInputMap

%% Scale bone length from static pose
% NOTE THAT THIS PROCEDURE SHOULD BE EXECUTED ITERATIVELY UNTIL SCALE
% FACTORS ARE NEAR 1
q = zeros(height(dofMap),1); q(dofMap('pelvis_ty')) = 0.95;

% Topology view:
% 1. [ground] (ROOT) --- ground_pelvis --[pelvis]--> hip_r --[femur_r]--> knee_r --[tibia_r]--> ankle_r --[foot_r]
% 2. [pelvis]--> hip_l --[femur_l]--> knee_l --[tibia_l]--> ankle_l --[foot_l]
% 3. [pelvis]--> back --[torso]--> neck --[head]
% 4. [torso]--> shoulder_r --[arm_r]--> elbow_r --[forearm_r]--> wrist_r --[hand_r]
% 5. [torso]--> shoulder_l --[arm_l]--> elbow_l --[forearm_l]--> wrist_l --[hand_l]

% ----- route 1 ----- %
% [ground]--> ground_pelvis --[pelvis]
pelvis.T(1,4) = q(dofMap('pelvis_tx'));
pelvis.T(2,4) = q(dofMap('pelvis_ty'));
pelvis.T(3,4) = q(dofMap('pelvis_tz'));

% [pelvis]--> hip_r --[femur_r]
P = hip_r.translation_parent;
R = eye(3);
femur_r.T = [R, R*P+pelvis.T(1:3,4); 0 0 0 1];

% [femur_r]--> knee_r --[tibia_r]
P = knee_r.translation_parent;
R1 = Rot(hip_r.axis(1:3,1), q(dofMap('hip_flexion_r')));
R2 = Rot(hip_r.axis(1:3,2), q(dofMap('hip_adduction_r')));
R3 = Rot(hip_r.axis(1:3,3), q(dofMap('hip_rotation_r')));
R = R*R1*R2*R3;
tibia_r.T = [R, R*P+femur_r.T(1:3,4); 0 0 0 1];
femur_r.T(1:3,1:3) = R;

% [tibia_r]--> ankle_r --[foot_r]
P = ankle_r.translation_parent;
R = R*Rot(knee_r.axis(1:3,1), q(dofMap('knee_flexion_r')));
foot_r.T = [R, R*P+tibia_r.T(1:3,4); 0 0 0 1];
tibia_r.T(1:3,1:3) = R;

R1 = Rot(ankle_r.axis(1:3,1), q(dofMap('ankle_dorsiflexion_r')));
R2 = Rot(ankle_r.axis(1:3,2), q(dofMap('ankle_adduction_r')));
R = R*R1*R2;
foot_r.T(1:3,1:3) = R;

% ----- route 2 ----- %
% [pelvis]--> hip_l --[femur_l]
P = hip_l.translation_parent;
R = eye(3);
femur_l.T = [R, R*P+pelvis.T(1:3,4); 0 0 0 1];

% [femur_l]--> knee_l --[tibia_l]
P = knee_l.translation_parent;
R1 = Rot(hip_l.axis(1:3,1), q(dofMap('hip_flexion_l')));
R2 = Rot(hip_l.axis(1:3,2), q(dofMap('hip_adduction_l')));
R3 = Rot(hip_l.axis(1:3,3), q(dofMap('hip_rotation_l')));
R = R*R1*R2*R3;
tibia_l.T = [R, R*P+femur_l.T(1:3,4); 0 0 0 1];
femur_l.T(1:3,1:3) = R;

% [tibia_l]--> ankle_l --[foot_l]
P = ankle_l.translation_parent;
R = R*Rot(knee_l.axis(1:3,1), q(dofMap('knee_flexion_l')));
foot_l.T = [R, R*P+tibia_l.T(1:3,4); 0 0 0 1];
tibia_l.T(1:3,1:3) = R;

R1 = Rot(ankle_l.axis(1:3,1), q(dofMap('ankle_dorsiflexion_l')));
R2 = Rot(ankle_l.axis(1:3,2), q(dofMap('ankle_adduction_l')));
R = R*R1*R2;
foot_l.T(1:3,1:3) = R;

% ----- route 3 ----- %
% 3. [pelvis]--> back --[torso]--> neck --[head]
% [pelvis]--> back --[torso]
P = back.translation_parent;
R = eye(3);
torso.T = [R, R*P+pelvis.T(1:3,4); 0 0 0 1];

% [torso]--> neck --[head]
P = neck.translation_parent;
R1 = Rot(back.axis(1:3,1), q(dofMap('lumbar_extension')));
R2 = Rot(back.axis(1:3,2), q(dofMap('lumbar_bending')));
R3 = Rot(back.axis(1:3,3), q(dofMap('lumbar_rotation')));
R = R*R1*R2*R3;
head.T = [R, R*P+torso.T(1:3,4); 0 0 0 1];
torso.T(1:3,1:3) = R;

R1 = Rot(neck.axis(1:3,1), q(dofMap('neck_extension')));
R2 = Rot(neck.axis(1:3,2), q(dofMap('neck_bending')));
R3 = Rot(neck.axis(1:3,3), q(dofMap('neck_rotation')));
R = R*R1*R2*R3;
head.T(1:3,1:3) = R;

% ----- route 4 ----- %
% 4. [torso]--> shoulder_r --[arm_r]--> elbow_r --[forearm_r]--> wrist_r --[hand_r]
% [torso]--> shoulder_r --[arm_r]
P = shoulder_r.translation_parent;
R = torso.T(1:3,1:3);
arm_r.T = [R, R*P+torso.T(1:3,4); 0 0 0 1];

% [arm_r]--> elbow_r -- [forearm_r]
P = elbow_r.translation_parent;
R1 = Rot(shoulder_r.axis(1:3,1), q(dofMap('arm_flex_r')));
R2 = Rot(shoulder_r.axis(1:3,2), q(dofMap('arm_add_r')));
R3 = Rot(shoulder_r.axis(1:3,2), q(dofMap('arm_rot_r')));
R = R*R1*R2*R3;
forearm_r.T = [R, R*P+arm_r.T(1:3,4); 0 0 0 1];
arm_r.T(1:3,1:3) = R;

% [forearm_r]--> wrist_r -- [hand_r]
P = wrist_r.translation_parent;
R1 = Rot(elbow_r.axis(1:3,1), q(dofMap('elbow_flex_r')));
R2 = Rot(elbow_r.axis(1:3,2), q(dofMap('elbow_rot_r')));
R = R*R1*R2;
hand_r.T = [R, R*P+forearm_r.T(1:3,4); 0 0 0 1];
forearm_r.T(1:3,1:3) = R;

R = R*Rot(wrist_r.axis(1:3,1), q(dofMap('wrist_flex_r')));
hand_r.T(1:3,1:3) = R;

% ----- route 5 ----- %
% 5. [torso]--> shoulder_l --[arm_l]--> elbow_l --[forearm_l]--> wrist_l --[hand_l]
% [torso]--> shoulder_l --[arm_l]
P = shoulder_l.translation_parent;
R = torso.T(1:3,1:3);
arm_l.T = [R, R*P+torso.T(1:3,4); 0 0 0 1];

% [arm_l]--> elbow_l -- [forearm_l]
P = elbow_l.translation_parent;
R1 = Rot(shoulder_l.axis(1:3,1), q(dofMap('arm_flex_l')));
R2 = Rot(shoulder_l.axis(1:3,2), q(dofMap('arm_add_l')));
R3 = Rot(shoulder_l.axis(1:3,2), q(dofMap('arm_rot_l')));
R = R*R1*R2*R3;
forearm_l.T = [R, R*P+arm_l.T(1:3,4); 0 0 0 1];
arm_l.T(1:3,1:3) = R;

% [forearm_l]--> wrist_l -- [hand_l]
P = wrist_l.translation_parent;
R1 = Rot(elbow_l.axis(1:3,1), q(dofMap('elbow_flex_l')));
R2 = Rot(elbow_l.axis(1:3,2), q(dofMap('elbow_rot_l')));
R = R*R1*R2;
hand_l.T = [R, R*P+forearm_l.T(1:3,4); 0 0 0 1];
forearm.T(1:3,1:3) = R;

R = R*Rot(wrist_l.axis(1:3,1), q(dofMap('wrist_flex_l')));
hand_l.T(1:3,1:3) = R;


% --- Marker position calculation --- %
% 1. markers attached to torso
location = eval(Sternum.attachedBody).T * [Sternum.deviation; 1];
Sternum.location = location(1:3);
location = eval(LAcromium.attachedBody).T * [LAcromium.deviation; 1];
LAcromium.location = location(1:3);
location = eval(TopHead.attachedBody).T * [TopHead.deviation; 1];
TopHead.location = location(1:3);
location = eval(RASIS.attachedBody).T * [RASIS.deviation; 1];
RASIS.location = location(1:3);
location = eval(LASIS.attachedBody).T * [LASIS.deviation; 1];
LASIS.location = location(1:3);
location = eval(VSacral.attachedBody).T * [VSacral.deviation; 1];
VSacral.location = location(1:3);
location = eval(RThighUpper.attachedBody).T * [RThighUpper.deviation; 1];
RThighUpper.location = location(1:3);
location = eval(RThighFront.attachedBody).T * [RThighFront.deviation; 1];
RThighFront.location = location(1:3);
location = eval(RThighRear.attachedBody).T * [RThighRear.deviation; 1];
RThighRear.location = location(1:3);
location = eval(RKneeLat.attachedBody).T * [RKneeLat.deviation; 1];
RKneeLat.location = location(1:3);
location = eval(RKneeMed.attachedBody).T * [RKneeMed.deviation; 1];
RKneeMed.location = location(1:3);
location = eval(RShankUpper.attachedBody).T * [RShankUpper.deviation; 1];
RShankUpper.location = location(1:3);
location = eval(RShankFront.attachedBody).T * [RShankFront.deviation; 1];
RShankFront.location = location(1:3);
location = eval(RShankRear.attachedBody).T * [RShankRear.deviation; 1];
RShankRear.location = location(1:3);
location = eval(RAnkleLat.attachedBody).T * [RAnkleLat.deviation; 1];
RAnkleLat.location = location(1:3);
location = eval(RAnkleMed.attachedBody).T * [RAnkleMed.deviation; 1];
RAnkleMed.location = location(1:3);
location = eval(RHeel.attachedBody).T * [RHeel.deviation; 1];
RHeel.location = location(1:3);
location = eval(RMidfootSup.attachedBody).T * [RMidfootSup.deviation; 1];
RMidfootSup.location = location(1:3);
location = eval(RMidfootLat.attachedBody).T * [RMidfootLat.deviation; 1];
RMidfootLat.location = location(1:3);
location = eval(RMidToeLat.attachedBody).T * [RMidToeLat.deviation; 1];
RMidToeLat.location = location(1:3);
location = eval(RMidToeMed.attachedBody).T * [RMidToeMed.deviation; 1];
RMidToeMed.location = location(1:3);
location = eval(RMidToeTip.attachedBody).T * [RMidToeTip.deviation; 1];
RMidToeTip.location = location(1:3);

location = eval(LThighUpper.attachedBody).T * [LThighUpper.deviation; 1];
LThighUpper.location = location(1:3);
location = eval(LThighFront.attachedBody).T * [LThighFront.deviation; 1];
LThighFront.location = location(1:3);
location = eval(LThighRear.attachedBody).T * [LThighRear.deviation; 1];
LThighRear.location = location(1:3);
location = eval(LKneeLat.attachedBody).T * [LKneeLat.deviation; 1];
LKneeLat.location = location(1:3);
location = eval(LKneeMed.attachedBody).T * [LKneeMed.deviation; 1];
LKneeMed.location = location(1:3);
location = eval(LShankUpper.attachedBody).T * [LShankUpper.deviation; 1];
LShankUpper.location = location(1:3);
location = eval(LShankFront.attachedBody).T * [LShankFront.deviation; 1];
LShankFront.location = location(1:3);
location = eval(LShankRear.attachedBody).T * [LShankRear.deviation; 1];
LShankRear.location = location(1:3);
location = eval(LAnkleLat.attachedBody).T * [LAnkleLat.deviation; 1];
LAnkleLat.location = location(1:3);
location = eval(LAnkleMed.attachedBody).T * [LAnkleMed.deviation; 1];
LAnkleMed.location = location(1:3);
location = eval(LHeel.attachedBody).T * [LHeel.deviation; 1];
LHeel.location = location(1:3);
location = eval(LMidfootSup.attachedBody).T * [LMidfootSup.deviation; 1];
LMidfootSup.location = location(1:3);
location = eval(LMidfootLat.attachedBody).T * [LMidfootLat.deviation; 1];
LMidfootLat.location = location(1:3);
location = eval(LMidToeLat.attachedBody).T * [LMidToeLat.deviation; 1];
LMidToeLat.location = location(1:3);
location = eval(LMidToeMed.attachedBody).T * [LMidToeMed.deviation; 1];
LMidToeMed.location = location(1:3);
location = eval(LMidToeTip.attachedBody).T * [LMidToeTip.deviation; 1];
LMidToeTip.location = location(1:3);


scaleFactor.pelvis = norm(markerPos_input(markerInputMap('RASIS'),:) - markerPos_input(markerInputMap('LASIS'),:)) / norm(RASIS.location - LASIS.location);
scaleFactor.femur_r = norm(markerPos_input(markerInputMap('RASIS'),:) - markerPos_input(markerInputMap('RKneeLat'),:)) / norm(RASIS.location - RKneeLat.location);
scaleFactor.femur_l = norm(markerPos_input(markerInputMap('LASIS'),:) - markerPos_input(markerInputMap('LKneeLat'),:)) / norm(LASIS.location - LKneeLat.location);
scaleFactor.tibia_r = norm(markerPos_input(markerInputMap('RKneeLat'),:) - markerPos_input(markerInputMap('RAnkleLat'),:)) / norm(RKneeLat.location - RAnkleLat.location);
scaleFactor.tibia_l = norm(markerPos_input(markerInputMap('LKneeLat'),:) - markerPos_input(markerInputMap('LAnkleLat'),:)) / norm(LKneeLat.location - LAnkleLat.location);
scaleFactor.foot_r = norm(markerPos_input(markerInputMap('RHeel'),:) - markerPos_input(markerInputMap('RToeTip'),:)) / norm(RHeel.location - RMidToeTip.location);
scaleFactor.foot_l = norm(markerPos_input(markerInputMap('LHeel'),:) - markerPos_input(markerInputMap('LToeTip'),:)) / norm(LHeel.location - LMidToeTip.location);
scaleFactor.torso = norm(markerPos_input(markerInputMap('VSacral'),:) - markerPos_input(markerInputMap('Sternum'),:)) / norm(VSacral.location - Sternum.location);
scaleFactor.Head = norm(markerPos_input(markerInputMap('TopHead'),:) - markerPos_input(markerInputMap('Sternum'),:)) / norm(TopHead.location - Sternum.location);

save scaleFactor.mat scaleFactor

%% Create Canvas

screenSize = get(0, 'ScreenSize');
width = screenSize(3) * 0.8;
left = screenSize(3) * 0.1;
bottom = screenSize(4) * 0.1;
height = screenSize(4) * 0.8;
fig = figure('Position', [left bottom width height], 'Color', 'w');
set(fig, 'DoubleBuffer', 'on');
% nextFrameTime = 0;
% timescale = 0.01;
FramePerSec = 100;

mov = VideoWriter(['SkeletonPoseAnimation_' char(datetime('today'))]);
movFrameRate = FramePerSec;
mov.Quality = 100;
open(mov);

clear width left bottom height fig FramePerSec screenSize

%% Loop Start
for i = 1 : npts
    %% Calculate body positions according to IK input

    % q = data(i,:);
    q(4:end) = deg2rad(q(4:end)); % joint angles: deg -> rad
    q = 0*q; q(dofMap('pelvis_ty')) = 0.93;

    % Topology view:
    % 1. [ground] (ROOT) --- ground_pelvis --[pelvis]--> hip_r --[femur_r]--> knee_r --[tibia_r]--> ankle_r --[foot_r]
    % 2. [pelvis]--> hip_l --[femur_l]--> knee_l --[tibia_l]--> ankle_l --[foot_l]
    % 3. [pelvis]--> back --[torso]--> neck --[head]
    % 4. [torso]--> shoulder_r --[arm_r]--> elbow_r --[forearm_r]--> wrist_r --[hand_r]
    % 5. [torso]--> shoulder_l --[arm_l]--> elbow_l --[forearm_l]--> wrist_l --[hand_l]

    % ----- route 1 ----- %
    % [ground]--> ground_pelvis --[pelvis]
    pelvis.T(1,4) = q(dofMap('pelvis_tx'));
    pelvis.T(2,4) = q(dofMap('pelvis_ty'));
    pelvis.T(3,4) = q(dofMap('pelvis_tz'));

    % [pelvis]--> hip_r --[femur_r]
    P = hip_r.translation_parent * scaleFactor.pelvis;
    R = eye(3);
    femur_r.T = [R, R*P+pelvis.T(1:3,4); 0 0 0 1];

    % [femur_r]--> knee_r --[tibia_r]
    P = knee_r.translation_parent * scaleFactor.femur_r;
    R1 = Rot(hip_r.axis(1:3,1), q(dofMap('hip_flexion_r')));
    R2 = Rot(hip_r.axis(1:3,2), q(dofMap('hip_adduction_r')));
    R3 = Rot(hip_r.axis(1:3,3), q(dofMap('hip_rotation_r')));
    R = R*R1*R2*R3;
    tibia_r.T = [R, R*P+femur_r.T(1:3,4); 0 0 0 1];
    femur_r.T(1:3,1:3) = R;

    % [tibia_r]--> ankle_r --[foot_r]
    P = ankle_r.translation_parent * scaleFactor.tibia_r;
    R = R*Rot(knee_r.axis(1:3,1), q(dofMap('knee_flexion_r')));
    foot_r.T = [R, R*P+tibia_r.T(1:3,4); 0 0 0 1];
    tibia_r.T(1:3,1:3) = R;

    R1 = Rot(ankle_r.axis(1:3,1), q(dofMap('ankle_dorsiflexion_r')));
    R2 = Rot(ankle_r.axis(1:3,2), q(dofMap('ankle_adduction_r')));
    R = R*R1*R2;
    foot_r.T(1:3,1:3) = R;

    % ----- route 2 ----- %
    % [pelvis]--> hip_l --[femur_l]
    P = hip_l.translation_parent * scaleFactor.pelvis;
    R = eye(3);
    femur_l.T = [R, R*P+pelvis.T(1:3,4); 0 0 0 1];

    % [femur_l]--> knee_l --[tibia_l]
    P = knee_l.translation_parent * scaleFactor.femur_l;
    R1 = Rot(hip_l.axis(1:3,1), q(dofMap('hip_flexion_l')));
    R2 = Rot(hip_l.axis(1:3,2), q(dofMap('hip_adduction_l')));
    R3 = Rot(hip_l.axis(1:3,3), q(dofMap('hip_rotation_l')));
    R = R*R1*R2*R3;
    tibia_l.T = [R, R*P+femur_l.T(1:3,4); 0 0 0 1];
    femur_l.T(1:3,1:3) = R;

    % [tibia_l]--> ankle_l --[foot_l]
    P = ankle_l.translation_parent * scaleFactor.tibia_l;
    R = R*Rot(knee_l.axis(1:3,1), q(dofMap('knee_flexion_l')));
    foot_l.T = [R, R*P+tibia_l.T(1:3,4); 0 0 0 1];
    tibia_l.T(1:3,1:3) = R;

    R1 = Rot(ankle_l.axis(1:3,1), q(dofMap('ankle_dorsiflexion_l')));
    R2 = Rot(ankle_l.axis(1:3,2), q(dofMap('ankle_adduction_l')));
    R = R*R1*R2;
    foot_l.T(1:3,1:3) = R;

    % ----- route 3 ----- %
    % 3. [pelvis]--> back --[torso]--> neck --[head]
    % [pelvis]--> back --[torso]
    P = back.translation_parent * scaleFactor.torso;
    R = eye(3);
    torso.T = [R, R*P+pelvis.T(1:3,4); 0 0 0 1];

    % [torso]--> neck --[head]
    P = neck.translation_parent * scaleFactor.torso;
    R1 = Rot(back.axis(1:3,1), q(dofMap('lumbar_extension')));
    R2 = Rot(back.axis(1:3,2), q(dofMap('lumbar_bending')));
    R3 = Rot(back.axis(1:3,3), q(dofMap('lumbar_rotation')));
    R = R*R1*R2*R3;
    head.T = [R, R*P+torso.T(1:3,4); 0 0 0 1];
    torso.T(1:3,1:3) = R;

    R1 = Rot(neck.axis(1:3,1), q(dofMap('neck_extension')));
    R2 = Rot(neck.axis(1:3,2), q(dofMap('neck_bending')));
    R3 = Rot(neck.axis(1:3,3), q(dofMap('neck_rotation')));
    R = R*R1*R2*R3;
    head.T(1:3,1:3) = R;

    % ----- route 4 ----- %
    % 4. [torso]--> shoulder_r --[arm_r]--> elbow_r --[forearm_r]--> wrist_r --[hand_r]
    % [torso]--> shoulder_r --[arm_r]
    P = shoulder_r.translation_parent * scaleFactor.torso;
    R = torso.T(1:3,1:3);
    arm_r.T = [R, R*P+torso.T(1:3,4); 0 0 0 1];

    % [arm_r]--> elbow_r -- [forearm_r]
    P = elbow_r.translation_parent * scaleFactor.height;
    R1 = Rot(shoulder_r.axis(1:3,1), q(dofMap('arm_flex_r')));
    R2 = Rot(shoulder_r.axis(1:3,2), q(dofMap('arm_add_r')));
    R3 = Rot(shoulder_r.axis(1:3,2), q(dofMap('arm_rot_r')));
    R = R*R1*R2*R3;
    forearm_r.T = [R, R*P+arm_r.T(1:3,4); 0 0 0 1];
    arm_r.T(1:3,1:3) = R;

    % [forearm_r]--> wrist_r -- [hand_r]
    P = wrist_r.translation_parent * scaleFactor.height;
    R1 = Rot(elbow_r.axis(1:3,1), q(dofMap('elbow_flex_r')));
    R2 = Rot(elbow_r.axis(1:3,2), q(dofMap('elbow_rot_r')));
    R = R*R1*R2;
    hand_r.T = [R, R*P+forearm_r.T(1:3,4); 0 0 0 1];
    forearm_r.T(1:3,1:3) = R;

    R = R*Rot(wrist_r.axis(1:3,1), q(dofMap('wrist_flex_r')));
    hand_r.T(1:3,1:3) = R;

    % ----- route 5 ----- %
    % 5. [torso]--> shoulder_l --[arm_l]--> elbow_l --[forearm_l]--> wrist_l --[hand_l]
    % [torso]--> shoulder_l --[arm_l]
    P = shoulder_l.translation_parent * scaleFactor.torso;
    R = torso.T(1:3,1:3);
    arm_l.T = [R, R*P+torso.T(1:3,4); 0 0 0 1];

    % [arm_l]--> elbow_l -- [forearm_l]
    P = elbow_l.translation_parent * scaleFactor.height;
    R1 = Rot(shoulder_l.axis(1:3,1), q(dofMap('arm_flex_l')));
    R2 = Rot(shoulder_l.axis(1:3,2), q(dofMap('arm_add_l')));
    R3 = Rot(shoulder_l.axis(1:3,2), q(dofMap('arm_rot_l')));
    R = R*R1*R2*R3;
    forearm_l.T = [R, R*P+arm_l.T(1:3,4); 0 0 0 1];
    arm_l.T(1:3,1:3) = R;

    % [forearm_l]--> wrist_l -- [hand_l]
    P = wrist_l.translation_parent * scaleFactor.height;
    R1 = Rot(elbow_l.axis(1:3,1), q(dofMap('elbow_flex_l')));
    R2 = Rot(elbow_l.axis(1:3,2), q(dofMap('elbow_rot_l')));
    R = R*R1*R2;
    hand_l.T = [R, R*P+forearm_l.T(1:3,4); 0 0 0 1];
    forearm.T(1:3,1:3) = R;

    R = R*Rot(wrist_l.axis(1:3,1), q(dofMap('wrist_flex_l')));
    hand_l.T(1:3,1:3) = R;

    %% Calculate marker positions according to IK input

    % --- Marker position calculation --- %
    % 1. markers attached to torso
    location = eval(Sternum.attachedBody).T * [Sternum.deviation; 1];
    Sternum.location = location(1:3);
    location = eval(LAcromium.attachedBody).T * [LAcromium.deviation; 1];
    LAcromium.location = location(1:3);
    location = eval(TopHead.attachedBody).T * [TopHead.deviation; 1];
    TopHead.location = location(1:3);
    location = eval(RASIS.attachedBody).T * [RASIS.deviation; 1];
    RASIS.location = location(1:3);
    location = eval(LASIS.attachedBody).T * [LASIS.deviation; 1];
    LASIS.location = location(1:3);
    location = eval(VSacral.attachedBody).T * [VSacral.deviation; 1];
    VSacral.location = location(1:3);
    location = eval(RThighUpper.attachedBody).T * [RThighUpper.deviation; 1];
    RThighUpper.location = location(1:3);
    location = eval(RThighFront.attachedBody).T * [RThighFront.deviation; 1];
    RThighFront.location = location(1:3);
    location = eval(RThighRear.attachedBody).T * [RThighRear.deviation; 1];
    RThighRear.location = location(1:3);
    location = eval(RKneeLat.attachedBody).T * [RKneeLat.deviation; 1];
    RKneeLat.location = location(1:3);
    location = eval(RKneeMed.attachedBody).T * [RKneeMed.deviation; 1];
    RKneeMed.location = location(1:3);
    location = eval(RShankUpper.attachedBody).T * [RShankUpper.deviation; 1];
    RShankUpper.location = location(1:3);
    location = eval(RShankFront.attachedBody).T * [RShankFront.deviation; 1];
    RShankFront.location = location(1:3);
    location = eval(RShankRear.attachedBody).T * [RShankRear.deviation; 1];
    RShankRear.location = location(1:3);
    location = eval(RAnkleLat.attachedBody).T * [RAnkleLat.deviation; 1];
    RAnkleLat.location = location(1:3);
    location = eval(RAnkleMed.attachedBody).T * [RAnkleMed.deviation; 1];
    RAnkleMed.location = location(1:3);
    location = eval(RHeel.attachedBody).T * [RHeel.deviation; 1];
    RHeel.location = location(1:3);
    location = eval(RMidfootSup.attachedBody).T * [RMidfootSup.deviation; 1];
    RMidfootSup.location = location(1:3);
    location = eval(RMidfootLat.attachedBody).T * [RMidfootLat.deviation; 1];
    RMidfootLat.location = location(1:3);
    location = eval(RMidToeLat.attachedBody).T * [RMidToeLat.deviation; 1];
    RMidToeLat.location = location(1:3);
    location = eval(RMidToeMed.attachedBody).T * [RMidToeMed.deviation; 1];
    RMidToeMed.location = location(1:3);
    location = eval(RMidToeTip.attachedBody).T * [RMidToeTip.deviation; 1];
    RMidToeTip.location = location(1:3);

    location = eval(LThighUpper.attachedBody).T * [LThighUpper.deviation; 1];
    LThighUpper.location = location(1:3);
    location = eval(LThighFront.attachedBody).T * [LThighFront.deviation; 1];
    LThighFront.location = location(1:3);
    location = eval(LThighRear.attachedBody).T * [LThighRear.deviation; 1];
    LThighRear.location = location(1:3);
    location = eval(LKneeLat.attachedBody).T * [LKneeLat.deviation; 1];
    LKneeLat.location = location(1:3);
    location = eval(LKneeMed.attachedBody).T * [LKneeMed.deviation; 1];
    LKneeMed.location = location(1:3);
    location = eval(LShankUpper.attachedBody).T * [LShankUpper.deviation; 1];
    LShankUpper.location = location(1:3);
    location = eval(LShankFront.attachedBody).T * [LShankFront.deviation; 1];
    LShankFront.location = location(1:3);
    location = eval(LShankRear.attachedBody).T * [LShankRear.deviation; 1];
    LShankRear.location = location(1:3);
    location = eval(LAnkleLat.attachedBody).T * [LAnkleLat.deviation; 1];
    LAnkleLat.location = location(1:3);
    location = eval(LAnkleMed.attachedBody).T * [LAnkleMed.deviation; 1];
    LAnkleMed.location = location(1:3);
    location = eval(LHeel.attachedBody).T * [LHeel.deviation; 1];
    LHeel.location = location(1:3);
    location = eval(LMidfootSup.attachedBody).T * [LMidfootSup.deviation; 1];
    LMidfootSup.location = location(1:3);
    location = eval(LMidfootLat.attachedBody).T * [LMidfootLat.deviation; 1];
    LMidfootLat.location = location(1:3);
    location = eval(LMidToeLat.attachedBody).T * [LMidToeLat.deviation; 1];
    LMidToeLat.location = location(1:3);
    location = eval(LMidToeMed.attachedBody).T * [LMidToeMed.deviation; 1];
    LMidToeMed.location = location(1:3);
    location = eval(LMidToeTip.attachedBody).T * [LMidToeTip.deviation; 1];
    LMidToeTip.location = location(1:3);

    %% plot
    % skeleton points
    % disp(['pelvis---hip_r---knee_r---ankle_r: [' num2str(pelvis.T(1:3,4)'), ']---[' num2str(femur_r.T(1:3,4)') ']---[' num2str(tibia_r.T(1:3,4)') ']---[' num2str(foot_r.T(1:3,4)') ']']);
    % figure(1),
    hold off
    plot3([torso.T(3,4) femur_r.T(3,4) tibia_r.T(3,4) foot_r.T(3,4)], ...
        [torso.T(1,4) femur_r.T(1,4) tibia_r.T(1,4) foot_r.T(1,4)], ...
        [torso.T(2,4) femur_r.T(2,4) tibia_r.T(2,4) foot_r.T(2,4)], ...
        'LineWidth', 2, 'Color', 'k');
    % disp(['pelvis---hip_l---knee_l---ankle_l: [' num2str(pelvis.T(1:3,4)'), ']---[' num2str(femur_l.T(1:3,4)') ']---[' num2str(tibia_l.T(1:3,4)') ']---[' num2str(foot_l.T(1:3,4)') ']']);
    hold on
    scatter3([torso.T(3,4) femur_r.T(3,4) tibia_r.T(3,4) foot_r.T(3,4)], ...
        [torso.T(1,4) femur_r.T(1,4) tibia_r.T(1,4) foot_r.T(1,4)], ...
        [torso.T(2,4) femur_r.T(2,4) tibia_r.T(2,4) foot_r.T(2,4)], ...
        'LineWidth', 2, 'MarkerFaceColor',[0.8 0.2 0.2], 'MarkerEdgeColor',[0.8 0 0]);

    plot3([torso.T(3,4) femur_l.T(3,4) tibia_l.T(3,4) foot_l.T(3,4)], ...
        [torso.T(1,4) femur_l.T(1,4) tibia_l.T(1,4) foot_l.T(1,4)], ...
        [torso.T(2,4) femur_l.T(2,4) tibia_l.T(2,4) foot_l.T(2,4)], ...
        'LineWidth', 2, 'Color', 'k');
    scatter3([torso.T(3,4) femur_l.T(3,4) tibia_l.T(3,4) foot_l.T(3,4)], ...
        [torso.T(1,4) femur_l.T(1,4) tibia_l.T(1,4) foot_l.T(1,4)], ...
        [torso.T(2,4) femur_l.T(2,4) tibia_l.T(2,4) foot_l.T(2,4)], ...
        'LineWidth', 2, 'MarkerFaceColor',[0.8 0.2 0.2], 'MarkerEdgeColor',[0.8 0 0]);
    % disp(['pelvis---torso---head: [' num2str(pelvis.T(1:3,4)'), ']---[' num2str(torso.T(1:3,4)') ']---[' num2str(head.T(1:3,4)') ']']);
    plot3([torso.T(3,4) torso.T(3,4) head.T(3,4)], ...
        [torso.T(1,4) torso.T(1,4) head.T(1,4)], ...
        [torso.T(2,4) torso.T(2,4) head.T(2,4)], ...
        'LineWidth', 2, 'Color', 'k');
    scatter3([torso.T(3,4) torso.T(3,4) head.T(3,4)], ...
        [torso.T(1,4) torso.T(1,4) head.T(1,4)], ...
        [torso.T(2,4) torso.T(2,4) head.T(2,4)], ...
        'LineWidth', 2, 'MarkerFaceColor',[0.8 0.2 0.2], 'MarkerEdgeColor',[0.8 0 0]);
    % disp(['arm_r---forearm_r---hand_r: [' num2str(torso.T(1:3,4)'), ']---[' num2str(arm_r.T(1:3,4)') ']---[' num2str(forearm_r.T(1:3,4)') ']---[' num2str(hand_r.T(1:3,4)') ']']);
    plot3([arm_r.T(3,4) forearm_r.T(3,4) hand_r.T(3,4)], ...
        [arm_r.T(1,4) forearm_r.T(1,4) hand_r.T(1,4)], ...
        [arm_r.T(2,4) forearm_r.T(2,4) hand_r.T(2,4)], ...
        'LineWidth', 2, 'Color', 'k');
    scatter3([arm_r.T(3,4) forearm_r.T(3,4) hand_r.T(3,4)], ...
        [arm_r.T(1,4) forearm_r.T(1,4) hand_r.T(1,4)], ...
        [arm_r.T(2,4) forearm_r.T(2,4) hand_r.T(2,4)], ...
        'LineWidth', 2, 'MarkerFaceColor',[0.8 0.2 0.2], 'MarkerEdgeColor',[0.8 0 0]);
    % disp(['torso---arm_l---forearm_l---hand_l: [' num2str(torso.T(1:3,4)'), ']---[' num2str(arm_l.T(1:3,4)') ']---[' num2str(forearm_l.T(1:3,4)') ']---[' num2str(hand_l.T(1:3,4)') ']']);
    plot3([arm_r.T(3,4) arm_l.T(3,4) forearm_l.T(3,4) hand_l.T(3,4)], ...
        [arm_r.T(1,4) arm_l.T(1,4) forearm_l.T(1,4) hand_l.T(1,4)], ...
        [arm_r.T(2,4) arm_l.T(2,4) forearm_l.T(2,4) hand_l.T(2,4)], ...
        'LineWidth', 2, 'Color', 'k');
    scatter3([arm_r.T(3,4) arm_l.T(3,4) forearm_l.T(3,4) hand_l.T(3,4)], ...
        [arm_r.T(1,4) arm_l.T(1,4) forearm_l.T(1,4) hand_l.T(1,4)], ...
        [arm_r.T(2,4) arm_l.T(2,4) forearm_l.T(2,4) hand_l.T(2,4)], ...
        'LineWidth', 2, 'MarkerFaceColor',[0.8 0.2 0.2], 'MarkerEdgeColor',[0.8 0 0]);

    % virtual marker points
    scatter3(Sternum.location(3), Sternum.location(1), Sternum.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(LAcromium.location(3), LAcromium.location(1), LAcromium.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(TopHead.location(3), TopHead.location(1), TopHead.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(RASIS.location(3), RASIS.location(1), RASIS.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(LASIS.location(3), LASIS.location(1), LASIS.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(VSacral.location(3), VSacral.location(1), VSacral.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(RThighUpper.location(3), RThighUpper.location(1), RThighUpper.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(RThighFront.location(3), RThighFront.location(1), RThighFront.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(RThighRear.location(3), RThighRear.location(1), RThighRear.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(RKneeLat.location(3), RKneeLat.location(1), RKneeLat.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(RKneeMed.location(3), RKneeMed.location(1), RKneeMed.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(RShankUpper.location(3), RShankUpper.location(1), RShankUpper.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(RShankFront.location(3), RShankFront.location(1), RShankFront.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(RShankRear.location(3), RShankRear.location(1), RShankRear.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(RAnkleLat.location(3), RAnkleLat.location(1), RAnkleLat.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(RAnkleMed.location(3), RAnkleMed.location(1), RAnkleMed.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(RHeel.location(3), RHeel.location(1), RHeel.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(RMidfootSup.location(3), RMidfootSup.location(1), RMidfootSup.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(RMidfootLat.location(3), RMidfootLat.location(1), RMidfootLat.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(RMidToeLat.location(3), RMidToeLat.location(1), RMidToeLat.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(RMidToeMed.location(3), RMidToeMed.location(1), RMidToeMed.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(RMidToeTip.location(3), RMidToeTip.location(1), RMidToeTip.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(LThighUpper.location(3), LThighUpper.location(1), LThighUpper.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(LThighFront.location(3), LThighFront.location(1), LThighFront.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(LThighRear.location(3), LThighRear.location(1), LThighRear.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(LKneeLat.location(3), LKneeLat.location(1), LKneeLat.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(LKneeMed.location(3), LKneeMed.location(1), LKneeMed.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(LShankUpper.location(3), LShankUpper.location(1), LShankUpper.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(LShankFront.location(3), LShankFront.location(1), LShankFront.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(LShankRear.location(3), LShankRear.location(1), LShankRear.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(LAnkleLat.location(3), LAnkleLat.location(1), LAnkleLat.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(LAnkleMed.location(3), LAnkleMed.location(1), LAnkleMed.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(LHeel.location(3), LHeel.location(1), LHeel.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(LMidfootSup.location(3), LMidfootSup.location(1), LMidfootSup.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(LMidfootLat.location(3), LMidfootLat.location(1), LMidfootLat.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(LMidToeLat.location(3), LMidToeLat.location(1), LMidToeLat.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(LMidToeMed.location(3), LMidToeMed.location(1), LMidToeMed.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(LMidToeTip.location(3), LMidToeTip.location(1), LMidToeTip.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    % experimental marker points
    scatter3(markerPos_input(:,3), markerPos_input(:,1), markerPos_input(:,2), 10, 'filled', 'MarkerFaceColor', [0.2 0.9 0.6], 'MarkerEdgeColor', [0.2 0.9 0.6])

    axis equal, view(45,30)
    xlim([-1 1]), ylim([-1.5 1.5]), zlim([-0.5 2]);
    xlabel('z'), ylabel('x'), zlabel('y');
    drawnow;
    F = getframe(gcf);
    writeVideo(mov, F);

end
clear F location i R R1 R2 R3 P
% disp(['pelvis_tx: ', num2str(q(dofMap('pelvis_tx')))])
% disp(['pelvis_ty: ', num2str(q(dofMap('pelvis_ty')))])
% disp(['pelvis_tz: ', num2str(q(dofMap('pelvis_tz')))])
% disp(['hip_flexion_r: ', num2str(rad2deg(q(dofMap('hip_flexion_r'))))])
% disp(['hip_adduction_r: ', num2str(rad2deg(q(dofMap('hip_adduction_r'))))])
% disp(['hip_rotation_r: ', num2str(rad2deg(q(dofMap('hip_rotation_r'))))])
% disp(['hip_flexion_l: ', num2str(rad2deg(q(dofMap('hip_flexion_l'))))])
% disp(['hip_adduction_l: ', num2str(rad2deg(q(dofMap('hip_adduction_l'))))])
% disp(['hip_rotation_l: ', num2str(rad2deg(q(dofMap('hip_rotation_l'))))])
% disp(['knee_flexion_r: ', num2str(rad2deg(q(dofMap('knee_flexion_r'))))])
% disp(['knee_flexion_l: ', num2str(rad2deg(q(dofMap('knee_flexion_l'))))])
% disp(['ankle_dorsiflexion_r: ', num2str(rad2deg(q(dofMap('ankle_dorsiflexion_r'))))])
% disp(['ankle_adduction_r: ', num2str(rad2deg(q(dofMap('ankle_adduction_r'))))])
% disp(['ankle_dorsiflexion_l: ', num2str(rad2deg(q(dofMap('ankle_dorsiflexion_l'))))])
% disp(['ankle_adduction_l: ', num2str(rad2deg(q(dofMap('ankle_adduction_l'))))])
% disp(['lumbar_extension: ', num2str(rad2deg(q(dofMap('lumbar_extension'))))])
% disp(['lumbar_bending: ', num2str(rad2deg(q(dofMap('lumbar_bending'))))])
% disp(['lumbar_rotation: ', num2str(rad2deg(q(dofMap('lumbar_rotation'))))])
% disp(['neck_extension: ', num2str(rad2deg(q(dofMap('neck_extension'))))])
% disp(['neck_bending: ', num2str(rad2deg(q(dofMap('neck_bending'))))])
% disp(['neck_rotation: ', num2str(rad2deg(q(dofMap('neck_rotation'))))])
% disp(['arm_flex_r: ', num2str(rad2deg(q(dofMap('arm_flex_r'))))])
% disp(['arm_add_r: ', num2str(rad2deg(q(dofMap('arm_add_r'))))])
% disp(['arm_rot_r: ', num2str(rad2deg(q(dofMap('arm_rot_r'))))])
% disp(['arm_flex_l: ', num2str(rad2deg(q(dofMap('arm_flex_l'))))])
% disp(['arm_add_l: ', num2str(rad2deg(q(dofMap('arm_add_l'))))])
% disp(['arm_rot_l: ', num2str(rad2deg(q(dofMap('arm_rot_l'))))])
% disp(['elbow_flex_r: ', num2str(rad2deg(q(dofMap('elbow_flex_r'))))])
% disp(['elbow_rot_r: ', num2str(rad2deg(q(dofMap('elbow_rot_r'))))])
% disp(['elbow_flex_l: ', num2str(rad2deg(q(dofMap('elbow_flex_l'))))])
% disp(['elbow_rot_l: ', num2str(rad2deg(q(dofMap('elbow_rot_l'))))])
% disp(['wrist_flex_r: ', num2str(rad2deg(q(dofMap('wrist_flex_r'))))])
% disp(['wrist_flex_l: ', num2str(rad2deg(q(dofMap('wrist_flex_l'))))])
