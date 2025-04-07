function bodies = calc_bodyPoses(q, dofMap)
% Input:
% - q: generalized coordiantes. the order can be referenced in the
% definition of dofMap.
% Output: 
% - body classes, with transformation matrix T modified.

load bodies.mat, load joints.mat

% Topology view:
% 1. [ground] (ROOT) --- ground_pelvis --[pelvis]--> hip_r --[femur_r]--> knee_r --[tibia_r]--> ankle_r --[foot_r]
% 2. [pelvis]--> hip_l --[femur_l]--> knee_l --[tibia_l]--> ankle_l --[foot_l]
% 3. [pelvis]--> back --[lumbar]--> t10 --[thorax]--> neck --[head]
% 4. [thorax]--> shoulder_r --[arm_r]--> elbow_r --[forearm_r]--> wrist_r --[hand_r]
% 5. [thorax]--> shoulder_l --[arm_l]--> elbow_l --[forearm_l]--> wrist_l --[hand_l]

% Note:
% For every body frame, the orientation is determined by the rotation of
% joint located at the same position, while the position is determined by
% the rotation of the parent joint. 
% E.g. the orientation of tibia_r is determined by knee_r, while the 
% position of tibia_r is determined by hip_r.

% ----- route 1 ----- %
% [ground]--> ground_pelvis --[pelvis]
pelvis.T(1,4) = q(dofMap('pelvis_tx'));
pelvis.T(2,4) = q(dofMap('pelvis_ty'));
pelvis.T(3,4) = q(dofMap('pelvis_tz'));
R = eye(3);

% [pelvis]--> hip_r --[femur_r]
P = hip_r.translation_parent;
R1 = Rot(ground_pelvis.axis(1:3,4), q(dofMap('pelvis_tilt')));
R2 = Rot(ground_pelvis.axis(1:3,5), q(dofMap('pelvis_list')));
R3 = Rot(ground_pelvis.axis(1:3,6), q(dofMap('pelvis_rotation')));
R = R*R1*R2*R3;
femur_r.T = [R, R*P+pelvis.T(1:3,4); 0 0 0 1];
pelvis.T(1:3,1:3) = R;

% [femur_r]--> knee_r --[tibia_r]
P = knee_r.translation_parent;
P(1) = ppval(xSpline, q(dofMap('knee_flexion_r')));
P(2) = ppval(ySpline, q(dofMap('knee_flexion_r')));
R1 = Rot(hip_r.axis(1:3,1), q(dofMap('hip_flexion_r')));
R2 = Rot(hip_r.axis(1:3,2), q(dofMap('hip_adduction_r')));
R3 = Rot(hip_r.axis(1:3,3), q(dofMap('hip_rotation_r')));
R = R*R1*R2*R3;
tibia_r.T = [R, R*P+femur_r.T(1:3,4); 0 0 0 1];
femur_r.T(1:3,1:3) = R;

% [tibia_r]--> ankle_r --[foot_r]
P = ankle_r.translation_parent;
R1 = Rot(knee_r.axis(1:3,1), q(dofMap('knee_flexion_r')));
R2 = Rot(knee_r.axis(1:3,2), q(dofMap('knee_adduction_r')));
R3 = Rot(knee_r.axis(1:3,3), q(dofMap('knee_rotation_r')));
R = R*R1*R2*R3;
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
P(1) = ppval(xSpline, q(dofMap('knee_flexion_l')));
P(2) = ppval(ySpline, q(dofMap('knee_flexion_l')));
R1 = Rot(hip_l.axis(1:3,1), q(dofMap('hip_flexion_l')));
R2 = Rot(hip_l.axis(1:3,2), q(dofMap('hip_adduction_l')));
R3 = Rot(hip_l.axis(1:3,3), q(dofMap('hip_rotation_l')));
R = R*R1*R2*R3;
tibia_l.T = [R, R*P+femur_l.T(1:3,4); 0 0 0 1];
femur_l.T(1:3,1:3) = R;

% [tibia_l]--> ankle_l --[foot_l]
P = ankle_l.translation_parent;
R1 = Rot(knee_l.axis(1:3,1), q(dofMap('knee_flexion_l')));
R2 = Rot(knee_l.axis(1:3,2), q(dofMap('knee_adduction_l')));
R3 = Rot(knee_l.axis(1:3,3), q(dofMap('knee_rotation_l')));
R = R*R1*R2*R3;
foot_l.T = [R, R*P+tibia_l.T(1:3,4); 0 0 0 1];
tibia_l.T(1:3,1:3) = R;

R1 = Rot(ankle_l.axis(1:3,1), q(dofMap('ankle_dorsiflexion_l')));
R2 = Rot(ankle_l.axis(1:3,2), q(dofMap('ankle_adduction_l')));
R = R*R1*R2;
foot_l.T(1:3,1:3) = R;

% ----- route 3 ----- %
% 3. [pelvis]--> back --[lumbar]--> t10 --[thorax]--> neck --[head]
% [pelvis]--> back --[lumbar]
P = back.translation_parent;
R = eye(3);
lumbar.T = [R, R*P+pelvis.T(1:3,4); 0 0 0 1];

% [lumbar]--> t10 --[thorax]
P = neck.translation_parent;
R1 = Rot(back.axis(1:3,1), q(dofMap('lumbar_extension')));
R2 = Rot(back.axis(1:3,2), q(dofMap('lumbar_bending')));
R3 = Rot(back.axis(1:3,3), q(dofMap('lumbar_rotation')));
R = R*R1*R2*R3;
thorax.T = [R, R*P+lumbar.T(1:3,4); 0 0 0 1];
lumbar.T(1:3,1:3) = R;

% [thorax]--> neck --[head]
P = neck.translation_parent;
R1 = Rot(back.axis(1:3,1), q(dofMap('t10_extension')));
R2 = Rot(back.axis(1:3,2), q(dofMap('t10_bending')));
R3 = Rot(back.axis(1:3,3), q(dofMap('t10_rotation')));
R = R*R1*R2*R3;
head.T = [R, R*P+thorax.T(1:3,4); 0 0 0 1];
thorax.T(1:3,1:3) = R;

R1 = Rot(neck.axis(1:3,1), q(dofMap('neck_extension')));
R2 = Rot(neck.axis(1:3,2), q(dofMap('neck_bending')));
R3 = Rot(neck.axis(1:3,3), q(dofMap('neck_rotation')));
R = R*R1*R2*R3;
head.T(1:3,1:3) = R;

% ----- route 4 ----- %
% 4. [thorax]--> shoulder_r --[arm_r]--> elbow_r --[forearm_r]--> wrist_r --[hand_r]
% [thorax]--> shoulder_r --[arm_r]
P = shoulder_r.translation_parent;
R = thorax.T(1:3,1:3);
arm_r.T = [R, R*P+thorax.T(1:3,4); 0 0 0 1];

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
% 5. [thorax]--> shoulder_l --[arm_l]--> elbow_l --[forearm_l]--> wrist_l --[hand_l]
% [thorax]--> shoulder_l --[arm_l]
P = shoulder_l.translation_parent;
R = thorax.T(1:3,1:3);
arm_l.T = [R, R*P+thorax.T(1:3,4); 0 0 0 1];

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
forearm_l.T(1:3,1:3) = R;

R = R*Rot(wrist_l.axis(1:3,1), q(dofMap('wrist_flex_l')));
hand_l.T(1:3,1:3) = R;

bodies.pelvis = pelvis;
bodies.femur_r = femur_r;
bodies.femur_l = femur_l;
bodies.tibia_r = tibia_r;
bodies.tibia_l = tibia_l;
bodies.foot_r = foot_r;
bodies.foot_l = foot_l;
bodies.lumbar = lumbar;
bodies.thorax = thorax;
bodies.head = head;
bodies.arm_r = arm_r;
bodies.arm_l = arm_l;
bodies.forearm_r = forearm_r;
bodies.forearm_l = forearm_l;
bodies.hand_r = hand_r;
bodies.hand_l = hand_l;

end