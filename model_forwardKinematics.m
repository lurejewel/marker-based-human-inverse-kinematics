close all
import org.opensim.modeling.*

%% Read model information

% from model_definition.m:
load markers.mat, load bodies.mat, load joints.mat
load dofMap.mat, load markerInputMap.mat
% from model_scale.m:
load scaleFactor.mat

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
    q = 0*q; q(dofMap('pelvis_ty')) = 0.95;

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
