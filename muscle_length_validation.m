% 跨mtp关节的肌肉通过点 √
% 膝关节后滚模拟 √
% 条件通过点 √
% 移动通过点
% 踝关节内外翻 √
% 上肢肌肉

clear all, close all, close all, clc
import org.opensim.modeling.*

%% Read model information

% load bodies.mat, load joints.mat
load dofMap.mat, load muscles.mat, load muscleMap.mat

%% Validate length of muscles

% 1. right hip flexion/extension
% [refdata, labels] = read_opensim_ref_data('./OpenSim muslen-dof refs/muscles - hip_flex_r.sto');
% range = linspace(deg2rad(-120),deg2rad(120),100); % define joint range
% musLenMat = zeros(length(range), length(muscles)); % init muscle length matrix
% q = zeros(1,33); q(dofMap('pelvis_ty')) = 0.95; % init values of generalized coordinates
% for i = 1 : length(range)
%     q(dofMap('hip_flexion_r')) = range(i); % assign joint movement
%     bodies = calc_bodyPoses(q, dofMap); % compute poses of bodies
%     muscles = calc_muscleLengths(bodies, muscles, q, dofMap); % compute positions of muscle path points (in world coordinate), and thus, muscle lengths
%     for j = 1 : length(muscles)
%         musLenMat(i,j) = muscles(j).muscleLength;
%     end
% end
% 
% for i = 2 : length(labels)
%     muscleName = labels{i};
%     figure, plot(rad2deg(range'), musLenMat(:,muscleMap(muscleName)), 'LineWidth', 1.5) % plot calculated data
%     hold on, plot(refdata(:,1), refdata(:,i)); % plot reference data
%     title(['hip flex range - ' muscleName], 'Interpreter','none');
%     xlabel('angle'), ylabel('muscle length'), legend('model', 'ref');
% end

% 2. right hip adduction/abduction
% [refdata, labels] = read_opensim_ref_data('./OpenSim muslen-dof refs/muscles - hip_add_r.sto');
% range = linspace(deg2rad(-120),deg2rad(120),100); % define joint range
% musLenMat = zeros(length(range), length(muscles)); % init muscle length matrix
% q = zeros(1,33); q(dofMap('pelvis_ty')) = 0.95; % init values of generalized coordinates
% for i = 1 : length(range)
%     q(dofMap('hip_adduction_r')) = range(i); % assign joint movement
%     bodies = calc_bodyPoses(q, dofMap); % compute poses of bodies
%     muscles = calc_muscleLengths(bodies, muscles, q, dofMap); % compute positions of muscle path points (in world coordinate), and thus, muscle lengths
%     for j = 1 : length(muscles)
%         musLenMat(i,j) = muscles(j).muscleLength;
%     end
% end
% 
% for i = 2 : length(labels)
%     muscleName = labels{i};
%     figure, plot(rad2deg(range'), musLenMat(:,muscleMap(muscleName)), 'LineWidth', 1.5) % plot calculated data
%     hold on, plot(refdata(:,1), refdata(:,i)); % plot reference data
%     title(['hip add range - ' muscleName], 'Interpreter','none');
%     xlabel('angle'), ylabel('muscle length'), legend('model', 'ref');
% end

% 3. right hip rotation
% [refdata, labels] = read_opensim_ref_data('./OpenSim muslen-dof refs/muscles - hip_rot_r.sto');
% range = linspace(deg2rad(-120),deg2rad(120),100); % define joint range
% musLenMat = zeros(length(range), length(muscles)); % init muscle length matrix
% q = zeros(1,33); q(dofMap('pelvis_ty')) = 0.95; % init values of generalized coordinates
% for i = 1 : length(range)
%     q(dofMap('hip_rotation_r')) = range(i); % assign joint movement
%     bodies = calc_bodyPoses(q, dofMap); % compute poses of bodies
%     muscles = calc_muscleLengths(bodies, muscles, q, dofMap); % compute positions of muscle path points (in world coordinate), and thus, muscle lengths
%     for j = 1 : length(muscles)
%         musLenMat(i,j) = muscles(j).muscleLength;
%     end
% end
% 
% for i = 2 : length(labels)
%     muscleName = labels{i};
%     figure, plot(rad2deg(range'), musLenMat(:,muscleMap(muscleName)), 'LineWidth', 1.5) % plot calculated data
%     hold on, plot(refdata(:,1), refdata(:,i)); % plot reference data
%     title(['hip rot range - ' muscleName], 'Interpreter','none');
%     xlabel('angle'), ylabel('muscle length'), legend('model', 'ref');
% end

% 4. right knee flexion/extension
% [refdata, labels] = read_opensim_ref_data('./OpenSim muslen-dof refs/muscles - knee_flex_r.sto');
% range = linspace(deg2rad(-120),deg2rad(10),100); % define joint range
% musLenMat = zeros(length(range), length(muscles)); % init muscle length matrix
% q = zeros(1,33); q(dofMap('pelvis_ty')) = 0.95; % init values of generalized coordinates
% for i = 1 : length(range)
%     q(dofMap('knee_flexion_r')) = range(i); % assign joint movement
%     bodies = calc_bodyPoses(q, dofMap); % compute poses of bodies
%     muscles = calc_muscleLengths(bodies, muscles, q, dofMap); % compute positions of muscle path points (in world coordinate), and thus, muscle lengths
%     for j = 1 : length(muscles)
%         musLenMat(i,j) = muscles(j).muscleLength;
%     end
% end
% 
% for i = 2 : length(labels)
%     muscleName = labels{i};
%     figure, plot(rad2deg(range'), musLenMat(:,muscleMap(muscleName)), 'LineWidth', 1.5) % plot calculated data
%     hold on, plot(refdata(:,1), refdata(:,i)); % plot reference data
%     title(['knee flex range - ' muscleName], 'Interpreter','none');
%     xlabel('angle'), ylabel('muscle length'), legend('model', 'ref');
% end

% 5. right ankle plantarflexion/dorsiflexion
% [refdata, labels] = read_opensim_ref_data('./OpenSim muslen-dof refs/muscles - ankle_flex_r.sto');
% range = linspace(deg2rad(-90),deg2rad(90),100); % define joint range
% musLenMat = zeros(length(range), length(muscles)); % init muscle length matrix
% q = zeros(1,33); q(dofMap('pelvis_ty')) = 0.95; % init values of generalized coordinates
% for i = 1 : length(range)
%     q(dofMap('ankle_dorsiflexion_r')) = range(i); % assign joint movement
%     bodies = calc_bodyPoses(q, dofMap); % compute poses of bodies
%     muscles = calc_muscleLengths(bodies, muscles, q, dofMap); % compute positions of muscle path points (in world coordinate), and thus, muscle lengths
%     for j = 1 : length(muscles)
%         musLenMat(i,j) = muscles(j).muscleLength;
%     end
% end
% 
% for i = 2 : length(labels)
%     muscleName = labels{i};
%     figure, plot(rad2deg(range'), musLenMat(:,muscleMap(muscleName)), 'LineWidth', 1.5) % plot calculated data
%     hold on, plot(refdata(:,1), refdata(:,i)); % plot reference data
%     title(['ankle flex range - ' muscleName], 'Interpreter','none');
%     xlabel('angle'), ylabel('muscle length'), legend('model', 'ref');
% end

% 6. right ankle adduction/abduction
% [refdata, labels] = read_opensim_ref_data('./OpenSim muslen-dof refs/muscles - ankle_add_r.sto');
% range = linspace(deg2rad(-90),deg2rad(90),100); % define joint range
% musLenMat = zeros(length(range), length(muscles)); % init muscle length matrix
% q = zeros(1,33); q(dofMap('pelvis_ty')) = 0.95; % init values of generalized coordinates
% for i = 1 : length(range)
%     q(dofMap('ankle_adduction_r')) = range(i); % assign joint movement
%     bodies = calc_bodyPoses(q, dofMap); % compute poses of bodies
%     muscles = calc_muscleLengths(bodies, muscles, q, dofMap); % compute positions of muscle path points (in world coordinate), and thus, muscle lengths
%     for j = 1 : length(muscles)
%         musLenMat(i,j) = muscles(j).muscleLength;
%     end
% end
% 
% for i = 2 : length(labels)
%     muscleName = labels{i};
%     figure, plot(rad2deg(range'), musLenMat(:,muscleMap(muscleName)), 'LineWidth', 1.5) % plot calculated data
%     hold on, plot(refdata(:,1), refdata(:,i)); % plot reference data
%     title(['ankle add range - ' muscleName], 'Interpreter','none');
%     xlabel('angle'), ylabel('muscle length'), legend('model', 'ref');
% end

% 7. lumbar flexion/extension
% [refdata, labels] = read_opensim_ref_data('./OpenSim muslen-dof refs/muscles - lumbar_ext_r.sto');
% range = linspace(deg2rad(-60),deg2rad(60),100); % define joint range
% musLenMat = zeros(length(range), length(muscles)); % init muscle length matrix
% q = zeros(1,33); q(dofMap('pelvis_ty')) = 0.95; % init values of generalized coordinates
% for i = 1 : length(range)
%     q(dofMap('lumbar_extension')) = range(i); % assign joint movement
%     bodies = calc_bodyPoses(q, dofMap); % compute poses of bodies
%     muscles = calc_muscleLengths(bodies, muscles, q, dofMap); % compute positions of muscle path points (in world coordinate), and thus, muscle lengths
%     for j = 1 : length(muscles)
%         musLenMat(i,j) = muscles(j).muscleLength;
%     end
% end
% 
% for i = 2 : length(labels)
%     muscleName = labels{i};
%     figure, plot(rad2deg(range'), musLenMat(:,muscleMap(muscleName)), 'LineWidth', 1.5) % plot calculated data
%     hold on, plot(refdata(:,1), refdata(:,i)); % plot reference data
%     title(['lumbar ext range - ' muscleName], 'Interpreter','none');
%     xlabel('angle'), ylabel('muscle length'), legend('model', 'ref');
% end

% 8. lumbar bending
% [refdata, labels] = read_opensim_ref_data('./OpenSim muslen-dof refs/muscles - lumbar_bending_r.sto');
% range = linspace(deg2rad(-60),deg2rad(60),100); % define joint range
% musLenMat = zeros(length(range), length(muscles)); % init muscle length matrix
% q = zeros(1,33); q(dofMap('pelvis_ty')) = 0.95; % init values of generalized coordinates
% for i = 1 : length(range)
%     q(dofMap('lumbar_bending')) = range(i); % assign joint movement
%     bodies = calc_bodyPoses(q, dofMap); % compute poses of bodies
%     muscles = calc_muscleLengths(bodies, muscles, q, dofMap); % compute positions of muscle path points (in world coordinate), and thus, muscle lengths
%     for j = 1 : length(muscles)
%         musLenMat(i,j) = muscles(j).muscleLength;
%     end
% end
% 
% for i = 2 : length(labels)
%     muscleName = labels{i};
%     figure, plot(rad2deg(range'), musLenMat(:,muscleMap(muscleName)), 'LineWidth', 1.5) % plot calculated data
%     hold on, plot(refdata(:,1), refdata(:,i)); % plot reference data
%     title(['lumbar bend range - ' muscleName], 'Interpreter','none');
%     xlabel('angle'), ylabel('muscle length'), legend('model', 'ref');
% end

% 9. lumbar rotation
% [refdata, labels] = read_opensim_ref_data('./OpenSim muslen-dof refs/muscles - lumbar_rot_r.sto');
% range = linspace(deg2rad(-60),deg2rad(60),100); % define joint range
% musLenMat = zeros(length(range), length(muscles)); % init muscle length matrix
% q = zeros(1,33); q(dofMap('pelvis_ty')) = 0.95; % init values of generalized coordinates
% for i = 1 : length(range)
%     q(dofMap('lumbar_rotation')) = range(i); % assign joint movement
%     bodies = calc_bodyPoses(q, dofMap); % compute poses of bodies
%     muscles = calc_muscleLengths(bodies, muscles, q, dofMap); % compute positions of muscle path points (in world coordinate), and thus, muscle lengths
%     for j = 1 : length(muscles)
%         musLenMat(i,j) = muscles(j).muscleLength;
%     end
% end
% 
% for i = 2 : length(labels)
%     muscleName = labels{i};
%     figure, plot(rad2deg(range'), musLenMat(:,muscleMap(muscleName)), 'LineWidth', 1.5) % plot calculated data
%     hold on, plot(refdata(:,1), refdata(:,i)); % plot reference data
%     title(['lumbar rot range - ' muscleName], 'Interpreter','none');
%     xlabel('angle'), ylabel('muscle length'), legend('model', 'ref');
% end

% 10. walking validation
[dofInput, dofLabels] = read_opensim_ref_data('./OpenSim muslen-dof refs/walking_dof.mot');
dofInput = deg2rad(dofInput); % can be checked by 'Storage.isInDegrees'
[muscleLenRef, musLabels] = read_opensim_ref_data('./OpenSim muslen-dof refs/walking_muscleLen.sto');
muscleLenCal = zeros(size(muscleLenRef)); % init muscle length matrix
for t = 1 : height(dofInput) % for every time step

    % assign dofs
    q = zeros(1, length(dofMap));
    for i = 1 : length(dofLabels)
        q(dofMap(dofLabels{i})) = dofInput(t,i);
    end

    % compute body poses
    bodies = calc_bodyPoses(q, dofMap);

    % compute positions of muscle path points in world coordinate, and
    % thus, muscle lengths
    muscles = calc_muscleLengths(bodies, muscles, q, dofMap);
    for j = 1 : length(muscles)
        muscleLenCal(t,j) = muscles(j).muscleLength;
    end
    
end

for i = 1 : length(musLabels)
    muscleName = musLabels{i};
    figure, plot(muscleLenCal(:,muscleMap(muscleName)), 'LineWidth', 1.5); % plot calculated data
    hold on, plot(muscleLenRef(:,i)); % plot reference data
    title(['length change of ' muscleName ' during walking'], 'Interpreter','none');
    xlabel('time'), ylabel('muscle length'), legend('model', 'ref');
    saveas(gcf, ['length change of ' muscleName ' during walking'], 'meta'); % 'meta': .emf format
    close(gcf);
end

% %% plot
% % skeleton points
% % disp(['pelvis---hip_r---knee_r---ankle_r: [' num2str(pelvis.T(1:3,4)'), ']---[' num2str(femur_r.T(1:3,4)') ']---[' num2str(tibia_r.T(1:3,4)') ']---[' num2str(foot_r.T(1:3,4)') ']']);
% % figure(1),
% hold off
% plot3([torso.T(3,4) femur_r.T(3,4) tibia_r.T(3,4) foot_r.T(3,4)], ...
%     [torso.T(1,4) femur_r.T(1,4) tibia_r.T(1,4) foot_r.T(1,4)], ...
%     [torso.T(2,4) femur_r.T(2,4) tibia_r.T(2,4) foot_r.T(2,4)], ...
%     'LineWidth', 2, 'Color', 'k');
% % disp(['pelvis---hip_l---knee_l---ankle_l: [' num2str(pelvis.T(1:3,4)'), ']---[' num2str(femur_l.T(1:3,4)') ']---[' num2str(tibia_l.T(1:3,4)') ']---[' num2str(foot_l.T(1:3,4)') ']']);
% hold on
% scatter3([torso.T(3,4) femur_r.T(3,4) tibia_r.T(3,4) foot_r.T(3,4)], ...
%     [torso.T(1,4) femur_r.T(1,4) tibia_r.T(1,4) foot_r.T(1,4)], ...
%     [torso.T(2,4) femur_r.T(2,4) tibia_r.T(2,4) foot_r.T(2,4)], ...
%     'LineWidth', 2, 'MarkerFaceColor',[0.8 0.2 0.2], 'MarkerEdgeColor',[0.8 0 0]);
% plot3([torso.T(3,4) femur_l.T(3,4) tibia_l.T(3,4) foot_l.T(3,4)], ...
%     [torso.T(1,4) femur_l.T(1,4) tibia_l.T(1,4) foot_l.T(1,4)], ...
%     [torso.T(2,4) femur_l.T(2,4) tibia_l.T(2,4) foot_l.T(2,4)], ...
%     'LineWidth', 2, 'Color', 'k');
% scatter3([torso.T(3,4) femur_l.T(3,4) tibia_l.T(3,4) foot_l.T(3,4)], ...
%     [torso.T(1,4) femur_l.T(1,4) tibia_l.T(1,4) foot_l.T(1,4)], ...
%     [torso.T(2,4) femur_l.T(2,4) tibia_l.T(2,4) foot_l.T(2,4)], ...
%     'LineWidth', 2, 'MarkerFaceColor',[0.8 0.2 0.2], 'MarkerEdgeColor',[0.8 0 0]);
% % % disp(['pelvis---torso---head: [' num2str(pelvis.T(1:3,4)'), ']---[' num2str(torso.T(1:3,4)') ']---[' num2str(head.T(1:3,4)') ']']);
% % plot3([torso.T(3,4) torso.T(3,4) head.T(3,4)], ...
% %     [torso.T(1,4) torso.T(1,4) head.T(1,4)], ...
% %     [torso.T(2,4) torso.T(2,4) head.T(2,4)], ...
% %     'LineWidth', 2, 'Color', 'k');
% % scatter3([torso.T(3,4) torso.T(3,4) head.T(3,4)], ...
% %     [torso.T(1,4) torso.T(1,4) head.T(1,4)], ...
% %     [torso.T(2,4) torso.T(2,4) head.T(2,4)], ...
% %     'LineWidth', 2, 'MarkerFaceColor',[0.8 0.2 0.2], 'MarkerEdgeColor',[0.8 0 0]);
% % % disp(['arm_r---forearm_r---hand_r: [' num2str(torso.T(1:3,4)'), ']---[' num2str(arm_r.T(1:3,4)') ']---[' num2str(forearm_r.T(1:3,4)') ']---[' num2str(hand_r.T(1:3,4)') ']']);
% % plot3([arm_r.T(3,4) forearm_r.T(3,4) hand_r.T(3,4)], ...
% %     [arm_r.T(1,4) forearm_r.T(1,4) hand_r.T(1,4)], ...
% %     [arm_r.T(2,4) forearm_r.T(2,4) hand_r.T(2,4)], ...
% %     'LineWidth', 2, 'Color', 'k');
% % scatter3([arm_r.T(3,4) forearm_r.T(3,4) hand_r.T(3,4)], ...
% %     [arm_r.T(1,4) forearm_r.T(1,4) hand_r.T(1,4)], ...
% %     [arm_r.T(2,4) forearm_r.T(2,4) hand_r.T(2,4)], ...
% %     'LineWidth', 2, 'MarkerFaceColor',[0.8 0.2 0.2], 'MarkerEdgeColor',[0.8 0 0]);
% % % disp(['torso---arm_l---forearm_l---hand_l: [' num2str(torso.T(1:3,4)'), ']---[' num2str(arm_l.T(1:3,4)') ']---[' num2str(forearm_l.T(1:3,4)') ']---[' num2str(hand_l.T(1:3,4)') ']']);
% % plot3([arm_r.T(3,4) arm_l.T(3,4) forearm_l.T(3,4) hand_l.T(3,4)], ...
% %     [arm_r.T(1,4) arm_l.T(1,4) forearm_l.T(1,4) hand_l.T(1,4)], ...
% %     [arm_r.T(2,4) arm_l.T(2,4) forearm_l.T(2,4) hand_l.T(2,4)], ...
% %     'LineWidth', 2, 'Color', 'k');
% % scatter3([arm_r.T(3,4) arm_l.T(3,4) forearm_l.T(3,4) hand_l.T(3,4)], ...
% %     [arm_r.T(1,4) arm_l.T(1,4) forearm_l.T(1,4) hand_l.T(1,4)], ...
% %     [arm_r.T(2,4) arm_l.T(2,4) forearm_l.T(2,4) hand_l.T(2,4)], ...
% %     'LineWidth', 2, 'MarkerFaceColor',[0.8 0.2 0.2], 'MarkerEdgeColor',[0.8 0 0]);
% 
% % virtual marker points
% scatter3(Sternum.location(3), Sternum.location(1), Sternum.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
% 
% axis equal, view(45,-30)
% xlim([-1 1]), ylim([-0.5 1.5]), zlim([-0.5 2]);
% xlabel('z'), ylabel('x'), zlabel('y');
% drawnow;
% F = getframe(gcf);
% writeVideo(mov, F);
