function T = compute_homogeneous_transform(P, Q)
% 输入：
% - P: marker deviation（相对刚体坐标系的位置，3xN）
% - Q: marker position（实测位置，3xN）
% 输出：齐次变换矩阵T（4x4）

% 计算质心
centroid_P = mean(P, 2);
centroid_Q = mean(Q, 2);

% 去质心坐标
X = P - centroid_P;
Y = Q - centroid_Q;

% 计算协方差矩阵H
H = X * Y';

% SVD分解
[U, ~, V] = svd(H);

% 计算旋转矩阵
R = V * U';

% 调整反射（确保旋转矩阵行列式为+1）
if det(R) < 0
    V(:, 3) = -V(:, 3);
    R = V * U';
end

% 计算平移向量
t = centroid_Q - R * centroid_P;

% 构造齐次变换矩阵
T = [R, t; zeros(1, 3), 1];
end