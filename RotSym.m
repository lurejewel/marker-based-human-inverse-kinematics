function R = RotSym(f, q)
% general rotation transformation, symbolic version
% f: rotation axis, 3x1
% q: rotation angle
f = sym(f);
fx = f(1); fy = f(2); fz = f(3);
% middle variables
vq = 1 - cos(q);
cq = cos(q);
sq = sin(q);
% general rotation transformation
R = nan(3,3); R = sym(R);
R(1,1) = fx*fx*vq + cq;
R(2,1) = fx*fy*vq + fz*sq;
R(3,1) = fx*fz*vq - fy*sq;
R(1,2) = fy*fx*vq - fz*sq;
R(2,2) = fy*fy*vq + cq;
R(3,2) = fy*fz*vq + fx*sq;
R(1,3) = fz*fx*vq + fy*sq;
R(2,3) = fz*fy*vq - fx*sq;
R(3,3) = fz*fz*vq + cq;

end

