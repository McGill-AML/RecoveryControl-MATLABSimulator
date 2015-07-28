traj_head = 3*pi/4;
q0 = quatmultiply([0;-1;0;0],[cos(traj_head(1)/2);0;0;sin(traj_head(1)/2)]);
q0 = q0/norm(q0);
q_des = angle2quat(-(0+pi),0,-3*pi/4,'xyz')';
q_err_w = quatmultiply(q_des,quatinv(q0))

acos(q_err_w(1))*2*180/pi

axis_err = sign(q_err_w(1))*q_err_w(2:4)