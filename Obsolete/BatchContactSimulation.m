% 
% angles_array = deg2rad([5 10 15 20 25 30]);
% tilt_array = repmat(angles_array,1,4*5);
% e_array1 = [repmat(0.95,size(angles_array)),repmat(0.9,size(angles_array)),repmat(0.85,size(angles_array)),repmat(0.8,size(angles_array))];
% e_array = repmat(e_array1,1,5);
% Vc_array = [repmat(0.1,size(e_array1)),repmat(0.3,size(e_array1)),repmat(0.5,size(e_array1)),repmat(0.7,size(e_array1)),repmat(0.9,size(e_array1))];
% recovery_control_array = ones(size(tilt_array));

% % angles_array = deg2rad([-15 -10 -5]);
% angles_array = deg2rad([-15 -10 -5 0 5 10 15 20 25 30]);
% tilt_array = repmat(angles_array,1,4*5);
% e_array1 = [repmat(0.95,size(angles_array)),repmat(0.9,size(angles_array)),repmat(0.85,size(angles_array)),repmat(0.8,size(angles_array))];
% e_array = repmat(e_array1,1,5);
% Vc_array = [repmat(0.1,size(e_array1)),repmat(0.3,size(e_array1)),repmat(0.5,size(e_array1)),repmat(0.7,size(e_array1)),repmat(0.9,size(e_array1))];
% recovery_control_array = ones(size(tilt_array));

Vc = 0.4:0.05:1.5;
Vc_array = repmat(Vc,1,18);
tilt_array1 = [-20*ones(size(Vc)),-15*ones(size(Vc)),-10*ones(size(Vc)),-5*ones(size(Vc)),0*ones(size(Vc)),5*ones(size(Vc)),10*ones(size(Vc)),15*ones(size(Vc)),20*ones(size(Vc))];
tilt_array = deg2rad(repmat(tilt_array1,1,2));
e_array = 0.9*ones(size(Vc_array));
recovery_control_array = zeros(size(Vc_array)); %OFF
heading_array = [(pi/4)*ones(size(tilt_array1)),zeros(size(tilt_array1))];


tilt_rec = zeros(size(tilt_array));
Vc_rec = zeros(size(tilt_array));
defl_init_rec = zeros(size(tilt_array));
defl_max_rec = zeros(size(tilt_array));
Fn_init_rec = zeros(size(tilt_array));
Fn_max_rec = zeros(size(tilt_array));
numContacts_rec = zeros(size(tilt_array));
stable_rec = zeros(size(tilt_array));

for k = 1:size(tilt_array,2)
    display(['tilt:' num2str(rad2deg(tilt_array(k))) blanks(5) 'e:' num2str(e_array(k)) blanks(5) 'Vc:' num2str(Vc_array(k))]);
    [tilt_rec(k), Vc_rec(k), defl_init_rec(k), defl_max_rec(k), Fn_init_rec(k), Fn_max_rec(k), numContacts_rec(k), stable_rec(k) ] = StartSimulator(tilt_array(k), e_array(k), Vc_array(k), recovery_control_array(k),heading_array(k));
end
figure();

copy_this = [defl_init_rec',defl_max_rec',stable_rec',numContacts_rec'];
copy_this2 = [heading_array',rad2deg(tilt_array'),Vc_array',defl_init_rec',defl_max_rec',stable_rec', Fn_init_rec', Fn_max_rec', numContacts_rec', Vc_rec'];  

Vc_determine = [heading_array',rad2deg(tilt_array'), rad2deg(tilt_rec'), Vc_array', Vc_rec'];