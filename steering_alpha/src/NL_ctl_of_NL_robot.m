%apply nonlinear steering algorithm to nonlinear robot kinematics

%steering simulation, 2nd order
%assumes can command (and achieve) an arbitrary spin-rate command, omega,
%instantaneously, although limited to max (saturation) spin rate commands
%Assume constant speed, v

%Assume desired direction of motion is a line at angle psi_des passing
%through point x_start:
xy_start_coords = [1;0];
%choose direction for desired path segment
psi_path_seg = pi/4; %45-deg angle
t_hat = [cos(psi_path_seg);sin(psi_path_seg)]; %vector parallel to direction of desired path
%nhat is vector normal to desired direction of motion; used for computation
%of offset; n_hat = Rot_about_z(pi/2)*t_hat
n_hat = [0 -1; 1 0]*t_hat; %

%Controller should null out displacement from desired path and should null out
%heading error; displacement is measured positive to the "left" when facing
%along the path in the desired direction of travel

%control parameters:
d_threshold = 1.0; %decide where to begin transition from perpendicular approach to sloped convergence
K_omega = 30.0; % gain for correcting heading...experiment with this value
omega_sat = 2.0; %max rads/sec

%simulate robot motion w/ x_vec defined as [x;y] and heading is psi
%choose initial conditions: x,y,psi:
x0 = 0;
y0 = 1;
psi0 = 0;

%specify forward speed, v:
v = 1; %1m/sec;  note that should change gains if change speed

%create some arrays to store simulation results
d_history = []; 
psi_history = [];
psi_des_history = [];
t_history = [];
u_history= []; %save history of omega commands as well
x_history = [];
y_history = [];
xy_robot_coords = [x0;y0]; % initialize variables
psi_robot = psi0;
x_dot = [0;0]; %compute these terms to integrate differential equations
omega_cmd = 0;
t=0; %time starts at zero
dt = 0.01; %choose time step for simulation
t_final = 5; %choose simulation duration
for t = 0:dt:t_final
    %compute appropriate heading as a function of offset:
    
    %compute the offset error:
    d = (xy_robot_coords-xy_start_coords)'*n_hat;
    if d> d_threshold %if far to the left of the path, heading should point toward the path
        psi_des = psi_path_seg-pi/2; %this heading is perpendicular to the path, pointing towards the path
    elseif d< -d_threshold %same thing if offset too far to the right--point towards path
        psi_des = psi_path_seg+pi/2;
    else %if offset is not large, make the heading gradually parallel to the path;
        psi_des = psi_path_seg-(pi/2)*d/d_threshold;  % heading offset proportional to displacement offset
    end
    
    psi_err = psi_robot-psi_des; %compare the current robot heading to the scheduled heading
    if psi_err > pi  %express heading error in range -pi to pi
        psi_err = psi_err-2*pi;
    end
    if psi_err < -pi
        psi_err = psi_err+2*pi
    end
    
    %feedback to coerce robot heading to conform to scheduled heading
    omega_cmd = -K_omega*psi_err;  %linear control law-> omega_cmd
    %limit the spin command to the legal range
    if omega_cmd > omega_sat
        omega_cmd = omega_sat
    elseif omega_cmd < -omega_sat
        omega_cmd = -omega_sat
    end
    
    %store incremental results for post plotting
    t_history=[t_history,t];
    d_history = [d_history,d];
    x_history = [x_history,xy_robot_coords(1)];
    y_history = [y_history,xy_robot_coords(2)];
    psi_history = [psi_history,psi_robot];
    u_history = [u_history,omega_cmd];
    psi_des_history=[psi_des_history,psi_des];
  
    %differential robot kinematics:
    psidot = omega_cmd;
    xdot = v*cos(psi_robot);
    ydot = v*sin(psi_robot);
    
    %Euler one-step integration:
    psi_robot = psi_robot+psidot*dt;
    %force heading into range -pi to pi
    if psi_robot >pi
        psi_robot = psi_robot-2*pi;
    end
    if psi_robot< -pi
        psi_robot = psi_robot+2*pi;
    end
    %Euler one-step for x and y updates:
    xy_robot_coords = xy_robot_coords + [xdot;ydot]*dt;

end
figure(1)
plot(t_history,d_history)
xlabel('time (sec)')
ylabel('offset (m)')
title('offset response')
figure(2)
plot(t_history,psi_history,'b',t_history,psi_des_history,'r')
xlabel('time (sec)')
ylabel('psi (rad)')
title('heading response: des (r) and act (b)')
figure(3)
plot(t_history,u_history)
xlabel('time (sec)')
ylabel('omega command (rad/sec)')
title('control effort')
figure(4)
%plot the desired path...
xdes_vec = [xy_start_coords(1),xy_start_coords(1)+t_hat(1)*3];
ydes_vec = [xy_start_coords(2),xy_start_coords(2)+t_hat(2)*3];
plot(xdes_vec,ydes_vec,'b',x_history,y_history,'r')
xlabel('x axis')
ylabel('y axis')
title('desired (blue) and actual (red) path')
