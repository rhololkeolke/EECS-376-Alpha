%apply linear steering algorithm to nonlinear robot kinematics

%steering simulation, 2nd order
%assumes can command (and achieve) a commanded spin-rate command, omega,
%instantaneously, although omega saturates:
omega_max = 3; %max allowed omega command

%Assume constant speed, v
%Assume desired direction of motion is a line at angle psi_des passint
%through point x_start:
xy_start_coords = [1;0];
psi_des = pi/4; %45-deg angle
t_hat = [cos(psi_des);sin(psi_des)]; %vector parallel to direction of desired path
%nhat is vector normal to desired direction of motion; used for computation
%of offset; n_hat = Rot_about_z(pi/2)*t_hat
n_hat = [0 -1; 1 0]*t_hat; %

%Controller should null out displacement from desired path and should null out
%heading error; displacement is measured positive to the "left" when facing
%along the path in the desired direction of travel

%simulate robot motion w/ x_vec defined as [x;y] and heading is psi
%compute control law as: omega_cmd = -K_vec*[d;d_psi], where d_psi =
%psi-psi_des

%initial condition, x,y,psi:
x0 = 0.1;
y0 = 0.5;
psi0 = 1;



%specify forward speed, v:
v = 1; %1m/sec;  note that should change gains if change speed

%specify feedback gains Kd and Kpsi, as though the system were linear
omega_n = 12
Kd = omega_n*omega_n/v %Kd*v = omega_n^2
Kpsi = omega_n*2 %Kpsi = 2*zeta*omega_n
K = [Kd, Kpsi];

%create some arrays to store simulation results
d_history = []; 
psi_history = [];
t_history = [];
u_history= []; %save history of omega commands as well
x_history = [];
y_history = [];
xy_robot_coords = [x0;y0]; % initialize variables
psi_robot = psi0;
x_dot = [0;0]; %compute these terms to integrate differential equations
omega_cmd = 0;
t=0;
dt = 0.01; %choose time step for simulation
t_final = 10; %choose simulation duration
for t = 0:dt:t_final
    %compute the offset error:
    d = (xy_robot_coords-xy_start_coords)'*n_hat;
    %compute the heading error
    psi_err = psi_robot-psi_des;
    %express in range -pi to pi
    if psi_err > pi 
        psi_err = psi_err-2*pi;
    end
    if psi_err < -pi
        psi_err = psi_err+2*pi
    end
    
    omega_cmd = -K*[d;psi_err];  %linear control law-> omega_cmd
    %check saturation:
    if omega_cmd>omega_max
        omega_cmd= omega_max;
    elseif omega_cmd < -omega_max
        omega_cmd = -omega_max
    end
    
          
    %store incremental results for post plotting
    t_history=[t_history,t];
    d_history = [d_history,d];
    x_history = [x_history,xy_robot_coords(1)];
    y_history = [y_history,xy_robot_coords(2)];
    psi_history = [psi_history,psi_robot];
    u_history = [u_history,omega_cmd];

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
    xy_robot_coords = xy_robot_coords + [xdot;ydot]*dt;

end
figure(1)
plot(t_history,d_history)
xlabel('time (sec)')
ylabel('offset (m)')
title('offset response')
figure(2)
plot(t_history,psi_history)
xlabel('time (sec)')
ylabel('psi (rad)')
title('heading response')
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
