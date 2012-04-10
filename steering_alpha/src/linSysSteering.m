%steering simulation, 2nd order
%assumes can command (and achieve) an arbitrary spin-rate command, omega,
%instantaneously
%Assume constant speed, v
%Assume desired direction of motion is the x axis (may be generalized).
%Controller should null out displacement from x axis and should null out
%non-zero heading

%simulate ideal, linear system w/ x_vec defined as:
%x_vec = [displacement,heading]^T
%xdot = Ax + Bu
%u = -K_vec*(x_vec)
%initial condition, d0, psi0; introduce some error here to watch transient
%response
d0 = 0.1; %positive displacement is to the left of the desired path
psi0 = 0.1; %positive heading is measured CCW w/rt direction of path

psi_des = 0; %for this example, desire to travel along x axis

x_desired = [0;psi_des]; %specify desire 0 offset and heading = 0
n_hat = [0;1]; %vector normal to desired direction of motion; use to compute the path offset
%specify forward speed, v:
v = 1; %1m/sec
A = [0 v; 0 0]; %d/dt (d) = v*sin(psi) ~= v*psi
b = [0;1]; %d/dt (psi) = commanded sin rate, omega
%specify feedback gains Kd and Kpsi
omega_n = 6
Kd = omega_n*omega_n/v %Kd*v = omega_n^2
Kpsi = 2*omega_n %Kpsi = 2*zeta*omega_n
K = [Kd, Kpsi];
d_history = []; %create some arrays to store simulation results
psi_history = [];
t_history = [];
u_history= []; %save history of omega commands as well
x_vec = [d0;psi0]; % initialize variables--offset and heading errs
omega_cmd = 0;
t=0;
dt = 0.01; %choose time step for simulation
t_final = 3; %choose simulation duration
for t = 0:dt:t_final

    omega_cmd = -K*x_vec; %here is the control law!
    
    %save values in vectors for plotting
    t_history=[t_history,t];
    d_history = [d_history,x_vec(1)];
    psi_history = [psi_history,x_vec(2)];
    u_history = [u_history,omega_cmd];

    %for straight-line path, d/dt (x_des)= [0;0], so d/dt(dx_vec) =
    %d/dt[d;psi]
    xdot = A*x_vec + b*omega_cmd;  %here are the dynamic equations for the corresponding
        %linear system
    x_vec = x_vec+xdot*dt; %Euler one-step integration of the dynamic eqns
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