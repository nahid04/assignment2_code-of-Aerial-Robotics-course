function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


Kvy = 20;      
Kpy = 60;
Kvz = 15;      
Kpz = 60;
Kvphi = 15;     
Kpphi = 65;


y_des = des_state.pos(1); 
z_des = des_state.pos(2);
y_d_des = des_state.vel(1); 
z_d_des = des_state.vel(2);
y_dd_des = des_state.acc(1);
z_dd_des = des_state.acc(2);
y = state.pos(1);
z = state.pos(2);
y_d = state.vel(1); 
z_d = state.vel(2);
phi = state.rot;
phi_d = state.omega;


u1 = params.mass*(params.gravity + z_dd_des + Kvz*(z_d_des - z_d) + Kpz*(z_des - z));
phi_c = -(y_dd_des + Kvy*(y_d_des - y_d) + Kpy*(y_des - y))/params.gravity;
phi_c_dot = -(Kvy*(y_dd_des + params.gravity*phi) + Kpy*(y_d_des - y_d))/params.gravity;
u2 = params.Ixx*( Kvphi*(phi_c_dot - phi_d) + Kpphi*(phi_c - phi));


end

