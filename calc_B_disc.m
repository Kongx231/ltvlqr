function B_disc = calc_B_disc(in1,u,dt,in4)
%CALC_B_DISC
%    B_DISC = CALC_B_DISC(IN1,U,DT,IN4)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    11-Oct-2021 23:16:27

L = in4(:,3);
m = in4(:,1);
B_disc = [0.0;(1.0./L.^2.*dt)./m];
