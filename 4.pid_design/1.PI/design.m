%% Plant
clear
clc
plant_tf = tf([55.99],[1 33.95]);
%step(plant_tf);
step_response_parameters = stepinfo(plant_tf);
plant_ts = step_response_parameters.RiseTime;

%% Controller 

% Requirements
e_p = 0;
ts = 0.7*plant_ts;

% PID Tuner Obtained Parameters
P = 0.74773;
I = 31.7677;




