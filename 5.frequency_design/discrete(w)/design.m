%% Plant
clear
clc
close all
s = tf('s');
plant_tf = 55.99/(s+33.95);
step_response_parameters = stepinfo(plant_tf);
plant_ts = step_response_parameters.RiseTime;
%step(plant_tf);

%% Plant Discretization
Ts = 0.005;
plant_tf_z = c2d(plant_tf,Ts,'zoh');

%% Control Parameters
Mp = 0.25;
Ev=0;

%% Ev=0 using K/((z-1)^2) compensator
z = tf('z',0.005);
% Se ajusta la ganacia k para lograr incrementar el margen de ganancia dado
% que esta muy alto para ajustarse solo con un compensador.
k = 0.002;

pre_ctrl_z = k/((z-1)^2);
prectrl_plant_tf_z = series(pre_ctrl_z,plant_tf_z);
bode(prectrl_plant_tf_z);
%step(feedback(precontrolled_plant_tf,1))

%% W Transform
prectrl_plant_tf_w = w_transform.z2w(prectrl_plant_tf_z,Ts);
%bode(prectrl_plant_tf_z);
%hold on;
bode(prectrl_plant_tf_w);

%% Control Design (Lead)
required_pm = 40+10+(23.1);
b = (1 + sind(required_pm))/(1-sind(required_pm ));
w_crossover_gain = -10*log(b);
w_crossover = 68; %rad/s
T2=1/(sqrt(b)*w_crossover);
lead_ctrl_tf_w = (1+(b*T2*s))/(1+(T2*s));
%bode(series(lead_controller_tf_w,prectrl_plant_tf_w))
% K fine setting
bode(k*series(lead_ctrl_tf_w,prectrl_plant_tf_w))
%step(feedback(series(k*w_transform.w2z(lead_controller_tf_w,Ts),prectrl_plant_tf_z),1))
%% Control Design (Lead2) 
required_pm = 40+30+(-23.9);
b = (1 + sind(required_pm))/(1-sind(required_pm ));
w_crossover_gain = -10*log(b);
w_crossover = 56.1; %rad/s
T2=1/(sqrt(b)*w_crossover);
lead_ctrl2_tf_w = (1+(b*T2*s))/(1+(T2*s));
k2=1.3;
bode(series(k2*lead_ctrl2_tf_w,(series(lead_ctrl_tf_w,prectrl_plant_tf_w))))
step(feedback(series(k2*lead_ctrl2_tf_w,(series(lead_ctrl_tf_w,prectrl_plant_tf_w))),1))
%% Inverse w Transform
lead_ctrl_tf_z = w_transform.w2z(k2*series(lead_ctrl_tf_w,lead_ctrl2_tf_w),Ts);
bode(lead_ctrl_tf_z)

%% Full Controller
final_ctrl_tf_z=series(lead_ctrl_tf_z,pre_ctrl_z);
step(feedback(series(final_ctrl_tf_z,plant_tf_z),1))


%% Diference Equations

%nagative powers and coeff simplify
[num,den] = tfdata(zpk(final_ctrl_tf_z));
arrayNum = cell2mat(num);
arrayDen = cell2mat(den);
final_ctrl_tf_neg_z = tf(arrayNum,arrayDen,Ts,"variable",'z^-1');
[final_num,final_den] = tfdata(final_ctrl_tf_neg_z);
arrayFinalNum = cell2mat(final_num);
arrayFinalDen = cell2mat(final_den);

final_ctrl_tf_neg_z
%% Control Design (Lag)
%required_pm = 40+6+(34.3);
%w_crossover_current_phase = 180-required_pm;
%w_crossover = 34.8; %rad/s
%w_crossover_current_gain = 29.1;
%a = 10^(w_crossover_current_gain/20);
%T1=10/(a*w_crossover);
%lag_controller_tf_w = (1+(a*T1*s))/(1+(T1*s))

%step(feedback(series(w_transform.w2z(lag_controller_tf_w,Ts),prectrl_plant_tf_z),1))


%b = (1 + sind(required_pm))/(1-sind(required_pm ))
%w_crossover_gain = -10*log(b)
%w_crossover = 35.2 %rad/s
