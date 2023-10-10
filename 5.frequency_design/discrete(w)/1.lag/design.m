%% Plant
clear
clc
plant_tf = tf([55.99],[1 33.95]);
%step(plant_tf);
step_response_parameters = stepinfo(plant_tf);
plant_ts = step_response_parameters.RiseTime;

%% Plant Discretization
Ts = 0.005;
plant_tf_z = c2d(plant_tf,Ts,'zoh');

%% Control Parameters
Mp = 0.25;
Ev=0;
%% Ev to zero by K/((z-1)^2)

prectrl_plant_tf_z = series(tf([1],[1 -2 1],Ts),plant_tf_z);
%bode(prectrl_plant_tf_z);
%step(feedback(precontrolled_plant_tf,1))

%% W Transform
syms z w;

[prectr_plant_tf_z_num,prectr_plant_tf_z_den] = tfdata(prectrl_plant_tf_z);
prectr_plant_tf_z_num=cell2mat(prectr_plant_tf_z_num);
prectr_plant_tf_z_den=cell2mat(prectr_plant_tf_z_den);

prectr_plant_tf_z_sym = poly2sym(prectr_plant_tf_z_num,z)/...
    poly2sym(prectr_plant_tf_z_den,z);
substitution = ((1+(w*sym(Ts)/2)) / (1-(w*sym(Ts)/2)));

prectr_plant_tf_w_sym = subs(prectr_plant_tf_z_sym,z,substitution)
[num, den] = numden(prectr_plant_tf_w_sym);
prectr_plant_tf_w = tf(sym2poly(num),sym2poly(den));
bode(prectr_plant_tf_w);
step(prectr_plant_tf_w);

%% Control Design



