data=importdata('bufferFile_05_09.json')
datas=(cell2mat(data))
%% matlab 2022
replace(datas,'[{"series":["signalesss"],"data":[[','');
replace(ans,'{"x":','');
replace(ans,'}]],"labels":[""]}]','');
replace(ans,'"y":','');
replace(ans,'},',';');
m=str2num(ans);
y=m(:,2);
plot(y)
%% PostProceso

% Identificar donde esta el inicio de la trama
start_flag_value = 0;
i = 1;
while (y(i) ~= 10)
    i = i+1;
end
i = i+1;
while (y(i) ~= 10)
    i = i+1;
end
start_flag_value = i;

% Construccion de datos
u = zeros(1,160);
vel = zeros(1,160);
i = 1;
for i = 1:160
    if(i <= 10)
        u(i) = 0.5;
    elseif(i <=90) 
        u(i) = 0.9;       
    else
        u(i) = 0.5;
    end
    vel(i) = y(i + (start_flag_value - 1) - (10) + 1);
end
vel(10) = vel(9);
u = u';
vel = vel';
clf
plot(vel)
hold on
plot(u)


