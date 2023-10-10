%% Carga de datos
identification_data = readtable("identification_data.txt")
u = identification_data(:,"u").u;
omega = identification_data(:,"omega").omega;
u_400 = identification_data(1:400,"u").u;
omega_400 = identification_data(1:400,"omega").omega;

