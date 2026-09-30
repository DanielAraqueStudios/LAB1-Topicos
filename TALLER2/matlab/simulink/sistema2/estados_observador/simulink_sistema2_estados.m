%% Sistema 2 (RLC no lineal) - Realimentacion de estados + observador (servo
% tipo-2, seguimiento de rampa) - modelo Simulink.
% Se modela la dinamica de lazo cerrado aumentada (Ghat - Hhat*Khat) como un
% unico bloque Discrete State-Space cuya entrada es la referencia r(k) y cuya
% salida es Chat*x_aug; el vector Bref inyecta r(k) solo en el ultimo estado
% de seguimiento (xi2), igual que en el lazo de simulacion de referencia de
% sistema2_taller2.m (coder-core / coder-ex3).
clear; clc; close all;

T = 0.1;

%% Estados aumentados (2.6, sin cambios - coder-ex3)
% orden de estado: [x1, x2, xi1, xi2]
Ad = [0.736858, -0.040937; 0.163746, 0.900604];
Bd = [0.043127; 0.004381];
Cd = [6, 0];
n = size(Ad, 1);
CGd = Cd * Ad; CHd = Cd * Bd;
Ghat = [Ad, zeros(n, 2); -CGd, 1, 1; -CGd, 0, 1];
Hhat = [Bd; -CHd; -CHd];
Chat = [Cd, 0, 0];

% Khat verificado (coder-ex3): dominante z=0.7252+/-j0.2130, extra 0.3,0.25
Khat = [58.685, 433.608, -2.576, 13.370];

% Observador (coder-ex3): z_obs=-0.0584+/-j0.0171
L = [0.2924; -3.7179];

Acl = Ghat - Hhat * Khat;
Bref = [0; 0; 0; 1]; % r(k) entra solo en xi2 (Bri=[0;1] en el lazo tipo-2)

disp('Polos lazo cerrado servo (verificacion, sistema 2):'); disp(eig(Acl));

try
    modelName = 'sistema2_estados_sim';
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
    new_system(modelName);
    open_system(modelName);

    add_block('simulink/Sources/Ramp', [modelName '/Ramp'], ...
        'Slope', '1', 'Start', '0', 'Position', [30, 100, 60, 130]);
    add_block('simulink/Discrete/Discrete State-Space', [modelName '/LazoCerrado_Aumentado'], ...
        'A', mat2str(Acl), 'B', mat2str(Bref), 'C', mat2str(Chat), 'D', '0', ...
        'SampleTime', num2str(T), 'Position', [140, 85, 280, 145]);
    add_block('simulink/Sinks/Scope', [modelName '/Scope'], 'Position', [330, 100, 360, 130]);

    add_line(modelName, 'Ramp/1', 'LazoCerrado_Aumentado/1');
    add_line(modelName, 'LazoCerrado_Aumentado/1', 'Scope/1');

    set_param(modelName, 'StopTime', '10');
    save_system(modelName, fullfile(fileparts(mfilename('fullpath')), [modelName '.slx']));
    sim(modelName);

    fprintf('Sistema 2 estados+observador: max|eig(Acl)|=%.6f\n', max(abs(eig(Acl))));
    disp('Polos observador (info, no afectan Acl al ser separacion x/xhat exacta):');
    disp(eig(Ad - L * Cd));
catch err
    warning('Simulink no disponible o fallo la construccion del modelo: %s', err.message);
end
