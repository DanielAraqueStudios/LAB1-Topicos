%% Sistema 2 (RLC no lineal) - Controlador deadbeat - modelo Simulink
% Lazo cerrado discreto: Ramp -> Sum -> Deadbeat (Discrete Transfer Fcn) ->
% Planta (Discrete State-Space) -> Scope, con realimentacion unitaria negativa.
clear; clc; close all;

T = 0.1; % (coder-ex3)

%% Planta discreta (coder-ex3)
Ad = [0.736858, -0.040937; 0.163746, 0.900604];
Bd = [0.043127; 0.004381];
Cd = [6, 0];
Dd = 0;

%% Deadbeat (2.5, sin cambios respecto a sistema2_taller2.m, raiz doble p=0.935)
numDB = [0.34043, -0.32560];
denDB = [1, -2, 1];
% Polos lazo cerrado esperados: 0.884+/-j0.270, 0.935 (doble).

try
    modelName = 'sistema2_deadbeat_sim';
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
    new_system(modelName);
    open_system(modelName);

    add_block('simulink/Sources/Ramp', [modelName '/Ramp'], ...
        'Slope', '1', 'Start', '0', 'Position', [30, 100, 60, 130]);
    add_block('simulink/Math Operations/Sum', [modelName '/Sum'], ...
        'Inputs', '+-', 'Position', [110, 95, 130, 135]);
    add_block('simulink/Discrete/Discrete Transfer Fcn', [modelName '/Deadbeat'], ...
        'Numerator', mat2str(numDB), 'Denominator', mat2str(denDB), 'SampleTime', num2str(T), ...
        'Position', [180, 90, 270, 140]);
    add_block('simulink/Discrete/Discrete State-Space', [modelName '/Planta_Discreta'], ...
        'A', mat2str(Ad), 'B', mat2str(Bd), 'C', mat2str(Cd), 'D', mat2str(Dd), 'SampleTime', num2str(T), ...
        'Position', [320, 85, 410, 145]);
    add_block('simulink/Sinks/Scope', [modelName '/Scope'], 'Position', [460, 100, 490, 130]);

    add_line(modelName, 'Ramp/1', 'Sum/1');
    add_line(modelName, 'Sum/1', 'Deadbeat/1');
    add_line(modelName, 'Deadbeat/1', 'Planta_Discreta/1');
    add_line(modelName, 'Planta_Discreta/1', 'Scope/1');
    add_line(modelName, 'Planta_Discreta/1', 'Sum/2', 'autorouting', 'on');

    set_param(modelName, 'StopTime', '5');
    save_system(modelName, fullfile(fileparts(mfilename('fullpath')), [modelName '.slx']));
    sim(modelName);

    lazoCerradoDB = feedback(tf(numDB, denDB, T) * tf(ss(Ad, Bd, Cd, Dd, T)), 1);
    disp('Polos lazo cerrado deadbeat (sistema 2):'); disp(pole(lazoCerradoDB));
catch err
    warning('Simulink no disponible o fallo la construccion del modelo: %s', err.message);
end
