%% Sistema 4 (dos tanques neumaticos) - Controlador deadbeat - modelo Simulink
% Lazo cerrado discreto: Step -> Sum -> Deadbeat (Discrete Transfer Fcn) ->
% Planta (Discrete State-Space) -> Scope, con realimentacion unitaria negativa.
clear; clc; close all;

T = 0.1; % (coder-ex3)

%% Planta discreta (coder-ex3)
Ad = [0.8629, 0.0889; 0.0444, 0.9185];
Bd = [0.04705; 0.01313];
Cd = [0, 0.5];
Dd = 0;

%% Deadbeat (4.5, sin cambios - coder-ex3): raiz doble en p=0, tercer root z=0.7218
numDB = [313.6195, -170.6446];
denDB = [1, -1];

try
    modelName = 'sistema4_deadbeat_sim';
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
    new_system(modelName);
    open_system(modelName);

    add_block('simulink/Sources/Step', [modelName '/Step'], ...
        'Time', '0', 'Before', '0', 'After', '1', 'Position', [30, 100, 60, 130]);
    add_block('simulink/Math Operations/Sum', [modelName '/Sum'], ...
        'Inputs', '+-', 'Position', [110, 95, 130, 135]);
    add_block('simulink/Discrete/Discrete Transfer Fcn', [modelName '/Deadbeat'], ...
        'Numerator', mat2str(numDB), 'Denominator', mat2str(denDB), 'SampleTime', num2str(T), ...
        'Position', [180, 90, 270, 140]);
    add_block('simulink/Discrete/Discrete State-Space', [modelName '/Planta_Discreta'], ...
        'A', mat2str(Ad), 'B', mat2str(Bd), 'C', mat2str(Cd), 'D', mat2str(Dd), 'SampleTime', num2str(T), ...
        'Position', [320, 85, 410, 145]);
    add_block('simulink/Sinks/Scope', [modelName '/Scope'], 'Position', [460, 100, 490, 130]);

    add_line(modelName, 'Step/1', 'Sum/1');
    add_line(modelName, 'Sum/1', 'Deadbeat/1');
    add_line(modelName, 'Deadbeat/1', 'Planta_Discreta/1');
    add_line(modelName, 'Planta_Discreta/1', 'Scope/1');
    add_line(modelName, 'Planta_Discreta/1', 'Sum/2', 'autorouting', 'on');

    set_param(modelName, 'StopTime', '5');
    save_system(modelName, fullfile(fileparts(mfilename('fullpath')), [modelName '.slx']));
    sim(modelName);

    lazoCerradoDB = feedback(tf(numDB, denDB, T) * tf(ss(Ad, Bd, Cd, Dd, T)), 1);
    disp('Polos lazo cerrado deadbeat (sistema 4):'); disp(pole(lazoCerradoDB));
catch err
    warning('Simulink no disponible o fallo la construccion del modelo: %s', err.message);
end
