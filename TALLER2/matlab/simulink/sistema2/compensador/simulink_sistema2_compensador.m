%% Sistema 2 (RLC no lineal) - Compensador digital - modelo Simulink
% Lazo cerrado discreto: Ramp -> Sum -> Controlador (Discrete Transfer Fcn) ->
% Planta (Discrete State-Space) -> Scope, con realimentacion unitaria negativa.
clear; clc; close all;

T = 0.1; % (coder-ex3)

%% Planta discreta (coder-ex3, discrete_tools.c2d_zoh)
Ad = [0.736858, -0.040937; 0.163746, 0.900604];
Bd = [0.043127; 0.004381];
Cd = [6, 0];
Dd = 0;

%% Compensador digital CORREGIDO (2.4, coder-ex3)
% Tipo 2 (rampa), zeta=0.7, wn reducido 4.0->2.0, n=2 secciones lead. NO usar
% los coeficientes antiguos de sistema2_taller2.m (bug de error de estado
% estacionario corregido aqui).
num_c = [0.097148, -0.167193, 0.071935];
den_c = [1, -3.191944, 3.739072, -1.902310, 0.355183];
% Raices lazo cerrado verificadas |z|<0.945; error de rampa -> 2.1e-10 en k=399.

try
    modelName = 'sistema2_compensador_sim';
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
    new_system(modelName);
    open_system(modelName);

    add_block('simulink/Sources/Ramp', [modelName '/Ramp'], ...
        'Slope', '1', 'Start', '0', 'Position', [30, 100, 60, 130]);
    add_block('simulink/Math Operations/Sum', [modelName '/Sum'], ...
        'Inputs', '+-', 'Position', [110, 95, 130, 135]);
    add_block('simulink/Discrete/Discrete Transfer Fcn', [modelName '/Controlador'], ...
        'Numerator', mat2str(num_c), 'Denominator', mat2str(den_c), 'SampleTime', num2str(T), ...
        'Position', [180, 90, 270, 140]);
    add_block('simulink/Discrete/Discrete State-Space', [modelName '/Planta_Discreta'], ...
        'A', mat2str(Ad), 'B', mat2str(Bd), 'C', mat2str(Cd), 'D', mat2str(Dd), 'SampleTime', num2str(T), ...
        'Position', [320, 85, 410, 145]);
    add_block('simulink/Sinks/Scope', [modelName '/Scope'], 'Position', [460, 100, 490, 130]);

    add_line(modelName, 'Ramp/1', 'Sum/1');
    add_line(modelName, 'Sum/1', 'Controlador/1');
    add_line(modelName, 'Controlador/1', 'Planta_Discreta/1');
    add_line(modelName, 'Planta_Discreta/1', 'Scope/1');
    add_line(modelName, 'Planta_Discreta/1', 'Sum/2', 'autorouting', 'on');

    set_param(modelName, 'StopTime', '15');
    save_system(modelName, fullfile(fileparts(mfilename('fullpath')), [modelName '.slx']));
    sim(modelName);

    lazoCerradoComp = feedback(tf(num_c, den_c, T) * tf(ss(Ad, Bd, Cd, Dd, T)), 1);
    p = pole(lazoCerradoComp);
    if max(abs(p)) < 1
        estado = 'estable';
    else
        estado = 'INESTABLE';
    end
    fprintf('Sistema 2 compensador: max|pole|=%.6f (%s)\n', max(abs(p)), estado);
catch err
    warning('Simulink no disponible o fallo la construccion del modelo: %s', err.message);
end
