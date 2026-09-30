%% Sistema 4 (dos tanques neumaticos) - Compensador digital - modelo Simulink
% Lazo cerrado discreto: Step -> Sum -> Controlador (Discrete Transfer Fcn) ->
% Planta (Discrete State-Space) -> Scope, con realimentacion unitaria negativa.
clear; clc; close all;

T = 0.1; % (coder-ex3)

%% Planta discreta (coder-ex3)
Ad = [0.8629, 0.0889; 0.0444, 0.9185];
Bd = [0.04705; 0.01313];
Cd = [0, 0.5];
Dd = 0;

%% Compensador digital CORREGIDO (4.4, coder-ex3)
% Tipo 1 (escalon), zeta=0.8, wn=1.5 (sin reduccion), n=2 secciones lead. NO
% usar los coeficientes antiguos de sistema4_taller2.m (bug de error de
% estado estacionario corregido aqui: ts pasa de 9.9s a 3.6s).
num_c = [6.270181, -11.077288, 4.892455];
den_c = [1, -2.441241, 1.960535, -0.519294];
% Raices lazo cerrado verificadas |z|<0.888; error de escalon -> 2.2e-15 en k=399.

try
    modelName = 'sistema4_compensador_sim';
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
    new_system(modelName);
    open_system(modelName);

    add_block('simulink/Sources/Step', [modelName '/Step'], ...
        'Time', '0', 'Before', '0', 'After', '1', 'Position', [30, 100, 60, 130]);
    add_block('simulink/Math Operations/Sum', [modelName '/Sum'], ...
        'Inputs', '+-', 'Position', [110, 95, 130, 135]);
    add_block('simulink/Discrete/Discrete Transfer Fcn', [modelName '/Controlador'], ...
        'Numerator', mat2str(num_c), 'Denominator', mat2str(den_c), 'SampleTime', num2str(T), ...
        'Position', [180, 90, 270, 140]);
    add_block('simulink/Discrete/Discrete State-Space', [modelName '/Planta_Discreta'], ...
        'A', mat2str(Ad), 'B', mat2str(Bd), 'C', mat2str(Cd), 'D', mat2str(Dd), 'SampleTime', num2str(T), ...
        'Position', [320, 85, 410, 145]);
    add_block('simulink/Sinks/Scope', [modelName '/Scope'], 'Position', [460, 100, 490, 130]);

    add_line(modelName, 'Step/1', 'Sum/1');
    add_line(modelName, 'Sum/1', 'Controlador/1');
    add_line(modelName, 'Controlador/1', 'Planta_Discreta/1');
    add_line(modelName, 'Planta_Discreta/1', 'Scope/1');
    add_line(modelName, 'Planta_Discreta/1', 'Sum/2', 'autorouting', 'on');

    set_param(modelName, 'StopTime', '10');
    save_system(modelName, fullfile(fileparts(mfilename('fullpath')), [modelName '.slx']));
    sim(modelName);

    lazoCerradoComp = feedback(tf(num_c, den_c, T) * tf(ss(Ad, Bd, Cd, Dd, T)), 1);
    p = pole(lazoCerradoComp);
    if max(abs(p)) < 1
        estado = 'estable';
    else
        estado = 'INESTABLE';
    end
    fprintf('Sistema 4 compensador: max|pole|=%.6f (%s)\n', max(abs(p)), estado);
catch err
    warning('Simulink no disponible o fallo la construccion del modelo: %s', err.message);
end
