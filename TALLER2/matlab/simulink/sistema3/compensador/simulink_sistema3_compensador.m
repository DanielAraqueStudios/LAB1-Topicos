%% Sistema 3 (termico, theta_m->R->theta_w->R_a->theta_a) - Compensador
% digital - modelo Simulink.
% Lazo cerrado discreto: Parabola (From Workspace) -> Sum -> Controlador
% (Discrete Transfer Fcn) -> Planta (Discrete State-Space) -> Scope, con
% realimentacion unitaria negativa. Referencia tipo 3 (parabola): caso mas
% dificil, ver docs/taller2_puntos_2_4.tex 3.4 (coder-ex3).
clear; clc; close all;

T = 0.1; % (coder-ex3)

%% Planta discreta (3.2/3.3, coder-ex3): A=[-1,1;1,-2], B=[1;0], C=[1,-1],
% R=Ra=Cm=Cw=1 (supuesto documentado). Autovalores continuos -0.381966,-2.618034.
Ad = [0.909218, 0.086250; 0.086250, 0.822968];
Bd = [0.095314; 0.004532];
Cd = [1, -1];
Dd = 0;

%% Compensador digital (3.4, coder-ex3): tipo 3 (parabola), zeta=0.2, wn=0.5,
% n=2 secciones. Diseno mas exigente genuinamente alcanzable (no rapido):
% dos raices muy cercanas al circulo unitario (~0.99); error de parabola
% converge a 0 pero requiere ~600+ muestras (~65s) para bajar de 0.05.
num_c = [0.043530, -0.086091, 0.042566];
den_c = [1, -3.443832, 4.380744, -2.479237, 0.591573, -0.049247];

%% Referencia parabola r(k) = 0.5*(k*T)^2 (coder-ex3), horizonte largo por la
% convergencia lenta documentada.
Nsim = 700;
t = (0:Nsim - 1)' * T;
r = 0.5 * t.^2;
simin = [t, r]; %#ok<NASGU> % usado por el bloque From Workspace

try
    modelName = 'sistema3_compensador_sim';
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
    new_system(modelName);
    open_system(modelName);

    add_block('simulink/Sources/From Workspace', [modelName '/Parabola'], ...
        'VariableName', 'simin', 'Position', [30, 100, 90, 130]);
    add_block('simulink/Math Operations/Sum', [modelName '/Sum'], ...
        'Inputs', '+-', 'Position', [140, 95, 160, 135]);
    add_block('simulink/Discrete/Discrete Transfer Fcn', [modelName '/Controlador'], ...
        'Numerator', mat2str(num_c), 'Denominator', mat2str(den_c), 'SampleTime', num2str(T), ...
        'Position', [210, 90, 300, 140]);
    add_block('simulink/Discrete/Discrete State-Space', [modelName '/Planta_Discreta'], ...
        'A', mat2str(Ad), 'B', mat2str(Bd), 'C', mat2str(Cd), 'D', mat2str(Dd), 'SampleTime', num2str(T), ...
        'Position', [350, 85, 440, 145]);
    add_block('simulink/Sinks/Scope', [modelName '/Scope'], 'Position', [490, 100, 520, 130]);

    add_line(modelName, 'Parabola/1', 'Sum/1');
    add_line(modelName, 'Sum/1', 'Controlador/1');
    add_line(modelName, 'Controlador/1', 'Planta_Discreta/1');
    add_line(modelName, 'Planta_Discreta/1', 'Scope/1');
    add_line(modelName, 'Planta_Discreta/1', 'Sum/2', 'autorouting', 'on');

    set_param(modelName, 'StopTime', num2str(t(end)));
    save_system(modelName, fullfile(fileparts(mfilename('fullpath')), [modelName '.slx']));
    sim(modelName);

    lazoCerradoComp = feedback(tf(num_c, den_c, T) * tf(ss(Ad, Bd, Cd, Dd, T)), 1);
    p = pole(lazoCerradoComp);
    fprintf('Sistema 3 compensador: max|pole|=%.6f (estable, cerca del limite)\n', max(abs(p)));
catch err
    warning('Simulink no disponible o fallo la construccion del modelo: %s', err.message);
end
