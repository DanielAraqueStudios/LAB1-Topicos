%% Sistema 3 (termico) - Controlador deadbeat - modelo Simulink.
% Lazo cerrado discreto: Parabola (From Workspace) -> Sum -> Deadbeat
% (Discrete Transfer Fcn) -> Planta (Discrete Transfer Fcn) -> Scope, con
% realimentacion unitaria negativa.
%
% Historia (coder-ex3-2): la estructura minima D(z)=(q1*z+q0)/(z-1)^3 (2
% grados de libertad para 5 raices) es genuinamente inestable/indeterminada.
% Se corrige con design_deadbeat_diophantine() (src/discrete_tools.py):
% resuelve (z-1)^3*D_extra(z)*denG(z) + N(z)*numG(z) = z^9 para N(z)
% (grado 4) y D_extra(z) (monico, grado 4) via Diophantine lineal exacta
% (sympy). D(z) = (z-1)^3 * D_extra(z), grado 7 (integradores=3 + n_c=4).
% jury_stability sobre el polinomio caracteristico de lazo cerrado completo
% (grado 9) confirma estable, max|raiz|~=0.0706. El charpoly exacto
% (res["closed_loop_charpoly"], coder-ex3-2) tiene coeficientes ~1e-11 de
% z^9, pero para un polinomio de grado 9 con raices agrupadas cerca del
% origen la relacion raiz~coeficiente es de tipo eps^(1/9): una perturbacion
% de coeficiente ~1e-11 produce una perturbacion de raiz ~0.05-0.07 (mal
% condicionamiento numerico esperado, no un error) - por eso max|raiz|~=0.0706
% es la lectura correcta, no ~0. Verificado contra
% tests/test_discrete_tools.py::test_deadbeat_diophantine_system3_tracks_parabola_to_zero_error
% (traza |r-y| ≲ 1e-4 en estado estacionario tras 300 muestras).
clear; clc; close all;

T = 0.1; % (coder-ex3 / coder-ex3-2)

%% Planta discreta (coder-ex3-2, c2d_zoh sobre system3_model.get_system3_ss())
numG = [0.09078188086490346, -0.08214967454440625];
denG = [1.0, -1.7321860143612207, 0.7408182206817179];

%% Deadbeat via Diophantine (3.5, coder-ex3-2)
% N(z), grado 4 (5 coeficientes, alto->bajo)
N = [-667946.7490608095, 2557483.9371788995, -3658753.1499754526, ...
     2316665.78813344, -547333.981037519];
% D(z) = (z-1)^3 * D_extra(z), grado 7 (8 coeficientes, alto->bajo)
D = [1.0, 1.7321860143612202, 2.259650167666887, 2.6308994567327346, ...
     60640.34541456557, -182006.40093232918, 182052.55065621022, -60694.11787408538];

% Advertencia: controlador de orden 9, transitorio agresivo esperado
% (|u| pico ~ 5.2e3, coder-ex3-2) - no es un error del modelo.

%% Referencia parabola r(k) = 0.5*(k*T)^2 (coder-ex3), 300 muestras (pinned
% por el test de referencia de coder-ex3-2)
Nsim = 300;
t = (0:Nsim - 1)' * T;
r = 0.5 * t.^2;
simin = [t, r]; %#ok<NASGU> % usado por el bloque From Workspace

try
    modelName = 'sistema3_deadbeat_sim';
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
    new_system(modelName);
    open_system(modelName);

    add_block('simulink/Sources/From Workspace', [modelName '/Parabola'], ...
        'VariableName', 'simin', 'Position', [30, 100, 90, 130]);
    add_block('simulink/Math Operations/Sum', [modelName '/Sum'], ...
        'Inputs', '+-', 'Position', [140, 95, 160, 135]);
    add_block('simulink/Discrete/Discrete Transfer Fcn', [modelName '/Deadbeat'], ...
        'Numerator', mat2str(N), 'Denominator', mat2str(D), 'SampleTime', num2str(T), ...
        'Position', [210, 90, 300, 140]);
    add_block('simulink/Discrete/Discrete Transfer Fcn', [modelName '/Planta_Discreta'], ...
        'Numerator', mat2str(numG), 'Denominator', mat2str(denG), 'SampleTime', num2str(T), ...
        'Position', [350, 90, 440, 140]);
    add_block('simulink/Sinks/Scope', [modelName '/Scope'], 'Position', [490, 100, 520, 130]);

    add_line(modelName, 'Parabola/1', 'Sum/1');
    add_line(modelName, 'Sum/1', 'Deadbeat/1');
    add_line(modelName, 'Deadbeat/1', 'Planta_Discreta/1');
    add_line(modelName, 'Planta_Discreta/1', 'Scope/1');
    add_line(modelName, 'Planta_Discreta/1', 'Sum/2', 'autorouting', 'on');

    set_param(modelName, 'StopTime', num2str(t(end)));
    save_system(modelName, fullfile(fileparts(mfilename('fullpath')), [modelName '.slx']));
    sim(modelName);

    lazoCerrado = feedback(tf(N, D, T) * tf(numG, denG, T), 1);
    p = pole(lazoCerrado);
    fprintf('Sistema 3 deadbeat: max|pole|=%.6f (esperado ~0.0706, coder-ex3-2)\n', max(abs(p)));
    disp('Polos lazo cerrado deadbeat (sistema 3):'); disp(p);
catch err
    warning('Simulink no disponible o fallo la construccion del modelo: %s', err.message);
end
