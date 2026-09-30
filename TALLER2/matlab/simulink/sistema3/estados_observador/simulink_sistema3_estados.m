%% Sistema 3 (termico) - Realimentacion de estados + observador (servo
% tipo-3, seguimiento de parabola) - modelo Simulink.
% Se modela la dinamica de lazo cerrado aumentada (Ghat - Hhat*Khat) como un
% unico bloque Discrete State-Space. Estado = [x1, x2, v, w, z] (cascada de
% seguimiento orden 3, coder-ex3); Bref inyecta la referencia solo en el
% ultimo estado (z).
%
% IMPORTANTE (coder-ex3, verificado numericamente): se debe inyectar r(k+1),
% NO r(k), o queda un residuo de error distinto de cero. Como la parabola es
% una senal de referencia externa conocida (no realimentada), esto se logra
% generando la senal de la fuente ya adelantada una muestra: el bloque
% "From Workspace" recibe simin(k) = r(k+1) = 0.5*((k+1)*T)^2 alineado al
% mismo vector de tiempo t(k)=k*T que usa el bloque de estado discreto.
clear; clc; close all;

T = 0.1;

%% Estados aumentados (3.6, coder-ex3)
Ad = [0.909218, 0.086250; 0.086250, 0.822968];
Bd = [0.095314; 0.004532];
Cd = [1, -1];

Ghat = [0.909218, 0.086250, 0, 0, 0; ...
        0.086250, 0.822968, 0, 0, 0; ...
        -0.822968, 0.736718, 1, 1, 1; ...
        -0.822968, 0.736718, 0, 1, 1; ...
        -0.822968, 0.736718, 0, 0, 1];
Hhat = [0.095314; 0.004532; -0.090782; -0.090782; -0.090782];
Chat = [1, -1, 0, 0, 0];

% Khat verificado (coder-ex3): dominante zeta=0.7,wn=2.0 -> z=0.860506+/-j0.123747;
% extra tracking z=0.3,0.25,0.2.
Khat = [-25.245, -658.410, -1.692, 2.506, -85.094];

% Observador (coder-ex3): z_obs=0.035024+/-j0.244097
L = [11.300; 9.638];

Acl = Ghat - Hhat * Khat;
Bref = [0; 0; 0; 0; 1]; % r entra solo en el ultimo estado de seguimiento (z)

disp('Polos lazo cerrado servo (verificacion, sistema 3):'); disp(eig(Acl));

%% Referencia parabola adelantada una muestra: simin(k) = r(k+1) (ver nota arriba)
Nsim = 400;
t = (0:Nsim - 1)' * T;
r_shift = 0.5 * (t + T).^2;
simin = [t, r_shift]; %#ok<NASGU> % usado por el bloque From Workspace

try
    modelName = 'sistema3_estados_sim';
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
    new_system(modelName);
    open_system(modelName);

    add_block('simulink/Sources/From Workspace', [modelName '/Parabola_adelantada'], ...
        'VariableName', 'simin', 'Position', [30, 100, 90, 130]);
    add_block('simulink/Discrete/Discrete State-Space', [modelName '/LazoCerrado_Aumentado'], ...
        'A', mat2str(Acl), 'B', mat2str(Bref), 'C', mat2str(Chat), 'D', '0', ...
        'SampleTime', num2str(T), 'Position', [150, 85, 290, 145]);
    add_block('simulink/Sinks/Scope', [modelName '/Scope'], 'Position', [340, 100, 370, 130]);

    add_line(modelName, 'Parabola_adelantada/1', 'LazoCerrado_Aumentado/1');
    add_line(modelName, 'LazoCerrado_Aumentado/1', 'Scope/1');

    set_param(modelName, 'StopTime', num2str(t(end)));
    save_system(modelName, fullfile(fileparts(mfilename('fullpath')), [modelName '.slx']));
    sim(modelName);

    fprintf('Sistema 3 estados+observador: max|eig(Acl)|=%.6f\n', max(abs(eig(Acl))));
    disp('Polos observador (info):'); disp(eig(Ad - L * Cd));
catch err
    warning('Simulink no disponible o fallo la construccion del modelo: %s', err.message);
end
