%% Sistema 4 (dos tanques neumaticos) - Realimentacion de estados + observador
% (servo tipo-1, seguimiento de escalon) - modelo Simulink.
% Se modela la dinamica de lazo cerrado aumentada (Ghat - Hhat*Khat) como un
% unico bloque Discrete State-Space; Bref inyecta r(k) en el unico estado de
% seguimiento xi (coder-ex3: "sin cambios, unaffected by the bug").
clear; clc; close all;

T = 0.1;

%% Estados aumentados (4.6, coder-ex3: valores no recalculados, reutiliza
% place() de sistema4_taller2.m ya que la planta y el diseno no cambiaron)
% orden de estado: [x1, x2, xi]
Ad = [0.8629, 0.0889; 0.0444, 0.9185];
Bd = [0.04705; 0.01313];
Cd = [0, 0.5];
n = size(Ad, 1);
CGd = Cd * Ad; CHd = Cd * Bd;
Ghat = [Ad, zeros(n, 1); -CGd, 1];
Hhat = [Bd; -CHd];
Chat = [Cd, 0];

% Polos deseados: zeta=0.8, wn=1.5 -> z=0.883331+/-j0.079715; extra servo z=0.4
polos_lc = [0.883331 + 0.079715i, 0.883331 - 0.079715i, 0.4];
Khat = place(Ghat, Hhat, polos_lc);

% Observador: z_obs=0.187225+/-j0.235934
polos_obs = [0.187225 + 0.235934i, 0.187225 - 0.235934i];
L = place(Ad', Cd', polos_obs)';

Acl = Ghat - Hhat * Khat;
Bref = [0; 0; 1]; % r(k) entra solo en xi (servo tipo-1)

disp('Polos lazo cerrado servo (verificacion, sistema 4):'); disp(eig(Acl));

try
    modelName = 'sistema4_estados_sim';
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
    new_system(modelName);
    open_system(modelName);

    add_block('simulink/Sources/Step', [modelName '/Step'], ...
        'Time', '0', 'Before', '0', 'After', '1', 'Position', [30, 100, 60, 130]);
    add_block('simulink/Discrete/Discrete State-Space', [modelName '/LazoCerrado_Aumentado'], ...
        'A', mat2str(Acl), 'B', mat2str(Bref), 'C', mat2str(Chat), 'D', '0', ...
        'SampleTime', num2str(T), 'Position', [140, 85, 280, 145]);
    add_block('simulink/Sinks/Scope', [modelName '/Scope'], 'Position', [330, 100, 360, 130]);

    add_line(modelName, 'Step/1', 'LazoCerrado_Aumentado/1');
    add_line(modelName, 'LazoCerrado_Aumentado/1', 'Scope/1');

    set_param(modelName, 'StopTime', '10');
    save_system(modelName, fullfile(fileparts(mfilename('fullpath')), [modelName '.slx']));
    sim(modelName);

    fprintf('Sistema 4 estados+observador: max|eig(Acl)|=%.6f\n', max(abs(eig(Acl))));
    disp('Polos observador (info):'); disp(eig(Ad - L * Cd));
catch err
    warning('Simulink no disponible o fallo la construccion del modelo: %s', err.message);
end
