%% Sistema 4 - Taller 2: proceso de presion neumatica de dos tanques (linealizado)
% Estado=[p1,p2], entrada=p_i, salida=w_o=p2/R4. R1=2,R2=4,R3=1,R4=2,C1=1,C2=2.
clear; clc; close all;

%% 1. Planta en tiempo continuo
A = [-1.5, 1; 0.5, -0.875];   % -(1/R1+1/R3)/C1,(1/R3)/C1; (1/R3)/C2,-(1/R2+1/R3+1/R4)/C2
B = [0.5; 0.125];
C = [0, 0.5];
D = 0;
sysC = ss(A, B, C, D);
Gs = tf(sysC);
disp('G(s) sistema 4:'); Gs

%% 2. Discretizacion
T = 0.1; % modo rapido wn=1.9606 rad/s, wn*T~0.196 (coder-core; modo lento wn*T~0.041)
sysd = c2d(sysC, T, 'zoh');
disp('G(z) sistema 4:'); tf(sysd)
% Referencia (coder-core, discrete_tools.c2d_zoh): Ad=[0.862906,0.088891;0.044446,0.918463]
% Bd=[0.047046;0.013134], Cd=[0,0.5], Dd=0 - debe coincidir con sysd salvo redondeo.

%% 3. Respuesta en lazo abierto
figure; step(sysC); title('Sistema 4 - Lazo abierto - respuesta al escalon');
S = stepinfo(sysC);
fprintf('Tiempo de asentamiento lazo abierto: %.4f s\n', S.SettlingTime);

%% 4. Compensador digital (design_type_compensator, criterio de angulo con Tipo
% garantizado, docs/taller2_puntos_2_4.tex sec. 4.4, coder-ex3). Correccion:
% un diseno que solo ubicaba el par dominante en z_d (sin polo en z=1)
% reportaba erroneamente error nulo; verificado por simulacion directa, el
% error de escalon real se estabiliza en 0.243 porque D(z)G(z) nunca llega a
% Tipo 1. Tipo 1 (escalon): planta es Tipo 0, el criterio de angulo se aplica
% sobre la planta EXTENDIDA G(z)/(z-1) y el resultado se multiplica por
% 1/(z-1) al final; jury_stability confirma el polinomio caracteristico
% completo antes de aceptar el diseno.
% z_d=0.883331+j0.079715, alpha=52.202deg, theta=127.798deg, n=2 secciones
% (cero=0.883331, polo=0.720621 c/u), K_c=6.270181.
% C(z) = 6.270181*(z-0.883331)^2 / [(z-0.720621)^2*(z-1)]
num_c = [6.270181, -11.077288, 4.892455];
den_c = [1, -2.441241, 1.960535, -0.519294];
Cz = tf(num_c, den_c, T);
lazoCerradoComp = feedback(Cz * sysd, 1);
figure; step(lazoCerradoComp); title('Sistema 4 - Lazo cerrado - Compensador digital');
disp('Polos lazo cerrado compensador (sistema 4):'); pole(lazoCerradoComp)
% Referencia (coder-ex3, verificado con jury_stability, grado 5):
% 0.875+/-j0.113, 0.883+/-j0.080(=z_d), 0.705; error de escalon ~0 (400 muestras)

%% 5. Controlador deadbeat (raiz doble impuesta en p=0, ver docs/4.5;
% minimos cuadrados de design_deadbeat da los mismos coeficientes pero con
% signo tal que resulta inestable en z=4.308 -- se usa la solucion de raiz
% doble verificada estable en su lugar)
% D(z) = (313.619 z - 170.644) / (z-1)
numDB = [313.619, -170.644];
denDB = [1, -1];
Cdb = tf(numDB, denDB, T);
lazoCerradoDB = feedback(Cdb * sysd, 1);
figure; step(lazoCerradoDB); title('Sistema 4 - Lazo cerrado - Deadbeat');
disp('Polos lazo cerrado deadbeat (sistema 4):'); pole(lazoCerradoDB)
% Referencia (docs 4.5): raices esperadas {0, 0, 0.7218}

%% 6. Realimentacion de estados + observador + servo tipo-1 (seguimiento de escalon)
Ad = sysd.A; Bd = sysd.B; Cd = sysd.C; Dd = sysd.D;
n = size(Ad, 1);

% Forma "-CG" del servo sistema tipo-1 (Ogata / augment_for_tracking, coder-core,
% order=1 para seguir escalon): estado aumentado = [x; xi].
CGd = Cd * Ad;
CHd = Cd * Bd;
Ghat = [Ad, zeros(n, 1); -CGd, 1];
Hhat = [Bd; -CHd];

% Polos deseados lazo cerrado: zeta=0.8, wn=1.5 rad/s -> z=0.883331+/-j0.079715;
% polo extra de servo en z=0.4 (coder-core).
polos_lc = [0.883331 + 0.079715i, 0.883331 - 0.079715i, 0.4];
Khat = place(Ghat, Hhat, polos_lc);
disp('Polos lazo cerrado servo (verificacion, sistema 4):'); eig(Ghat - Hhat * Khat)

% Polos observador: tsdo=tsd/10 -> z=0.187225+/-j0.235934 (coder-core).
polos_obs = [0.187225 + 0.235934i, 0.187225 - 0.235934i];
L = place(Ad', Cd', polos_obs)';

Nsim = 100;
t = (0:Nsim - 1)' * T;
r = ones(Nsim, 1);

x = zeros(n, 1); xhat = zeros(n, 1); xi = 0;
X = zeros(Nsim, n); Xhat = zeros(Nsim, n); Y = zeros(Nsim, 1);
for k = 1:Nsim
    y = Cd * x + Dd;
    u = -Khat * [xhat; xi];
    X(k, :) = x'; Xhat(k, :) = xhat'; Y(k) = y;
    xi = xi - CGd * xhat - CHd * u + r(k);
    xhat_next = Ad * xhat + Bd * u + L * (y - Cd * xhat);
    x = Ad * x + Bd * u;
    xhat = xhat_next;
end

figure; plot(t, Y, t, r, '--'); title('Sistema 4 - Servo tipo-1 + observador - seguimiento de escalon');
xlabel('t [s]'); ylabel('w_o'); legend('salida', 'referencia');

figure; plot(t, X - Xhat); title('Sistema 4 - Error de observacion x - xhat');
xlabel('t [s]'); legend('e_1', 'e_2');

%% 7. Simulink (auto-generado)
try
    modelName = 'sistema4_taller2_model';
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
    new_system(modelName);
    open_system(modelName);

    add_block('simulink/Sources/Step', [modelName '/Step'], 'Position', [30, 100, 60, 130]);
    add_block('simulink/Math Operations/Sum', [modelName '/Sum'], 'Inputs', '+-', 'Position', [110, 95, 130, 135]);
    add_block('simulink/Discrete/Discrete Transfer Fcn', [modelName '/Controlador'], ...
        'Numerator', mat2str(num_c), 'Denominator', mat2str(den_c), 'SampleTime', num2str(T), ...
        'Position', [180, 90, 260, 140]);
    add_block('simulink/Discrete/Discrete State-Space', [modelName '/Planta_Discreta'], ...
        'A', mat2str(Ad), 'B', mat2str(Bd), 'C', mat2str(Cd), 'D', mat2str(Dd), 'SampleTime', num2str(T), ...
        'Position', [310, 85, 400, 145]);
    add_block('simulink/Sinks/Scope', [modelName '/Scope'], 'Position', [450, 100, 480, 130]);

    add_line(modelName, 'Step/1', 'Sum/1');
    add_line(modelName, 'Sum/1', 'Controlador/1');
    add_line(modelName, 'Controlador/1', 'Planta_Discreta/1');
    add_line(modelName, 'Planta_Discreta/1', 'Scope/1');
    add_line(modelName, 'Planta_Discreta/1', 'Sum/2', 'autorouting', 'on');

    save_system(modelName);
    sim(modelName);
catch err
    warning('Simulink no disponible o fallo la construccion del modelo: %s', err.message);
end
