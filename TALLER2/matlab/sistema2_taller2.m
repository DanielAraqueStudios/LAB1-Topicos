%% Sistema 2 - Taller 2: circuito RLC con resistencia no lineal (linealizado)
% Estado=[i_L, v_C], salida=e_o. Punto de operacion I0=1A, R_NL=6*I0^2=6 ohm.
clear; clc; close all;

%% 1. Planta en tiempo continuo
A = [-3, -0.5; 2, -1];   % L=2,R=2,C=0.5,R_NL=6 -> [-R_NL/L,-1/L; 1/C,-1/(R*C)]
B = [0.5; 0];
C = [6, 0];
D = 0;
sysC = ss(A, B, C, D);
Gs = tf(sysC);
disp('G(s) sistema 2:'); Gs

%% 2. Discretizacion
T = 0.1; % wn=2 rad/s (zeta=1, polos repetidos en -2), wn*T~0.2 -> T=0.1s (coder-core)
sysd = c2d(sysC, T, 'zoh');
disp('G(z) sistema 2:'); tf(sysd)
% Referencia (coder-core, discrete_tools.c2d_zoh): Ad=[0.736858,-0.040937;0.163746,0.900604]
% Bd=[0.043127;0.004381], Cd=[6,0], Dd=0 - debe coincidir con sysd salvo redondeo.

%% 3. Respuesta en lazo abierto
figure; step(sysC); title('Sistema 2 - Lazo abierto - respuesta al escalon');
S = stepinfo(sysC);
fprintf('Tiempo de asentamiento lazo abierto: %.4f s\n', S.SettlingTime);

%% 4. Compensador digital (design_type_compensator, criterio de angulo con Tipo
% garantizado, docs/taller2_puntos_2_4.tex sec. 2.4, coder-ex3). Correccion:
% un diseno con wn=4 solo ubicaba el par dominante pero dejaba raices no
% dominantes fuera del circulo unitario para TODO angulo/seccion probado
% (verificado con jury_stability sobre el polinomio caracteristico completo,
% no solo z_d); wn=2 (mismo zeta=0.7) si es estable. Tipo 2 (rampa): D(z)G(z)
% debe aportar los dos integradores 1/(z-1)^2 (planta es Tipo 0), asi que el
% criterio de angulo se aplica sobre la planta EXTENDIDA G(z)/(z-1)^2 y el
% resultado se multiplica por 1/(z-1)^2 al final.
% z_d=0.860506+j0.123747, alpha=50.140deg, theta=129.860deg, n=2 secciones
% (cero=0.860506, polo=0.595972 c/u), K_c=0.097148.
% C(z) = 0.097148*(z-0.860506)^2 / [(z-0.595972)^2*(z-1)^2]
num_c = [0.097148, -0.167193, 0.071935];
den_c = [1, -3.191944, 3.739072, -1.902310, 0.355183];
Cz = tf(num_c, den_c, T);
lazoCerradoComp = feedback(Cz * sysd, 1);
figure; step(lazoCerradoComp); title('Sistema 2 - Lazo cerrado - Compensador digital');
disp('Polos lazo cerrado compensador (sistema 2):'); pole(lazoCerradoComp)
% Referencia (coder-ex3, verificado con jury_stability, grado 6):
% 0.939+/-j0.100, 0.860+/-j0.124(=z_d), 0.840, 0.391; error de rampa ~0 (400 muestras)

%% 5. Controlador deadbeat (raiz doble impuesta en p=0.935, ver docs/2.5;
% minimos cuadrados de design_deadbeat con integrators=2 da raices inestables,
% 1.322+/-2.184j -- se usa la solucion estable de raiz doble en p en su lugar)
% D(z) = (0.34043 z - 0.32560) / (z-1)^2
numDB = [0.34043, -0.32560];
denDB = [1, -2, 1];
Cdb = tf(numDB, denDB, T);
lazoCerradoDB = feedback(Cdb * sysd, 1);
figure; step(lazoCerradoDB); title('Sistema 2 - Lazo cerrado - Deadbeat');
disp('Polos lazo cerrado deadbeat (sistema 2):'); pole(lazoCerradoDB)
% Referencia (docs 2.5): raices esperadas {0.884+/-j0.270, 0.935 (doble)}

%% 6. Realimentacion de estados + observador + servo tipo-2 (seguimiento de rampa)
Ad = sysd.A; Bd = sysd.B; Cd = sysd.C; Dd = sysd.D;
n = size(Ad, 1);

% Forma "-CG" del servo sistema tipo-2 (Ogata / augment_for_tracking, coder-core,
% order=2 para seguir rampa): estado aumentado = [x; xi1; xi2].
CGd = Cd * Ad;
CHd = Cd * Bd;
Ghat = [Ad, zeros(n, 2); -CGd, 1, 1; -CGd, 0, 1];
Hhat = [Bd; -CHd; -CHd];

% Polos deseados lazo cerrado: zeta=0.7, wn=4 rad/s -> z=0.725157+/-j0.212971;
% dos polos extra de servo en z=0.3 y z=0.25 (coder-core).
polos_lc = [0.725157 + 0.212971i, 0.725157 - 0.212971i, 0.3, 0.25];
Khat = place(Ghat, Hhat, polos_lc);
disp('Polos lazo cerrado servo (verificacion, sistema 2):'); eig(Ghat - Hhat * Khat)

% Polos observador: tsdo=tsd/10 -> z=-0.058357+/-j0.017098 (coder-core).
polos_obs = [-0.058357 + 0.017098i, -0.058357 - 0.017098i];
L = place(Ad', Cd', polos_obs)';

Mi = [1, 1; 0, 1];
Bri = [0; 1];

% Simulacion con referencia tipo rampa (servo tipo-2)
Nsim = 100;
t = (0:Nsim - 1)' * T;
r = t;

x = zeros(n, 1); xhat = zeros(n, 1); xi = zeros(2, 1);
X = zeros(Nsim, n); Xhat = zeros(Nsim, n); Y = zeros(Nsim, 1);
for k = 1:Nsim
    y = Cd * x + Dd;
    u = -Khat * [xhat; xi];
    X(k, :) = x'; Xhat(k, :) = xhat'; Y(k) = y;
    comun = -CGd * xhat - CHd * u;
    xi = Mi * xi + ones(2, 1) * comun + Bri * r(k);
    xhat_next = Ad * xhat + Bd * u + L * (y - Cd * xhat);
    x = Ad * x + Bd * u;
    xhat = xhat_next;
end

figure; plot(t, Y, t, r, '--'); title('Sistema 2 - Servo tipo-2 + observador - seguimiento de rampa');
xlabel('t [s]'); ylabel('e_o'); legend('salida', 'referencia');

figure; plot(t, X - Xhat); title('Sistema 2 - Error de observacion x - xhat');
xlabel('t [s]'); legend('e_1', 'e_2');

%% 7. Simulink (auto-generado)
try
    modelName = 'sistema2_taller2_model';
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
