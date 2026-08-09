%% POST-PROCESAMIENTO: Genera figuras de respuesta a partir del diseño real
clear all; clc; close all;

outdir = fullfile(pwd, 'figures');
if ~exist(outdir, 'dir'); mkdir(outdir); end

%% Parámetros físicos (idénticos a simulation.m)
Mb = 0.7; Lb = 0.1555; Mr = 0.375; L = 0.25; Ir = 6.231e-4; Ib = 0.0247; g = 9.81;
Km = 202.5; pm = 24.93;
a = Ib + Mr*L^2 + 2*Ir;
b = (Mb*Lb + Mr*L)*g;

matA = zeros(3,3);
matA(1,2) = 1; matA(2,1) = b/a; matA(2,3) = -(2*Ir*pm)/a; matA(3,3) = -pm;
matB = zeros(3,1); matB(2,1) = -(2*Ir*Km)/a; matB(3,1) = Km;
matC = [1,0,0]; matD = 0;
dim = 3; dim_aug = 4;

syms s real

function [K, Ki, poles_out] = disena_controlador(matA, matB, matC, matD, dim_aug, zita_d, ts_d, s)
    wn_d = 4/(zita_d*ts_d);
    pol_dom = s^2 + 2*zita_d*wn_d*s + wn_d^2;
    polo_rapido = 10*(zita_d*wn_d);
    pol_nd = s + polo_rapido;
    pol_des = pol_dom;
    for k = 1:(dim_aug-2), pol_des = pol_des*pol_nd; end
    pol_des = vpa(expand(pol_des),5);
    coef_pd = double(coeffs(pol_des, s, 'All'));
    poles_out = double(solve(pol_des==0,s));

    A_aug = [matA, zeros(3,1); -matC, 0];
    B_aug = [matB; -matD];
    vec_en = [0 0 0 1];
    K_ack = vec_en*inv(ctrb(A_aug,B_aug))*polyvalm(coef_pd, A_aug);
    K = K_ack(1:end-1);
    Ki = -K_ack(end);
end

%% Diseño 1: subamortiguado (zeta = 0.6, ts = 2.0 s)  -- igual que simulation.m
[K1, Ki1, p1] = disena_controlador(matA, matB, matC, matD, dim_aug, 0.6, 2.0, s);

%% Diseño 2: críticamente amortiguado (zeta = 1.0, ts = 2.0 s) -- mismo método, otro zeta
[K2, Ki2, p2] = disena_controlador(matA, matB, matC, matD, dim_aug, 1.0, 2.0, s);

fprintf('K1 (subamortiguado, zeta=0.6) = [%g %g %g], Ki1 = %g\n', K1, Ki1);
fprintf('K2 (critico, zeta=1.0)        = [%g %g %g], Ki2 = %g\n', K2, Ki2);

%% Sistema en lazo cerrado con acción integral: xdot=[x;xi], r escalón
function sys_cl = lazo_cerrado(matA, matB, matC, K, Ki)
    A_cl = [matA - matB*K, matB*Ki; -matC, 0];
    B_cl = [zeros(3,1); 1];
    C_cl = [matC, 0];
    sys_cl = ss(A_cl, B_cl, C_cl, 0);
end

sys1 = lazo_cerrado(matA, matB, matC, K1, Ki1);
sys2 = lazo_cerrado(matA, matB, matC, K2, Ki2);

t = 0:0.001:3;
r = 0.1*ones(size(t)); % referencia: 0.1 rad
[y1,~,x1] = lsim(sys1, r, t);
[y2,~,x2] = lsim(sys2, r, t);

f1 = figure('Visible','off');
plot(t,y1,'b-','LineWidth',1.5); hold on; plot(t,y2,'r--','LineWidth',1.5);
yline(0.1,'k:'); grid on;
xlabel('Tiempo (s)'); ylabel('\theta_1 (rad)');
legend('Subamortiguado (\zeta=0.6)','Criticamente amortiguado (\zeta=1.0)','Referencia','Location','southeast');
title('Respuesta del sistema controlado ante referencia escalon');
saveas(f1, fullfile(outdir,'respuesta_controlador.png'));

%% Observador: dinámica del error e = x - xhat, con L_std_m1
% Polos del observador (10x más rápido, zeta=0.6, ts=0.2s)
zita_do = 0.6; ts_do = 0.2;
wn_do = 4/(zita_do*ts_do);
pol_dom_o = s^2 + 2*zita_do*wn_do*s + wn_do^2;
polo_rapido_o = 10*(zita_do*wn_do);
pol_nd_o = s + polo_rapido_o;
pol_des_o = expand(pol_dom_o*pol_nd_o);
coef_pdo = double(coeffs(vpa(pol_des_o,5), s, 'All'));

vec_en_obs = [0;0;1];
L1 = polyvalm(coef_pdo, matA) * inv(obsv(matA, matC)) * vec_en_obs;

A_err = matA - L1*matC;
sys_err = ss(A_err, zeros(3,1), eye(3), 0);
e0 = [0.05; 0.02; 0.01]; % error inicial de estimación
[~, ~, e] = initial(sys_err, e0, t);

f2 = figure('Visible','off');
plot(t, e, 'LineWidth', 1.3); grid on;
xlabel('Tiempo (s)'); ylabel('Error de estimacion');
legend('e_1 = \theta_1-\theta_1^{est}','e_2 = \omega_1-\omega_1^{est}','e_3 = \omega_r-\omega_r^{est}');
title('Convergencia del error del observador de estados');
saveas(f2, fullfile(outdir,'respuesta_observador.png'));

%% Servosistema: seguimiento de referencia tipo escalón (usa sys1, diseño subamortiguado)
f3 = figure('Visible','off');
plot(t, y1, 'b-', 'LineWidth', 1.5); hold on; yline(0.1,'k--');
grid on; xlabel('Tiempo (s)'); ylabel('\theta_1 (rad)');
legend('Salida del servosistema','Referencia r=0.1 rad','Location','southeast');
title('Respuesta del servosistema con compensador integrante');
saveas(f3, fullfile(outdir,'respuesta_servosistema.png'));

%% Guardar tabla resumen
fid = fopen(fullfile(outdir,'resumen_ganancias.txt'),'w');
fprintf(fid, 'K1=[%g %g %g] Ki1=%g\n', K1, Ki1);
fprintf(fid, 'K2=[%g %g %g] Ki2=%g\n', K2, Ki2);
fprintf(fid, 'L1=[%g %g %g]\n', L1);
fclose(fid);

fprintf('Listo. Figuras guardadas en %s\n', outdir);
