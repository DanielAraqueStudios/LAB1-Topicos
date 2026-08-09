%% 1. PARÁMETROS DEL SISTEMA Y PUNTOS DE OPERACIÓN (PÉNDULO CON RUEDA DE REACCIÓN)
clear all; clc;

% Parámetros físicos (Masas en kg, Longitudes en m, Inercias en kg*m^2)
Mb = 0.7;       
Lb = 0.1555;    
Mr = 0.375;     
L  = 0.25;      
Ir = 6.231e-4;  
Ib = 0.0247;    
g  = 9.81;      

% Parámetros del motor
Km = 202.5;     
pm = 24.93;     

% Constantes agrupadas de simplificación
a = Ib + Mr*L^2 + 2*Ir;
b = (Mb*Lb + Mr*L)*g;

% Puntos de Operación (Péndulo en la vertical)
ang_op = 0; 
S_ang = sin(ang_op); C_ang = cos(ang_op);

fprintf('--- EQUILIBRIO PÉNDULO CON RUEDA DE REACCIÓN ---\nÁngulo de Operación: %.4f rad (Vertical)\n\n', ang_op);

%% 2. MATRICES DEL ESPACIO DE ESTADOS (ESTÁNDAR)
% REDUCCIÓN A 3 ESTADOS: [theta, omega_p, omega_r]
% Se elimina 'phi' porque es inobservable y no afecta la dinámica de estabilización.
matA = zeros(3,3);
matA(1,2) = 1;         
matA(2,1) = (b/a)*C_ang; 
matA(2,3) = -(2*Ir*pm)/a;
matA(3,3) = -pm;

matB = zeros(3,1);
matB(2,1) = -(2*Ir*Km)/a;      
matB(3,1) = Km;       

matE = zeros(3,1); 
matC = [1, 0, 0]; 
matD = 0;
matF = 0;

%% 3. FORMAS CANÓNICAS (FCC y FCO)
dim = length(matA);
M_ctrl = ctrb(matA, matB)
M_obsv = obsv(matA, matC)

% Matriz de Toeplitz
coef_carac = poly(matA);
a_vec = coef_carac(2:end);
M_toep = zeros(dim, dim);
for i = 1:dim
    for j = 1:dim
        k = (dim + 1) - (i + j);
        if k == 0, M_toep(i,j) = 1;
        elseif k > 0, M_toep(i,j) = a_vec(k);
        end
    end
end

% Transformaciones
T_cc = M_ctrl * M_toep    
T_cc_inv = inv(T_cc)
T_co = inv(M_toep * M_obsv)
T_co_inv = inv(T_co)

% Matrices FCC
A_cc = T_cc_inv*matA*T_cc  
B_cc = T_cc_inv*matB  
C_cc = matC*T_cc
E_cc = T_cc_inv * matE;         
D_cc = matD;                    
F_cc = matF;

% Matrices FCO
A_co = T_co_inv*matA*T_co  
B_co = T_co_inv*matB 
C_co = matC*T_co
E_co = T_co_inv * matE;         
D_co = matD;                    
F_co = matF;                    

%% -------------------------------------------------------------------------
%% 4. DISEÑO DE CONTROLADOR (CON INTEGRADOR)
%% -------------------------------------------------------------------------
syms s real
dim_aug = dim + 1;
zita_d = 0.6;  ts_d = 2.0;
wn_d = 4 / (zita_d * ts_d);

% Polos Dominantes y Rápidos
pol_dom = s^2 + 2*zita_d*wn_d*s + wn_d^2;
polo_rapido = 10 * (zita_d * wn_d);
pol_nd = s + polo_rapido;

% Polinomio total deseado (Controlador)
pol_des_sym = pol_dom;
for k = 1:(dim_aug - 2), pol_des_sym = pol_des_sym * pol_nd; end
pol_des_sym = vpa(expand(pol_des_sym), 5);
coef_pd = double(coeffs(pol_des_sym, s, 'All'))

% --- MÉTODO 1 (ASIGNACIÓN DIRECTA) ---
syms Ki_sym real; K_vec_sym = sym('K', [1 dim], 'real');
A_aug_std = [(matA - matB*K_vec_sym), matB*Ki_sym; (-matC + matD*K_vec_sym), -matD*Ki_sym];
res_std = solve(coeffs(det(eye(dim_aug)*s - A_aug_std) - pol_des_sym, s), [K_vec_sym, Ki_sym], 'Real', true);
K_std_m1 = double(subs(K_vec_sym, res_std))
Ki_std_m1 = double(res_std.Ki_sym)

A_aug_cc = [(A_cc - B_cc*K_vec_sym), B_cc*Ki_sym; (-C_cc + matD*K_vec_sym), -matD*Ki_sym];
res_cc = solve(coeffs(det(eye(dim_aug)*s - A_aug_cc) - pol_des_sym, s), [K_vec_sym, Ki_sym], 'Real', true);
K_cc_m1 = double(subs(K_vec_sym, res_cc))
Ki_cc_m1 = double(res_cc.Ki_sym)

A_aug_co = [(A_co - B_co*K_vec_sym), B_co*Ki_sym; (-C_co + matD*K_vec_sym), -matD*Ki_sym];
res_co = solve(coeffs(det(eye(dim_aug)*s - A_aug_co) - pol_des_sym, s), [K_vec_sym, Ki_sym], 'Real', true);
K_co_m1 = double(subs(K_vec_sym, res_co)) 
Ki_co_m1 = double(res_co.Ki_sym)

% --- MÉTODO 2 (MATRIZ DE PESOS T) ---
calc_pesos = @(A, B, C, D) deal([A, zeros(length(A),1); -C, 0], [B; -D]);
[A_aug_T_std, B_aug_T_std] = calc_pesos(matA, matB, matC, matD);
[A_aug_T_cc, B_aug_T_cc]   = calc_pesos(A_cc, B_cc, C_cc, matD);
[A_aug_T_co, B_aug_T_co]   = calc_pesos(A_co, B_co, C_co, matD);

[K_std_m2, Ki_std_m2] = metodo_T(A_aug_T_std, B_aug_T_std, coef_pd, dim_aug)
[K_cc_m2, Ki_cc_m2]   = metodo_T(A_aug_T_cc, B_aug_T_cc, coef_pd, dim_aug)
[K_co_m2, Ki_co_m2]   = metodo_T(A_aug_T_co, B_aug_T_co, coef_pd, dim_aug)

% --- MÉTODO 3 (ACKERMANN) ---
vec_en = [zeros(1, dim_aug - 1), 1];
K_ack_std = vec_en * inv(ctrb(A_aug_T_std, B_aug_T_std)) * polyvalm(coef_pd, A_aug_T_std);
K_std_m3 = K_ack_std(1:end-1) 
Ki_std_m3 = -K_ack_std(end)

K_ack_cc = vec_en * inv(ctrb(A_aug_T_cc, B_aug_T_cc)) * polyvalm(coef_pd, A_aug_T_cc);
K_cc_m3 = K_ack_cc(1:end-1) 
Ki_cc_m3 = -K_ack_cc(end)

K_ack_co = vec_en * inv(ctrb(A_aug_T_co, B_aug_T_co)) * polyvalm(coef_pd, A_aug_T_co);
K_co_m3 = K_ack_co(1:end-1)
Ki_co_m3 = -K_ack_co(end)

%% -------------------------------------------------------------------------
%% 5. DISEÑO DE OBSERVADOR DE ESTADOS (10 VECES MÁS RÁPIDO)
%% -------------------------------------------------------------------------
zita_do = zita_d;       
ts_do   = ts_d / 10;    
wn_do   = 4 / (zita_do * ts_do);

% Polos Dominantes y Rápidos Observador
pol_dom_obs = s^2 + 2*zita_do*wn_do*s + wn_do^2;
polo_rapido_obs = 10 * (zita_do * wn_do);
pol_nd_obs = s + polo_rapido_obs;

% Polinomio total deseado (Observador)
pol_des_obs_sym = pol_dom_obs;
for k = 1:(dim - 2), pol_des_obs_sym = pol_des_obs_sym * pol_nd_obs; end
pol_des_obs_sym = vpa(expand(pol_des_obs_sym), 5);
coef_pdo = double(coeffs(pol_des_obs_sym, s, 'All'))

% --- MÉTODO 1 (ASIGNACIÓN DIRECTA) ---
L_vec_sym = sym('L', [dim 1], 'real');
res_obs_std = solve(coeffs(det(eye(dim)*s - (matA - L_vec_sym*matC)) - pol_des_obs_sym, s), L_vec_sym, 'Real', true);
L_std_m1 = double(subs(L_vec_sym, res_obs_std))

res_obs_cc = solve(coeffs(det(eye(dim)*s - (A_cc - L_vec_sym*C_cc)) - pol_des_obs_sym, s), L_vec_sym, 'Real', true);
L_cc_m1 = double(subs(L_vec_sym, res_obs_cc))

res_obs_co = solve(coeffs(det(eye(dim)*s - (A_co - L_vec_sym*C_co)) - pol_des_obs_sym, s), L_vec_sym, 'Real', true);
L_co_m1 = double(subs(L_vec_sym, res_obs_co))

% --- MÉTODO 2 (MATRIZ DE TRANSFORMACIÓN Q) ---
L_std_m2 = metodo_Q_obs(matA, matC, coef_pdo, dim)
L_cc_m2  = metodo_Q_obs(A_cc, C_cc, coef_pdo, dim)
L_co_m2  = metodo_Q_obs(A_co, C_co, coef_pdo, dim)

% --- MÉTODO 3 (ACKERMANN OBSERVADOR) ---
vec_en_obs = [zeros(dim - 1, 1); 1]; 
L_std_m3 = polyvalm(coef_pdo, matA) * inv(obsv(matA, matC)) * vec_en_obs
L_cc_m3  = polyvalm(coef_pdo, A_cc) * inv(obsv(A_cc, C_cc)) * vec_en_obs
L_co_m3  = polyvalm(coef_pdo, A_co) * inv(obsv(A_co, C_co)) * vec_en_obs


%% =========================================================================
%% FUNCIONES LOCALES 
%% =========================================================================

function [K, Ki] = metodo_T(A_aug, B_aug, coef_pd, dim_aug)
    a_vec = poly(A_aug); 
    a_vec = a_vec(2:end);
    
    M_t = zeros(dim_aug, dim_aug);
    for i = 1:dim_aug
        for j = 1:dim_aug
            k = (dim_aug + 1) - (i + j); 
            if k == 0
                M_t(i,j) = 1; 
            elseif k > 0
                M_t(i,j) = a_vec(k); 
            end
        end
    end
    K_total = fliplr(coef_pd(2:end) - a_vec) * inv(ctrb(A_aug, B_aug) * M_t);
    K = K_total(1:end-1); 
    Ki = -K_total(end);
end

function L_out = metodo_Q_obs(A_mat, C_mat, coef_pdo, dim)
    a_vec = poly(A_mat); 
    a_vec = a_vec(2:end);
    
    M_q = zeros(dim, dim);
    for i = 1:dim
        for j = 1:dim
            k = (dim + 1) - (i + j); 
            if k == 0
                M_q(i,j) = 1; 
            elseif k > 0
                M_q(i,j) = a_vec(k); 
            end
        end
    end
    Q_trans = inv(M_q * obsv(A_mat, C_mat));
    L_out = Q_trans * fliplr(coef_pdo(2:end) - a_vec)';
end