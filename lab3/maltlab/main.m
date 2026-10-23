%% =========================================================
%  CONTROL DE CARRITO + ROTACIÓN (CASCADA BASE/PÉNDULO) — MATLAB (v4)
%  Requiere: Control System Toolbox (tf, c2d, ss, place)
%  Firmware: lab3/fimware/main.ino (v4, con péndulo medido)
% =========================================================
%  Cambios respecto a la v3:
%   - Eje lineal: se agrega un 4º tipo de controlador, Lugar de Raíces (z)
%     (compensador de adelanto discreto), además de PID/OscMuertas/Servo.
%   - Eje de rotación: deja de ser un lazo SISO independiente y pasa a
%     una CASCADA (Figura 2 de la guía):
%       * Lazo EXTERNO: referencia de ángulo de base (slider) vs ángulo
%         de base medido (encoder, columna 6 de telemetría). Su salida
%         es una referencia de ángulo de péndulo (acotada).
%       * Lazo INTERNO: esa referencia de péndulo vs ángulo de péndulo
%         medido (potenciómetro, columna 8). Su salida, tras compensar
%         zona muerta, es el PWM final que se envía como V<pwmRot>.
%     Ambos lazos de la cascada usan la MISMA familia de controlador
%     (PID / Lugar de Raíces (z) / Osc. Muertas / Servosistema), elegida
%     en un único selector, pero con parámetros de planta independientes.
%   - La telemetría ahora trae 8 columnas (se agrega Angulo_pendulo_deg).
%   - El ESP32 sigue sin alterar el PWM; toda la inteligencia (zona
%     muerta, rampas, controladores, cascada) está en MATLAB.
%  Antes de usar el carrito real: calibra U_DZ/K (lineal), K/wc (externo
%  rotación) y U_DZ/K (interno péndulo) y escríbelos en los campos de la GUI.
% =========================================================

function ControlMotor_GUI()
    clc; close all;

    %% ── PARÁMETROS FIJOS — LINEAL ───────────────────────────
    COM_PORT     = 'COM3';
    BAUD_RATE    = 115200;
    SAMPLE_TIME  = 0.05;     % s (debe coincidir con el ESP32: 50 ms)
    MAX_DIST_CM  = 23.0;     % referencia máxima (el ESP32 frena a ~23.7)
    U_MAX        = 230;      % PWM físico máximo (compartido por todos los motores)
    MAX_DELTA_UC = 60;       % cambio máximo del comando por muestra
    BANDA_IN     = 0.3;      % cm: entra en banda muerta
    BANDA_OUT    = 0.6;      % cm: sale de la banda muerta
    RAMP_PID     = 2.0;      % cm/s
    RAMP_OD      = 2.0;      % cm/s
    RAMP_SRV     = 1.5;      % cm/s
    RAMP_RLZ     = 1.5;      % cm/s (lugar de raíces en z)
    TAU_MOTOR    = 0.10;     % s, constante de tiempo supuesta del motor
    OD_N         = 4;        % oscilaciones muertas: actualiza cada N muestras
    OD_TIPO      = 'escalon';% 'escalon' o 'rampa'
    OD_GANANCIA  = 0.7;      % <1 da margen ante errores de K (dead-beat es frágil)
    DEF_UDZ      = 110;      % valores por defecto de los campos
    DEF_K        = 0.034;
    DEF_WC       = 2.0;
    SIM_K = 0.040;  SIM_UDZ = 125;  SIM_TAU = 0.12;   % planta simulada

    %% ── PARÁMETROS FIJOS — ROTACIÓN (CASCADA) ───────────────
    MAX_ROT_DEG      = 170.0;    % referencia máxima de ángulo base, en grados
                                   % (coincide con ROT_LIM_SUP_DEG del firmware)
    % TODO: calibrar/ajustar según el rig real. Amplitud máxima que el lazo
    % externo puede pedirle al péndulo como referencia.
    PEND_REF_MAX_DEG = 15.0;

    MAX_DELTA_UC_EXT = 10;       % cambio máx. de la "referencia de péndulo" por muestra (°)
    MAX_DELTA_UC_INT = 60;       % cambio máx. del comando interno (PWM virtual) por muestra
    BANDA_IN_EXT     = 1.0;      % grados (base): entra en banda muerta del lazo externo
    BANDA_OUT_EXT    = 2.0;
    BANDA_IN_INT     = 1.0;      % grados (péndulo): entra en banda muerta del lazo interno
    BANDA_OUT_INT    = 2.0;
    RAMP_PID_ROT     = 25.0;     % deg/s (referencia de ángulo base)
    RAMP_OD_ROT      = 25.0;
    RAMP_SRV_ROT     = 20.0;
    RAMP_RLZ_ROT     = 20.0;
    TAU_MOTOR_EXT    = 0.12;     % s, constante de tiempo supuesta del lazo externo (base)
    TAU_MOTOR_INT    = 0.12;     % s, constante de tiempo supuesta del lazo interno (péndulo)
    OD_N_ROT         = 4;
    OD_TIPO_ROT      = 'escalon';
    OD_GANANCIA_ROT  = 0.7;

    % Externo (base): no hay zona muerta propia, su "ucMax" es la amplitud
    % máxima de la referencia de péndulo que puede generar.
    DEF_K_EXT        = 0.60;     % TODO: deg_pendulo_ref por deg_error_base, calibrar
    DEF_WC_EXT       = 2.0;

    % Interno (péndulo): sí tiene zona muerta, porque su salida ES el PWM
    % final del motor de rotación.
    DEF_UDZ_INT      = 110;      % TODO: calibrar con el motor de rotación real
    DEF_K_INT        = 0.60;     % TODO: deg_pendulo/s por PWM, calibrar
    DEF_WC_INT       = 2.0;

    % Plantas simuladas (modo sin ESP32).
    % Simplificación para probar la GUI sin hardware: el ángulo de base
    % responde directamente al PWM de rotación con la misma estructura que
    % el carro lineal (zona muerta + retardo de primer orden), y el péndulo
    % tiene su propia dinámica ligeramente acoplada, también excitada por
    % el MISMO PWM (en la realidad ambos son accionados por un solo motor
    % de rotación; esto no es un modelo físico riguroso, solo sirve para
    % ejercitar la lógica de la cascada en el escritorio).
    SIM_K_BASE = 0.70;   SIM_UDZ_BASE = 125;  SIM_TAU_BASE = 0.15;
    SIM_K_PEND = 0.35;   SIM_TAU_PEND = 0.25;   % el péndulo no tiene zona muerta propia (no tiene actuador propio)

    %% ── ESTADO ──────────────────────────────────────────────
    estado = struct('corriendo',false,'arduino',[],'timerObj',[], ...
        'controlador','PID','controladorRot','PID', ...
        'distRef',0,'distRefRamp',0,'rotRef',0,'rotRefRamp',0, ...
        'tiempoVec',[],'refVec',[],'distVec',[],'controlVec',[],'errorVec',[],'velVec',[], ...
        'refRotVec',[],'posRotVec',[],'controlRotVec',[],'errorRotVec',[],'velRotVec',[], ...
        'pendRefVec',[],'pendVec',[],'errorPendVec',[],'velPendVec',[], ...
        'modoManual',true,'u_prev_c',0,'u_prev_c_ext',0,'u_prev_c_int',0, ...
        'enBanda',false,'enBandaExt',false,'enBandaInt',false, ...
        'sinDatos',0,'sinDatosRot',0, ...
        'alarma','','tick',0);

    P      = struct('Udz',DEF_UDZ,'K',DEF_K,'wc',DEF_WC,'ucMax',U_MAX-DEF_UDZ);
    Pext   = struct('K',DEF_K_EXT,'wc',DEF_WC_EXT,'ucMax',PEND_REF_MAX_DEG);   % lazo externo: sin Udz
    Pint   = struct('Udz',DEF_UDZ_INT,'K',DEF_K_INT,'wc',DEF_WC_INT,'ucMax',U_MAX-DEF_UDZ_INT);

    meas    = struct('y',0,'t',0,'vhat',0);          % posición lineal
    measRot = struct('y',0,'vhat',0);                % ángulo de base
    measPen = struct('y',0,'vhat',0,'yPrev',0);       % ángulo de péndulo
    simu    = struct('y',0,'v',0,'up',0,'yf',0,'t',0);
    simuRot = struct('y',0,'v',0,'up',0,'yf',0);
    simuPen = struct('y',0,'v',0,'yf',0);

    pidc = struct('Kp',0,'Ki',0,'Kd',0,'I',0,'Imax',1);
    od   = struct('Cn',1,'Cd',1,'be',0,'bu',zeros(1,0),'cnt',0,'uh',0);
    srv  = struct('Ad',eye(2),'Bd',zeros(2,1),'Cd',[1 0],'Ka',[0 0 0], ...
                  'L',zeros(2,1),'x',[0;0],'xi',0,'ximax',1);
    rlz  = struct('Cn',1,'Cd',1,'be',zeros(1,0),'bu',zeros(1,0));

    % Externo (cascada rotación → referencia de péndulo)
    pidcExt = struct('Kp',0,'Ki',0,'Kd',0,'I',0,'Imax',1);
    odExt   = struct('Cn',1,'Cd',1,'be',0,'bu',zeros(1,0),'cnt',0,'uh',0);
    srvExt  = struct('Ad',eye(2),'Bd',zeros(2,1),'Cd',[1 0],'Ka',[0 0 0], ...
                  'L',zeros(2,1),'x',[0;0],'xi',0,'ximax',1);
    rlzExt  = struct('Cn',1,'Cd',1,'be',zeros(1,0),'bu',zeros(1,0));

    % Interno (cascada rotación → PWM final)
    pidcInt = struct('Kp',0,'Ki',0,'Kd',0,'I',0,'Imax',1);
    odInt   = struct('Cn',1,'Cd',1,'be',0,'bu',zeros(1,0),'cnt',0,'uh',0);
    srvInt  = struct('Ad',eye(2),'Bd',zeros(2,1),'Cd',[1 0],'Ka',[0 0 0], ...
                  'L',zeros(2,1),'x',[0;0],'xi',0,'ximax',1);
    rlzInt  = struct('Cn',1,'Cd',1,'be',zeros(1,0),'bu',zeros(1,0));

    %% ══════════════════════════════════════════════════════════
    %  INTERFAZ
    %% ══════════════════════════════════════════════════════════
    fig = figure('Name','Control de Carrito + Rotación (cascada)','NumberTitle','off', ...
        'MenuBar','none','ToolBar','figure','Color',[0.94 0.94 0.94], ...
        'Position',[40 30 1650 820],'CloseRequestFcn',@cerrarAplicacion);

    BG = [0.16 0.16 0.22];  FG = [0.7 0.7 0.9];

    pControl = uipanel(fig,'Title','Eje Lineal', ...
        'BackgroundColor',BG,'ForegroundColor',[0.85 0.85 1.0], ...
        'FontSize',11,'FontWeight','bold','Units','normalized', ...
        'Position',[0.01 0.01 0.19 0.98]);

    uicontrol(pControl,'Style','text','String','Modo de operación:', ...
        'Units','normalized','Position',[0.04 0.95 0.92 0.03], ...
        'BackgroundColor',BG,'ForegroundColor',FG,'FontSize',9,'HorizontalAlignment','left');
    tglModo = uicontrol(pControl,'Style','togglebutton','String','Simulación (Sin ESP32)', ...
        'Units','normalized','Position',[0.04 0.905 0.92 0.045], ...
        'BackgroundColor',[0.25 0.45 0.65],'ForegroundColor','white', ...
        'FontSize',9,'FontWeight','bold','Callback',@cambiarModo,'Value',1);

    uicontrol(pControl,'Style','text','String','Controlador (lineal):', ...
        'Units','normalized','Position',[0.04 0.865 0.92 0.03], ...
        'BackgroundColor',BG,'ForegroundColor',FG,'FontSize',9,'HorizontalAlignment','left');
    uicontrol(pControl,'Style','popupmenu', ...
        'String',{'PID Discreto','Lugar de Raíces (z)','Oscilaciones Muertas','Servosistema'}, ...
        'Units','normalized','Position',[0.04 0.815 0.92 0.045], ...
        'BackgroundColor',[0.22 0.22 0.30],'ForegroundColor',[0.95 0.95 1.0], ...
        'FontSize',9,'Callback',@cambiarControlador);

    uicontrol(pControl,'Style','text','String','Referencia de posición (cm):', ...
        'Units','normalized','Position',[0.04 0.775 0.92 0.03], ...
        'BackgroundColor',BG,'ForegroundColor',FG,'FontSize',9,'HorizontalAlignment','left');
    lblDist = uicontrol(pControl,'Style','text','String','0.0 cm', ...
        'Units','normalized','Position',[0.64 0.725 0.32 0.045], ...
        'BackgroundColor',BG,'ForegroundColor',[0.4 0.9 0.6],'FontSize',10,'FontWeight','bold');
    sldDist = uicontrol(pControl,'Style','slider','Min',0,'Max',MAX_DIST_CM,'Value',0, ...
        'Units','normalized','Position',[0.04 0.725 0.56 0.045], ...
        'BackgroundColor',[0.28 0.28 0.38],'Callback',@cambiarDistancia);
    uicontrol(pControl,'Style','pushbutton','String','◄', ...
        'Units','normalized','Position',[0.04 0.675 0.13 0.04], ...
        'BackgroundColor',[0.22 0.35 0.50],'ForegroundColor','white','FontSize',10, ...
        'Callback',@(~,~) ajustarSlider(sldDist,-0.5,'lin'));
    uicontrol(pControl,'Style','pushbutton','String','►', ...
        'Units','normalized','Position',[0.19 0.675 0.13 0.04], ...
        'BackgroundColor',[0.22 0.35 0.50],'ForegroundColor','white','FontSize',10, ...
        'Callback',@(~,~) ajustarSlider(sldDist,+0.5,'lin'));
    uicontrol(pControl,'Style','text','String','−0.5 cm      +0.5 cm', ...
        'Units','normalized','Position',[0.35 0.675 0.61 0.035], ...
        'BackgroundColor',BG,'ForegroundColor',[0.5 0.5 0.65],'FontSize',7,'HorizontalAlignment','left');

    uicontrol(pControl,'Style','text','String','Parámetros de planta (lineal):', ...
        'Units','normalized','Position',[0.04 0.635 0.92 0.03], ...
        'BackgroundColor',BG,'ForegroundColor',FG,'FontSize',8,'HorizontalAlignment','left');
    edtUdz = filaEdit(pControl, 'PWM mínimo (U_DZ)',       num2str(DEF_UDZ), 0.595);
    edtK   = filaEdit(pControl, 'K planta (cm/s por PWM)', num2str(DEF_K),   0.550);
    edtWc  = filaEdit(pControl, 'Ancho de banda (rad/s)',  num2str(DEF_WC),  0.505);
    edtCOM = filaEdit(pControl, 'Puerto COM',              COM_PORT,         0.460);

    uicontrol(pControl,'Style','pushbutton','String','▶  INICIAR', ...
        'Units','normalized','Position',[0.04 0.375 0.44 0.07], ...
        'BackgroundColor',[0.15 0.60 0.35],'ForegroundColor','white', ...
        'FontSize',10,'FontWeight','bold','Callback',@iniciarControl);
    uicontrol(pControl,'Style','pushbutton','String','■  DETENER', ...
        'Units','normalized','Position',[0.52 0.375 0.44 0.07], ...
        'BackgroundColor',[0.65 0.20 0.20],'ForegroundColor','white', ...
        'FontSize',10,'FontWeight','bold','Callback',@detenerControl);
    uicontrol(pControl,'Style','pushbutton','String','↺  TODOS LOS EJES A CERO', ...
        'Units','normalized','Position',[0.04 0.295 0.92 0.07], ...
        'BackgroundColor',[0.55 0.35 0.05],'ForegroundColor','white', ...
        'FontSize',10,'FontWeight','bold','Callback',@irACero);
    uicontrol(pControl,'Style','pushbutton','String','Limpiar gráficas', ...
        'Units','normalized','Position',[0.04 0.215 0.44 0.07], ...
        'BackgroundColor',[0.28 0.28 0.40],'ForegroundColor','white','FontSize',9, ...
        'Callback',@limpiarGraficas);
    uicontrol(pControl,'Style','pushbutton','String','Exportar datos', ...
        'Units','normalized','Position',[0.52 0.215 0.44 0.07], ...
        'BackgroundColor',[0.28 0.28 0.40],'ForegroundColor','white','FontSize',9, ...
        'Callback',@exportarDatos);
    lblInfo = uicontrol(pControl,'Style','text', ...
        'String','Presione INICIAR para calcular los controladores', ...
        'Units','normalized','Position',[0.04 0.02 0.92 0.17], ...
        'BackgroundColor',[0.13 0.13 0.18],'ForegroundColor',[0.45 0.65 0.45], ...
        'FontSize',8,'HorizontalAlignment','center');

    %% ── Panel de rotación (cascada base → péndulo) ──────────
    pControlRot = uipanel(fig,'Title','Eje de Rotación (cascada: base → péndulo)', ...
        'BackgroundColor',BG,'ForegroundColor',[0.85 0.85 1.0], ...
        'FontSize',11,'FontWeight','bold','Units','normalized', ...
        'Position',[0.21 0.01 0.27 0.98]);

    uicontrol(pControlRot,'Style','text','String','Controlador (cascada, mismo tipo en ambos lazos):', ...
        'Units','normalized','Position',[0.04 0.965 0.92 0.025], ...
        'BackgroundColor',BG,'ForegroundColor',FG,'FontSize',8,'HorizontalAlignment','left');
    uicontrol(pControlRot,'Style','popupmenu', ...
        'String',{'PID Discreto','Lugar de Raíces (z)','Oscilaciones Muertas','Servosistema'}, ...
        'Units','normalized','Position',[0.04 0.925 0.92 0.04], ...
        'BackgroundColor',[0.22 0.22 0.30],'ForegroundColor',[0.95 0.95 1.0], ...
        'FontSize',9,'Callback',@cambiarControladorRot);

    uicontrol(pControlRot,'Style','text','String','Referencia de ángulo BASE (°):', ...
        'Units','normalized','Position',[0.04 0.885 0.92 0.025], ...
        'BackgroundColor',BG,'ForegroundColor',FG,'FontSize',8,'HorizontalAlignment','left');
    lblRot = uicontrol(pControlRot,'Style','text','String','0.0 °', ...
        'Units','normalized','Position',[0.64 0.840 0.32 0.04], ...
        'BackgroundColor',BG,'ForegroundColor',[0.4 0.9 0.6],'FontSize',10,'FontWeight','bold');
    sldRot = uicontrol(pControlRot,'Style','slider','Min',-MAX_ROT_DEG,'Max',MAX_ROT_DEG,'Value',0, ...
        'Units','normalized','Position',[0.04 0.840 0.56 0.04], ...
        'BackgroundColor',[0.28 0.28 0.38],'Callback',@cambiarRotacion);
    uicontrol(pControlRot,'Style','pushbutton','String','◄', ...
        'Units','normalized','Position',[0.04 0.795 0.13 0.035], ...
        'BackgroundColor',[0.22 0.35 0.50],'ForegroundColor','white','FontSize',9, ...
        'Callback',@(~,~) ajustarSlider(sldRot,-5,'rot'));
    uicontrol(pControlRot,'Style','pushbutton','String','►', ...
        'Units','normalized','Position',[0.19 0.795 0.13 0.035], ...
        'BackgroundColor',[0.22 0.35 0.50],'ForegroundColor','white','FontSize',9, ...
        'Callback',@(~,~) ajustarSlider(sldRot,+5,'rot'));

    uicontrol(pControlRot,'Style','text','String','Péndulo medido:', ...
        'Units','normalized','Position',[0.04 0.755 0.44 0.025], ...
        'BackgroundColor',BG,'ForegroundColor',FG,'FontSize',8,'HorizontalAlignment','left');
    lblPend = uicontrol(pControlRot,'Style','text','String','0.0 °', ...
        'Units','normalized','Position',[0.50 0.750 0.46 0.035], ...
        'BackgroundColor',BG,'ForegroundColor',[0.9 0.7 0.3],'FontSize',9,'FontWeight','bold');
    uicontrol(pControlRot,'Style','text','String','Ref. péndulo (lazo externo):', ...
        'Units','normalized','Position',[0.04 0.715 0.44 0.025], ...
        'BackgroundColor',BG,'ForegroundColor',FG,'FontSize',8,'HorizontalAlignment','left');
    lblPendRef = uicontrol(pControlRot,'Style','text','String','0.0 °', ...
        'Units','normalized','Position',[0.50 0.710 0.46 0.035], ...
        'BackgroundColor',BG,'ForegroundColor',[0.9 0.7 0.3],'FontSize',9,'FontWeight','bold');

    uicontrol(pControlRot,'Style','text','String','Planta lazo EXTERNO (ángulo base):', ...
        'Units','normalized','Position',[0.04 0.670 0.92 0.025], ...
        'BackgroundColor',BG,'ForegroundColor',FG,'FontSize',8,'HorizontalAlignment','left');
    edtKExt  = filaEdit(pControlRot, 'K (deg_pend_ref/deg_error)', num2str(DEF_K_EXT),  0.632);
    edtWcExt = filaEdit(pControlRot, 'Ancho de banda (rad/s)',     num2str(DEF_WC_EXT), 0.590);

    uicontrol(pControlRot,'Style','text','String','Planta lazo INTERNO (péndulo):', ...
        'Units','normalized','Position',[0.04 0.545 0.92 0.025], ...
        'BackgroundColor',BG,'ForegroundColor',FG,'FontSize',8,'HorizontalAlignment','left');
    edtUdzInt = filaEdit(pControlRot, 'PWM mínimo (U_DZ)',        num2str(DEF_UDZ_INT), 0.507);
    edtKInt   = filaEdit(pControlRot, 'K (°/s por PWM)',          num2str(DEF_K_INT),   0.465);
    edtWcInt  = filaEdit(pControlRot, 'Ancho de banda (rad/s)',   num2str(DEF_WC_INT),  0.423);

    lblInfoRot = uicontrol(pControlRot,'Style','text', ...
        'String','Presione INICIAR para calcular los controladores', ...
        'Units','normalized','Position',[0.04 0.02 0.92 0.38], ...
        'BackgroundColor',[0.13 0.13 0.18],'ForegroundColor',[0.45 0.65 0.45], ...
        'FontSize',8,'HorizontalAlignment','center');

    %% ── Gráficas: col1=lineal, col2=rotación(externo/base), col3=péndulo(interno) ──
    pGraficas = uipanel(fig,'Title','Monitoreo en Tiempo Real', ...
        'BackgroundColor',[0.94 0.94 0.94],'ForegroundColor',[0.10 0.10 0.10], ...
        'FontSize',11,'FontWeight','bold','Units','normalized', ...
        'Position',[0.49 0.01 0.50 0.98]);

    ax1 = subplot(4,3,1,'Parent',pGraficas);
    configurarEjes(ax1,'Posición lineal vs Tiempo','Tiempo (s)','Posición (cm)');
    hRefLine = line(ax1,NaN,NaN,'Color',[0 0.6 0],'LineStyle','--','LineWidth',1.5,'DisplayName','Referencia');
    hMedLine = line(ax1,NaN,NaN,'Color',[0 0.15 0.7],'LineWidth',1.5,'DisplayName','Posición real');
    legend(ax1,'show','FontSize',7,'Location','southeast');

    ax2 = subplot(4,3,4,'Parent',pGraficas);
    configurarEjes(ax2,'Velocidad lineal vs Tiempo','Tiempo (s)','cm/s');
    hVelLine = line(ax2,NaN,NaN,'Color',[0.8 0 0],'LineWidth',1.5);

    ax3 = subplot(4,3,7,'Parent',pGraficas);
    configurarEjes(ax3,'PWM lineal vs Tiempo','Tiempo (s)','PWM');
    hCtrlLine = line(ax3,NaN,NaN,'Color',[0 0 0],'LineWidth',1.5);

    ax4 = subplot(4,3,10,'Parent',pGraficas);
    configurarEjes(ax4,'Error lineal vs Tiempo','Tiempo (s)','Error (cm)');
    hErrLine = line(ax4,NaN,NaN,'Color',[0.72 0.27 0],'LineWidth',1.5);

    % columna 2: lazo externo (ángulo base)
    ax1r = subplot(4,3,2,'Parent',pGraficas);
    configurarEjes(ax1r,'Ángulo BASE vs Tiempo','Tiempo (s)','Ángulo (°)');
    hRefLineRot = line(ax1r,NaN,NaN,'Color',[0 0.6 0],'LineStyle','--','LineWidth',1.5,'DisplayName','Referencia');
    hMedLineRot = line(ax1r,NaN,NaN,'Color',[0 0.15 0.7],'LineWidth',1.5,'DisplayName','Ángulo real');
    legend(ax1r,'show','FontSize',7,'Location','southeast');

    ax2r = subplot(4,3,5,'Parent',pGraficas);
    configurarEjes(ax2r,'Velocidad angular BASE','Tiempo (s)','°/s');
    hVelLineRot = line(ax2r,NaN,NaN,'Color',[0.8 0 0],'LineWidth',1.5);

    ax3r = subplot(4,3,8,'Parent',pGraficas);
    configurarEjes(ax3r,'PWM rotación (salida cascada)','Tiempo (s)','PWM');
    hCtrlLineRot = line(ax3r,NaN,NaN,'Color',[0 0 0],'LineWidth',1.5);

    ax4r = subplot(4,3,11,'Parent',pGraficas);
    configurarEjes(ax4r,'Error ángulo BASE','Tiempo (s)','Error (°)');
    hErrLineRot = line(ax4r,NaN,NaN,'Color',[0.72 0.27 0],'LineWidth',1.5);

    % columna 3: lazo interno (péndulo) — sin subplot de PWM propio
    % (comparte el PWM mostrado en ax3r)
    ax1p = subplot(4,3,3,'Parent',pGraficas);
    configurarEjes(ax1p,'Ángulo PÉNDULO vs Tiempo','Tiempo (s)','Ángulo (°)');
    hRefLinePend = line(ax1p,NaN,NaN,'Color',[0 0.6 0],'LineStyle','--','LineWidth',1.5,'DisplayName','Ref. péndulo');
    hMedLinePend = line(ax1p,NaN,NaN,'Color',[0.9 0.5 0],'LineWidth',1.5,'DisplayName','Péndulo real');
    legend(ax1p,'show','FontSize',7,'Location','southeast');

    ax2p = subplot(4,3,6,'Parent',pGraficas);
    configurarEjes(ax2p,'Velocidad angular PÉNDULO','Tiempo (s)','°/s');
    hVelLinePend = line(ax2p,NaN,NaN,'Color',[0.8 0 0],'LineWidth',1.5);

    ax4p = subplot(4,3,12,'Parent',pGraficas);
    configurarEjes(ax4p,'Error ángulo PÉNDULO','Tiempo (s)','Error (°)');
    hErrLinePend = line(ax4p,NaN,NaN,'Color',[0.72 0.27 0],'LineWidth',1.5);

    lblStatus = uicontrol(fig,'Style','text','String','Estado: DETENIDO', ...
        'Units','normalized','Position',[0.01 0.00 0.98 0.02], ...
        'BackgroundColor',[0.85 0.85 0.85],'ForegroundColor',[0.1 0.1 0.1], ...
        'FontSize',8,'HorizontalAlignment','left');

    %% ══════════════════════════════════════════════════════════
    %  CALLBACKS
    %% ══════════════════════════════════════════════════════════
    function h = filaEdit(parent, etiqueta, valor, y)
        uicontrol(parent,'Style','text','String',etiqueta, ...
            'Units','normalized','Position',[0.04 y 0.56 0.035], ...
            'BackgroundColor',BG,'ForegroundColor',FG,'FontSize',8,'HorizontalAlignment','left');
        h = uicontrol(parent,'Style','edit','String',valor, ...
            'Units','normalized','Position',[0.62 y 0.34 0.035], ...
            'BackgroundColor',[0.22 0.22 0.30],'ForegroundColor','white','FontSize',9);
    end

    function cambiarModo(src,~)
        if estado.corriendo
            src.Value = double(estado.modoManual);
            return;
        end
        if src.Value == 1
            estado.modoManual = true;
            src.String = 'Simulación (Sin ESP32)';
            src.BackgroundColor = [0.25 0.45 0.65];
        else
            estado.modoManual = false;
            src.String = 'Conectado al ESP32';
            src.BackgroundColor = [0.20 0.55 0.30];
        end
    end

    function cambiarControlador(src,~)
        opciones = {'PID','RootLocusZ','OscMuertas','Servosistema'};
        estado.controlador = opciones{src.Value};
        resetearEstadosControladores();
        actualizarStatus();
    end

    function cambiarControladorRot(src,~)
        opciones = {'PID','RootLocusZ','OscMuertas','Servosistema'};
        estado.controladorRot = opciones{src.Value};
        resetearEstadosControladores();
        actualizarStatus();
    end

    function cambiarDistancia(src,~)
        estado.distRef = src.Value;
        lblDist.String = sprintf('%.1f cm', estado.distRef);
    end

    function cambiarRotacion(src,~)
        estado.rotRef = src.Value;
        lblRot.String = sprintf('%.1f °', estado.rotRef);
    end

    function ajustarSlider(slider, delta, eje)
        nuevo = max(slider.Min, min(slider.Max, slider.Value + delta));
        slider.Value = nuevo;
        if strcmp(eje,'lin')
            estado.distRef = nuevo;
            lblDist.String = sprintf('%.1f cm', nuevo);
        else
            estado.rotRef = nuevo;
            lblRot.String = sprintf('%.1f °', nuevo);
        end
    end

    function irACero(~,~)
        estado.distRef = 0;  sldDist.Value = 0;  lblDist.String = '0.0 cm';
        estado.rotRef  = 0;  sldRot.Value  = 0;  lblRot.String  = '0.0 °';
    end

    function iniciarControl(~,~)
        if estado.corriendo; return; end

        calcularControladores();
        estado.alarma = '';

        if ~estado.modoManual
            COM = strtrim(edtCOM.String);
            try
                estado.arduino = serialport(COM, BAUD_RATE);
                configureTerminator(estado.arduino,"LF");
                pause(2);                       % el ESP32 se reinicia al abrir el puerto
                flush(estado.arduino);
            catch
                estado.arduino = [];
                errordlg(['No se pudo abrir ' COM '. Cierre el Serial Monitor y revise el COM.'],'Error');
                return;
            end
            [y0, yr0, yp0, t0] = esperarMedicion(3.0);
            if isnan(y0)
                errordlg('El ESP32 no envía datos. Revise sensor, baudios y firmware.','Error');
                try; delete(estado.arduino); catch; end
                estado.arduino = [];
                return;
            end
            try; writeline(estado.arduino,'START'); catch; end
            meas.y = y0;  meas.t = t0;
            measRot.y = yr0;
            measPen.y = yp0;
        else
            simu.y = 0; simu.v = 0; simu.up = 0; simu.yf = 0; simu.t = 0;
            simuRot.y = 0; simuRot.v = 0; simuRot.up = 0; simuRot.yf = 0;
            simuPen.y = 0; simuPen.v = 0; simuPen.yf = 0;
            meas.y = 0;  meas.t = 0;
            measRot.y = 0;
            measPen.y = 0;
        end
        meas.vhat = 0;
        measRot.vhat = 0;
        measPen.vhat = 0;

        % las referencias arrancan donde está el carrito/rotación (error inicial = 0)
        y0c = max(0, min(MAX_DIST_CM, meas.y));
        estado.distRef = y0c;  estado.distRefRamp = y0c;
        sldDist.Value  = y0c;  lblDist.String = sprintf('%.1f cm', y0c);

        r0c = max(-MAX_ROT_DEG, min(MAX_ROT_DEG, measRot.y));
        estado.rotRef = r0c;  estado.rotRefRamp = r0c;
        sldRot.Value  = r0c;  lblRot.String = sprintf('%.1f °', r0c);

        estado.tiempoVec = [];  estado.refVec = [];  estado.distVec = [];
        estado.controlVec = []; estado.errorVec = []; estado.velVec = [];
        estado.refRotVec = []; estado.posRotVec = [];
        estado.controlRotVec = []; estado.errorRotVec = []; estado.velRotVec = [];
        estado.pendRefVec = []; estado.pendVec = []; estado.errorPendVec = []; estado.velPendVec = [];
        estado.tick = 0;

        resetearEstadosControladores();
        estado.corriendo = true;
        actualizarStatus();

        estado.timerObj = timer('ExecutionMode','fixedRate','Period',SAMPLE_TIME, ...
            'BusyMode','drop','TimerFcn',@loopControl,'ErrorFcn',@errorTimer);
        start(estado.timerObj);
    end

    function detenerControl(~,~)
        if ~estado.corriendo; return; end
        estado.corriendo = false;
        if ~isempty(estado.timerObj) && isvalid(estado.timerObj)
            stop(estado.timerObj); delete(estado.timerObj);
        end
        estado.timerObj = [];
        if ~isempty(estado.arduino)
            try
                writeline(estado.arduino,'D0.0,U0,R0.0,V0'); pause(0.1);
                writeline(estado.arduino,'STOP');             pause(0.05);
                delete(estado.arduino);
            catch
            end
            estado.arduino = [];
        end
        actualizarStatus();
    end

    function limpiarGraficas(~,~)
        set([hRefLine hMedLine hVelLine hCtrlLine hErrLine ...
             hRefLineRot hMedLineRot hVelLineRot hCtrlLineRot hErrLineRot ...
             hRefLinePend hMedLinePend hVelLinePend hErrLinePend], ...
            'XData',NaN,'YData',NaN);
        estado.tiempoVec = [];  estado.refVec = [];  estado.distVec = [];
        estado.controlVec = []; estado.errorVec = []; estado.velVec = [];
        estado.refRotVec = []; estado.posRotVec = [];
        estado.controlRotVec = []; estado.errorRotVec = []; estado.velRotVec = [];
        estado.pendRefVec = []; estado.pendVec = []; estado.errorPendVec = []; estado.velPendVec = [];
    end

    function exportarDatos(~,~)
        if isempty(estado.tiempoVec)
            msgbox('No hay datos para exportar.','Aviso'); return;
        end
        tabla = table(estado.tiempoVec', estado.refVec', estado.distVec', ...
            estado.velVec', estado.controlVec', estado.errorVec', ...
            estado.refRotVec', estado.posRotVec', estado.velRotVec', ...
            estado.controlRotVec', estado.errorRotVec', ...
            estado.pendRefVec', estado.pendVec', estado.velPendVec', estado.errorPendVec', ...
            'VariableNames',{'Tiempo_s','Referencia_cm','Posicion_cm', ...
                             'Velocidad_cms','PWM','Error_cm', ...
                             'Referencia_base_deg','Posicion_base_deg','Velocidad_base_degs', ...
                             'PWM_rot','Error_base_deg', ...
                             'Referencia_pendulo_deg','Posicion_pendulo_deg', ...
                             'Velocidad_pendulo_degs','Error_pendulo_deg'});
        archivo = ['datos_' datestr(now,'yyyymmdd_HHMMSS') '.csv'];
        writetable(tabla,archivo);
        msgbox(['Guardado: ' archivo],'OK');
    end

    function cerrarAplicacion(~,~)
        try; detenerControl([],[]); catch; end
        try; delete(fig); catch; end
    end

    function errorTimer(~,ev)
        try; disp(['[ERROR] Timer: ' ev.Data.message]); catch; end
        detenerControl([],[]);
    end

    %% ══════════════════════════════════════════════════════════
    %  DISEÑO DE CONTROLADORES
    %% ══════════════════════════════════════════════════════════
    function v = leerEdit(h, def, lo, hi)
        v = str2double(h.String);
        if isnan(v); v = def; end
        v = max(lo, min(hi, v));
        h.String = num2str(v);
    end

    function resetearEstadosControladores()
        pidc.I = 0;
        od.be  = zeros(1,numel(od.Cn));
        od.bu  = zeros(1,max(0,numel(od.Cd)-1));
        od.cnt = 0;  od.uh = 0;
        rlz.be = zeros(1,max(1,numel(rlz.Cn)));
        rlz.bu = zeros(1,max(1,numel(rlz.Cd)-1));
        srv.x  = [meas.y; 0];
        srv.xi = 0;
        estado.u_prev_c = 0;
        estado.enBanda  = false;
        estado.sinDatos = 0;

        pidcExt.I = 0;
        odExt.be  = zeros(1,numel(odExt.Cn));
        odExt.bu  = zeros(1,max(0,numel(odExt.Cd)-1));
        odExt.cnt = 0;  odExt.uh = 0;
        rlzExt.be = zeros(1,max(1,numel(rlzExt.Cn)));
        rlzExt.bu = zeros(1,max(1,numel(rlzExt.Cd)-1));
        srvExt.x  = [measRot.y; 0];
        srvExt.xi = 0;

        pidcInt.I = 0;
        odInt.be  = zeros(1,numel(odInt.Cn));
        odInt.bu  = zeros(1,max(0,numel(odInt.Cd)-1));
        odInt.cnt = 0;  odInt.uh = 0;
        rlzInt.be = zeros(1,max(1,numel(rlzInt.Cn)));
        rlzInt.bu = zeros(1,max(1,numel(rlzInt.Cd)-1));
        srvInt.x  = [measPen.y; 0];
        srvInt.xi = 0;

        estado.u_prev_c_ext = 0;
        estado.u_prev_c_int = 0;
        estado.enBandaExt    = false;
        estado.enBandaInt    = false;
        estado.sinDatosRot   = 0;
    end

    % Calcula ganancias PID / osc. muertas / servosistema para una planta
    % de primer orden con integrador: Gz = K/(TAU*s*(s+1/TAU)) aprox.
    function [pidOut, odOut, srvOut] = disenarControladores(Ts, Pcfg, tau, wc, odN, odTipo, odGanancia, x0)
        pidOut.Kp   = wc / Pcfg.K;
        pidOut.Ki   = pidOut.Kp * wc / 4;
        pidOut.Kd   = pidOut.Kp * 0.25 / wc;
        pidOut.Imax = 0.5 * Pcfg.ucMax / max(pidOut.Ki, 1e-6);

        Tod = Ts * odN;
        Gz  = c2d(tf(Pcfg.K, [tau 1 0]), Tod, 'zoh');
        [nd, dd] = tfdata(Gz, 'v');
        nd = nd(find(abs(nd) > 1e-12, 1, 'first'):end);
        if strcmp(odTipo,'rampa')
            Cn = conv([2 -1], dd);   Cd = conv([1 -2 1], nd);
        else
            Cn = dd;                 Cd = conv([1 -1], nd);
        end
        odOut.Cn = Cn / Cd(1);
        odOut.Cd = Cd / Cd(1);
        odOut.ganancia = odGanancia;

        A = [0 1; 0 -1/tau];  B = [0; Pcfg.K/tau];  C = [1 0];
        sd = c2d(ss(A,B,C,0), Ts, 'zoh');
        Ad = sd.A;  Bd = sd.B;  Cdd = sd.C;
        Aa = [Ad, zeros(2,1); -Cdd, 1];
        Ba = [Bd; 0];
        wn = 1.5 * wc;  zt = 0.9;
        sc = [-zt*wn + 1j*wn*sqrt(1-zt^2), -zt*wn - 1j*wn*sqrt(1-zt^2), -2*wn];
        Ka = place(Aa, Ba, exp(Ts*sc));
        L  = place(Ad', Cdd', exp(Ts*[-3*wn, -3.5*wn]))';
        srvOut.Ad = Ad;  srvOut.Bd = Bd;  srvOut.Cd = Cdd;  srvOut.Ka = Ka;  srvOut.L = L;
        srvOut.ximax = 2*Pcfg.ucMax / max(abs(Ka(3)), 1e-6);
        srvOut.x = x0;
    end

    % Lugar geométrico de raíces en z: compensador de adelanto discreto,
    % diseñado en continuo por condición de ángulo/magnitud del lugar de
    % raíces y luego discretizado (tustin). Misma planta supuesta que
    % PID/Servosistema: G(s) = K / (s*(tau*s + 1)).
    function [Cn, Cd] = disenarLeadZ(Ts, Pcfg, tau, wc)
        zt = 0.9; wn = 1.5*wc;                 % mismo criterio de diseño que Servosistema
        sigma = zt*wn; omega = wn*sqrt(1-zt^2);
        s0 = complex(-sigma, omega);           % polo dominante deseado
        Gs0 = Pcfg.K / (s0*(tau*s0+1));
        phiG = angle(Gs0);
        target = -pi - phiG;
        phiNeeded = atan2(sin(target), cos(target));   % fase que debe aportar el compensador, en (-pi,pi]

        zc = max(sigma, 0.05);                 % cero del compensador = parte real del polo deseado (simplificación estándar)
        angPolo = (pi/2) - phiNeeded;          % con zc=sigma, angle(s0+zc) = 90° exactamente
        angPolo = max(min(angPolo, pi/2 - 1e-3), -pi/2 + 1e-3);  % evita tan() singular
        pc = sigma + omega/tan(angPolo);
        pc = max(pc, zc + 0.05);               % el polo debe quedar a la izquierda del cero
        Kc = abs(s0+pc) / (abs(s0+zc) * abs(Gs0));

        Gc  = tf(Kc*[1 zc], [1 pc]);
        Gcz = c2d(Gc, Ts, 'tustin');
        [nd, dd] = tfdata(Gcz, 'v');
        Cn = nd / dd(1);
        Cd = dd / dd(1);
    end

    % Aplica un filtro discreto recursivo genérico (dados Cn/Cd y buffers
    % de historial de error/salida), usado por el controlador Lugar de
    % Raíces (z) en los tres lazos (lineal, externo, interno) para no
    % triplicar la misma mecánica de filtro recursivo.
    function [uc, st] = aplicarRecursivo(st, e, ucMax)
        st.be = [e, st.be(1:end-1)];
        nCd = numel(st.Cd);
        if nCd > 1
            u = st.Cn * st.be(1:numel(st.Cn))' - st.Cd(2:end) * st.bu(1:nCd-1)';
        else
            u = st.Cn * st.be(1:numel(st.Cn))';
        end
        u = max(-ucMax, min(ucMax, u));
        st.bu = [u, st.bu(1:end-1)];
        uc = u;
    end

    function calcularControladores()
        Ts = SAMPLE_TIME;
        P.Udz   = leerEdit(edtUdz, DEF_UDZ, 0, 200);
        P.K     = leerEdit(edtK,   DEF_K, 0.002, 1);
        P.wc    = leerEdit(edtWc,  DEF_WC, 0.3, 6);
        P.ucMax = U_MAX - P.Udz;

        Pext.K     = leerEdit(edtKExt,  DEF_K_EXT, 0.01, 10);
        Pext.wc    = leerEdit(edtWcExt, DEF_WC_EXT, 0.3, 6);
        Pext.ucMax = PEND_REF_MAX_DEG;

        Pint.Udz   = leerEdit(edtUdzInt, DEF_UDZ_INT, 0, 200);
        Pint.K     = leerEdit(edtKInt,   DEF_K_INT, 0.01, 10);
        Pint.wc    = leerEdit(edtWcInt,  DEF_WC_INT, 0.3, 6);
        Pint.ucMax = U_MAX - Pint.Udz;

        [pid1, od1, srv1] = disenarControladores(Ts, P, TAU_MOTOR, P.wc, OD_N, OD_TIPO, OD_GANANCIA, [meas.y;0]);
        pidc.Kp = pid1.Kp; pidc.Ki = pid1.Ki; pidc.Kd = pid1.Kd; pidc.Imax = pid1.Imax;
        od.Cn = od1.Cn; od.Cd = od1.Cd;
        srv.Ad = srv1.Ad; srv.Bd = srv1.Bd; srv.Cd = srv1.Cd; srv.Ka = srv1.Ka; srv.L = srv1.L; srv.ximax = srv1.ximax;
        [rlz.Cn, rlz.Cd] = disenarLeadZ(Ts, P, TAU_MOTOR, P.wc);

        [pid2, od2, srv2] = disenarControladores(Ts, Pext, TAU_MOTOR_EXT, Pext.wc, OD_N_ROT, OD_TIPO_ROT, OD_GANANCIA_ROT, [measRot.y;0]);
        pidcExt.Kp = pid2.Kp; pidcExt.Ki = pid2.Ki; pidcExt.Kd = pid2.Kd; pidcExt.Imax = pid2.Imax;
        odExt.Cn = od2.Cn; odExt.Cd = od2.Cd;
        srvExt.Ad = srv2.Ad; srvExt.Bd = srv2.Bd; srvExt.Cd = srv2.Cd; srvExt.Ka = srv2.Ka; srvExt.L = srv2.L; srvExt.ximax = srv2.ximax;
        [rlzExt.Cn, rlzExt.Cd] = disenarLeadZ(Ts, Pext, TAU_MOTOR_EXT, Pext.wc);

        [pid3, od3, srv3] = disenarControladores(Ts, Pint, TAU_MOTOR_INT, Pint.wc, OD_N_ROT, OD_TIPO_ROT, OD_GANANCIA_ROT, [measPen.y;0]);
        pidcInt.Kp = pid3.Kp; pidcInt.Ki = pid3.Ki; pidcInt.Kd = pid3.Kd; pidcInt.Imax = pid3.Imax;
        odInt.Cn = od3.Cn; odInt.Cd = od3.Cd;
        srvInt.Ad = srv3.Ad; srvInt.Bd = srv3.Bd; srvInt.Cd = srv3.Cd; srvInt.Ka = srv3.Ka; srvInt.L = srv3.L; srvInt.ximax = srv3.ximax;
        [rlzInt.Cn, rlzInt.Cd] = disenarLeadZ(Ts, Pint, TAU_MOTOR_INT, Pint.wc);

        fprintf('[LIN  PID]   Kp=%.2f Ki=%.2f Kd=%.2f\n', pidc.Kp, pidc.Ki, pidc.Kd);
        fprintf('[LIN  Servo] Ka=[%s]  L=[%s]\n', num2str(srv.Ka,'%.3f '), num2str(srv.L','%.3f '));
        fprintf('[ROT-EXT PID]   Kp=%.2f Ki=%.2f Kd=%.2f\n', pidcExt.Kp, pidcExt.Ki, pidcExt.Kd);
        fprintf('[ROT-INT PID]   Kp=%.2f Ki=%.2f Kd=%.2f\n', pidcInt.Kp, pidcInt.Ki, pidcInt.Kd);

        % C(z) numéricos — Lugar de Raíces (z) y Oscilaciones Muertas, para
        % las 3 plantas. Cn/Cd ya están normalizados (Cd(1)=1), por lo que
        % C(z) = Cn(z)/Cd(z) tal cual se imprime aquí coincide exactamente
        % con los coeficientes usados en aplicarRecursivo()/calcularOscMuertas*().
        fprintf('[LIN     RootLocusZ] C(z): num=[%s]  den=[%s]\n', num2str(rlz.Cn,'%.5f '), num2str(rlz.Cd,'%.5f '));
        fprintf('[ROT-EXT RootLocusZ] C(z): num=[%s]  den=[%s]\n', num2str(rlzExt.Cn,'%.5f '), num2str(rlzExt.Cd,'%.5f '));
        fprintf('[ROT-INT RootLocusZ] C(z): num=[%s]  den=[%s]\n', num2str(rlzInt.Cn,'%.5f '), num2str(rlzInt.Cd,'%.5f '));
        fprintf('[LIN     OscMuertas] C(z) (cada %d muestras): num=[%s]  den=[%s]  ganancia=%.2f\n', OD_N,     num2str(od.Cn,'%.5f '),    num2str(od.Cd,'%.5f '),    OD_GANANCIA);
        fprintf('[ROT-EXT OscMuertas] C(z) (cada %d muestras): num=[%s]  den=[%s]  ganancia=%.2f\n', OD_N_ROT, num2str(odExt.Cn,'%.5f '), num2str(odExt.Cd,'%.5f '), OD_GANANCIA_ROT);
        fprintf('[ROT-INT OscMuertas] C(z) (cada %d muestras): num=[%s]  den=[%s]  ganancia=%.2f\n', OD_N_ROT, num2str(odInt.Cn,'%.5f '), num2str(odInt.Cd,'%.5f '), OD_GANANCIA_ROT);

        lblInfo.String = sprintf(['PID: Kp=%.1f Ki=%.1f Kd=%.2f\n' ...
            'Servo: Ka=[%.1f %.2f %.2f]\n' ...
            'U_DZ=%.0f  K=%.3f  wc=%.1f'], ...
            pidc.Kp, pidc.Ki, pidc.Kd, srv.Ka(1), srv.Ka(2), srv.Ka(3), P.Udz, P.K, P.wc);

        lblInfoRot.String = sprintf(['EXTERNO (base->ref.péndulo): PID Kp=%.1f Ki=%.1f Kd=%.2f\n' ...
            'K_ext=%.3f  wc=%.1f  ref_pend_max=%.1f°\n' ...
            'INTERNO (péndulo->PWM): PID Kp=%.1f Ki=%.1f Kd=%.2f\n' ...
            'U_DZ_int=%.0f  K_int=%.3f  wc=%.1f\n' ...
            'TODO: calibrar K_ext/K_int/U_DZ_int con planta real'], ...
            pidcExt.Kp, pidcExt.Ki, pidcExt.Kd, Pext.K, Pext.wc, PEND_REF_MAX_DEG, ...
            pidcInt.Kp, pidcInt.Ki, pidcInt.Kd, Pint.Udz, Pint.K, Pint.wc);
    end

    %% ══════════════════════════════════════════════════════════
    %  LAZO PRINCIPAL
    %% ══════════════════════════════════════════════════════════
    function loopControl(~,~)
        if ~estado.corriendo; return; end

        % ── rampas de referencia (independientes por eje) ─────
        switch estado.controlador
            case 'PID';          rr = RAMP_PID * SAMPLE_TIME;
            case 'RootLocusZ';   rr = RAMP_RLZ * SAMPLE_TIME;
            case 'OscMuertas';   rr = RAMP_OD  * SAMPLE_TIME;
            case 'Servosistema'; rr = RAMP_SRV * SAMPLE_TIME;
            otherwise;           rr = RAMP_PID * SAMPLE_TIME;
        end
        dr = estado.distRef - estado.distRefRamp;
        if abs(dr) <= rr; estado.distRefRamp = estado.distRef;
        else; estado.distRefRamp = estado.distRefRamp + sign(dr)*rr; end

        switch estado.controladorRot
            case 'PID';          rrR = RAMP_PID_ROT * SAMPLE_TIME;
            case 'RootLocusZ';   rrR = RAMP_RLZ_ROT * SAMPLE_TIME;
            case 'OscMuertas';   rrR = RAMP_OD_ROT  * SAMPLE_TIME;
            case 'Servosistema'; rrR = RAMP_SRV_ROT * SAMPLE_TIME;
            otherwise;           rrR = RAMP_PID_ROT * SAMPLE_TIME;
        end
        drR = estado.rotRef - estado.rotRefRamp;
        if abs(drR) <= rrR; estado.rotRefRamp = estado.rotRef;
        else; estado.rotRefRamp = estado.rotRefRamp + sign(drR)*rrR; end

        % ── medición (lineal + base + péndulo comparten la misma línea serial) ─
        [y, yr, yp, tt, nuevo] = leerSensor();
        if nuevo
            dtm = tt - meas.t;
            if dtm > 0.01 && dtm < 0.5
                meas.vhat    = 0.7*meas.vhat    + 0.3*(y  - meas.y)/dtm;
                measRot.vhat = 0.7*measRot.vhat + 0.3*(yr - measRot.y)/dtm;
                measPen.vhat = 0.7*measPen.vhat + 0.3*(yp - measPen.y)/dtm;
            end
            meas.y = y;  meas.t = tt;  measRot.y = yr;  measPen.y = yp;
            estado.sinDatos = 0;  estado.sinDatosRot = 0;
        else
            estado.sinDatos = estado.sinDatos + 1;
            estado.sinDatosRot = estado.sinDatosRot + 1;
        end
        e  = estado.distRefRamp - meas.y;
        eExt = estado.rotRefRamp - measRot.y;   % error del lazo externo (base)

        % ── controlador lineal ─────────────────────────────────
        if estado.sinDatos > 10
            uc = 0;
            estado.alarma = 'SIN DATOS DEL ESP32';
        else
            switch estado.controlador
                case 'PID';          uc = calcularPID(e);
                case 'RootLocusZ';   uc = calcularRootLocusZ(e);
                case 'OscMuertas';   uc = calcularOscMuertas(e);
                case 'Servosistema'; uc = calcularServosistema(meas.y, e);
                otherwise;           uc = 0;
            end
        end
        uc = max(-P.ucMax, min(P.ucMax, uc));
        du = uc - estado.u_prev_c;
        uc = estado.u_prev_c + max(-MAX_DELTA_UC, min(MAX_DELTA_UC, du));
        if estado.enBanda
            if abs(e) > BANDA_OUT; estado.enBanda = false; end
        elseif abs(e) < BANDA_IN && abs(meas.vhat) < 1.5
            estado.enBanda = true;
        end
        if estado.enBanda; uc = 0; end
        estado.u_prev_c = uc;
        if abs(uc) < 2
            u = 0;
        else
            u = sign(uc) * (P.Udz + abs(uc));
        end
        u = max(-U_MAX, min(U_MAX, u));

        % ── cascada de rotación: lazo EXTERNO (base -> ref. péndulo) ──
        if estado.sinDatosRot > 10
            pendRef = 0;
            estado.alarma = 'SIN DATOS DEL ESP32';
        else
            switch estado.controladorRot
                case 'PID';          pendRef = calcularPIDExt(eExt);
                case 'RootLocusZ';   pendRef = calcularRootLocusZExt(eExt);
                case 'OscMuertas';   pendRef = calcularOscMuertasExt(eExt);
                case 'Servosistema'; pendRef = calcularServosistemaExt(measRot.y, eExt);
                otherwise;           pendRef = 0;
            end
        end
        pendRef = max(-Pext.ucMax, min(Pext.ucMax, pendRef));
        dExt = pendRef - estado.u_prev_c_ext;
        pendRef = estado.u_prev_c_ext + max(-MAX_DELTA_UC_EXT, min(MAX_DELTA_UC_EXT, dExt));
        if estado.enBandaExt
            if abs(eExt) > BANDA_OUT_EXT; estado.enBandaExt = false; end
        elseif abs(eExt) < BANDA_IN_EXT && abs(measRot.vhat) < 3
            estado.enBandaExt = true;
        end
        if estado.enBandaExt; pendRef = 0; end
        estado.u_prev_c_ext = pendRef;

        % ── cascada de rotación: lazo INTERNO (péndulo -> PWM final) ──
        eInt = pendRef - measPen.y;
        if estado.sinDatosRot > 10
            ucInt = 0;
        else
            switch estado.controladorRot
                case 'PID';          ucInt = calcularPIDInt(eInt);
                case 'RootLocusZ';   ucInt = calcularRootLocusZInt(eInt);
                case 'OscMuertas';   ucInt = calcularOscMuertasInt(eInt);
                case 'Servosistema'; ucInt = calcularServosistemaInt(measPen.y, eInt);
                otherwise;           ucInt = 0;
            end
        end
        ucInt = max(-Pint.ucMax, min(Pint.ucMax, ucInt));
        dInt = ucInt - estado.u_prev_c_int;
        ucInt = estado.u_prev_c_int + max(-MAX_DELTA_UC_INT, min(MAX_DELTA_UC_INT, dInt));
        if estado.enBandaInt
            if abs(eInt) > BANDA_OUT_INT; estado.enBandaInt = false; end
        elseif abs(eInt) < BANDA_IN_INT && abs(measPen.vhat) < 3
            estado.enBandaInt = true;
        end
        if estado.enBandaInt; ucInt = 0; end
        estado.u_prev_c_int = ucInt;
        if abs(ucInt) < 2
            uR = 0;
        else
            uR = sign(ucInt) * (Pint.Udz + abs(ucInt));
        end
        uR = max(-U_MAX, min(U_MAX, uR));

        % ── salida ───────────────────────────────────────────
        if estado.modoManual
            avanzarPlantaSim(u);
            avanzarPlantaSimRot(uR);
        elseif ~isempty(estado.arduino)
            try
                writeline(estado.arduino, sprintf('D%.1f,U%d,R%.1f,V%d', ...
                    estado.distRefRamp, round(u), estado.rotRefRamp, round(uR)));
            catch ex
                disp(['[Serial] ' ex.message]);
            end
        end

        % ── registro ─────────────────────────────────────────
        t_k = SAMPLE_TIME * numel(estado.tiempoVec);
        estado.tiempoVec  = [estado.tiempoVec,  t_k];
        estado.refVec     = [estado.refVec,     estado.distRefRamp];
        estado.distVec    = [estado.distVec,    meas.y];
        estado.controlVec = [estado.controlVec, u];
        estado.errorVec   = [estado.errorVec,   e];
        estado.velVec     = [estado.velVec,     meas.vhat];

        estado.refRotVec     = [estado.refRotVec,     estado.rotRefRamp];
        estado.posRotVec     = [estado.posRotVec,     measRot.y];
        estado.controlRotVec = [estado.controlRotVec, uR];
        estado.errorRotVec   = [estado.errorRotVec,   eExt];
        estado.velRotVec     = [estado.velRotVec,     measRot.vhat];

        estado.pendRefVec   = [estado.pendRefVec,   pendRef];
        estado.pendVec      = [estado.pendVec,      measPen.y];
        estado.errorPendVec = [estado.errorPendVec, eInt];
        estado.velPendVec   = [estado.velPendVec,   measPen.vhat];

        lblPend.String    = sprintf('%.1f °', measPen.y);
        lblPendRef.String = sprintf('%.1f °', pendRef);

        estado.tick = estado.tick + 1;
        if mod(estado.tick,2) == 0; actualizarGraficas(); end
        if mod(estado.tick,10) == 0; actualizarStatus(); end
    end

    %% ── CONTROLADORES — LINEAL ──────────────────────────────
    % PID discreto (lineal). NO es un C(z) lineal único: el término
    % derivativo usa la VELOCIDAD MEDIDA (meas.vhat, estimada por EMA sobre
    % la señal del sensor), no la derivada del error, y la parte integral
    % tiene anti-windup condicional (se congela si |e|>=3 o si |uc|>=ucMax).
    % Por eso se documenta como ecuación de diferencias, no como C(z)=U(z)/E(z).
    %   Ecuación de diferencias implementada:
    %     uc(k) = Kp*e(k) + Ki*I(k) - Kd*vhat(k)
    %     I(k)  = I(k-1) + e(k)*Ts     si |e(k)| < 3  y  |uc(k)| < ucMax
    %           = I(k-1)               en otro caso (anti-windup)
    function uc = calcularPID(e)
        uc = pidc.Kp*e + pidc.Ki*pidc.I - pidc.Kd*meas.vhat;
        if abs(e) < 3 && abs(uc) < P.ucMax          % anti-windup
            pidc.I = max(-pidc.Imax, min(pidc.Imax, pidc.I + e*SAMPLE_TIME));
        end
    end

    % Lugar de Raíces (z) (lineal): compensador de adelanto discretizado por
    % Tustin en disenarLeadZ, aplicado como filtro recursivo (aplicarRecursivo).
    %   C(z) = Kc*(z - zcz) / (z - pcz) = rlz.Cn(z) / rlz.Cd(z)
    %        (rlz.Cn, rlz.Cd ya normalizados con rlz.Cd(1)=1; valores
    %         numéricos impresos en consola por calcularControladores())
    %   Ecuación de diferencias (con Cd(1)=1):
    %     uc(k) = Cn(1)*e(k) + Cn(2)*e(k-1) - Cd(2)*uc(k-1)
    function uc = calcularRootLocusZ(e)
        [uc, rlz] = aplicarRecursivo(rlz, e, P.ucMax);
    end

    % Oscilaciones Muertas (lineal): filtro recursivo de dead-beat, diseñado
    % en disenarControladores a partir de Gz=c2d(K/(tau*s^2+s), Ts*OD_N) y
    % evaluado solo cada OD_N muestras (mantiene el valor entre muestras);
    % se aplica una ganancia OD_GANANCIA<1 para dar margen ante errores de K.
    %   C(z) = od.Cn(z) / od.Cd(z)   (coeficientes numéricos impresos en
    %          consola por calcularControladores(); evaluado a Ts*OD_N)
    %   Ecuación de diferencias, solo cuando mod(cnt-1,OD_N)==0 (k' = índice
    %   de actualización lenta, k'=k/OD_N):
    %     u_raw(k') = Cn(1)*e(k') + Cn(2)*e(k'-1) + ...
    %                 - Cd(2)*u_raw(k'-1) - Cd(3)*u_raw(k'-2) - ...
    %     u(k') = sat(u_raw(k') * OD_GANANCIA, ±ucMax)
    %     uc(k) = u(k')   para todo k entre dos actualizaciones (hold)
    function uc = calcularOscMuertas(e)
        od.cnt = od.cnt + 1;
        if mod(od.cnt-1, OD_N) == 0
            od.be = [e, od.be(1:end-1)];
            u = od.Cn * od.be' - od.Cd(2:end) * od.bu';
            u = max(-P.ucMax, min(P.ucMax, u*OD_GANANCIA));
            od.bu = [u/OD_GANANCIA, od.bu(1:end-1)];
            od.uh = u;
        end
        uc = od.uh;
    end

    % Servosistema (lineal): controlador de estados por realimentación +
    % integrador discreto + observador (estimador) de estado, diseñado en
    % disenarControladores (place sobre el sistema ampliado Aa/Ba, y sobre
    % Ad'/Cd' para el observador). No es un C(z) escalar simple: es un
    % sistema en espacio de estados con entrada e(k) y salida de medición y(k).
    %   Ecuaciones de diferencias implementadas:
    %     xi(k+1) = xi(k) + e(k)        si |u_raw(k)|<=ucMax o sign(e)~=sign(u_raw)
    %             = xi(k)                en otro caso (anti-windup)
    %     u_raw(k) = -Ka(1:2)*x(k) - Ka(3)*xi(k)
    %     uc(k)    = sat(u_raw(k), ±ucMax)
    %     x(k+1)   = Ad*x(k) + Bd*uc(k) + L*(y(k) - Cd*x(k))      (observador)
    function uc = calcularServosistema(y, e)
        x     = srv.x;
        u_raw = -(srv.Ka(1:2) * x) - srv.Ka(3) * srv.xi;
        uc    = max(-P.ucMax, min(P.ucMax, u_raw));
        if abs(u_raw) <= P.ucMax || sign(e) ~= sign(u_raw)
            srv.xi = max(-srv.ximax, min(srv.ximax, srv.xi + e));
        end
        innov  = y - srv.Cd * x;
        srv.x  = srv.Ad * x + srv.Bd * uc + srv.L * innov;
    end

    %% ── CONTROLADORES — ROTACIÓN, LAZO EXTERNO (base -> ref. péndulo) ──
    % e aquí es eExt = rotRefRamp - measRot.y (error de ángulo de BASE);
    % la salida uc es la referencia de péndulo (antes de rate-limit/banda
    % muerta, aplicados en loopControl), NO un PWM.
    %
    % PID discreto (externo). Misma estructura que calcularPID: derivativo
    % sobre velocidad angular de BASE medida (measRot.vhat), integral con
    % anti-windup condicional (congela si |e|>=5 o |uc|>=ucMax=Pext.ucMax).
    %   Ecuación de diferencias:
    %     uc(k) = Kp*e(k) + Ki*I(k) - Kd*measRot.vhat(k)
    %     I(k)  = I(k-1) + e(k)*Ts    si |e(k)|<5 y |uc(k)|<Pext.ucMax
    function uc = calcularPIDExt(e)
        uc = pidcExt.Kp*e + pidcExt.Ki*pidcExt.I - pidcExt.Kd*measRot.vhat;
        if abs(e) < 5 && abs(uc) < Pext.ucMax
            pidcExt.I = max(-pidcExt.Imax, min(pidcExt.Imax, pidcExt.I + e*SAMPLE_TIME));
        end
    end

    % Lugar de Raíces (z) (externo): mismo compensador de adelanto que el
    % lineal, diseñado con la planta Pext (K, wc) y TAU_MOTOR_EXT.
    %   C(z) = Kc*(z - zcz) / (z - pcz) = rlzExt.Cn(z) / rlzExt.Cd(z)
    %   Ecuación de diferencias (con Cd(1)=1):
    %     uc(k) = Cn(1)*e(k) + Cn(2)*e(k-1) - Cd(2)*uc(k-1)
    function uc = calcularRootLocusZExt(e)
        [uc, rlzExt] = aplicarRecursivo(rlzExt, e, Pext.ucMax);
    end

    % Oscilaciones Muertas (externo): misma mecánica que calcularOscMuertas,
    % evaluado cada OD_N_ROT muestras, planta Pext.
    %   C(z) = odExt.Cn(z) / odExt.Cd(z)
    %   Ecuación de diferencias (k' = índice de actualización lenta):
    %     u_raw(k') = Cn(1)*e(k') + Cn(2)*e(k'-1) + ...
    %                 - Cd(2)*u_raw(k'-1) - ...
    %     u(k') = sat(u_raw(k') * OD_GANANCIA_ROT, ±Pext.ucMax)
    %     uc(k) = u(k')   (hold entre actualizaciones)
    function uc = calcularOscMuertasExt(e)
        odExt.cnt = odExt.cnt + 1;
        if mod(odExt.cnt-1, OD_N_ROT) == 0
            odExt.be = [e, odExt.be(1:end-1)];
            u = odExt.Cn * odExt.be' - odExt.Cd(2:end) * odExt.bu';
            u = max(-Pext.ucMax, min(Pext.ucMax, u*OD_GANANCIA_ROT));
            odExt.bu = [u/OD_GANANCIA_ROT, odExt.bu(1:end-1)];
            odExt.uh = u;
        end
        uc = odExt.uh;
    end

    % Servosistema (externo): mismo esquema estados+integrador+observador
    % que calcularServosistema, planta Pext, con y = measRot.y (ángulo base).
    %   Ecuaciones de diferencias:
    %     xi(k+1) = xi(k) + e(k)        si |u_raw(k)|<=Pext.ucMax o sign(e)~=sign(u_raw)
    %     u_raw(k) = -Ka(1:2)*x(k) - Ka(3)*xi(k)
    %     uc(k)    = sat(u_raw(k), ±Pext.ucMax)
    %     x(k+1)   = Ad*x(k) + Bd*uc(k) + L*(y(k) - Cd*x(k))
    function uc = calcularServosistemaExt(y, e)
        x     = srvExt.x;
        u_raw = -(srvExt.Ka(1:2) * x) - srvExt.Ka(3) * srvExt.xi;
        uc    = max(-Pext.ucMax, min(Pext.ucMax, u_raw));
        if abs(u_raw) <= Pext.ucMax || sign(e) ~= sign(u_raw)
            srvExt.xi = max(-srvExt.ximax, min(srvExt.ximax, srvExt.xi + e));
        end
        innov     = y - srvExt.Cd * x;
        srvExt.x  = srvExt.Ad * x + srvExt.Bd * uc + srvExt.L * innov;
    end

    %% ── CONTROLADORES — ROTACIÓN, LAZO INTERNO (péndulo -> PWM) ──
    % e aquí es eInt = pendRef - measPen.y (error de ángulo de PÉNDULO);
    % la salida uc es el comando virtual que, tras la compensación de zona
    % muerta en loopControl (sign(uc)*(Pint.Udz+abs(uc))), se vuelve el PWM
    % final V<pwmRot> enviado al ESP32 (motor de rotación).
    %
    % PID discreto (interno). Derivativo sobre velocidad angular de PÉNDULO
    % medida (measPen.vhat), integral con anti-windup condicional (congela
    % si |e|>=5 o |uc|>=Pint.ucMax).
    %   Ecuación de diferencias:
    %     uc(k) = Kp*e(k) + Ki*I(k) - Kd*measPen.vhat(k)
    %     I(k)  = I(k-1) + e(k)*Ts    si |e(k)|<5 y |uc(k)|<Pint.ucMax
    function uc = calcularPIDInt(e)
        uc = pidcInt.Kp*e + pidcInt.Ki*pidcInt.I - pidcInt.Kd*measPen.vhat;
        if abs(e) < 5 && abs(uc) < Pint.ucMax
            pidcInt.I = max(-pidcInt.Imax, min(pidcInt.Imax, pidcInt.I + e*SAMPLE_TIME));
        end
    end

    % Lugar de Raíces (z) (interno): mismo compensador de adelanto,
    % diseñado con la planta Pint (Udz, K, wc) y TAU_MOTOR_INT.
    %   C(z) = Kc*(z - zcz) / (z - pcz) = rlzInt.Cn(z) / rlzInt.Cd(z)
    %   Ecuación de diferencias (con Cd(1)=1):
    %     uc(k) = Cn(1)*e(k) + Cn(2)*e(k-1) - Cd(2)*uc(k-1)
    function uc = calcularRootLocusZInt(e)
        [uc, rlzInt] = aplicarRecursivo(rlzInt, e, Pint.ucMax);
    end

    % Oscilaciones Muertas (interno): misma mecánica, evaluado cada
    % OD_N_ROT muestras, planta Pint.
    %   C(z) = odInt.Cn(z) / odInt.Cd(z)
    %   Ecuación de diferencias (k' = índice de actualización lenta):
    %     u_raw(k') = Cn(1)*e(k') + Cn(2)*e(k'-1) + ...
    %                 - Cd(2)*u_raw(k'-1) - ...
    %     u(k') = sat(u_raw(k') * OD_GANANCIA_ROT, ±Pint.ucMax)
    %     uc(k) = u(k')   (hold entre actualizaciones)
    function uc = calcularOscMuertasInt(e)
        odInt.cnt = odInt.cnt + 1;
        if mod(odInt.cnt-1, OD_N_ROT) == 0
            odInt.be = [e, odInt.be(1:end-1)];
            u = odInt.Cn * odInt.be' - odInt.Cd(2:end) * odInt.bu';
            u = max(-Pint.ucMax, min(Pint.ucMax, u*OD_GANANCIA_ROT));
            odInt.bu = [u/OD_GANANCIA_ROT, odInt.bu(1:end-1)];
            odInt.uh = u;
        end
        uc = odInt.uh;
    end

    % Servosistema (interno): mismo esquema estados+integrador+observador,
    % planta Pint, con y = measPen.y (ángulo de péndulo).
    %   Ecuaciones de diferencias:
    %     xi(k+1) = xi(k) + e(k)        si |u_raw(k)|<=Pint.ucMax o sign(e)~=sign(u_raw)
    %     u_raw(k) = -Ka(1:2)*x(k) - Ka(3)*xi(k)
    %     uc(k)    = sat(u_raw(k), ±Pint.ucMax)
    %     x(k+1)   = Ad*x(k) + Bd*uc(k) + L*(y(k) - Cd*x(k))
    function uc = calcularServosistemaInt(y, e)
        x     = srvInt.x;
        u_raw = -(srvInt.Ka(1:2) * x) - srvInt.Ka(3) * srvInt.xi;
        uc    = max(-Pint.ucMax, min(Pint.ucMax, u_raw));
        if abs(u_raw) <= Pint.ucMax || sign(e) ~= sign(u_raw)
            srvInt.xi = max(-srvInt.ximax, min(srvInt.ximax, srvInt.xi + e));
        end
        innov     = y - srvInt.Cd * x;
        srvInt.x  = srvInt.Ad * x + srvInt.Bd * uc + srvInt.L * innov;
    end

    %% ══════════════════════════════════════════════════════════
    %  SENSOR / SIMULACIÓN
    %% ══════════════════════════════════════════════════════════
    function [y, yr, yp, tt, nuevo] = leerSensor()
        if estado.modoManual
            raw  = simu.y    + 0.15*randn();
            rawR = simuRot.y + 0.30*randn();
            rawP = simuPen.y + 0.30*randn();
            simu.yf    = 0.5*raw  + 0.5*simu.yf;
            simuRot.yf = 0.5*rawR + 0.5*simuRot.yf;
            simuPen.yf = 0.5*rawP + 0.5*simuPen.yf;
            simu.t  = simu.t + SAMPLE_TIME;
            y = simu.yf;  yr = simuRot.yf;  yp = simuPen.yf;  tt = simu.t;  nuevo = true;
        else
            [yy, yyR, yyP, t2, ok] = leerUltimaLinea();
            if ok
                y = yy;  yr = yyR;  yp = yyP;  tt = t2;  nuevo = true;
            else
                y = meas.y;  yr = measRot.y;  yp = measPen.y;  tt = meas.t;  nuevo = false;
            end
        end
    end

    % Vacía el buffer y se queda con la última telemetría válida.
    % Formato: Tiempo_s,Referencia_rel_cm,Posicion_rel_cm,PWM_lin,
    %          Referencia_rot_deg,Posicion_rot_deg,PWM_rot,Angulo_pendulo_deg
    function [y, yr, yp, tt, ok] = leerUltimaLinea()
        y = NaN;  yr = NaN;  yp = NaN;  tt = NaN;  ok = false;
        try
            while estado.arduino.NumBytesAvailable > 0
                linea = char(strtrim(readline(estado.arduino)));
                if isempty(linea); continue; end
                if startsWith(linea,'ERROR:')
                    estado.alarma = linea;  continue;
                end
                if startsWith(linea,'OK:') || startsWith(linea,'PONG') || contains(linea,'Tiempo')
                    continue;
                end
                partes = strsplit(linea, ',');
                if numel(partes) >= 8
                    tv  = str2double(partes{1});
                    yv  = str2double(partes{3});
                    yrv = str2double(partes{6});
                    ypv = str2double(partes{8});
                    if ~isnan(tv) && ~isnan(yv)
                        y = yv;  tt = tv;  ok = true;
                        if ~isnan(yrv); yr = yrv; end
                        if ~isnan(ypv); yp = ypv; end
                        if startsWith(estado.alarma,'ERROR:'); estado.alarma = ''; end
                    end
                end
            end
        catch
        end
    end

    function [y, yr, yp, tt] = esperarMedicion(tmax)
        y = NaN;  yr = NaN;  yp = NaN;  tt = NaN;
        t0 = tic;
        while toc(t0) < tmax && isnan(y)
            [yy, yyR, yyP, t2, ok] = leerUltimaLinea();
            if ok
                y = yy;  yr = yyR;  yp = yyP;  tt = t2;
            else
                pause(0.02);
            end
        end
        if isnan(yr); yr = 0; end
        if isnan(yp); yp = 0; end
    end

    % Planta simulada lineal: velocidad con zona muerta y lag, 1 muestra de retardo
    function avanzarPlantaSim(u)
        ua = simu.up;  simu.up = u;
        dt = 0.005;
        for i = 1:round(SAMPLE_TIME/dt)
            if abs(ua) > SIM_UDZ
                vss = SIM_K * (ua - sign(ua)*SIM_UDZ);
            else
                vss = 0;
            end
            simu.v = simu.v + dt/SIM_TAU * (vss - simu.v);
            simu.y = simu.y + simu.v*dt;
        end
        simu.y = max(-1, min(MAX_DIST_CM + 1, simu.y));
    end

    % Planta simulada de rotación (CASCADA): el ángulo de BASE responde
    % directamente al PWM de rotación igual que el carro lineal (zona
    % muerta + lag de primer orden); el PÉNDULO tiene su propia dinámica
    % de primer orden, excitada por el MISMO PWM pero sin zona muerta
    % propia (no tiene actuador propio, solo se mide). Esto es una
    % aproximación para probar la lógica de la GUI en el escritorio, no
    % un modelo físico riguroso del acoplamiento base/péndulo real.
    function avanzarPlantaSimRot(u)
        ua = simuRot.up;  simuRot.up = u;
        dt = 0.005;
        for i = 1:round(SAMPLE_TIME/dt)
            if abs(ua) > SIM_UDZ_BASE
                vss = SIM_K_BASE * (ua - sign(ua)*SIM_UDZ_BASE);
            else
                vss = 0;
            end
            simuRot.v = simuRot.v + dt/SIM_TAU_BASE * (vss - simuRot.v);
            simuRot.y = simuRot.y + simuRot.v*dt;

            vssP = SIM_K_PEND * ua;
            simuPen.v = simuPen.v + dt/SIM_TAU_PEND * (vssP - simuPen.v);
            simuPen.y = simuPen.y + simuPen.v*dt;
        end
        simuRot.y = max(-MAX_ROT_DEG-1, min(MAX_ROT_DEG+1, simuRot.y));
        simuPen.y = max(-PEND_REF_MAX_DEG-5, min(PEND_REF_MAX_DEG+5, simuPen.y));
    end

    %% ══════════════════════════════════════════════════════════
    %  GRÁFICAS Y UTILIDADES
    %% ══════════════════════════════════════════════════════════
    function actualizarGraficas()
        n = numel(estado.tiempoVec);
        if n == 0; return; end
        i0 = max(1, n - 300);
        idx = i0:n;
        t = estado.tiempoVec(idx);
        ventana = 11;
        xLim = [max(0, t(end)-ventana), max(ventana, t(end))];

        set(hRefLine, 'XData',t,'YData',estado.refVec(idx));
        set(hMedLine, 'XData',t,'YData',estado.distVec(idx));
        ax1.XLim = xLim;
        ylo = min([estado.distVec(idx), estado.refVec(idx)]) - 0.6;
        yhi = max([estado.distVec(idx), estado.refVec(idx)]) + 0.6;
        ax1.YLim = [ylo, max(yhi, ylo+1)];

        set(hVelLine, 'XData',t,'YData',estado.velVec(idx));
        ax2.XLim = xLim;
        pk = max(1, max(abs(estado.velVec(idx))));
        ax2.YLim = [-pk*1.1, pk*1.1];

        set(hCtrlLine,'XData',t,'YData',estado.controlVec(idx));
        ax3.XLim = xLim;
        ax3.YLim = [-U_MAX-10, U_MAX+10];

        set(hErrLine, 'XData',t,'YData',estado.errorVec(idx));
        ax4.XLim = xLim;
        pe = max(0.5, max(abs(estado.errorVec(idx))));
        ax4.YLim = [-pe*1.1, pe*1.1];

        set(hRefLineRot, 'XData',t,'YData',estado.refRotVec(idx));
        set(hMedLineRot, 'XData',t,'YData',estado.posRotVec(idx));
        ax1r.XLim = xLim;
        yloR = min([estado.posRotVec(idx), estado.refRotVec(idx)]) - 2;
        yhiR = max([estado.posRotVec(idx), estado.refRotVec(idx)]) + 2;
        ax1r.YLim = [yloR, max(yhiR, yloR+1)];

        set(hVelLineRot, 'XData',t,'YData',estado.velRotVec(idx));
        ax2r.XLim = xLim;
        pkR = max(1, max(abs(estado.velRotVec(idx))));
        ax2r.YLim = [-pkR*1.1, pkR*1.1];

        set(hCtrlLineRot,'XData',t,'YData',estado.controlRotVec(idx));
        ax3r.XLim = xLim;
        ax3r.YLim = [-U_MAX-10, U_MAX+10];

        set(hErrLineRot, 'XData',t,'YData',estado.errorRotVec(idx));
        ax4r.XLim = xLim;
        peR = max(0.5, max(abs(estado.errorRotVec(idx))));
        ax4r.YLim = [-peR*1.1, peR*1.1];

        set(hRefLinePend, 'XData',t,'YData',estado.pendRefVec(idx));
        set(hMedLinePend, 'XData',t,'YData',estado.pendVec(idx));
        ax1p.XLim = xLim;
        yloP = min([estado.pendVec(idx), estado.pendRefVec(idx)]) - 2;
        yhiP = max([estado.pendVec(idx), estado.pendRefVec(idx)]) + 2;
        ax1p.YLim = [yloP, max(yhiP, yloP+1)];

        set(hVelLinePend, 'XData',t,'YData',estado.velPendVec(idx));
        ax2p.XLim = xLim;
        pkP = max(1, max(abs(estado.velPendVec(idx))));
        ax2p.YLim = [-pkP*1.1, pkP*1.1];

        set(hErrLinePend, 'XData',t,'YData',estado.errorPendVec(idx));
        ax4p.XLim = xLim;
        peP = max(0.5, max(abs(estado.errorPendVec(idx))));
        ax4p.YLim = [-peP*1.1, peP*1.1];

        drawnow limitrate;
    end

    function configurarEjes(ax,titulo,xlab,ylab)
        ax.Color = [1 1 1];  ax.XColor = [0.15 0.15 0.15];  ax.YColor = [0.15 0.15 0.15];
        ax.GridColor = [0.7 0.7 0.7];  ax.GridAlpha = 0.8;
        ax.XGrid = 'on';  ax.YGrid = 'on';  ax.Box = 'on';  ax.FontSize = 8;
        ax.Title.String = titulo;  ax.Title.FontSize = 9;  ax.Title.FontWeight = 'bold';
        xlabel(ax,xlab,'FontSize',8);
        ylabel(ax,ylab,'FontSize',8);
    end

    function actualizarStatus()
        nombres = {'PID Discreto','Lugar de Raíces (z)','Osc. Muertas','Servosistema'};
        idx  = strcmp({'PID','RootLocusZ','OscMuertas','Servosistema'}, estado.controlador);
        idxR = strcmp({'PID','RootLocusZ','OscMuertas','Servosistema'}, estado.controladorRot);
        if any(idx);  nom  = nombres{idx};  else; nom  = estado.controlador;    end
        if any(idxR); nomR = nombres{idxR}; else; nomR = estado.controladorRot; end
        if estado.corriendo
            s = 'CORRIENDO';  c = [0.10 0.60 0.20];
        else
            s = 'DETENIDO';   c = [0.70 0.10 0.10];
        end
        txt = sprintf(['Estado: %s  |  Lineal: %s Ref:%.1fcm(%.1f) Pos:%.2fcm  |  ' ...
            'Rot(cascada): %s Base Ref:%.1f°(%.1f) Pos:%.2f°  Péndulo Ref:%.1f° Pos:%.2f°'], ...
            s, nom, estado.distRef, estado.distRefRamp, meas.y, ...
            nomR, estado.rotRef, estado.rotRefRamp, measRot.y, ...
            estado.u_prev_c_ext, measPen.y);
        if ~isempty(estado.alarma); txt = [txt '  |  ' estado.alarma]; end
        lblStatus.String = txt;
        lblStatus.ForegroundColor = c;
    end

end % ── fin ControlMotor_GUI
