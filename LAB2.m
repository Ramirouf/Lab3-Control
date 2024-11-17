%% LAB2 - G1 - Sistemas de Control y Automatización 
s = tf('s');

Vel_nominal= 1200*(pi/30);
Vcc=12;

% Mejor modelo de 1er orden del LAB1 (Sundaresan-Krishnaswamy) 
tau = 0.045559;
Kp = Vel_nominal/Vcc;          % veloc. nominal en rad/s / Tension nominal
Gp = Kp/(tau*s+1);

Ka = 12;                       % ganancia del actuador (Tension de Alimentacion)
Ks = (66.845e-3)*(10/21);      % vel. nom. en rad/s * acondicionador de 8,4 a 4 V
Kn = (1/4);                    % ganancia de normalizacion para obtener 1pu
Kla = Ka*Ks*Kn;
Vref = (0.9)*Ks*Kn*Vel_nominal;

%FTLA No compensada
Glanc = Kla * Gp ;
%FTLC No compensada
Glcnc = feedback(Glanc,1);

%% Especificaciones de desempeño transitorio
% El sobrepaso debe ser menor al 5% Y ts menor a 0.4 s
Mp=5/100; ts=0.4;
sigma=4.5/ts; wd=-pi*sigma/log(Mp);
xita=-log(Mp)/sqrt(pi^2+(log(Mp))^2);
wn=sigma/xita;

% Resolución por reubicación de polos
Kc = (2*xita*wn*tau-1)/(Kla*Kp);
zpi = (wn*wn*tau)/(Kla*Kp*Kc);

%controlador
Gc = Kc*(s+zpi)/s;

%FTLA compensada
Glac = Kla*Gp*Gc;
%FTLC compensada
Glcc = feedback(Glac,1);

%% Respuesta al escalón compensado y no compensado
figure;
subplot(1, 2, 1); 
step(Glcnc,Glcc);
legend('Glnc(s)','Glcc(s)');
title('Respuesta al Escalón a Lazo Cerrado con PI');
xlabel('Tiempo (s)');
ylabel('Amplitud');

Unc=feedback(1,Kla* Gp);
U = feedback(Gc, Gp*Kla);

subplot(1, 2, 2);
step(Unc,U);
legend('U(s) No Compensado','U(s) Compensado');
title('Acción de Control a Lazo Cerrado con PI');
xlabel('Tiempo (s)');
ylabel('Amplitud');

%% Reajuste mediante sisotool
% sisotool(Gp*Ka,Gc,Ks*Kn,Vref);

  % 0.17538 (s+46.12)
  % -----------------
  %         s

Kc = 0.17538; zpi = 46.12;
Gc = Kc*((s+zpi)/s);

Glac = Gp*Ka*Gc*Ks*Kn;
Glcc = feedback(Glac,1);

%% Respuesta al escalón compensado y no compensado
figure;
subplot(1, 2, 1); 
step(Glcnc,Glcc);
legend('Glnc(s)','Glcc(s)');
title('Respuesta al Escalón a Lazo Cerrado con PI y Ajuste Sisotool');
xlabel('Tiempo (s)');
ylabel('Amplitud');

Unc=feedback(1,Kla* Gp);
U = feedback(Gc, Gp*Kla);

subplot(1, 2, 2);
step(Unc,U);
legend('U(s) No Compensado','U(s) Compensado');
title('Acción de Control a Lazo Cerrado con PI y Ajuste Sisotool');
xlabel('Tiempo (s)');
ylabel('Amplitud');

%% Tiempo discreto
%Definicion del periodo de muestreo a partir del tiempo de subida
info = stepinfo(Glcc);
tr = info.RiseTime;
Nr = 20; %Numero de muestras entre 4 y 10
T_s = floor((tr/Nr)*1000);
T = T_s/1000;
z = tf('z',T);

% Se aproxima el controlador en tiempo discreto mediante Tustin
sT = (2/T)*((z-1)/(z+1));
Gcd = minreal(Kc*((sT+zpi)/(sT)));

% muestreo de la planta en tiempo discreto
Gpd = c2d(Gp,T)*(1/z); %se tiene en cuenta atraso de implementacion

Glacd = Kla*Gpd*Gcd;
Glccd = feedback(Glacd,1);

%% Respuestas al escalon FTLC y Accion de control TC vs. TD
figure;
subplot(1, 2, 1); 
h=stepplot(Vref*Glcc,Vref*Glccd);
h.Response(1).Name = 'FTLC compensada en TC';
h.Response(2).Name = 'FTLC compensada en TD';
legend show;
title('Respuesta al escalon PI con atraso de 1 período');

Ud = feedback(Gcd, Gpd*Kla);
subplot(1, 2, 2);
h=stepplot(Vref*U,Vref*Ud);
h.Response(1).Name = 'U(s) en TC';
h.Response(2).Name = 'U(z) en TD';
legend show;
title('Accion de control PI con atraso de 1 período');

%% Compensación ZOH
epsi = 0.3;
Cz = (2*(z-epsi))/(z+1-2*epsi);
Gczd = Gcd*Cz;
Glaczd = Kla*Gpd*Gczd;
Glcczd = feedback(Glaczd,1);

%% Respuesta y Accion de control PI con compensación de ZOH
figure;
subplot(1, 2, 1); 
h=stepplot(Vref*Glcc,Vref*Glccd, Vref*Glcczd);
h.Response(1).Name = 'FTLC en TC';
h.Response(2).Name = 'FTLC en TD';
h.Response(3).Name = 'FTLC en TD c/ compensación de ZOH';
legend show;
title('Respuesta al escalon PI con atraso de 1 período');

Uczd = feedback(Gczd, Gpd*Kla);

subplot(1, 2, 2); 
h=stepplot(Vref*U,Vref*Ud,Vref*Uczd);
h.Response(1).Name = 'U(s) en TC';
h.Response(2).Name = 'U(z) en TD';
h.Response(3).Name = 'U(z) en TD c/ compensación de ZOH';
legend show;
title('Accion de control PI con atraso de 1 período');

%% Obtener valores de PD a partir de PI
%% Especificaciones de desempeño transitorio
% El sobrepaso debe ser menor al 1% Y ts menor a 0.2 s
Mp=1/100; ts=0.35;
sigma=4.5/ts; wd=-pi*sigma/log(Mp);
xita=-log(Mp)/sqrt(pi^2+(log(Mp))^2);
wn=sigma/xita;
pd1=-sigma+1i*wd; pd2=pd1';

%TRANSFORMACION CONFORME e^(pd*T)
pdz1 = exp(pd1*T); pdz2=pdz1';

%% Sistema 3 ecuaciones 3 incognitas
% resuelto en Mathcad
zpd=-0.53337267048575644048; Kpd = 0.85543884160060501599;
Gcpd = Kpd*(z-zpd)/z; % compensador PD calculado.

Gcpid = Gcd*Gcpd;
Gczpid = Gczd*Gcpd;

Glccd_PID = feedback(Gcpid*Kla*Gpd,1);
Glccd_PIDcz = feedback(Gczpid*Kla*Gpd, 1);

%% Respuesta y Accion de control con compensacion de ZOH para PID
figure; 
subplot(1, 2, 1);
h=stepplot(Vref*Glcc,Vref*Glccd_PID, Vref*Glccd_PIDcz);
h.Response(1).Name = 'FTLC en TC';
h.Response(2).Name = 'FTLC en TD';
h.Response(3).Name = 'FTLC en TD c/ compensación ZOH';
legend show;
title('Respuesta al escalón con PID considerando atraso de implementación');

% Acciones de control con y sin Cz
Upid = feedback(Gcpid, Gpd*Kla);
Uczpid = feedback(Gczpid, Gpd*Kla);

subplot(1, 2, 2); 
h=stepplot(Vref*U,Vref*Upid,Vref*Uczpid);
h.Response(1).Name = 'U(s) en TC';
h.Response(2).Name = 'U(z) en TD';
h.Response(3).Name = 'U(z) en TD c/ compensación de ZOH';
legend show;
title('Acción de control con PID considerando atraso de implementación');

%% Calculo D a partir de PID
Kpid= 0.18808;
b=-0.5953;
c=0.5334;

Kd=T*b*c*Kpid;
Kpi=Kpid-(Kd/T);
a =(((Kpi*(b+c)) + ((2*Kd)/T))/Kpi);

