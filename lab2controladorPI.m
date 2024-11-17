%% LAB2 - G1 - Sistemas de Control y Automatizaci�n 
s = tf('s');

Vel_nominal= 1200*(pi/30);
Vcc=12;

% Mejor modelo de 1er orden del LAB1 (Sundaresan-Krishnaswamy) 
tau = 0.045559;
Kp = Vel_nominal/Vcc;               % veloc. nominal en rad/s / Tension nominal
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

%% Respuesta al escal�n LCNC
figure;
step(Glcnc);
legend('Glcc(s)');
title('Respuesta al Escal�n a lazo cerrado No compensado');

%% Especificaciones de desempe�o transitorio
% El sobrepaso debe ser menor al 5% Y ts menor a 1500 s
Mp=5/100; ts=0.2;
sigma=4.5/ts; wd=-pi*sigma/log(Mp);
xita=-log(Mp)/sqrt(pi^2+(log(Mp))^2);
wn=sigma/xita;

% Resoluci�n por reubicaci�n de polos
Kc = (2*xita*wn*tau-1)/(Kla*Kp);
zpi = (wn*wn*tau)/(Kla*Kp*Kc);

%controlador
Gc = Kc*(s+zpi)/s;

%FTLA compensada
Glac = Kla*Gp*Gc;
%FTLC compensada
Glcc = feedback(Glac,1);

%% Respuesta al escal�n
figure;
% gr�fico en el lado izquierdo (Respuesta al Escal�n)
subplot(1, 2, 1); % 1 fila, 2 columnas, posici�n 1
step(Glcc);
legend('Glcc(s)');
title('Respuesta al Escal�n a Lazo Cerrado Compensado');
xlabel('Tiempo (s)');
ylabel('Amplitud');

U = feedback(Gc, Gp*Kla);
% gr�fico en el lado derecho (Acci�n de Control)
subplot(1, 2, 2); % 1 fila, 2 columnas, posici�n 2
step(U);
legend('U(s)');
title('Acci�n de Control a Lazo Cerrado Compensado');
xlabel('Tiempo (s)');
ylabel('Amplitud');

%% Reajuste mediante sisotool

% sisotool(Gp*Ka,Gc,Ks*Kn,Vref);

  % 0.50946 (s+36.29)
  % -----------------
  %         s

Kc = 0.50946; zpi = 36.29;
Gc = Kc*((s+zpi)/s);

Glac = Gp*Ka*Gc*Ks*Kn;
Glcc = feedback(Glac,1);

%% Respuesta al escal�n
figure;
% gr�fico en el lado izquierdo (Respuesta al Escal�n)
subplot(1, 2, 1); % 1 fila, 2 columnas, posici�n 1
step(Vref*Glcc);
legend('Glcc(s)');
title('Respuesta al Escal�n a Lazo Cerrado Compensado');
xlabel('Tiempo (s)');
ylabel('Amplitud');

U = feedback(Gc, Gp*Kla);
% gr�fico en el lado derecho (Acci�n de Control)
subplot(1, 2, 2); % 1 fila, 2 columnas, posici�n 2
step(Vref*U);
legend('U(s)');
title('Acci�n de Control a Lazo Cerrado Compensado');
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
h=stepplot(Vref*Glcc,Vref*Glccd);
h.Response(1).Name = 'FTLC compensada en TC';
h.Response(2).Name = 'FTLC compensada en TD';
legend show;
title('Respuesta al escalon FTLC en tiempo continuo y discreto');

Ud = feedback(Gcd, Gpd*Kla);
figure; 
h=stepplot(Vref*U,Vref*Ud);
h.Response(1).Name = 'U(s) en TC';
h.Response(2).Name = 'U(z) en TD';
legend show;
title('Accion de control FTLC en tiempo continuo y discreto');

%% Compensaci�n ZOH
epsi = 0.2;
Cz = (2*(z-epsi))/(z+1-2*epsi);
Gczd = Gcd*Cz;
Glaczd = Kla*Gpd*Gczd;
Glcczd = feedback(Glaczd,1);

%% Respuesta y Accion de control PI con compensaci�n de ZOH
figure; 
h=stepplot(Vref*Glcc,Vref*Glccd, Vref*Glcczd);
h.Response(1).Name = 'FTLC en TC';
h.Response(2).Name = 'FTLC en TD';
h.Response(3).Name = 'FTLC en TD c/ compensaci�n de ZOH';
legend show;
title('Respuesta al escal�n con PI considerando atraso de implementaci�n');


Uczd = feedback(Gczd, Gpd*Kla);
figure; 
h=stepplot(Vref*U,Vref*Ud,Vref*Uczd);
h.Response(1).Name = 'U(s) en TC';
h.Response(2).Name = 'U(z) en TD';
h.Response(3).Name = 'U(z) en TD c/ compensaci�n de ZOH';
legend show;
title('Acci�n de control con PI considerando atraso de implementaci�n');

%% Obtener valores de PD a partir de PI
%% Especificaciones de desempe�o transitorio
% El sobrepaso debe ser menor al 1% Y ts menor a 0.2 s
Mp=1/100; ts=0.2;
sigma=4.5/ts; wd=-pi*sigma/log(Mp);
xita=-log(Mp)/sqrt(pi^2+(log(Mp))^2);
wn=sigma/xita;
pd1=-sigma+1i*wd; pd2=pd1';
%TRANSFORMACION CONFORME e^(pd*T)
pdz1 = exp(pd1*T); pdz2=pdz1';

% hallado en Mathcad a partir de resolucion de ecuacion
zpd=0.50749; Kpd = 1.85;
Gcpd = Kpd*(z-zpd)/z; % compensador PD calculado.

Glccd_PID = feedback(Gcd*Kla*Gpd*Gcpd,1);
Glccd_PIDcz = feedback(Gczd*Kla*Gpd*Gcpd, 1);

%% Respuesta y Accion de control con compensacion de ZOH para PID
figure; 
h=stepplot(Vref*Glcc,Vref*Glccd_PID, Vref*Glccd_PIDcz);
h.Response(1).Name = 'FTLC en TC';
h.Response(2).Name = 'FTLC en TD';
h.Response(3).Name = 'FTLC en TD c/ compensaci�n ZOH';
legend show;
title('Respuesta al escal�n con PID considerando atraso de implementaci�n');

Upid = feedback(Gcd*Gcpd, Gpd*Kla);
Uczpid = feedback(Gczd*Gcpd, Gpd*Kla);
figure; 
h=stepplot(Vref*U,Vref*Upid,Vref*Uczpid);
h.Response(1).Name = 'U(s) en TC';
h.Response(2).Name = 'U(z) en TD';
h.Response(3).Name = 'U(z) en TD c/ compensaci�n de ZOH';
legend show;
title('Acci�n de control con PID considerando atraso de implementaci�n');