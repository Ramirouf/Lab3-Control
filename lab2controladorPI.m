%% Planta-controlador-sensor-actuador contemplando el acondicionador
s = tf('s');
tau = 0.045559;
Kp = 125.6637/12;
Ka = 12;
Kn = (1/4);
Ks = (66.845e-3)*(10/21)*Kn;%se agrega 1/4 para obtener 1pu
ts = 0.2;
psita = 0.7;
wn = 4.5/(psita*ts);
Kc = (2*psita*wn*tau-1)/(Ka*Ks*Kp);
zpi = (wn*wn*tau)/(Ka*Ks*Kp*Kc);
Gc = Kc*(s+zpi)/s;
Gp = Kp/(tau*s+1);
Glac = Ka*Ks*Gp*Gc;
Glcc = feedback(Glac,1);

%% Respuesta al escal�n
figure;
step(Glcc);

% Acci�n de control
U = feedback(Gc,Ka*Gp*Ks);
figure; 
step(U);

%% Tiempo discreto
Kcs = 0.58165;
zpis = 29.51;
Gcs = (Kcs*(s+zpis))/s;
Glacs = Gp*Ks*Ka*Gcs;
Glccs = feedback(Glacs,1);
T = 12e-3;
z = tf('z',T);
% Se calcula el controlador en tiempo discreto
st = (2/T)*(((z-1)/(z+1)));
Gcd = minreal((Kcs*(st+zpis))/(st));
%planta en tiempo discreto
Gpd = c2d(Gp,T,'zoh');

Glacd = Ks*Ka*Gpd*Gcd;
Glccd = feedback(Glacd,1);

stepplot(0.9*Glccs,0.9*Glccd);
%% Compensaci�n ZOH
epsi = 0.175;
Cz = (2*(z-epsi))/(z+1-2*epsi);
Gcd2 = Gcd*Cz;
Glacdc = Ks*Ka*Gpd*Gcd2;
Glccdc = feedback(Glacdc,1);
stepplot(0.9*Glccs,0.9*Glccdc);

