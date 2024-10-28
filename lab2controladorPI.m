%% Planta-controlador-sensor-actuador
s = tf('s');
tau = 0.045559;
Kp = 125.6637/12;
Ka = 12;
Ks = 66.845e-3;
Kc = 0.125;
Gc = Kc*(s+44.82)/s;
Gp = Kp/(tau*s+1);
Glac = Ka*Ks*Gp*Gc;
Glcc = feedback(Glac,1);

%% Respuesta al escalón

step(Glcc);


%% Planta-controlador-sensor-actuador contemplando el acondicionador
s = tf('s');
tau = 0.045559;
Kp = 125.6637/12;
Ka = 12;
Ks = (66.845e-3)*(10/21);
Kc = 0.2625;
Gc = Kc*(s+44.82)/s;
Gp = Kp/(tau*s+1);
Glac = Ka*Ks*Gp*Gc;
Glcc = feedback(Glac,1);

%% Respuesta al escalón

step(12*Glcc);

%% Acción de control
U = feedback(Gc,Ka*Gp*Ks);
step(U);

%% Tiempo discreto
Kcs = 0.20102;
Gcs = (Kcs*(s+40.92))/s;
Glacs = Gp*Ks*Ka*Gcs;
Glccs = feedback(Glacs,1);
T = 6e-3;
z = tf('z',T);

%Gcd1 = Kp+(Kcs*T*z)/(z-1);
Gcd2 = (Kcs*(((2/T)*(((z-1)/(z+1))+40.92)))/((2/T)*((z-1)/(z+1))));
Gcd2 = minreal(Gcd2);
Gpd = c2d(Gp,T,'zoh');
Gcd = c2d(Gcs,T,'tustin');
Glacd = Ks*Ka*Gpd*Gcd2;
Glccd = feedback(Glacd,1);

stepplot(Glccs,Glccd);



