Rs = 0.5; Ld = 1e-3; Lq = 1e-3;
psi_f = 0.01; p = 4;
J = 1e-5; B = 1e-5; Vdc = 24;
wc = 2*pi*1000;
Kp = Lq*wc; Ki = Rs*wc;
Vmax = Vdc/sqrt(3);

% Bouble de courant
Kt = 1.5*p*psi_f;
wn = 200; zeta = 1;
Kpw = 2*zeta*wn*J/Kt;   % 0.0667
Kiw = wn^2*J/Kt;        % 6.67
Imax = 2;

% Boucle de vitesse
Kt = 1.5*p*psi_f;      % constante de couple, N.m/A
wn = 200;              % pulsation propre, rad/s
zeta = 1;              % amortissement
Kpw = 2*zeta*wn*J/Kt;  % gain proportionnel
Kiw = wn^2*J/Kt;       % gain integral
Imax = 2;              % limite de courant, A

