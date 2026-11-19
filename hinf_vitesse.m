% hinf_vitesse.m
% Synthese H-infini par LMI (YALMIP) du correcteur de vitesse,
% robuste a une inertie J comprise entre J0 et 3*J0.
%
% Etat augmente : x = [e ; xi], avec e = wm - wm* et xi = integrale de e.
% Retour d'etat u = iq* = K*x. Comme le procede est du premier ordre,
% ce retour d'etat a exactement la structure d'un PI :
%   iq* = Kpw*(wm* - wm) + Kiw*integrale(wm* - wm), avec Kpw = -K(1), Kiw = -K(2).
% Les gains obtenus se mettent donc dans le meme bloc PID que le PI classique.
%
% Prerequis : YALMIP et un solveur SDP (SeDuMi, SDPT3 ou MOSEK).

params;                 % charge Rs, J, B, Kt, Kpw, Kiw...
J0 = J;
Kpw_pi = Kpw;  Kiw_pi = Kiw; % PI de reference (placement de poles, wn = 200)

% --- Choix de synthese ---
TLmax = 0.05;                % couple de charge de reference, N.m (normalise w)
q1 = 1;                      % poids sur l'erreur de vitesse
q2 = 100;                    % poids sur l'integrale de l'erreur
r  = 0.1;                    % poids sur la commande iq*
alpha = 100;                 % decroissance minimale : Re(poles) < -alpha
beta  = 1000;                % bande passante maximale : Re(poles) > -beta
theta = pi/4;                % cone : amortissement >= cos(theta) = 0,707
Jsommets = [J0, 3*J0];       % sommets du polytope d'incertitude

Cz = [q1 0; 0 q2; 0 0];
Dz = [0; 0; r];

% --- Variables de decision ---
X = sdpvar(2,2);             % symetrique, X = P^-1
W = sdpvar(1,2);             % W = K*X
g = sdpvar(1);               % borne gamma sur la norme H-infini

F = [X >= 1e-6*eye(2)];
for Jv = Jsommets
    A  = [-B/Jv 0; 1 0];
    B2 = [Kt/Jv; 0];         % entree de commande iq*
    B1 = [-TLmax/Jv; 0];     % entree de perturbation (couple de charge normalise)

    AX = A*X + B2*W;         % (A + B2*K)*X
    L  = AX + AX';
    Z  = Cz*X + Dz*W;

    % Lemme borne reel : ||T_wz||_inf < gamma
    M = [L,   B1,  Z';
         B1', -g,  zeros(1,3);
         Z,   zeros(3,1), -g*eye(3)];
    F = [F, M <= 0];

    % Region de poles (D-stabilite)
    F = [F, L + 2*alpha*X <= 0];                 % Re(poles) < -alpha
    F = [F, -L - 2*beta*X <= 0];                 % Re(poles) > -beta
    F = [F, [sin(theta)*L,          cos(theta)*(AX - AX');
             cos(theta)*(AX' - AX), sin(theta)*L] <= 0];   % cone d'amortissement
end

opts = sdpsettings('solver', 'sedumi', 'verbose', 0);
res  = optimize(F, g, opts);
if res.problem ~= 0
    error('Synthese LMI : %s', res.info);
end

K = value(W) / value(X);
Kpw_hinf = -K(1);
Kiw_hinf = -K(2);
gamma = value(g);

fprintf('\nSynthese H-infini (LMI)\n');
fprintf('  gamma garanti = %.3f\n', gamma);
fprintf('  Kpw_hinf = %.4f A/(rad/s), Kiw_hinf = %.2f A/rad\n', Kpw_hinf, Kiw_hinf);
fprintf('  PI de reference : Kpw = %.4f, Kiw = %.2f\n', Kpw_pi, Kiw_pi);

% --- Verification a posteriori sur une grille de J ---
fprintf('\n  J/J0   norme Hinf   zeta_min   poles\n');
for k = [1 1.5 2 2.5 3]
    Jv  = k*J0;
    A   = [-B/Jv 0; 1 0];
    B2  = [Kt/Jv; 0];
    B1  = [-TLmax/Jv; 0];
    Acl = A + B2*K;
    sys = ss(Acl, B1, Cz + Dz*K, 0);
    vp  = eig(Acl);
    zeta_min = min(-real(vp)./abs(vp));
    fprintf('  %4.1f   %9.3f   %8.3f   %s\n', k, norm(sys, inf), zeta_min, mat2str(vp.', 4));
end

% Meme verification pour le PI de reference (hors region pour J > J0)
fprintf('\n  PI de reference :\n');
Kpi = [-Kpw_pi, -Kiw_pi];
for k = [1 2 3]
    Jv  = k*J0;
    Acl = [-B/Jv 0; 1 0] + [Kt/Jv; 0]*Kpi;
    vp  = eig(Acl);
    fprintf('  %4.1f   zeta_min = %.3f   %s\n', k, min(-real(vp)./abs(vp)), mat2str(vp.', 4));
end