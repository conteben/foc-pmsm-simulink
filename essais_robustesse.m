% essais_robustesse.m
% Compare le PI de reference et le correcteur H-infini pour J = J0, 2*J0, 3*J0.
%
% Prerequis dans foc_pmsm.slx :
%   - bloc Step de consigne de vitesse : Final value = w_ref
%   - bloc Step de couple de charge    : Final value = TL_step (instant 0,1 s)
%   - bloc PID de vitesse : P = Kpw, I = Kiw (deja le cas)
%   - deux blocs To Workspace, format Timeseries : wm_log (vitesse) et iq_log (iq)
% Lancer hinf_vitesse.m avant ce script.

w_ref   = 10;          % rad/s : assez petit pour que iq* ne sature pas
TL_step = 0.05;        % N.m

noms     = {'PI (wn = 200)', 'H-infini LMI'};
gains    = [Kpw_pi, Kiw_pi; Kpw_hinf, Kiw_hinf];
fichiers = {'robustesse_pi.png', 'robustesse_hinf.png'};
facteurs = [1 2 3];

fprintf('\n%-16s %5s %12s %14s\n', 'Correcteur', 'J/J0', 'Depassement', 'Creux (rad/s)');
for c = 1:2
    Kpw = gains(c,1);
    Kiw = gains(c,2);
    figure('Name', noms{c});
    for k = facteurs
        J = k*J0;
        out = sim('commande_vectorielle_FOC');
        t  = out.wm_log.Time;   wm = out.wm_log.Data;
        ti = out.iq_log.Time;   iq = out.iq_log.Data;

        m1 = t > 0.01 & t < 0.1;
        m2 = t > 0.1;
        dep   = (max(wm(m1))/w_ref - 1)*100;
        creux = w_ref - min(wm(m2));
        fprintf('%-16s %5d %11.1f%% %14.2f\n', noms{c}, k, dep, creux);

        subplot(2,1,1); plot(t, wm, 'DisplayName', sprintf('J = %d J_0', k)); hold on;
        subplot(2,1,2); plot(ti, iq, 'DisplayName', sprintf('J = %d J_0', k)); hold on;
    end
    subplot(2,1,1); grid on; ylabel('\omega_m (rad/s)'); title(noms{c}); legend('Location', 'southeast');
    subplot(2,1,2); grid on; ylabel('i_q (A)'); xlabel('t (s)');
    exportgraphics(gcf, fullfile('figures', fichiers{c}), 'Resolution', 150);
end

% Remise des valeurs nominales
J = J0;  Kpw = Kpw_pi;  Kiw = Kiw_pi;
