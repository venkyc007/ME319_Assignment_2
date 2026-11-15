function make_figures(scenarios, modes)
% Reproduces the scenario figures (figs/scen_<S>_<mode>.png) for the paper.
if nargin < 1, scenarios = {'A','B','C'}; end
if nargin < 2, modes = {'track','ambush'}; end
if exist('OCTAVE_VERSION','builtin') && isempty(getenv('DISPLAY')), graphics_toolkit('gnuplot'); end
if ~exist('figs','dir'), mkdir('figs'); end
P = guardian_params(); d2r = pi/180;
for im = 1:numel(modes)
for is = 1:numel(scenarios)
    sc = scenarios{is}; md = modes{im};
    R = runmode(sc, md, false); L = R.log; t = L.t;
    f = figure('visible',figvis(),'position',[0 0 1100 900]);
    % (a) elevation
    subplot(3,2,[1 2]); hold on;
    shade_blackout(L, [min(L.th_true)/d2r-3, max(L.th_true)/d2r+3]);
    h1 = plot(t, L.th_true/d2r, 'k', 'linewidth', 1.5);
    h2 = plot(t, L.th_hat/d2r, 'b--');
    h3 = plot(t, atan2(L.xpred(:,2), L.xpred(:,1))/d2r, 'm:');
    h4 = plot(t, L.th_ref/d2r, 'g');
    h5 = plot(t, L.th/d2r, 'r', 'linewidth', 1.2);
    ylabel('elevation [deg]'); xlabel('t [s]');
    legend([h1 h2 h3 h4 h5], {'true LOS','estimated LOS','predicted LOS (t+t_p)','reference','turret'},'location','northwest');
    title(sprintf('Scenario %s, %s mode: %d/%d hits, first hit %.2f s', sc, md, R.n_hits, R.n_shots, R.t_intercept));
    grid on; ylim([min(L.th_true)/d2r-3, max(L.th_true)/d2r+3]);
    % (b) trajectory
    subplot(3,2,3); plot(L.X(:,1), L.X(:,2), 'k', L.xhat(:,1), L.xhat(:,2), 'b--'); hold on;
    for k = 1:R.n_shots
        ik = find(t >= R.shots(k,1), 1); th = L.th(ik);
        plot([0 2100*cos(th)], [0 2100*sin(th)], 'color', [1 .6 .6]);
    end
    plot(L.X(1,1), L.X(1,2), 'ko'); axis equal; grid on;
    xlabel('x [m]'); ylabel('y [m]'); title('Target path (truth/estimate) and fired beams');
    xlim([min(L.X(:,1))-100, max(L.X(:,1))+100]); ylim([0, max(L.X(:,2))+150]);
    % (c) perpendicular miss + shots
    subplot(3,2,4); hold on; shade_blackout(L, [0 10]);
    semilogy(t, max(L.true_miss, 1e-2), 'k');
    plot([0 P.t_end], P.R_eng*[1 1], 'r--');
    if R.n_shots > 0
        h = R.shots(:,3) == 1;
        plot(R.shots(h,1), max(R.shots(h,2),1e-2), 'go', 'markerfacecolor', 'g');
        plot(R.shots(~h,1), max(R.shots(~h,2),1e-2), 'rx', 'markersize', 10, 'linewidth', 2);
    end
    set(gca,'yscale','log'); ylim([1e-2 1e3]); grid on;
    xlabel('t [s]'); ylabel('beam-target miss [m]'); title('True perpendicular miss, shots (o hit, x miss)');
    % (d) estimation error and 1-sigma
    subplot(3,2,5); hold on; shade_blackout(L, [0 20]);
    e = hypot(L.xhat(:,1)-L.X(:,1), L.xhat(:,2)-L.X(:,2));
    h1 = plot(t, e, 'b'); h2 = plot(t, L.sig_perp, 'm');
    ylim([0 20]); grid on; xlabel('t [s]'); ylabel('[m]');
    legend([h1 h2], {'|position est. error|','\sigma_\perp (EKF)'}); title('Estimator (grey = radar blackout)');
    % (e) motor voltage
    subplot(3,2,6); plot(t, L.V, 'k'); grid on; xlabel('t [s]'); ylabel('V_m [V]');
    yl = max(abs(L.V(t>0.5)))*1.3 + 1; ylim([-yl yl]);
    title(sprintf('Motor voltage (t<0.5 s peak %.0f V clipped)', max(abs(L.V))));
    print(f, sprintf('figs/scen_%s_%s.png', sc, md), '-dpng', '-r110'); close(f);
    fprintf('figs/scen_%s_%s.png\n', sc, md);
end
end
end
function shade_blackout(L, yl)
v = L.valid; t = L.t;
% blackout = gap between valid samples > 0.075 s
tv = t(v == 1); g = find(diff(tv) > 0.075);
for k = g.'
    patch([tv(k) tv(k+1) tv(k+1) tv(k)], [yl(1) yl(1) yl(2) yl(2)], [0.9 0.9 0.9], 'edgecolor', 'none');
end
end
