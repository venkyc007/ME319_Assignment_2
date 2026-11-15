% Monte-Carlo tuning/validation summary figure (figs/montecarlo.png)
if isempty(getenv('DISPLAY')), graphics_toolkit('gnuplot'); end
ks = [0.5 1 1.5 2 2.5 3 4];
Tt = analyse_sweep('tq20', ks, true);           % track, q=20^2, wc=20 (replayed gate)
Ta = [analyse_sweep('ak1',[],true) analyse_sweep('ak2',[],true) analyse_sweep('ak3',[],true)];
f = figure('visible',figvis(),'position',[0 0 1100 420]);
subplot(1,2,1);
[ax, h1, h2] = plotyy(ks, 100*[Tt.precision], ks, [Tt.t_first_mean]);
hold(ax(1),'on'); hold(ax(2),'on');
plot(ax(1), [1 2 3], 100*[Ta.precision], 'bs--', 'markerfacecolor','b');
plot(ax(2), [1 2 3], [Ta.t_first_mean], 'rs--', 'markerfacecolor','r');
set(h1,'color','b','marker','o'); set(h2,'color','r','marker','o');
ylabel(ax(1),'shot precision [%]'); ylabel(ax(2),'mean first-hit time [s]'); xlabel('fire\_k');
set(ax(1),'ycolor','b','ylim',[85 100.5]); set(ax(2),'ycolor','r','ylim',[0.8 2.4]);
title('R1-R100: fire\_k trade-off (o track, s ambush)'); grid on;
subplot(1,2,2); hold on;
F = {Tt(ks==3).first, Ta(3).first}; nm = {'track R1-100','ambush R1-100'}; c = {[0 0 1],[1 0 0]}; hh = [];
V = {}; try V = {analyse_sweep('vtrack',[],true).first, analyse_sweep('vamb',[],true).first}; catch, end
for j = 1:2
  x = sort(F{j}(~isnan(F{j}))); hh(end+1) = stairs(x, (1:numel(x))/100, 'color', c{j}, 'linewidth', 1.5);
end
for j = 1:numel(V)
  x = sort(V{j}(~isnan(V{j}))); hh(end+1) = stairs(x, (1:numel(x))/100, 'color', 0.5*c{j}+0.5, 'linestyle', '--', 'linewidth', 1.5);
  nm{end+1} = strrep(nm{j}, 'R1-100', 'R101-200 (held out)');
end
legend(hh, nm, 'location', 'southeast'); grid on; xlim([0 4]); ylim([0 1.02]);
xlabel('time of first successful engagement [s]'); ylabel('fraction of cases intercepted');
title('Final design: interception-time CDF');
print(f, 'figs/montecarlo.png', '-dpng', '-r110');
