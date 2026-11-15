function v = figvis()
% figures are hidden in headless gnuplot runs, visible when a display exists (fltk/qt print needs it)
if isempty(getenv('DISPLAY')), v = 'off'; else, v = 'on'; end
end
