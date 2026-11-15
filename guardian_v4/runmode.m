function R = runmode(sc, mode, verbose, ovr)
global GUARDIAN_OVERRIDE
if nargin<3, verbose=true; end
if nargin<4, ovr=struct(); end
ovr.mode = mode; GUARDIAN_OVERRIDE = ovr;
clear turret_controller turret_plant
R = run_scenario(sc, [], verbose);
GUARDIAN_OVERRIDE = [];
end
