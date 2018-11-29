
% -----------------------------------------------
% script for ACC 2019 simulation results analysis
% -----------------------------------------------

clear;
close all;

load('ACC2019_scene1_20steps_100run.mat'); % input data
idxSim = floor(rand(1)*100)+1;

for iClock = 1:sim(idxSim).clock.nt+1
    ComputePDFMixture(squeeze(sim(idxSim).PF(1).hist.pt(:,:,iClock)),planner.w,planner.param,'Gaussian');

end