function [ agentParam, targetParam, landmarkParam, centEstiParam, locEstiParam ] = ImportInput(obj)
%IMPORTINPUTS incodes all files related to the simulation from text files

%--- Landmark input file import ----
landmarkFileID = fopen('LandmarkProfile.txt');
landmarkParam = textscan(landmarkFileID,'%s %s');
fclose(landmarkFileID);

%--- Agent input file import ----
agentFileID = fopen('AgentProfile.txt');
agentParam = textscan(agentFileID,'%s %s');
fclose(agentFileID);

%--- Target input file import ----
targetFileID = fopen('TargetProfile.txt');
targetParam = textscan(targetFileID,'%s %s');
fclose(targetFileID);

%--- Local Guessed Target (and bias) input file import ----
locEstiFileID = fopen('LocalEstimatorProfile.txt');
locEstiParam = textscan(locEstiFileID,'%s %s');
fclose(locEstiFileID);

%--- Central Guessed Target (and bias) input file import ----
centEstiFileID = fopen('CentralEstimatorProfile.txt');
centEstiParam = textscan(centEstiFileID,'%s %s');
fclose(centEstiFileID);


end

