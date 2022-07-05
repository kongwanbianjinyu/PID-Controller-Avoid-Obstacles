clear all; close all; clc;
tic

%%
load TestTrack.mat
load ROB535_ControlProject_part1_Team26.mat


[Y,T] = forwardIntegrateControlInput(ROB535_ControlProject_part1_input);

%% Results
info = getTrajectoryInfo(Y,ROB535_ControlProject_part1_input)
%%
figure
    hold all
    plot(TestTrack.bl(1,:),TestTrack.bl(2,:),'k')
    plot(TestTrack.br(1,:),TestTrack.br(2,:),'k')    
    plot(info.Y(:,1),info.Y(:,3),'r')
    
toc
    
    

