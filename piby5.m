clear all;
clc;
bdclose classes;
close all;

run('piby5tunnel.m')
sim('test_01_3d_veh_piby567_2018')
run('display_in_3d.m')