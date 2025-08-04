%% close 
close all
clear all
clc

format long

%% Include dirs
addpath("quaternions/");
addpath("KF_utilities/");

%% Waypoints

traj_sel = 8;  % select a number from 1 to 5
WP_gen;

%% Trajectory generation

trajectory_gen_cartesian;

%% Sensors

sensors_2;


