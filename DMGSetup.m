
% -------------------- %
% Setup File for DMG %
% -------------------- %
% Notes: 
%   (1) This file assumes that you have write permissions
%       to change the MATLAB path.  If you do cannot change the
%       MATLAB path, contact your system administrator.
%   (2) Precompiled SNOPT and IPOPT libraries are included in the DMG distribution.
%   (3) Built-in automatic differentiation is included in the DMG
%       distribution and is installed by default using this setup
%       file.

currdir = pwd;
addir = strcat(currdir,'/ad/');
addpath(addir,path,'-begin');
libdir = strcat(currdir,'/lib/');
addpath(libdir,path,'-begin');
snoptdir = strcat(currdir,'/NLPsolvers/SNOPT/');
addpath(snoptdir,path,'-begin');
ipoptdir = strcat(currdir,'/NLPsolvers/IPOPT/');
addpath(ipoptdir,path,'-begin');
savepath;
