%clear all;clc;

load 'Dados_teste.txt'
ax_offset = -mean(Dados_teste(2:end,1))/16
ay_offset = -mean(Dados_teste(2:end,2))/16
az_offset = (16384-mean(Dados_teste(2:end,3)))/16

% gx_offset = -mean(Dados_teste(2:end,4))/4
% gy_offset = -mean(Dados_teste(2:end,5))/4
% gz_offset = -mean(Dados_teste(2:end,6))/4
