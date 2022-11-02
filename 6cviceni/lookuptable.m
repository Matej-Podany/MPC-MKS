clear all; %#ok<CLALL> 
close all;
clc;

% CSV read
raw = csvread("ntc.csv")'; %#ok<CSVRD> 

temp1 = raw(1,:); % temperature
r = raw(2,:); % resistance

ad = r./(r+10).*1023;

figure;
plot(ad, temp1, 'ro');
grid on;
hold on;

p = polyfit(ad, temp1, 10 );
ad2 = 0:1023;

t2 = round(polyval(p, ad2), 1);
hold on, plot(ad2, t2, 'g');

dlmwrite('data.dlm', t2*10, ','); %#ok<DLMWT> 
