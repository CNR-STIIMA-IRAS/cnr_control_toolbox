clear all;close all;clc;
st=1e-3;
P=tf(1,[.1 1]);
Pd=c2d(P,st);

max_acc=5;
max_vel=2;
w_carrier=0.1;
A_carrier=.1;
w_min=1;
w_max=100;
n_harmonics=20;

[u,Du,DDu,t,harmonics,coefs_u,max_position]=computeHarmonics(w_carrier,A_carrier,st,w_min,w_max,n_harmonics,max_vel,max_acc);

y=lsim(Pd,u,t)+0.01*randn(length(t),1);
subplot(4,1,1)
plot(t,y)
grid on

subplot(4,1,2)
plot(t,u)
grid on

subplot(4,1,3)
plot(t,Du)
grid on

subplot(4,1,4)
plot(t,DDu)
grid on

coefs_y = fourierCoefficients(t,y,w_carrier,w_carrier*harmonics);
%%
figure(3)
fr_estim=coefs_y./coefs_u;
fr_data=freqresp(Pd,w_carrier*harmonics);fr_data=fr_data(:);

subplot(2,1,1)
semilogx(w_carrier*harmonics,20*log10(abs(fr_estim)),'ok');
hold on
semilogx(w_carrier*harmonics,20*log10(abs(fr_data)));
grid on

subplot(2,1,2)
semilogx(w_carrier*harmonics,rad2deg(angle(fr_estim)),'ok');
hold on
semilogx(w_carrier*harmonics,rad2deg(angle(fr_data)));
grid on
