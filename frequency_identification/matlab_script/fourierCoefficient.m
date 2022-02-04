function c=fourierCoefficient(t,y,omega0,n)


T=2*pi/omega0;
periods=floor(t(end)/T);
assert(periods>=1)

idx=find(t>=(t(end)-periods*T),1); %take only the last periods*T seconds
t=t(idx:end);
y=y(idx:end);


%c=trapz(t,y.*exp(-1i*n*omega0*t))/T/periods; % from Fourier definition, averaged on multiple periods

dt=mean(diff(t));
c=sum(dt.*y.*exp(-1i*n*omega0*t))/T/periods; % from Fourier definition, averaged on multiple periods