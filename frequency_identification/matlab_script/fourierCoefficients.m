function c=fourierCoefficients(t,y,omega0,omega)


for idx=1:length(omega)
    n=round(omega(idx)/omega0);
    assert(abs(n-omega(idx)/omega0)<1e-3); % check if omega is multiple of omega0
    c(idx,1)=fourierCoefficient(t,y,omega0,n);
end