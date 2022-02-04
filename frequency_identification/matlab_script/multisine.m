function [u,Du,DDu,t,harmonics]=multisine(w_carrier,A_carrier,st,harmonics,coefs)


T_portante=2*pi/w_carrier;
t=(0:st:(1.5*T_portante))';
carrier=A_carrier*sin(w_carrier*t);
Dcarrier=w_carrier*A_carrier*cos(w_carrier*t);
DDcarrier=-w_carrier^2*A_carrier*sin(w_carrier*t);
omega=harmonics*w_carrier;

x=zeros(length(t),1);
Dx=zeros(length(t),1);
DDx=zeros(length(t),1);
for idx=1:length(omega)
%     x   =   x+2*real(coefs(idx))*cos(omega(idx)*t)+...
%         imag(coefs(idx))*sin(omega(idx)*t);
%     
%     Dx  =  Dx+2*(-real(coefs(idx))*sin(omega(idx)*t)+...
%         -imag(coefs(idx))*cos(omega(idx)*t))*omega(idx);
%     
%     DDx = DDx+2*(-real(coefs(idx))*cos(omega(idx)*t)+...
%         imag(coefs(idx))*sin(omega(idx)*t))*omega(idx)^2;

    x   =   x +                 (coefs(idx)*exp(1i*omega(idx)*t)+conj(coefs(idx))*exp(-1i*omega(idx)*t));
    Dx  =  Dx + 1i*omega(idx) * (coefs(idx)*exp(1i*omega(idx)*t)-conj(coefs(idx))*exp(-1i*omega(idx)*t));
    DDx = DDx - omega(idx)^2  * (coefs(idx)*exp(1i*omega(idx)*t)+conj(coefs(idx))*exp(-1i*omega(idx)*t));
    
end


u   = carrier   + x;
Du  = Dcarrier  + Dx;
DDu = DDcarrier + DDx;