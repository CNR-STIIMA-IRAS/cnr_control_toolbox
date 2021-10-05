function [u,Du,DDu,t,harmonics,opt_coefs,max_position]=computeHarmonics(w_carrier,A_carrier,...
    st,w_min,w_max,n_harmonics,max_vel,max_acc)


min_harm=floor(w_min/w_carrier);
max_harm=ceil(w_max/w_carrier);


harmonics=sort(min_harm+randperm([max_harm-min_harm])');
n_harmonics=min(length(harmonics),n_harmonics);

harmonics=(harmonics(1:n_harmonics));
omega=harmonics*w_carrier;

maxDDu=inf;

for iter=1:10
    coefs=(randn(n_harmonics,1)+1i*randn(n_harmonics,1)).*omega.^-2;
    [u,Du,DDu,t,harmonics]=multisine(w_carrier,A_carrier,st,harmonics,coefs);
    maxDDu_iter=max(DDu);
    if maxDDu_iter<maxDDu
        maxDDu=maxDDu_iter;
        opt_coefs=coefs;
    end
end
[u,Du,DDu,t,harmonics]=multisine(w_carrier,A_carrier,st,harmonics,opt_coefs);
maxDu=max(Du);
maxDDu=max(DDu);
scale=min([1 max_vel/maxDu max_acc/maxDDu]);
opt_coefs=opt_coefs*scale;
[u,Du,DDu,t,harmonics]=multisine(w_carrier,A_carrier,st,harmonics,opt_coefs);

max_position=max(u);