% gaussian process example
% https://www.robots.ox.ac.uk/~mebden/reports/GPtutorial.pdf

% control example 
% http://mlg.eng.cam.ac.uk/pub/pdf/KocMurRasLik03.pdf

% multiple gps for estimating a persons pose
% http://www.maths.lth.se/matematiklth/personal/sminchis/code/TGP.html

% multi input gp for control
% http://www.dcs.gla.ac.uk/~rod/publications/KocGirBanMur03.pdf

% gp for uav sys id
% file:///home/andy/Downloads/Hemakumara2013_ICRA_GaussianProcess.pdf

close all
clear all
clc

x = [-1.5, -1.0, -0.75, -0.4, -0.25, 0.00, 7, 4, 1, 2];
y = [-1.6, -1.1, -0.4, 0.1, 0.5, 0.75, 1, -1, -1.5, 0];
s = ones(length(x),1)*0.3;

l = 1; % length parameter - how much distance between x's matters, large l means the distance matters less
sigma_f = 1.27; % maximum allowable covariance - large with functions with large range of y values
sigma_f_sq = sigma_f^2;
sigma_n = 0.3;
sigma_n_sq = sigma_n^2;

k =zeros(length(x), length(x));
for i=1:length(x)
    for j=1:length(x)
        if(x(i) == x(j))
            kroneckerDelta = 1;
        else
            kroneckerDelta = 0;
        end
        k(i,j) = sigma_f_sq * exp(-(x(i)-x(j))^2/2*l^2) + sigma_n_sq*kroneckerDelta;
    end
end

figure
hold all

x_min = min(x);
x_max = max(x);

xs = x_min - 1;

while xs < x_max + 1;
    xs = xs + 0.1;
    ks = zeros(1,length(x));
    for i=1:length(x)
        if(x(i) == xs)
            kroneckerDelta = 1;
        else
            kroneckerDelta = 0;
        end
        ks(i) = sigma_f_sq * exp(-(x(i)-xs)^2/2*l^2) + sigma_n_sq*kroneckerDelta;
    end

    if(xs == xs)
        kroneckerDelta = 1;
    else
        kroneckerDelta = 0;
    end
    kss = sigma_f_sq * exp(-(xs-xs)^2/2*l^2) + sigma_n_sq*kroneckerDelta;

    ys = ks/k*y';
    yv = kss-ks/k*ks';

    errorbar(xs,ys,yv, 'g.')
end

errorbar(x,y,s, 'r.')
plot(x,y,'k.')

