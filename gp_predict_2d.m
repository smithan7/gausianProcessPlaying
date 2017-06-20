function [ y_m, y_v ] = gp_predict_2d( xp, x, y, r )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    if nargin == 3
        r = zeros(length(x),length(x));
    else
        r = diag(r);
    end
    
    l = 0.9; % length parameter - how much distance between x's matters, large l means the distance matters less
    sigma_f = 1.27; % maximum allowable covariance - large with functions with large range of y values
    sigma_f_sq = sigma_f^2;
    sigma_n = 1.0; % actual uncertainty in the data
    sigma_n_sq = sigma_n^2;

    % build covariance matrix
    k = zeros(length(x), length(x));
    for i=1:length(x)
        for j=1:length(x)
            if x(1,i) == x(1,j) && x(2,i) == x(2,j)
                kroneckerDelta = 1;
            else
                kroneckerDelta = 0;
            end
            k(i,j) = sigma_f_sq * exp(-((x(1,i)-x(1,j))^2 + (x(2,i)-x(2,j))^2)/(2*l^2)) + sigma_n_sq*kroneckerDelta;
        end
    end

    ks = zeros(1,length(x));
    for i=1:length(x)
        if x(1,i) == xp(1) && x(2,i) == xp(2)
            kroneckerDelta = 1;
        else
            kroneckerDelta = 0;
        end
        ks(i) = sigma_f_sq * exp(-((x(1,i)-xp(1))^2 + (x(2,i)-xp(2))^2)/2*l^2) + sigma_n_sq*kroneckerDelta;
    end
    
    if(x(i) == xp)
        kroneckerDelta = 1;
    else
        kroneckerDelta = 0;
    end
    kss = sigma_f_sq * exp(-((xp(1)-xp(1))^2+(xp(2)-xp(2))^2)/2*l^2) + sigma_n_sq*kroneckerDelta;

    
    % this is the computationally expensive bit, you can reduce it when the
    % number of samples gets large by selecting only the sensor
    % measurements that are in the local vicinity of the point you care
    % about. The idea being, snesor measurments from very different
    % attitudes, ect don't matter as much as sensor readings from nearby
    % the point of interest
    y_m = ks/(k+r)*y';
    y_v = kss-ks/(k+r)*ks';

end

