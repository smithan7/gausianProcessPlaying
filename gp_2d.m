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

% I think to go to multiple observations, X(i) is a vector not number, for covariance you bustract the complete whole vector
% to get the distance between each entry and then sum the distances. so for X(i) = [1xn] -> xs = sum_j=1^n (X(i,j) - X'(l,j))^2

% next goal is to include a model of a car and predict it's model as it drives around, from x,y,theta,xdot,ydot,thetadot predict x',y',theta',xdot',ydot',thetadot'
% really this means predicting just the dots and then solving for the velocity. I think a key will be to use greedy selection of points so that 
% the matrix inverse is manageable

% maybe as part of the vector include my state information? for puruit evasion.


close all
clear all
clc

l = 0.9129; % length parameter - how much distance between x's matters, large l means the distance matters less
sigma_f = 1.3234; % maximum allowable covariance - large with functions with large range of y values
sigma_f_sq = sigma_f^2;
sigma_n = 0.3; % actual uncertainty in the data
sigma_n_sq = sigma_n^2;

s_min = sigma_f;
l_min = l;
y_min = inf;

for opt_iter = 1:1
   
    sigma_f = s_min + 0.005*randn();
    sigma_f_sq = sigma_f^2;
    l = l_min + 0.005*randn();
    
    y_sum = 0;
    
    for test_iter = 1:1
        for i=1:500
            x(1,i) = rand()*20-10;
            x(2,i) = rand()*20-10;
            y(i) = x(1,i)*sin(x(2,i));
        end
        s = ones(length(x),1)*0.3;
    

        % build covariance matrix
        k =zeros(length(x), length(x));
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

        xcnt = 0;
        for xs1 = -10:0.5:10
            xcnt = xcnt +1;
            ycnt = 0;
            for xs2 = -10:0.5:10
                ycnt = ycnt+1;
                ks = zeros(1,length(x));
                for i=1:length(x)
                    if x(1,i) == xs1 && x(2,i) == xs2
                        kroneckerDelta = 1;
                    else
                        kroneckerDelta = 0;
                    end
                    ks(i) = sigma_f_sq * exp(-((x(1,i)-xs1)^2 + (x(2,i)-xs2)^2)/2*l^2) + sigma_n_sq*kroneckerDelta;
                end

                if xs1 == xs1 && xs2 == xs2
                    kroneckerDelta = 1;
                else
                    kroneckerDelta = 0;
                end
                kss = sigma_f_sq * exp(-((xs1-xs1)^2+(xs2-xs2)^2)/2*l^2) + sigma_n_sq*kroneckerDelta;

                % this is the computationally expensive bit, you can reduce it when the
                % number of samples gets large by selecting only the sensor
                % measurements that are in the local vicinity of the point you care
                % about. The idea being, snesor measurments from very different
                % attitudes, ect don't matter as much as sensor readings from nearby
                % the point of interest
                ys(xcnt,ycnt) = ks/k*y';
                yv(xcnt, ycnt) = kss-ks/k*ks';

                yt(xcnt,ycnt) = xs1*sin(xs2);

                ye(xcnt,ycnt)=abs(yt(xcnt,ycnt)-ys(xcnt,ycnt));
                y_sum = y_sum+ye(xcnt,ycnt);
            end
        end
    end
    
    if y_sum < y_min
        opt_iter
        l_min = l
        s_min = sigma_f
        y_min = y_sum
    end
end

surf(ys,yv)

figure
surf(ye)

figure
surf(yt)
