% gp_nd

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

x_range = 5;
n_pts = 1000;
n_dimensions = 2;

x = zeros(n_dimensions, n_pts);
y = zeros(1 ,n_pts);

for i=1:floor(n_pts*0.9)
    for j=1:n_dimensions
        x(j,i) = rand()*(2*x_range)-x_range;
    end
    y(i) = x(1,i)*sin(x(2,i))+randn()/2;
end

for i=floor(n_pts*0.9):n_pts
    for j=1:n_dimensions
        x(j,i) = rand()*(x_range)-x_range/2;
    end
    y(i) = x(1,i)*sin(x(2,i))+randn();
end


figure(1)
plot3(x(2,:),x(1,:),y, 'ro')

[ x, y ] = gp_sparse_2d( x, y, 200 );

xcnt = 0;
for xs1 = -x_range:0.5:x_range
    xcnt = xcnt +1;
    ycnt = 0;
    for xs2 = -x_range:0.5:x_range
        ycnt = ycnt+1;
        [ys(xcnt,ycnt), yv(xcnt,ycnt)] = gp_predict_2d([xs1, xs2], x, y);
        yt(xcnt,ycnt) = xs1*sin(xs2);
        ye(xcnt,ycnt)=abs(yt(xcnt,ycnt)-ys(xcnt,ycnt));
    end
end

figure(1)
surf(-x_range:0.5:x_range,-x_range:0.5:x_range,ys,yv)
hold on
plot3(x(2,:),x(1,:),y, 'ro')

figure(2)
surf(-x_range:0.5:x_range,-x_range:0.5:x_range,ye)

figure(3)
surf(-x_range:0.5:x_range,-x_range:0.5:x_range,yt)

figure(4)
plot3(x(2,:),x(1,:),y, 'ro')
grid on