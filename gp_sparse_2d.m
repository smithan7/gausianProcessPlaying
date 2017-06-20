function [ x, y ] = gp_sparse_2d( x, y, N )
%UNTITLED Summary of this function goes here
    while length(x) > N
        worst_v = inf;
        worst_i = -1;
        for i=1:length(x)
            v = 0;
            for j=1:length(x)
                v = (x(1,i)-x(1,j))^2 + (x(2,i)-x(2,j))^2 +(y(i)-y(j))^2;
                if v < worst_v
                    worst_v = v;
                    worst_i = i;
                end
            end
            
        end
        x(:,worst_i) = [];
        y(worst_i) = [];
end