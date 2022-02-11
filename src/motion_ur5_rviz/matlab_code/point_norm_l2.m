function [mean_norm_l2, norm_l2] = point_norm_l2(data, pos_x, pos_y, pos_z)

len = size(data,2);
norm_l2 = zeros(1, len);
for i=1:len
    norm_l2 (i)= sqrt(data(pos_x, i)^2 + data(pos_y, i)^2 + data(pos_z, i)^2);
end

mean_norm_l2 = mean(norm_l2);
end