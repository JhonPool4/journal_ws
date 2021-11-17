function [list] = get_subplot_index(img_index, rows,  cols, img_size, space_size, legend_flag)

   
    list = [];
    if legend_flag
        legend =1;
        space = 1;
    else
        space=1;
        legend =0;
    end
    for i=1:(rows-space-legend)
        % p0 = new line + space + 
        p0 = ((i+legend-space)*cols+1) + (img_index-1)*space_size + (img_index-1)*img_size
        for j = p0:(p0+img_size-1)
            list(end+1) = j;
        end
    end
end