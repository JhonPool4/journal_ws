function generate_figure(y_data_1, y_data_2, y_data_3, x_data, fc)

% time variables
time = fc.time;
t_start = fc.t_start;
t_end   = fc.t_end;
t_dt = fc.t_dt;


tiledlayout(fc.cnt_v_img,fc.cnt_h_img, 'Padding', 'none')

for img_index=1:fc.cnt_v_img*fc.cnt_h_img
    nexttile
    plot_name = strcat(fc.name_list(img_index), fc.unit_list(img_index));
    plot(x_data, y_data_1(img_index,:),'linestyle', fc.ls_list{1}, 'linewidth', fc.lw_list{1}, 'color', fc.color_list{1}), hold on, grid on, box on
    plot(x_data, y_data_2(img_index,:),'linestyle', fc.ls_list{2}, 'linewidth', fc.lw_list{2}, 'color', fc.color_list{2}), hold on, grid on, box on
    plot(x_data, y_data_3(img_index,:),'linestyle', fc.ls_list{3}, 'linewidth', fc.lw_list{3}, 'color', fc.color_list{3})
    
    xlabel('Time (s)', 'Interpreter','latex'),
    ylabel(plot_name, 'Interpreter','latex'),
    xticks(time(t_start):t_dt:time(t_end))
    xlim([time(t_start) time(t_end)])    
    ax = gca; % current axes
    ax.FontSize = fc.axis_font_size;
    set(gca,'TickLabelInterpreter','latex')   
    if (img_index==1 && fc.legend_space~=0)
        title(" ", "FontSize",fc.legend_space)
    end
        
end 

chosen_figure=gcf;
set(chosen_figure,'PaperUnits','centimeters');
set(chosen_figure,'PaperPositionMode','auto');
set(chosen_figure,'Units','centimeters');

set(chosen_figure,'Position',[0 0 fc.img_width fc.img_height]);

% add legend
if (fc.legend_space~=0)
    Lgnd = legend(fc.legend_list, 'interpreter', 'latex', 'Orientation','horizontal');
    Lgnd.FontSize = fc.legend_font_size;
    Lgnd.Position(1) = 0.5 - Lgnd.Position(3)/2; %fc.legend_xpos;
    Lgnd.Position(2) = 0.95- Lgnd.Position(4)/2;
end

% save figure
saveas(chosen_figure, fc.save_path, 'epsc'); %Set desired file name

end