% @info:    Generate plots of trajectory tracking error at joint space.
%               The plots compare performance of control method using fixed
%               gains and optimized gains.
clc, clear all, close all;

%pwd = '/home/jhon/catkin_ws/journal_ws/src/motion_ur5_rviz/';
pwd = '/home/emanuelsamir/Documentos/dev/concytec/journal_ws/src/motion_ur5_rviz/';
control_method = 'SMCi'; % change: (i) PDi or SMCi 
trajectory_type  = '/circular_traj/'; % change to database


file_name_1 = 'cartesian_L_5_K_1_alpha_0.0_beta_0.1_gamma_0.9_delta_0.1_t_30_uncertainty_0.1.csv';
file_name_2 = 'cartesian_L_5_K_1_alpha_0.01_beta_0.1_gamma_0.9_delta_0.1_t_30_uncertainty_0.1.csv';
%file_name_1 = 'cartesian_L5_K_1_alpha_0.0_beta_0.1_gamma_0.9_delta_0.1_t_50.csv';
%file_name_2 = 'cartesian_L5_K_1_alpha_0.01_beta_0.1_gamma_0.9_delta_0.1_t_50.csv';
file_path = fullfile(pwd, 'data/',control_method, trajectory_type);
image_path = fullfile(pwd,'document/images/', control_method, trajectory_type);

% read table
data_1 = readtable(fullfile(file_path, file_name_1));%,'PreserveVariableNames',true);
data_2 = readtable(fullfile(file_path, file_name_2));%,'PreserveVariableNames',true);

% time variables
time = data_1.t;
t_start = 1;
t_step  = 5;
t_end   = 3000;%length(time);
t_dt = (time(t_end) - time(t_start))/3;
      
% get articular data
[p_med_1, dp_med_1, ddp_med_1, p_ref_1, dp_ref_1, ddp_ref_1, p_e_1, dp_e_1, ddp_e_1] = get_cartesian_position_data( data_1, t_start, t_step, t_end);
[rpy_med_1, rpy_ref_1, rpy_e_1] = get_cartesian_orientation_data(data_1, t_start, t_step, t_end);

[p_med_2, dp_med_2, ddp_med_2, p_ref_2, dp_ref_2, ddp_ref_2, p_e_2, dp_e_2, ddp_e_2] = get_cartesian_position_data( data_2, t_start, t_step, t_end);
[rpy_med_2, rpy_ref_2, rpy_e_2] = get_cartesian_orientation_data(data_2, t_start, t_step, t_end);


x_data = time(t_start:t_step:t_end);
y_data_1 = [p_e_1(:,1)';p_e_1(:,2)';p_e_1(:,1)' ; ddp_e_1(:,1)';ddp_e_1(:,2)';ddp_e_1(:,3)'; rpy_e_1(:,1)'; rpy_e_1(:,2)'; rpy_e_1(:,3)'];
y_data_2 = [p_e_2(:,1)';p_e_2(:,2)';p_e_2(:,1)' ; ddp_e_2(:,1)';ddp_e_2(:,2)';ddp_e_2(:,3)'; rpy_e_2(:,1)'; rpy_e_2(:,2)'; rpy_e_2(:,3)'];

%%

close all
tiledlayout(3,3, 'Padding', 'none')

name_list_1 = ["$\mathrm{x}$", "$\mathrm{y}$","$\mathrm{z}$"];
name_list_2 = ["$\mathrm{\ddot{x}}$", "$\mathrm{\ddot{y}}$","$\mathrm{\ddot{z}}$"];
name_list_3 = ["$\mathrm{\phi_{roll}}$", "$\mathrm{\phi_{pitch}}$","$\mathrm{\phi_{roll}}$"];

unit_list_1 = [" ($\mathrm{mm}$)", " ($\mathrm{mm}$)", " ($\mathrm{mm}$)"];
unit_list_2 = [" ($\mathrm{\frac{mm}{s^2}}$)", " ($\mathrm{\frac{mm}{s^2}}$)", " ($\mathrm{\frac{mm}{s^2}}$)"];
unit_list_3 = [" ($\mathrm{^{\circ}}$)", " ($\mathrm{^{\circ}}$)", " ($\mathrm{^{\circ}}$)"];
name_list = [name_list_1, name_list_2, name_list_3];
unit_list  = [unit_list_1, unit_list_2, unit_list_3];

for img_index=1:9
    nexttile
    plot_name = strcat(name_list(img_index), unit_list(img_index));
    plot(x_data, y_data_1(img_index,:),'linestyle', '-', 'linewidth', 2), hold on, grid on, box on
    plot(x_data, y_data_2(img_index,:),'linestyle', '-', 'linewidth', 2)
    xlabel('Time (s)', 'Interpreter','latex'),
    ylabel(plot_name, 'Interpreter','latex'),
    xticks(time(t_start):t_dt:time(t_end))
    xlim([0 time(t_end)])    
    ax = gca; % current axes
    ax.FontSize = 10;
    set(gca,'TickLabelInterpreter','latex')   
    if img_index==1
        title(" ", "FontSize",40)
    end
        
end 

chosen_figure=gcf;
set(chosen_figure,'PaperUnits','centimeters');
set(chosen_figure,'PaperPositionMode','auto');
set(chosen_figure,'Units','centimeters');

set(chosen_figure,'Position',[0 0 16 20]);

% add legend
Lgnd = legend({'fixed gains', 'with optimization'}, 'interpreter', 'latex', 'Orientation','horizontal');
Lgnd.FontSize = 12;
Lgnd.Position(1) = 0.3;
Lgnd.Position(2) = 0.95;
saveas(chosen_figure,'/home/jhon/Desktop/filename', 'epsc'); %Set desired file name