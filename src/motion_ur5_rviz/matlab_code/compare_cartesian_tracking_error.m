% @info:    Generate plots of trajectory tracking error at joint space.
%               The plots compare performance of control method using fixed
%               gains and optimized gains.
clc, clear all, close all;

pwd = '/home/jhon/catkin_ws/journal_ws/src/motion_ur5_rviz/';
control_method = 'PDi'; % change: (i) PDi or SMCi 
trajectory_type  = '/circular_traj/'; % change to database
file_name_1 = 'cartesian_KP_9_KD_6_alpha_0.0_beta_0.1_gamma_0.9_delta_0.1_t_50.csv';
file_name_2 = 'cartesian_KP_9_KD_6_alpha_0.5_beta_0.1_gamma_0.9_delta_0.1_t_50.csv';
%file_name_1 = 'cartesian_L5_K_1_alpha_0.0_beta_0.1_gamma_0.9_delta_0.1_t_50.csv';
%file_name_2 = 'cartesian_L5_K_1_alpha_0.01_beta_0.1_gamma_0.9_delta_0.1_t_50.csv';
file_path = fullfile(pwd, 'data/',control_method, trajectory_type);
image_path = fullfile(pwd,'document/images/', control_method, trajectory_type);

% read table
data_1 = readtable(fullfile(file_path, file_name_1),'PreserveVariableNames',true);
data_2 = readtable(fullfile(file_path, file_name_2),'PreserveVariableNames',true);

% time variables
time = data_1.t;
t_start = 1;
t_step  = 5;
t_end   = 3001;%length(time);
t_dt = (time(t_end) - time(t_start))/3;


% get cartesian data
[p_med_1, dp_med_1, ddp_med_1, p_ref_1, dp_ref_1, ddp_ref_1, p_e_1, dp_e_1, ddp_e_1] = get_cartesian_position_data( data_1, t_start, t_step, t_end);
[rpy_med_1, rpy_ref_1, rpy_e_1] = get_cartesian_orientation_data(data_1, t_start, t_step, t_end);

[p_med_2, dp_med_2, ddp_med_2, p_ref_2, dp_ref_2, ddp_ref_2, p_e_2, dp_e_2, ddp_e_2] = get_cartesian_position_data( data_2, t_start, t_step, t_end);
[rpy_med_2, rpy_ref_2, rpy_e_2] = get_cartesian_orientation_data(data_2, t_start, t_step, t_end);

% get control parameters
[K1_1, K2_1, K1_2, K2_2] = get_control_parameters(control_method, data_2, t_start, t_step, t_end);

% colors
color1 = [0, 0.4470, 0.7410];                % dark red        --alpha=0.1 
color2 = [0.6350, 0.0780, 0.1840];       % dark blue      --alpha=0.5
color3 = [0.4660, 0.6740, 0.1880];       % dark green    --alpha=0.8

% page size
image_size_height = 5;
image_size_height_legend = 1.5*5;
image_size_width = 17;
%% cartesian position: tracking error
clc, close all;

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 image_size_width image_size_height_legend])

name_list = ["$\mathrm{x}$", "$\mathrm{y}$","$\mathrm{z}$"];

cnt_img = 3;
space_size = 1;
img_size = 3;
rows = img_size + 2;
cols  = img_size*cnt_img+space_size*(cnt_img-1);
legend_flag = true;

for i=1:3
    plot_name = strcat(name_list(i),' ($\mathrm{mm}$)');
    subplot_indexes = get_subplot_index(i, rows,  cols, img_size, space_size, legend_flag);
    subplot(rows, cols, subplot_indexes),
    plot(time(t_start:t_step:t_end), p_e_1(:, i), 'linestyle', '-', 'linewidth', 2, 'color', color1), hold on, grid on, box on
    plot(time(t_start:t_step:t_end), p_e_2(:, i), 'linestyle', '--','linewidth', 2, 'color', color2), hold on, grid on, box on
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(time(t_start):t_dt:time(t_end))
    xlim([0 time(t_end)])
    ax = gca; % current axes
    ax.FontSize = 11;
    set(gca,'TickLabelInterpreter','latex')
    
end         

% add legend
Lgnd = legend({'fixed gains', 'with optimization'}, 'interpreter', 'latex', 'Orientation','horizontal');
Lgnd.FontSize = 12;
Lgnd.Position(1) = 0.3;
Lgnd.Position(2) = 0.85;
    

% Save image
file_name     = fullfile(image_path, 'cartesian_position_error');
saveas(gcf,file_name,'epsc')  


%% cartesian velocity: tracking error
clc, close all;

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 image_size_width image_size_height])

name_list = ["$\mathrm{\dot{x}}$", "$\mathrm{\dot{y}}$","$\mathrm{\dot{z}}$"];

cnt_img = 3;
space_size = 1;
img_size = 3;
rows = img_size + 2;
cols  = img_size*cnt_img+space_size*(cnt_img-1);
legend_flag = false;

for i=1:3
    plot_name = strcat(name_list(i),' ($\mathrm{\frac{mm}{s}}$)');
    subplot_indexes = get_subplot_index(i, rows,  cols, img_size, space_size, legend_flag);
    subplot(rows, cols, subplot_indexes),
    plot(time(t_start:t_step:t_end), dp_e_1(:, i), 'linestyle', '-', 'linewidth', 2, 'color', color1), hold on, grid on, box on
    plot(time(t_start:t_step:t_end), dp_e_2(:, i), 'linestyle', '--','linewidth', 2, 'color', color2), hold on, grid on, box on
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(time(t_start):t_dt:time(t_end))
    xlim([0 time(t_end)])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
    
end           

% Save image
file_name     = fullfile(image_path, 'cartesian_velocity_error');
saveas(gcf,file_name,'epsc')  


%% cartesian orientation with Euler angles: tracking error
clc, close all;

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 image_size_width image_size_height])

name_list = ["$\mathrm{\phi_{roll}}$", "$\mathrm{\phi_{pitch}}$","$\mathrm{\phi_{roll}}$"];

cnt_img = 3;
space_size = 1;
img_size = 3;
rows = img_size + 2;
cols  = img_size*cnt_img+space_size*(cnt_img-1);
legend_flag = false;

for i=1:3
    plot_name = strcat(name_list(i),' ($\mathrm{^{\circ}}$)');
    subplot_indexes = get_subplot_index(i, rows,  cols, img_size, space_size, legend_flag);
    subplot(rows, cols, subplot_indexes),
    plot(time(t_start:t_step:t_end), rpy_e_1(:, i), 'linestyle', '-', 'linewidth', 2, 'color', color1), hold on, grid on, box on
    plot(time(t_start:t_step:t_end), rpy_e_2(:,i), 'linestyle', '--','linewidth', 2, 'color', color2), hold on, grid on, box on
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(time(t_start):t_dt:time(t_end))
    xlim([0 time(t_end)])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
    
end           

% Save image
file_name     = fullfile(image_path, 'cartesian_rpy_error');
saveas(gcf,file_name,'epsc')  

