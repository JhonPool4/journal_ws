% @info: generate plots of trajectory tracking at joint space 
clc, clear all, close all;


pwd = '/home/jhon/catkin_ws/journal_ws/src/motion_ur5_rviz/';
trajectory_type  = 'SMCi/circular_traj/'; % change: (i) PDi or SMCi 
file_name_1 = 'articular_lambda_5_k_1_alpha_[0 0]_beta_0.1_gamma_0.9_t_50.csv';
file_name_2 = 'articular_lambda_5_k_1_alpha_[0.01 0.01]_beta_0.1_gamma_0.9_t_50.csv';
file_path = fullfile(pwd, 'data/',trajectory_type);
image_path = fullfile(pwd,'document/images/', trajectory_type);

data_1 = readtable(fullfile(file_path, file_name_1),'PreserveVariableNames',true);
data_2 = readtable(fullfile(file_path, file_name_2),'PreserveVariableNames',true);

time = data_1.t;
t_start = 1;
t_step  = 1;
t_end   = length(time);
t_dt = (time(t_end) - time(t_start))/5;
      
[q_ref_1, dq_ref_1, ddq_ref_1, q_med_1, dq_med_1, ddq_med_1, q_e_1, dq_e_1, ddq_e_1] = get_cartesian_data(data_1, t_start, t_step, t_end);
[q_ref_2, dq_ref_2, ddq_ref_2, q_med_2, dq_med_2, ddq_med_2, q_e_2, dq_e_2, ddq_e_2] = get_cartesian_data(data_2, t_start, t_step, t_end);


% colors
color1 = [0, 0.4470, 0.7410];               % black         --alpha=0.1 
color2 = [0.6350, 0.0780, 0.1840];       % dark blue     --alpha=0.5
color3 = [0.4660, 0.6740, 0.1880];      % dark green    --alpha=0.8

%% joint position: tracking error
clc, close all;

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 16.0 20.0])

name_list = ["$\mathrm{q_1}$", "$\mathrm{q_2}$","$\mathrm{q_3}$","$\mathrm{q_4}$","$\mathrm{q_5}$","$\mathrm{q_6}$"];
    
for i=1:6
    plot_name = strcat(name_list(i),' ($\mathrm{rad}$)');
    subplot(3, 2, i),
    plot(time(t_start:t_step:t_end), q_e_1(:, i), 'linestyle', '-', 'linewidth', 2, 'color', color1), hold on, grid on, box on
    plot(time(t_start:t_step:t_end), q_e_2(:, i), 'linestyle', '--','linewidth', 2, 'color', color2), hold on, grid on, box on
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(time(t_start):t_dt:time(t_end))
    xlim([0 time(t_end)])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
    
end         
% add legend
Lgnd = legend({'fixed gains', 'with optimization'}, 'interpreter', 'latex', 'Orientation','horizontal');
Lgnd.FontSize = 12;
Lgnd.Position(1) = 0.3;
Lgnd.Position(2) = 0.95;
    

% Save image
file_name     = fullfile(image_path, 'joint_position_error');
saveas(gcf,file_name,'epsc')  


%% joint velocity
clc, close all;

figure(1), hold on, grid on, box on;
    screen = get(0, 'ScreenSize');    
    set(gcf,'units','centimeters','position', [0 0 15.0 20.0])

name_list = [   "$\mathrm{\dot{q}_1}$", ...
                        "$\mathrm{\dot{q}_2}$",...
                        "$\mathrm{\dot{q}_3}$",...
                        "$\mathrm{\dot{q}_4}$",...
                        "$\mathrm{\dot{q}_5}$",...
                        "$\mathrm{\dot{q}_6}$"];

for i=1:6
    plot_name = strcat(name_list(i),'  ($\mathrm{\frac{rad}{s}}$)');
    subplot(3, 2, i),
    plot(time(t_start:t_step:t_end), dq_e_1(:, i), 'linestyle', '-', 'linewidth', 2, 'color', color1), hold on, grid on, box on
    plot(time(t_start:t_step:t_end), dq_e_2(:, i), 'linestyle', '--','linewidth', 2, 'color', color2), hold on, grid on, box on
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(time(t_start):t_dt:time(t_end))
    xlim([0 time(t_end)])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
    
end         
% add legend
Lgnd = legend({'fixed gains', 'with optimization'}, 'interpreter', 'latex', 'Orientation','horizontal');
Lgnd.FontSize = 12;
Lgnd.Position(1) = 0.3;
Lgnd.Position(2) = 0.95;


% Save image
file_name     = fullfile(image_path, 'joint_velocity_error');
saveas(gcf,file_name,'epsc')  



%% joint acceleration
clc, close all

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 15.0 20.0])

name_list = [   "$\mathrm{\ddot{q}_1}$", ...
                        "$\mathrm{\ddot{q}_2}$",...
                        "$\mathrm{\ddot{q}_3}$",...
                        "$\mathrm{\ddot{q}_4}$",...
                        "$\mathrm{\ddot{q}_5}$",...
                        "$\mathrm{\ddot{q}_6}$"];    
for i=1:6
    plot_name = strcat(name_list(i),'  ($\mathrm{\frac{rad}{s^2}}$)');
    subplot(3, 2, i),
    plot(time(t_start:t_step:t_end), ddq_e_1(:, i), 'linestyle', '-', 'linewidth', 2, 'color', color1), hold on, grid on, box on
    plot(time(t_start:t_step:t_end), ddq_e_2(:, i), 'linestyle', '--','linewidth', 2, 'color', color2), hold on, grid on, box on
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(time(t_start):t_dt:time(t_end))
    xlim([0 time(t_end)])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
    
end         
% add legend
Lgnd = legend({'fixed gains', 'with optimization'}, 'interpreter', 'latex', 'Orientation','horizontal');
Lgnd.FontSize = 12;
Lgnd.Position(1) = 0.3;
Lgnd.Position(2) = 0.95;


% Save image
file_name     = fullfile(image_path, 'joint_acceleration_error');
saveas(gcf,file_name,'epsc')  