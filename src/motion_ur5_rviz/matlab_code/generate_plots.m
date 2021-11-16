% @info: generate plots of trajectory tracking at joint space 
clc, clear all, close all;


pwd = '/home/jhon/catkin_ws/journal_ws/src/motion_ur5_rviz/';
dir  = 'data/SMCi/circular_traj/'; % change: (i) PDi or SMCi 
file_name = 'articular_kp_9_kd_6_alpha_0.0_beta_0.1_gamma_0.9_t_60.csv';
file_path = fullfile(pwd, dir);
image_path = fullfile(pwd,'document/images/');

data = readtable(fullfile(file_path, file_name),'PreserveVariableNames',true);

time = data.t;
t_start = 1;
t_step  = 1;
t_end   = length(time);
t_dt = (time(t_end) - time(t_start))/4;
      
q_ref= [    data.q1_ref(t_start:t_step:t_end), ...
            data.q2_ref(t_start:t_step:t_end), ...
            data.q3_ref(t_start:t_step:t_end), ...
            data.q4_ref(t_start:t_step:t_end), ...
            data.q5_ref(t_start:t_step:t_end), ...
            data.q6_ref(t_start:t_step:t_end)];  
               
dq_ref= [   data.dq1_ref(t_start:t_step:t_end), ...
            data.dq2_ref(t_start:t_step:t_end), ...
            data.dq3_ref(t_start:t_step:t_end), ...
            data.dq4_ref(t_start:t_step:t_end), ...
            data.dq5_ref(t_start:t_step:t_end), ...
            data.dq6_ref(t_start:t_step:t_end)];  

ddq_ref= [  data.ddq1_ref(t_start:t_step:t_end), ...
            data.ddq2_ref(t_start:t_step:t_end), ...
            data.ddq3_ref(t_start:t_step:t_end), ...
            data.ddq4_ref(t_start:t_step:t_end), ...
            data.ddq5_ref(t_start:t_step:t_end), ...
            data.ddq6_ref(t_start:t_step:t_end)];  

% measured data
q_med= [    data.q1_med(t_start:t_step:t_end), ...
            data.q2_med(t_start:t_step:t_end), ...
            data.q3_med(t_start:t_step:t_end), ...
            data.q4_med(t_start:t_step:t_end), ...
            data.q5_med(t_start:t_step:t_end), ...
            data.q6_med(t_start:t_step:t_end)];  
               
dq_med= [   data.dq1_med(t_start:t_step:t_end), ...
            data.dq2_med(t_start:t_step:t_end), ...
            data.dq3_med(t_start:t_step:t_end), ...
            data.dq4_med(t_start:t_step:t_end), ...
            data.dq5_med(t_start:t_step:t_end), ...
            data.dq6_med(t_start:t_step:t_end)];  

ddq_med= [  data.ddq1_med(t_start:t_step:t_end), ...
            data.ddq2_med(t_start:t_step:t_end), ...
            data.ddq3_med(t_start:t_step:t_end), ...
            data.ddq4_med(t_start:t_step:t_end), ...
            data.ddq5_med(t_start:t_step:t_end), ...
            data.ddq6_med(t_start:t_step:t_end)];  
               
%% joint position: tracking performance
clc, close all;

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 15.0 20.0])

name_list = ["$\mathrm{q_1}$", "$\mathrm{q_2}$","$\mathrm{q_3}$","$\mathrm{q_4}$","$\mathrm{q_5}$","$\mathrm{q_6}$"];
    
for i=1:6
    plot_name = strcat(name_list(i),' ($\mathrm{rad}$)');
    subplot(3, 2, i),
    plot(time(t_start:t_step:t_end), q_ref(:, i), '-k', 'linewidth', 2), hold on, grid on, box on
    plot(time(t_start:t_step:t_end), q_med(:, i), '--r', 'linewidth', 2), hold on, grid on, box on
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(time(t_start):t_dt:time(t_end))
    xlim([0 time(t_end)])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
    
end         
    
% add legend
Lgnd = legend({'reference', 'measured'}, 'interpreter', 'latex', 'Orientation','horizontal');
Lgnd.FontSize = 12;
Lgnd.Position(1) = 0.3;
Lgnd.Position(2) = 0.95;

% Save image
file_name     = fullfile(image_path, 'joint_position');
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
    plot(time(t_start:t_step:t_end), dq_ref(:, i), '-k', 'linewidth', 2), hold on, grid on, box on
    plot(time(t_start:t_step:t_end), dq_med(:, i), '--r', 'linewidth', 2), hold on, grid on, box on
    
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(time(t_start):t_dt:time(t_end))
    xlim([0 time(t_end)])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
end         

% add legend
Lgnd = legend({'reference', 'measured'}, 'interpreter', 'latex', 'Orientation','horizontal');
Lgnd.FontSize = 12;
Lgnd.Position(1) = 0.3;
Lgnd.Position(2) = 0.95;

% Save image
file_name     = fullfile(image_path, 'joint_velocity');
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
    plot(time(t_start:t_step:t_end), ddq_ref(:, i), '-k', 'linewidth', 2), hold on, grid on, box on
    plot(time(t_start:t_step:t_end), ddq_med(:, i), '--r', 'linewidth', 2), hold on, grid on, box on
    
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(time(t_start):t_dt:time(t_end))
    xlim([0 time(t_end)])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
end         
% add legend
Lgnd = legend({'reference', 'measured'}, 'interpreter', 'latex', 'Orientation','horizontal');
Lgnd.FontSize = 12;
Lgnd.Position(1) = 0.3;
Lgnd.Position(2) = 0.95;

% Save image
file_name     = fullfile(image_path, 'joint_acceleration');
saveas(gcf,file_name,'epsc')  



