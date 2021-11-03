% plots - ur5 joint position
% act_1.3 y act_1.4
clc, clear all, close all;


pwd = '/home/jhon/catkin_ws/journal_ws/src/motion_ur5_rviz/';
dir  = 'data/PDi/circular_traj/'; 
file_name = 'articular_kp_0.5_alpha_0.5_beta_0.2_gamma_0.8_t_20.csv';
file_path = fullfile(pwd, dir);
image_path = fullfile(pwd,'document/images/');

data = readtable(fullfile(file_path, file_name),'PreserveVariableNames',true);

time = data.t;

t_start = 1;
t_step  = 1;
t_end   = length(time);
      
q_des= [  data.q1_des(t_start:t_step:t_end), ...
                    data.q2_des(t_start:t_step:t_end), ...
                    data.q3_des(t_start:t_step:t_end), ...
                    data.q4_des(t_start:t_step:t_end), ...
                    data.q5_des(t_start:t_step:t_end), ...
                    data.q6_des(t_start:t_step:t_end)];  
               
dq_des= [   data.dq1_des(t_start:t_step:t_end), ...
                    data.dq2_des(t_start:t_step:t_end), ...
                    data.dq3_des(t_start:t_step:t_end), ...
                    data.dq4_des(t_start:t_step:t_end), ...
                    data.dq5_des(t_start:t_step:t_end), ...
                    data.dq6_des(t_start:t_step:t_end)];  

ddq_des= [data.ddq1_des(t_start:t_step:t_end), ...
                    data.ddq2_des(t_start:t_step:t_end), ...
                    data.ddq3_des(t_start:t_step:t_end), ...
                    data.ddq4_des(t_start:t_step:t_end), ...
                    data.ddq5_des(t_start:t_step:t_end), ...
                    data.ddq6_des(t_start:t_step:t_end)];  

% measured data
q_med= [  data.q1(t_start:t_step:t_end), ...
                    data.q2(t_start:t_step:t_end), ...
                    data.q3(t_start:t_step:t_end), ...
                    data.q4(t_start:t_step:t_end), ...
                    data.q5(t_start:t_step:t_end), ...
                    data.q6(t_start:t_step:t_end)];  
               
dq_med= [   data.dq1(t_start:t_step:t_end), ...
                    data.dq2(t_start:t_step:t_end), ...
                    data.dq3(t_start:t_step:t_end), ...
                    data.dq4(t_start:t_step:t_end), ...
                    data.dq5(t_start:t_step:t_end), ...
                    data.dq6(t_start:t_step:t_end)];  

ddq_med= [data.ddq1(t_start:t_step:t_end), ...
                    data.ddq2(t_start:t_step:t_end), ...
                    data.ddq3(t_start:t_step:t_end), ...
                    data.ddq4(t_start:t_step:t_end), ...
                    data.ddq5(t_start:t_step:t_end), ...
                    data.ddq6(t_start:t_step:t_end)];  
               
%% joint position: tracking performance
clc, close all;

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 15.0 20.0])

name_list = ["$\mathrm{q_1}$", "$\mathrm{q_2}$","$\mathrm{q_3}$","$\mathrm{q_4}$","$\mathrm{q_5}$","$\mathrm{q_6}$"];
    
for i=1:6
    plot_name = strcat(name_list(i),' ($\mathrm{rad}$)');
    subplot(3, 2, i),
    plot(time(t_start:t_step:t_end), q_des(:, i), '-k', 'linewidth', 2), hold on, grid on, box on
    plot(time(t_start:t_step:t_end), q_med(:, i), '--r', 'linewidth', 2), hold on, grid on, box on
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(0:5:20)
    xlim([0 20])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
    
end         
    
% add legend
Lgnd = legend({'desired', 'measured'}, 'interpreter', 'latex', 'Orientation','horizontal');
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

for i=1:6
    plot_name = strcat('$\mathrm{\dot{q}}$',num2str(i),'  ($\mathrm{\frac{rad}{s}}$)');
    subplot(3, 2, i),
    plot(time(t_start:t_step:t_end), dq_des(:, i), '-k', 'linewidth', 2), hold on, grid on, box on
    plot(time(t_start:t_step:t_end), dq_med(:, i), '--r', 'linewidth', 2), hold on, grid on, box on
    
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(0:1:5)
    xlim([0 5])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
end         

% add legend
Lgnd = legend({'desired', 'measured'}, 'interpreter', 'latex', 'Orientation','horizontal');
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

for i=1:6
    plot_name = strcat('$\mathrm{\ddot{q}}$',num2str(i),'  ($\mathrm{\frac{rad}{s^2}}$)');
    subplot(3, 2, i),
    plot(time(t_start:t_step:t_end), ddq_des(:, i), '-k', 'linewidth', 2), hold on, grid on, box on
    plot(time(t_start:t_step:t_end), ddq_med(:, i), '--r', 'linewidth', 2), hold on, grid on, box on
    %title(plot_name, 'interpreter', 'latex')
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(0:1:5)
    xlim([0 5])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
    
end         
% add legend
Lgnd = legend({'desired', 'measured'}, 'interpreter', 'latex', 'Orientation','horizontal');
Lgnd.FontSize = 12;
Lgnd.Position(1) = 0.3;
Lgnd.Position(2) = 0.95;

% Save image
file_name     = fullfile(image_path, 'joint_acceleration');
saveas(gcf,file_name,'epsc')  



%% kd: critical damping
kd= [data.kd1(t_start:t_step:t_end), ...
        data.kd2(t_start:t_step:t_end), ...
        data.kd3(t_start:t_step:t_end), ...
        data.kd4(t_start:t_step:t_end), ...
        data.kd5(t_start:t_step:t_end), ...
        data.kd6(t_start:t_step:t_end)];  

clc, close all

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 15.0 20.0])

for i=1:6
    plot_name = strcat('$\mathrm{k_d}$',num2str(i),'  ($\mathrm{\frac{N.m.s}{rad}}$)');
    subplot(3, 2, i),
    plot(time(t_start:t_step:t_end), kd(:, i), '-k', 'linewidth', 2), hold on, grid on, box on
    %title(plot_name, 'interpreter', 'latex')
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(0:1:5)
    xlim([0 5])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
    
end         
%%
% add legend
%Lgnd = legend({'desired', 'measured'}, 'interpreter', 'latex', 'Orientation','horizontal');
%Lgnd.FontSize = 12;
%Lgnd.Position(1) = 0.3;
%Lgnd.Position(2) = 0.95;
% Save image
file_name     = fullfile(image_path, 'kd');
saveas(gcf,file_name,'epsc')  

%% g: gravity term
g= [data.g1(t_start:t_step:t_end), ...
       data.g2(t_start:t_step:t_end), ...
       data.g3(t_start:t_step:t_end), ...
       data.g4(t_start:t_step:t_end), ...
       data.g5(t_start:t_step:t_end), ...
       data.g6(t_start:t_step:t_end)];  

clc, close all

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 15.0 20.0])

for i=1:6
    plot_name = strcat('$\mathrm{g}$',num2str(i),'  ($\mathrm{N.m}$)');
    subplot(3, 2, i),
    plot(time(t_start:t_step:t_end), g(:, i), '-k', 'linewidth', 2), hold on, grid on, box on
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(0:1:5)
    xlim([0 5])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
end     
% Save image
file_name     = fullfile(image_path, 'g');
saveas(gcf,file_name,'epsc')  

%% C: coriolis and cetripetal forces
C= [data.C1(t_start:t_step:t_end), ...
       data.C2(t_start:t_step:t_end), ...
       data.C3(t_start:t_step:t_end), ...
       data.C4(t_start:t_step:t_end), ...
       data.C5(t_start:t_step:t_end), ...
       data.C6(t_start:t_step:t_end)];  

clc, close all

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 15.0 20.0])

for i=1:6
    plot_name = strcat('$\mathrm{C}$',num2str(i),'  ($\mathrm{N.m}$)');
    subplot(3, 2, i),
    plot(time(t_start:t_step:t_end), C(:, i), '-k', 'linewidth', 2), hold on, grid on, box on
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(0:1:5)
    xlim([0 5])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')

    if ((i==2) || (i==5))
        plot(time(2153), C(2153,i), 'marker','o', 'MarkerSize', 8, 'MarkerFaceColor', 'r'), hold on, grid on, box on
        plot(time(2212), C(2212,i), 'marker','o', 'MarkerSize', 8, 'MarkerFaceColor', 'r'), hold on, grid on, box on
    end
end     
% Save image
file_name     = fullfile(image_path, 'C');
saveas(gcf,file_name,'epsc')  

%% Joint Velocity: error
de= [%data.de1(t_start:t_step:t_end), ...
        data.de2(t_start:t_step:t_end), ...
        data.de5(t_start:t_step:t_end)%, ...
        %data.de6(t_start:t_step:t_end)
        ];  

clc, close all

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 15.0 7.0])

name_list = ["$\mathrm{\dot{e}_2}$", "$\mathrm{\dot{e}_5}$"];

for i=1:2
    plot_name = strcat(name_list(i),'  ($\mathrm{\frac{rad}{s}}$)');
    subplot(1, 2, i),
    plot(time(t_start:t_step:t_end), de(:, i), '-k', 'linewidth', 2), hold on, grid on, box on
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(0:1:5)
    xlim([0 5])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
end     
% Save image
file_name     = fullfile(image_path, 'de');
saveas(gcf,file_name,'epsc')  

%% Joint Position: error
e= [data.e1(t_start:t_step:t_end), ...
        data.e2(t_start:t_step:t_end), ...
        data.e3(t_start:t_step:t_end), ...
        data.e4(t_start:t_step:t_end), ...
        data.e5(t_start:t_step:t_end), ...
        data.e6(t_start:t_step:t_end)
        ];  

clc, close all

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 15.0 20.0])

name_list = ["$\mathrm{e_1}$", "$\mathrm{e_2}$","$\mathrm{e_3}$","$\mathrm{e_4}$","$\mathrm{e_5}$","$\mathrm{e_6}$"];

for i=1:6
    plot_name = strcat(name_list(i),'  ($\mathrm{{rad}}$)');
    subplot(3, 2, i),
    plot(time(t_start:t_step:t_end), e(:, i), '-k', 'linewidth', 2), hold on, grid on, box on
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(0:1:5)
    xlim([0 5])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
end     
% Save image
file_name     = fullfile(image_path, 'e');
saveas(gcf,file_name,'epsc')  
