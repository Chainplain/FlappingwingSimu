%%% 2022年12月12日

DarkRed = [0.7,0.2,0.2];
PatchRed = [0.5,0.25,0.25];
LighGrayRed = [0.9,0.5,0.5];

DarkRed_2 = [0.5,0.3,0.3];
PatchRed_2 = [0.8,0.6,0.6];
LighGrayRed_2 = [0.90,0.55,0.5];

DarkGreen = [0.2,0.7,0.2];
PatchGreen = [0.25,0.5,0.25];
LighGrayGreen = [0.4,0.75,0.4];

DarkGreen_2 = [0.3,0.5,0.3];
PatchGreen_2 = [0.6,0.8,0.6];
LighGrayGreen_2 = [0.55,0.95,0.65];

DarkBlue = [0.2,0.2 ,0.7];
PatchBlue = [0.25,0.25, 0.5];
LighGrayBlue = [0.5,0.5,0.9];

DarkBlue_2 = [0.3,0.3,0.5];
PatchBlue_2 = [0.6,0.6,0.8];
LighGrayBlue_2 = [0.55,0.7,0.95];

LighGray = [0.6,0.6,0.6];

x_tick =0 + [0  4000 8000 12000 16000 20000];

Experiment_name_list = {'5th_proposed'; '5th_UT2'; '5th_UT3'};
Task = '_C_1_0_0';
Rear_name = '_Traj_Tracking_File.mat';

LineWidth = 1.2;
Emph = 1.5;

for i = 1 : 3
    load([Experiment_name_list{i}, Task, Rear_name]);
    Length =  size(Total_body_rotation_list,1);
    % Total_body_rotation_in_Euler_list = zeros(Length,3);

    Total_body_rotation_list_a = permute( Total_body_rotation_list,[2,3,1]);
    for j = 1 : Length
        Total_body_rotation_list_a(:,:,j) = Total_body_rotation_list_a(:,:,j);
    end
    Total_body_rotation_in_Euler_list_S{i} = rotm2eul(Total_body_rotation_list_a);
    
    Total_desired_rotation_list_a = permute( Total_desired_rotation_list,[2,3,1]);
     for j = 1 : Length
        Total_desired_rotation_list_a(:,:,j) = Total_desired_rotation_list_a(:,:,j);
    end
    Total_desired_rotation_in_Euler_list_S{i} = rotm2eul(Total_desired_rotation_list_a);
    
    Total_body_translation_list_S{i} = Total_body_translation_list;
    Total_body_translation_desire_S{i} = Total_body_translation_desire;
    
    Total_body_angular_velocity_list_S{i} = Total_body_angular_velocity_list;
    Total_Angular_velocity_filtered_list_S{i} = Total_Angular_velocity_filtered_list;
    
    Total_desired_angular_velocity_list_S{i} = Total_desired_angular_velocity_list;
    
    Total_psi_rotation_error_list_S{i} = Total_psi_rotation_error_list;
    Total_pitch_input_S{i} = Total_pitch_input;
    Total_roll_input_S{i} = Total_roll_input;
    Total_yaw_input_S{i} = Total_yaw_input;
%     
%     for j = 1 : Length
%         Total_wing_torque(j,:) = ((BasisRotation' *Total_body_rotation_list_a(:,:,j))' * Total_wing_torque(j,:)')';
%         Total_wing_force(j,:) = ((BasisRotation' *Total_body_rotation_list_a(:,:,j))' * Total_wing_force(j,:)')';
%         Total_rudder_torque(j,:) = ((BasisRotation' *Total_body_rotation_list_a(:,:,j))' * Total_rudder_torque(j,:)')';
%         Total_rudder_force(j,:) = ((BasisRotation' *Total_body_rotation_list_a(:,:,j))' * Total_rudder_force(j,:)')';
%         Total_tail_torque(j,:) = ((BasisRotation' *Total_body_rotation_list_a(:,:,j))' * Total_tail_torque(j,:)')';
%         Total_tail_force(j,:) = ((BasisRotation' *Total_body_rotation_list_a(:,:,j))' * Total_tail_force(j,:)')';
%     end
%     
    
    Total_wing_torque_S{i} = Total_wing_torque;
    Total_wing_force_S{i} = Total_wing_force;
    Total_rudder_torque_S{i} = Total_rudder_torque;
    Total_rudder_force_S{i} = Total_rudder_force;
    Total_tail_torque_S{i} = Total_tail_torque;
    Total_tail_force_S{i} = Total_tail_force;
end

figure;


subplot(4,1,1);
hold on;
plot(Total_body_translation_list_S{1}(:,1),"-",'LineWidth',LineWidth* Emph,'Color',LighGrayRed * 0.4);
plot(Total_body_translation_list_S{2}(:,1),":",'LineWidth',LineWidth,'Color',LighGrayRed* 0.6);
plot(Total_body_translation_list_S{3}(:,1),"-.",'LineWidth',LineWidth,'Color',LighGrayRed* 0.8);
plot(Total_body_translation_desire_S{1}(:,1),"--",'LineWidth',LineWidth * Emph,'Color',LighGrayRed* 1);
axis([x_tick(1),x_tick(end),-3,5])
set(gca,'xtick',x_tick);
set(gca,'ytick',(-3:2:5));
grid on;
h = gca;
set(h,'LineWidth',1,'GridLineStyle','--','GridAlpha',1,'GridColor',[0.9,0.9,0.9])
hold off;


subplot(4,1,2);
hold on;
plot(Total_body_translation_list_S{1}(:,2),"-",'LineWidth',LineWidth* Emph,'Color',LighGrayGreen * 0.4);
plot(Total_body_translation_list_S{2}(:,2),":",'LineWidth',LineWidth,'Color',LighGrayGreen * 0.6);
plot(Total_body_translation_list_S{3}(:,2),"-.",'LineWidth',LineWidth,'Color',LighGrayGreen * 0.8);
plot(Total_body_translation_desire_S{1}(:,2),"--",'LineWidth',LineWidth * Emph,'Color',LighGrayGreen * 1);
axis([x_tick(1),x_tick(end),-3,5])
set(gca,'xtick',x_tick);
set(gca,'ytick',(-3:2:5));
grid on;
h = gca;
set(h,'LineWidth',1,'GridLineStyle','--','GridAlpha',1,'GridColor',[0.9,0.9,0.9])
hold off;

subplot(4,1,3);
hold on;
plot(Total_body_translation_list_S{1}(:,3),"-",'LineWidth',LineWidth* Emph,'Color',LighGrayBlue * 0.4);
plot(Total_body_translation_list_S{2}(:,3),":",'LineWidth',LineWidth,'Color',LighGrayBlue * 0.6);
plot(Total_body_translation_list_S{3}(:,3),"-.",'LineWidth',LineWidth,'Color',LighGrayBlue * 0.8);
plot(Total_body_translation_desire_S{1}(:,3),"--",'LineWidth',LineWidth * Emph,'Color',LighGrayBlue * 1);
axis([x_tick(1),x_tick(end),-3,5])
set(gca,'xtick',x_tick);
set(gca,'ytick',(-3:2:5));
grid on;
h = gca;
set(h,'LineWidth',1,'GridLineStyle','--','GridAlpha',1,'GridColor',[0.9,0.9,0.9])
hold off;

subplot(4,1,4);
hold on;
for j = 1 : 3
ErrorSquare{j} =( Total_body_translation_list_S{j}(:,1) - double(Total_body_translation_desire_S{j}(:,1))).^2 ... 
              + ( Total_body_translation_list_S{j}(:,2) - double(Total_body_translation_desire_S{j}(:,2))).^2 ...
              + ( Total_body_translation_list_S{j}(:,3) - double(Total_body_translation_desire_S{j}(:,3))).^2;
Error{j} = sqrt(ErrorSquare{j}) ;        
end
plot(Error{1},"-",'LineWidth',LineWidth* Emph,'Color',LighGray * 0.4);
plot(Error{2},":",'LineWidth',LineWidth,'Color',LighGray * 0.6);
plot(Error{3},"-.",'LineWidth',LineWidth,'Color',LighGray * 0.8);
axis([x_tick(1),x_tick(end),0,6])
set(gca,'xtick',x_tick);
set(gca,'ytick',(0:2:6));
grid on;
h = gca;
set(h,'LineWidth',1,'GridLineStyle','--','GridAlpha',1,'GridColor',[0.9,0.9,0.9])
hold off;

set(gcf,'Position',[400,100,600,700]);


Chosen1 = 4000:7000;
Chosen2 = 12000:20000;
Chosen = [Chosen1, Chosen2];

disp('tau_1 RMS');
disp(rms(Error{1}(Chosen)))
disp('tau_2 RMS');
disp(rms(Error{2}(Chosen)))
disp('tau_3 RMS');
disp(rms(Error{3}(Chosen)))

disp('tau_1 MAX');
disp(max(Error{1}(Chosen)))
disp('tau_2 MAX');
disp(max(Error{2}(Chosen)))
disp('tau_3 MAX');
disp(max(Error{3}(Chosen)))