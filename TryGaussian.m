clear all;
TIME_STEP = 0.001;
SampleNum = 300;
Final = 100;


Experiment_name_list = {'5th_proposed'; '5th_UT2'; '5th_UT3'};
Task_list = {'_C_0_1_0', '_C_1_0_0', '_L_1_0_0', '_P_1_0_0'};
Rear_name = '_Traj_Tracking_File.mat';
X_data_all = [];
Y_data_all = [];
Filter_width = 300;

for  kk = 1 : 3
    for hh = 1 : 4

        load([Experiment_name_list{kk}, Task_list{hh}, Rear_name]);



        Total_body_rotation_list_a = permute( Total_body_rotation_list,[2,3,1]);
        Total_body_rotation_list_euler_vec = rotm2eul(Total_body_rotation_list_a);

        % Total_body_angular_velocity_list;

        % Total_body_translation_list
        Total_Freq_stroke_vec = Total_Freq_stroke';
        Total_pitch_input_vec =  Total_pitch_input';
        Total_roll_input_vec  =  Total_roll_input';
        Total_yaw_input_vec  =  Total_yaw_input';


        Total_body_translation_pos_vec = Total_body_translation_list;
        Total_Angular_velocity_filtered_vec = Total_Angular_velocity_filtered_list;

        

        Total_body_translation_pos_vec_filtered = ave_filter( Total_body_translation_pos_vec, Filter_width);

        Total_body_translation_vel_vec = gnerate_rate_with_time_step( ...
                                            Total_body_translation_pos_vec_filtered, TIME_STEP );
        Total_body_translation_vel_vec_filtered = ave_filter( Total_body_translation_vel_vec, Filter_width);

        Total_body_translation_acc_vec = gnerate_rate_with_time_step( ...
                                            Total_body_translation_vel_vec_filtered, TIME_STEP );

        Total_body_translation_vel_vec_in_body = zeros(size(Total_body_translation_pos_vec));
        Total_body_translation_acc_vec_in_body = zeros(size(Total_body_translation_acc_vec));
        
%         Total_body_translation_ang_acc_vec_in_body = zeros(size(Total_Angular_velocity_filtered_vec));
        
        Total_body_translation_ang_acc_vec_in_body = gnerate_rate_with_time_step( ...
                                            Total_Angular_velocity_filtered_vec, TIME_STEP );
         Total_body_translation_ang_acc_vec_in_body_filtered = ...
             ave_filter( Total_body_translation_ang_acc_vec_in_body, Filter_width);
         
        Total_gravity_in_body = zeros(size(Total_body_translation_acc_vec));
        
        %%
        gravity = 9.8; 
        for i = 1 :  size(Total_body_translation_pos_vec, 1)
            Total_body_translation_vel_vec_in_body(i , :) = (Total_body_rotation_list_a(:,:,i)' *...
                                                            Total_body_translation_pos_vec(i , :)')';
            Total_body_translation_acc_vec_in_body(i , :) = (Total_body_rotation_list_a(:,:,i)' *...
                                                            Total_body_translation_acc_vec(i , :)')';
            Total_gravity_in_body(i , :) = gravity * (Total_body_rotation_list_a(:,:,i)' *...
                                                            [0; 0; 1])';
        end

        X_data = [Total_Freq_stroke_vec,...
                  Total_pitch_input_vec,...
                  Total_roll_input_vec,...
                  Total_yaw_input_vec,...
                  Total_body_translation_vel_vec_in_body, ...
                  Total_Angular_velocity_filtered_vec,...
                  Total_body_rotation_list_euler_vec];
        Y_data = [Total_body_translation_acc_vec_in_body + Total_gravity_in_body, Total_body_translation_ang_acc_vec_in_body_filtered];
        
        X_data_all = [X_data_all; X_data];
        Y_data_all = [Y_data_all; Y_data];
    end
end
random_index = randperm(size(Y_data_all,1));
sample_index = random_index(1:SampleNum);



gprMdl_1 = fitrgp(X_data_all(sample_index,:),Y_data_all(sample_index,1),'KernelFunction','squaredExponential','FitMethod', ...
    'sr','PredictMethod','sd','Basis','none','ActiveSetSize',Final, ...
    'ActiveSetMethod','sgma','Standardize',1,'KernelParameters',[1;1]);


gprMdl_2 = fitrgp(X_data_all(sample_index,:),Y_data_all(sample_index,2),'KernelFunction','squaredExponential','FitMethod', ...
    'sr','PredictMethod','sd','Basis','none','ActiveSetSize',Final, ...
    'ActiveSetMethod','sgma','Standardize',1,'KernelParameters',[1;1]);
gprMdl_3 = fitrgp(X_data_all(sample_index,:),Y_data_all(sample_index,3),'KernelFunction','squaredExponential','FitMethod', ...
    'sr','PredictMethod','sd','Basis','none','ActiveSetSize',Final, ...
    'ActiveSetMethod','sgma','Standardize',1,'KernelParameters',[1;1]);
gprMdl_4 = fitrgp(X_data_all(sample_index,:),Y_data_all(sample_index,4),'KernelFunction','squaredExponential','FitMethod', ...
    'sr','PredictMethod','sd','Basis','none','ActiveSetSize',Final, ...
    'ActiveSetMethod','sgma','Standardize',1,'KernelParameters',[1;1]);
gprMdl_5 = fitrgp(X_data_all(sample_index,:),Y_data_all(sample_index,5),'KernelFunction','squaredExponential','FitMethod', ...
    'sr','PredictMethod','sd','Basis','none','ActiveSetSize',Final, ...
    'ActiveSetMethod','sgma','Standardize',1,'KernelParameters',[1;1]);
gprMdl_6 = fitrgp(X_data_all(sample_index,:),Y_data_all(sample_index,6),'KernelFunction','squaredExponential','FitMethod', ...
    'sr','PredictMethod','sd','Basis','none','ActiveSetSize',Final, ...
    'ActiveSetMethod','sgma','Standardize',1,'KernelParameters',[1;1]);

Y_pre_1 = predict(gprMdl_1,X_data);
Y_pre_2 = predict(gprMdl_2,X_data);
Y_pre_3 = predict(gprMdl_3,X_data);
Y_pre_4 = predict(gprMdl_4,X_data);
Y_pre_5 = predict(gprMdl_5,X_data);
Y_pre_6 = predict(gprMdl_6,X_data);

%%
figure;
subplot(6,1,1);
hold on;
plot(ave_filter(Y_pre_1,100) ,'b','linewidth',1);
plot(Y_data(:,1),'r','linewidth',1);
ylabel('x_acc')
hold off;

subplot(6,1,2);
hold on;
plot(ave_filter(Y_pre_2,100),'b','linewidth',1);
plot(Y_data(:,2),'r','linewidth',1);
ylabel('y_acc')
hold off;

subplot(6,1,3);
hold on;
plot(ave_filter(Y_pre_3,100),'b','linewidth',1);
plot(Y_data(:,3),'r','linewidth',1);
ylabel('z_acc')
hold off;

subplot(6,1,4);
hold on;
plot(ave_filter(Y_pre_4,100),'b','linewidth',1);
plot(Y_data(:,4),'r','linewidth',1);
ylabel('x_ang_acc')
hold off;

subplot(6,1,5);
hold on;
plot(ave_filter(Y_pre_5,100),'b','linewidth',1);
plot(Y_data(:,5),'r','linewidth',1);
ylabel('y_ang_acc')
hold off;

subplot(6,1,6);
hold on;
plot(ave_filter(Y_pre_6,100),'b','linewidth',1);
plot(Y_data(:,6),'r','linewidth',1);
ylabel('z_ang_acc')
hold off;

% %%
% Experiment_name_list = {'5th_proposed'; '5th_UT2'; '5th_UT3'};
% Task_list = {'_C_0_1_0', '_C_1_0_0', '_L_1_0_0', '_P_1_0_0'};
% Rear_name = '_Traj_Tracking_File.mat';
% X_data_all = [];
% Y_data_all = [];
% Filter_width = 400;
% 
% kk = 3;
% hh = 4;
%         load([Experiment_name_list{kk}, Task_list{hh}, Rear_name]);
% 
% 
% 
%         Total_body_rotation_list_a = permute( Total_body_rotation_list,[2,3,1]);
%         Total_body_rotation_list_euler_vec = rotm2eul(Total_body_rotation_list_a);
% 
%         % Total_body_angular_velocity_list;
% 
%         % Total_body_translation_list
%         Total_Freq_stroke_vec = Total_Freq_stroke';
%         Total_pitch_input_vec =  Total_pitch_input';
%         Total_roll_input_vec  =  Total_roll_input';
%         Total_yaw_input_vec  =  Total_yaw_input';
% 
% 
%         Total_body_translation_pos_vec = Total_body_translation_list;
%         Total_Angular_velocity_filtered_vec = Total_Angular_velocity_filtered_list;
% 
%         
% 
%         Total_body_translation_pos_vec_filtered = ave_filter( Total_body_translation_pos_vec, Filter_width);
% 
%         Total_body_translation_vel_vec = gnerate_rate_with_time_step( ...
%                                             Total_body_translation_pos_vec_filtered, TIME_STEP );
%         Total_body_translation_vel_vec_filtered = ave_filter( Total_body_translation_vel_vec, Filter_width);
% 
%         Total_body_translation_acc_vec = gnerate_rate_with_time_step( ...
%                                             Total_body_translation_vel_vec_filtered, TIME_STEP );
% 
%         Total_body_translation_vel_vec_in_body = zeros(size(Total_body_translation_pos_vec));
%         Total_body_translation_acc_vec_in_body = zeros(size(Total_body_translation_acc_vec));
%         
% %         Total_body_translation_ang_acc_vec_in_body = zeros(size(Total_Angular_velocity_filtered_vec));
%         
%         Total_body_translation_ang_acc_vec_in_body = gnerate_rate_with_time_step( ...
%                                             Total_Angular_velocity_filtered_vec, TIME_STEP );
%          Total_body_translation_ang_acc_vec_in_body_filtered = ...
%              ave_filter( Total_body_translation_ang_acc_vec_in_body, Filter_width);
%          
%         Total_gravity_in_body = zeros(size(Total_body_translation_acc_vec));
%         
%         %%
%         gravity = 9.8; 
%         for i = 1 :  size(Total_body_translation_pos_vec, 1)
%             Total_body_translation_vel_vec_in_body(i , :) = (Total_body_rotation_list_a(:,:,i)' *...
%                                                             Total_body_translation_pos_vec(i , :)')';
%             Total_body_translation_acc_vec_in_body(i , :) = (Total_body_rotation_list_a(:,:,i)' *...
%                                                             Total_body_translation_acc_vec(i , :)')';
%             Total_gravity_in_body(i , :) = gravity * (Total_body_rotation_list_a(:,:,i)' *...
%                                                             [0; 0; 1])';
%         end
% 
%         X_data = [Total_Freq_stroke_vec,...
%                   Total_pitch_input_vec,...
%                   Total_roll_input_vec,...
%                   Total_yaw_input_vec,...
%                   Total_body_translation_vel_vec_in_body, ...
%                   Total_Angular_velocity_filtered_vec,...
%                   Total_body_rotation_list_euler_vec];
%         Y_data = [Total_body_translation_acc_vec_in_body + Total_gravity_in_body, Total_body_translation_ang_acc_vec_in_body_filtered];
% %%
% 
% Y_pre_1 = predict(gprMdl_1,X_data);
% Y_pre_2 = predict(gprMdl_2,X_data);
% Y_pre_3 = predict(gprMdl_3,X_data);
% Y_pre_4 = predict(gprMdl_4,X_data);
% Y_pre_5 = predict(gprMdl_5,X_data);
% [Y_pre_6,~,Y_int_6] = predict(gprMdl_6,X_data);
% 
% 
% figure;
% subplot(6,1,1);
% hold on;
% plot(ave_filter(Y_pre_1,100) ,'b','linewidth',1);
% plot(Y_data(:,1),'r','linewidth',1);
% ylabel('x acc')
% hold off;
% 
% subplot(6,1,2);
% hold on;
% plot(ave_filter(Y_pre_2,100),'b','linewidth',1);
% plot(Y_data(:,2),'r','linewidth',1);
% ylabel('y acc')
% hold off;
% 
% subplot(6,1,3);
% hold on;
% plot(ave_filter(Y_pre_3,100),'b','linewidth',1);
% plot(Y_data(:,3),'r','linewidth',1);
% ylabel('z acc')
% hold off;
% 
% subplot(6,1,4);
% hold on;
% plot(ave_filter(Y_pre_4,100),'b','linewidth',1);
% plot(Y_data(:,4),'r','linewidth',1);
% ylabel('x ang acc')
% hold off;
% 
% subplot(6,1,5);
% hold on;
% plot(ave_filter(Y_pre_5,100),'b','linewidth',1);
% plot(Y_data(:,5),'r','linewidth',1);
% ylabel('y ang acc')
% hold off;
% 
% subplot(6,1,6);
% hold on;
% plot(ave_filter(Y_pre_6,100),'b','linewidth',1);
% plot(Y_data(:,6),'r','linewidth',1);
% ylabel('z ang acc')
% x = (1:size(Y_data(:,6),1))';                  % GPR predictions
% patch([x;flipud(x)],[Y_int_6(:,1);flipud(Y_int_6(:,2))],'k','FaceAlpha',0.1); % Prediction intervals
% hold off;


%%
function A_rate = gnerate_rate_with_time_step(A, tims_step)
Diff = zeros(size(A));
Diff(2 : end-1, :) = (A(3 : end, :) -  A(1 : end-2, :)) / tims_step /2;
A_rate =  Diff;
end

function A_filtered = ave_filter(A, windowSize)
 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
A_filtered = filter(b,a,A);
end





