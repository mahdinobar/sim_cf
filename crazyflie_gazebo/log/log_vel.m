%load bag file and read its messages
clear all
close all
clc
sample=100;%the step size to ignore messages received in between 
%[e.g. sample=1 means all messages are read]
bag = rosbag('models_states_hlafmps_100hz_basic.bag')
bag.AvailableTopics
%bag.MessageList
bagselect = select(bag, 'Topic', '/gazebo/model_states');
%start = bag.StartTime
%bagselect3 = select(bag, 'Time', [start+1 start + 3], 'Topic', '/gazebo/model_states')
msgs = readMessages(bagselect,1:sample:bagselect.NumMessages);
%msgs2=msgs{2}
%%
%drones and obstacles position vector extraction
% num_drones=3;%number of drones
% num_obs=0; %number of obstacles
% r_d=zeros(length(msgs),3,num_drones);
arg_cf1=3;
figure(1)
title('Real and Commanded Velocity to Crazyflie 1 vs time')
subplot(3,1,1)
v_d_x=cellfun(@(m) double(m.Twist(arg_cf1, 1).Linear.X),msgs);
plot(bagselect.MessageList.Time(1:sample:bagselect.NumMessages),v_d_x(:,1,1))
xlabel('time (s)')
ylabel('V_x (m/s)')
grid on
hold on
plot(bagselect.MessageList.Time(1:sample:bagselect.NumMessages),0.*v_d_x(:,1,1)+0.5)
legend('real','command')
subplot(3,1,2)
v_d_y=cellfun(@(m) double(m.Twist(arg_cf1, 1).Linear.Y),msgs);
plot(bagselect.MessageList.Time(1:sample:bagselect.NumMessages),v_d_y(:,1,1))
xlabel('time (s)')
ylabel('V_y (m/s)')
grid on
hold on
plot(bagselect.MessageList.Time(1:sample:bagselect.NumMessages),0.*v_d_y(:,1,1))
legend('real','command')
subplot(3,1,3)
v_d_z=cellfun(@(m) double(m.Twist(arg_cf1, 1).Linear.Z),msgs);
plot(bagselect.MessageList.Time(1:sample:bagselect.NumMessages),v_d_z(:,1,1))
xlabel('time (s)')
ylabel('V_z (m/s)')
grid on
hold on
plot(bagselect.MessageList.Time(1:sample:bagselect.NumMessages),0.*v_d_z(:,1,1))
legend('real','command')


% for d=(1+num_obs+1):(1+num_obs+num_drones)
%     r_d(:,1,d-1-num_obs)=cellfun(@(m) double(m.Pose(d, 1).Position.X),msgs);
%     r_d(:,2,d-1-num_obs)=cellfun(@(m) double(m.Pose(d, 1).Position.Y),msgs);
%     r_d(:,3,d-1-num_obs)=cellfun(@(m) double(m.Pose(d, 1).Position.Z),msgs);
% end
% r_ob=zeros(length(msgs),3,num_obs);
% for ob=(1+1):(1+num_obs)
% %     r_ob(:,1,ob-1)=cellfun(@(m) double(m.Pose(ob, 1).Position.X),msgs);
% %     r_ob(:,2,ob-1)=cellfun(@(m) double(m.Pose(ob, 1).Position.Y),msgs);
% %     r_ob(:,3,ob-1)=cellfun(@(m) double(m.Pose(ob, 1).Position.Z),msgs);
%     r_ob(:,1,ob-1)=10.;
%     r_ob(:,2,ob-1)=1.;
%     r_ob(:,3,ob-1)=5.;
% end
% %%
% %distance calculation and plot over time
% if num_drones==1 %we exclude when we have only one drone for finding nchoosek[combination]
%     drone_combinations=[1 1];
% else
%     drone_combinations=nchoosek(1:num_drones,2);
% end
% dot_drones_obstacles=zeros(length(msgs),3,num_drones,num_obs);
% dot_drones = zeros(length(msgs),3,size(drone_combinations,1));
% distance_drones_obstacles=zeros(length(msgs),num_drones,num_obs);
% distance_drones=zeros(length(msgs),size(drone_combinations,1));
% min_distance=zeros(length(msgs),num_drones,num_obs);
% max_distance=zeros(length(msgs),num_drones,num_obs);
% mean_distance=zeros(length(msgs),num_drones,num_obs);
% min_distance_AllDrones_EachObstacle=zeros(length(msgs),num_obs);
% leg33=[];%used for plot legends
% leg22=[];
% leg44=[];
% for k=1:size(drone_combinations,1)
%     dot_drones(:,:,k)=[r_d(:,:,drone_combinations(k,1))-r_d(:,:,drone_combinations(k,2))];
%     for i=1:length(msgs)
%         distance_drones(i,k)=norm(dot_drones(i,:,k));                
%     end
% %     figure(4)
% %     plot(bag.MessageList.Time(1:100:bagselect.NumMessages),distance_drones(:,k))
% %     leg4=['drone ' '(' num2str(drone_combinations(k,1)) ' & drone ' num2str(drone_combinations(k,1))];
% %     leg44=strvcat(leg44,leg4);
% %     hold on
% %     title('Inter-agent distance over time')
% %     xlabel('time (s)')
% %     ylabel('distance (m)')
% %     grid on
% end
% figure(1)
% subplot(2,1,1)
% plot(bag.MessageList.Time(1:100:bagselect.NumMessages),mean(distance_drones(:,:),2))
% hold on
% plot(bag.MessageList.Time(1:100:bagselect.NumMessages),min(distance_drones(:,:),[],2))
% plot(bag.MessageList.Time(1:100:bagselect.NumMessages),max(distance_drones(:,:),[],2))
% legend('Average','Minimum','Maximum')
% title('Ave./Min./Max. inter-agent distance over time')
% xlabel('time (s)')
% ylabel('distance (m)')
% grid on
% for ob=1:num_obs
%     for d=1:num_drones
%         dot_drones_obstacles(:,:,d,ob)=[r_d(:,:,d)-r_ob(:,:,ob)];
%         dot_drones_obstacles(:,3,d,ob)=0;%here it is assumed obsticales are 
% %         pillar so z coordinate of drones are always equal to the obstacle
%         for i=1:length(msgs)
%             distance_drones_obstacles(i,d,ob)=norm(dot_drones_obstacles(i,:,d,ob));
%             min_distance(i,d,ob)=min(distance_drones_obstacles(1:i,d,ob));
%             max_distance(i,d,ob)=max(distance_drones_obstacles(1:i,d,ob));
%             mean_distance(i,d,ob)=mean(distance_drones_obstacles(1:i,d,ob));
%         end
% %         figure(2)
% %         plot(bag.MessageList.Time(1:100:bagselect.NumMessages),distance_drones_obstacles(:,d,ob))
% %         leg2=['drone ' num2str(d) ' & obstacle ' num2str(ob)];
% %         leg22=strvcat(leg22,leg2);
% %         hold on
% %         title('Drones distance from obstacles over time')
% %         xlabel('time (s)')
% %         ylabel('distance (m)')
% %         grid on
%     end
%     min_distance_AllDrones_EachObstacle(:,ob)=min(distance_drones_obstacles(:,:,ob),[],2);
%     figure(1)
%     subplot(2,1,2)
%     plot(bag.MessageList.Time(1:100:bagselect.NumMessages),min_distance_AllDrones_EachObstacle(:,ob))
%     leg3=['obstacle = ' '' num2str(ob) ''];
%     leg33=strvcat(leg33,leg3);
%     hold on
%     title('Minimum distance between all drones with each obstacle over time')
%     xlabel('time (s)')
%     ylabel('minimum distance (m)')
%     grid on
% end
% % figure(2)
% % legend(leg22)
% figure(1)
% subplot(2,1,2)
% legend(leg33)
% % figure(4)
% % legend(leg44)
% 
%%
% %read commanded velocity message messages
% bagselect_velocity_command = select(bag, 'Topic', '/iris_1/geometry/velocity');
% msgs_velocity_command = readMessages(bagselect_velocity_command,1:1:bagselect_velocity_command.NumMessages);
% 
% bagselect_velocity_obstacle = select(bag, 'Topic', '/iris_1/geometry/velocity_obstacle');
% msgs_velocity_obstacle = readMessages(bagselect_velocity_obstacle,1:1:bagselect_velocity_obstacle.NumMessages);
% bagselect_velocity_obstacle = select(bag, 'Topic', '/iris_1/geometry/velocity_obstacle');
% msgs_velocity_obstacle = readMessages(bagselect_velocity_obstacle,1:1:bagselect_velocity_obstacle.NumMessages);
% bagselect_velocity_potential = select(bag, 'Topic', '/iris_1/geometry/velocity_potential');
% msgs_velocity_potential = readMessages(bagselect_velocity_potential,1:1:bagselect_velocity_potential.NumMessages);
% bagselect_velocity_matching = select(bag, 'Topic', '/iris_1/geometry/velocity_matching');
% msgs_velocity_matching = readMessages(bagselect_velocity_matching,1:1:bagselect_velocity_matching.NumMessages);

%%
% %drones velocity vector extraction
% v_d=zeros(length(msgs),3,num_drones);
% v_d_command=zeros(length(msgs_velocity_command),3,num_drones);
% v_d_obstacle=zeros(length(msgs_velocity_command),3,num_drones);
% v_d_potential=zeros(length(msgs_velocity_command),3,num_drones);
% v_d_matching=zeros(length(msgs_velocity_command),3,num_drones);
% for d=(1+num_obs+1):(1+num_obs+num_drones)
%     d_h=d-num_obs-1;
%     v_d(:,1,d-1-num_obs)=cellfun(@(m) double(m.Twist(d, 1).Linear.X),msgs);
%     v_d(:,2,d-1-num_obs)=cellfun(@(m) double(m.Twist(d, 1).Linear.Y),msgs);
%     v_d(:,3,d-1-num_obs)=cellfun(@(m) double(m.Twist(d, 1).Linear.Z),msgs);
%     v_d_command(:,1,d_h)=cellfun(@(m) double(m(d_h, 1).X),msgs_velocity_command);
%     v_d_command(:,2,d_h)=cellfun(@(m) double(m(d_h, 1).Y),msgs_velocity_command);
%     v_d_command(:,3,d_h)=cellfun(@(m) double(m(d_h, 1).Z),msgs_velocity_command);
%     v_d_obstacle(:,1,d_h)=cellfun(@(m) double(m(d_h, 1).X),msgs_velocity_obstacle);
%     v_d_obstacle(:,2,d_h)=cellfun(@(m) double(m(d_h, 1).Y),msgs_velocity_obstacle);
%     v_d_obstacle(:,3,d_h)=cellfun(@(m) double(m(d_h, 1).Z),msgs_velocity_obstacle);
%     v_d_potential(:,1,d_h)=cellfun(@(m) double(m(d_h, 1).X),msgs_velocity_potential);
%     v_d_potential(:,2,d_h)=cellfun(@(m) double(m(d_h, 1).Y),msgs_velocity_potential);
%     v_d_potential(:,3,d_h)=cellfun(@(m) double(m(d_h, 1).Z),msgs_velocity_potential);
%     v_d_matching(:,1,d_h)=cellfun(@(m) double(m(d_h, 1).X),msgs_velocity_matching);
%     v_d_matching(:,2,d_h)=cellfun(@(m) double(m(d_h, 1).Y),msgs_velocity_matching);
%     v_d_matching(:,3,d_h)=cellfun(@(m) double(m(d_h, 1).Z),msgs_velocity_matching);
% end

%%
% %plot command and real velocity of one drone
% figure(5)
% subplot(3,1,1)
% plot(bagselect_velocity_command.MessageList.Time,v_d_command(:,1,1))
% hold on
% plot(bagselect_velocity_obstacle.MessageList.Time,v_d_obstacle(:,1,1))
% hold on
% plot(bagselect_velocity_potential.MessageList.Time,v_d_potential(:,1,1))
% hold on
% plot(bagselect_velocity_matching.MessageList.Time,v_d_matching(:,1,1))
% hold on
% plot(bagselect.MessageList.Time(1:sample:bagselect.NumMessages),v_d(:,1,1))
% legend('command','obstacle','potential','matching','real')
% title('Drone velocity along x axis over time')
% xlabel('time (s)')
% ylabel('V_x (m/s)')
% grid on
% subplot(3,1,2)
% plot(bagselect_velocity_command.MessageList.Time,v_d_command(:,2,1))
% hold on
% plot(bagselect_velocity_obstacle.MessageList.Time,v_d_obstacle(:,2,1))
% hold on
% plot(bagselect_velocity_potential.MessageList.Time,v_d_potential(:,2,1))
% hold on
% plot(bagselect_velocity_matching.MessageList.Time,v_d_matching(:,2,1))
% hold on
% plot(bagselect.MessageList.Time(1:sample:bagselect.NumMessages),v_d(:,2,1))
% legend('command','obstacle','potential','matching','real')
% title('Drone real velocity along y axis over time')
% xlabel('time (s)')
% ylabel('V_y (m/s)')
% grid on
% subplot(3,1,3)
% plot(bagselect_velocity_command.MessageList.Time,v_d_command(:,3,1))
% hold on
% plot(bagselect_velocity_obstacle.MessageList.Time,v_d_obstacle(:,3,1))
% hold on
% plot(bagselect_velocity_potential.MessageList.Time,v_d_potential(:,3,1))
% hold on
% plot(bagselect_velocity_matching.MessageList.Time,v_d_matching(:,3,1))
% hold on
% plot(bagselect.MessageList.Time(1:sample:bagselect.NumMessages),v_d(:,3,1))
% legend('command','obstacle','potential','matching','real')
% title('Drone real velocity along z axis over time')
% xlabel('time (s)')
% ylabel('V_z (m/s)')
% grid on
