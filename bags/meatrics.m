clear
clc

%declare name of the bag
experimentbag1 = rosbag('mpc/u_0,8_psi_1,0_2021-03-02-23-27-05.bag')
desiredheading1 = select(experimentbag1, "Topic", '/guidance/desired_heading');
desiredheadingts1 = timeseries(desiredheading1, 'Data');
start_time1 = desiredheadingts1.get.TimeInfo.Start;

%declare name of the bag
experimentbag2 = rosbag('pid/u_0,8_psi_1,0_2021-02-20-22-29-49.bag')
desiredheading2 = select(experimentbag2, "Topic", '/guidance/desired_heading');
desiredheadingts2 = timeseries(desiredheading2, 'Data');
start_time2 = desiredheadingts2.get.TimeInfo.Start;



%heading plot
heading1 = select(experimentbag1, "Topic", 'vectornav/ins_2d/NED_pose');
headingts1 = timeseries(heading1, 'Theta');
t = headingts1.get.Time - start_time1;
headingdata1 = headingts1.get.Data;
figure
plot(t,headingdata1)
hold on
%heading plot
heading2 = select(experimentbag2, "Topic", 'vectornav/ins_2d/NED_pose');
headingts2 = timeseries(heading2, 'Theta');
t = headingts2.get.Time - start_time2;
headingdata2 = headingts2.get.Data;
plot(t,headingdata2)
hold on
%desired heading plot
desiredheading = select(experimentbag1, "Topic", '/guidance/desired_heading');
desiredheadingts = timeseries(desiredheading1, 'Data');
t = desiredheadingts.get.Time - start_time1;
desiredheadingdata = desiredheadingts.get.Data;
plot(t,desiredheadingdata)
hold off
legend('$MPC$', '$PID$','$\psi_{d}$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$\psi$ [rad]', 'Interpreter', 'latex')
title('Heading MPC')


%speed plot
speed = select(experimentbag1, "Topic", '/vectornav/ins_2d/local_vel');
speedts = timeseries(speed, 'X');
t = speedts.get.Time - start_time1;
speeddata = speedts.get.Data;
figure
plot(t,speeddata)
hold on
%speed plot
speed = select(experimentbag2, "Topic", '/vectornav/ins_2d/local_vel');
speedts = timeseries(speed, 'X');
t = speedts.get.Time - start_time2;
speeddata = speedts.get.Data;
plot(t,speeddata)
hold on
%desired speed plot
desiredspeed = select(experimentbag1, "Topic", '/guidance/desired_speed');
desiredspeedts = timeseries(desiredspeed, 'Data');
t = desiredspeedts.get.Time - start_time1;
desiredspeeddata = desiredspeedts.get.Data;
plot(t,desiredspeeddata)
hold off
legend('PID','MPC','$u_{d}$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$u$ [m/s]', 'Interpreter', 'latex') 
title('Speed')


%Cross Track Error
cross_error_topic = select(experimentbag1, "Topic", '/usv_control/controller/heading_error');
cross_error = timeseries(cross_error_topic, 'Data');
t = cross_error.get.Time - start_time1;
cross_error_data = cross_error.get.Data;
figure
plot(t,cross_error_data)
legend('$Y_{e}$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$Y_{e}$ [m]', 'Interpreter', 'latex')
cross_error_data = cross_error_data(400:3000);
mae1 = mae(cross_error_data);
mse1 = mse(cross_error_data);
disp("cross error MPC");
fprintf("mae %f \n",mae1);
fprintf("mse %f \n",mse1);

hold on 
%Cross Track Error
cross_error_topic = select(experimentbag2, "Topic", '/usv_control/controller/heading_error');
cross_error = timeseries(cross_error_topic, 'Data');
t = cross_error.get.Time - start_time2;
cross_error_data = cross_error.get.Data;
plot(t,cross_error_data)
legend('MPC', 'PID', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$u$ [m]', 'Interpreter', 'latex') 
title('Cross Track Error')
cross_error_data = cross_error_data(400:3000);
mae1 = mae(cross_error_data);
mse1 = mse(cross_error_data);
disp("cross error PID");
fprintf("mae %f \n",mae1);
fprintf("mse %f \n",mse1);


%Speed Error
cross_error_topic = select(experimentbag1, "Topic", '/usv_control/controller/speed_error');
cross_error = timeseries(cross_error_topic, 'Data');
t = cross_error.get.Time - start_time1;
cross_error_data1 = cross_error.get.Data;
figure
plot(t,cross_error_data1)
legend('$Y_{e}$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$Y_{e}$ [m]', 'Interpreter', 'latex')
cross_error_data1 = cross_error_data1(400:3000);
mae1 = mae(cross_error_data1);
mse1 = mse(cross_error_data1);
disp("speed error MPC");
fprintf("mae %f \n",mae1);
fprintf("mse %f \n",mse1);

hold on 
%Speed Error
cross_error_topic = select(experimentbag2, "Topic", '/usv_control/controller/speed_error');
cross_error = timeseries(cross_error_topic, 'Data');
t = cross_error.get.Time - start_time2;
cross_error_data2 = cross_error.get.Data;
plot(t,cross_error_data2)
legend('MPC', 'PID', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$u$ [m/s]', 'Interpreter', 'latex') 
title('Speed Error')
cross_error_data2 = cross_error_data2(400:3000);
mae1 = mae(cross_error_data2);
mse1 = mse(cross_error_data2);
disp("speed error PID");
fprintf("mae %f \n",mae1);
fprintf("mse %f \n",mse1);

%Control Input X
cross_error_topic = select(experimentbag1, "Topic", '/usv_control/controller/control_input ');
cross_error = timeseries(cross_error_topic, 'X');
t = cross_error.get.Time - start_time1;
cross_error_data1 = cross_error.get.Data;
figure
plot(t,cross_error_data1)
legend('$Y_{e}$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$Y_{e}$ [m]', 'Interpreter', 'latex')
norm1 = norm(cross_error_data1);
disp("control input x MPC");
fprintf("norm %f \n",norm1);
hold on 
%Control Input X
cross_error_topic = select(experimentbag2, "Topic", '/usv_control/controller/control_input ');
cross_error = timeseries(cross_error_topic, 'X');
t = cross_error.get.Time - start_time2;
cross_error_data2 = cross_error.get.Data;
plot(t,cross_error_data2)
legend('MPC', 'PID', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$u$ [m/s]', 'Interpreter', 'latex') 
title('Control Input X')
norm1 = norm(cross_error_data2);
disp("Control input x PID");
fprintf("norm %f \n",norm1);



%Control Input Z
cross_error_topic = select(experimentbag1, "Topic", '/usv_control/controller/control_input ');
cross_error = timeseries(cross_error_topic, 'Theta');
t = cross_error.get.Time - start_time1;
cross_error_data1 = cross_error.get.Data;
figure
plot(t,cross_error_data1)
legend('$Y_{e}$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$Y_{e}$ [m]', 'Interpreter', 'latex')
norm1 = norm(cross_error_data1);
disp("Control input z MPC");
fprintf("norm %f \n",norm1);
hold on 
%Control Input Z
cross_error_topic = select(experimentbag2, "Topic", '/usv_control/controller/control_input ');
cross_error = timeseries(cross_error_topic, 'Theta');
t = cross_error.get.Time - start_time2;
cross_error_data2 = cross_error.get.Data;
plot(t,cross_error_data2)
legend('MPC', 'PID', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$u$ [m/s]', 'Interpreter', 'latex') 
title('Control Input Z PID')
norm1 = norm(cross_error_data2);
disp("Control input z PID");
fprintf("norm %f \n",norm1);
