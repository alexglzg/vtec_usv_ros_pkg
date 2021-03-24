%declare name of the bag
experimentbag = rosbag('pid_zz_2021-03-24-09-10-25.bag')
desiredheading = select(experimentbag, "Topic", '/guidance/ye');
desiredheadingts = timeseries(desiredheading, 'Data');
start_time = desiredheadingts.get.TimeInfo.Start;

%crosstrack error plot
cross_error_topic = select(experimentbag, "Topic", '/guidance/ye');
cross_error = timeseries(cross_error_topic, 'Data');
t = cross_error.get.Time - start_time;
cross_error_data = cross_error.get.Data;
figure
plot(t,cross_error_data)
legend('$Y_{e}$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$Y_{e}$ [m]', 'Interpreter', 'latex')
%cross_error_data = cross_error_data(400:3000);
%mae1 = mae(cross_error_data);
%mse1 = mse(cross_error_data);
%disp("cross error MPC");
%fprintf("mae %f \n",mae1);
%fprintf("mse %f \n",mse1);

%heading plot
%heading = select(experimentbag, "Topic", 'vectornav/ins_2d/NED_pose');
%headingts = timeseries(heading, 'Theta');
%t = headingts.get.Time - start_time;
%headingdata = headingts.get.Data;
%figure
%plot(t,headingdata)
%hold on

%desired heading plot
%desiredheading = select(experimentbag, "Topic", '/guidance/desired_heading');
%desiredheadingts = timeseries(desiredheading, 'Data');
%t = desiredheadingts.get.Time - start_time;
%desiredheadingdata = desiredheadingts.get.Data;
%plot(t,desiredheadingdata)
%hold off
%legend('$\psi$','$\psi_{d}$', 'Interpreter', 'latex')
%xlabel('Time [s]', 'Interpreter', 'latex') 
%ylabel('$\psi$ [rad]', 'Interpreter', 'latex')
%title('Heading MPC')

%right thruster plot
right = select(experimentbag, "Topic", '/usv_control/controller/right_thruster');
rightts = timeseries(right, 'Data');
t = rightts.get.Time - start_time;
rightdata = rightts.get.Data;
figure
plot(t,rightdata)
hold on
%left thruster plot
left = select(experimentbag, "Topic", '/usv_control/controller/left_thruster');
leftts = timeseries(left, 'Data');
t = leftts.get.Time - start_time;
leftdata = leftts.get.Data;
plot(t,leftdata)
hold off
legend('$T_{stbd}$','$T_{port}$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('Thrust [N]', 'Interpreter', 'latex') 
title('Thruster MPC')
%speed plot
speed = select(experimentbag, "Topic", '/vectornav/ins_2d/local_vel');
speedts = timeseries(speed, 'X');
t = speedts.get.Time - start_time;
speeddata = speedts.get.Data;
figure
plot(t,speeddata)
hold on
%desired speed plot
desiredspeed = select(experimentbag, "Topic", '/guidance/desired_speed');
desiredspeedts = timeseries(desiredspeed, 'Data');
t = desiredspeedts.get.Time - start_time;
desiredspeeddata = desiredspeedts.get.Data;
plot(t,desiredspeeddata)
hold off
legend('$u$','$u_{d}$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$u$ [m/s]', 'Interpreter', 'latex') 
title('Speed MPC')

%Publish desired Path
desired_path = select(experimentbag, 'Topic', '/guidance/target');
msgStructs = readMessages(desired_path,'DataFormat','struct');
msgStructs{1}
figure
xPoints = cellfun(@(m) double(m.X),msgStructs);
yPoints = cellfun(@(m) double(m.Y),msgStructs);
plot(yPoints,xPoints)
xlabel('Y(m)', 'Interpreter', 'latex') 
ylabel('X(m)', 'Interpreter', 'latex')
hold on 

%publish actual path
position = select(experimentbag, 'Topic', '/vectornav/ins_2d/ins_pose');
msgStructs = readMessages(position,'DataFormat','struct');
msgStructs{1}
xPoints = cellfun(@(m) double(m.X),msgStructs);
yPoints = cellfun(@(m) double(m.Y),msgStructs);
plot(yPoints,xPoints)
legend('Desired Path', 'Actual Path', 'Interpreter', 'latex')