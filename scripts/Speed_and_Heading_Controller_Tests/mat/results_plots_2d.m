%Declare name of the bag
experimentbag = rosbag('2d_trial1.bag')

%Velocity control input u
ctrl_input_speed = select(experimentbag, "Topic", '/u_speed'); %Variable reading from rosbag
ctrl_input_speed_ts = timeseries(ctrl_input_speed, 'Data'); %Time series creation
ctrl_input_speed_data = ctrl_input_speed_ts.get.Data; %Data gathering

%Output velocity y
velocity = select(experimentbag, "Topic", '/y_speed');
velocity_ts = timeseries(velocity, 'Data');
velocity_data = velocity_ts.get.Data;

%Reference velocity r
reference_speed = select(experimentbag, "Topic", '/r_speed');
reference_speed_ts = timeseries(reference_speed, 'Data');
reference_speed_data = reference_speed_ts.get.Data;

%Velocity racking error e
error_speed = select(experimentbag, "Topic", '/e_speed');
error_speed_ts = timeseries(error_speed, 'Data');
error_speed_data = error_speed_ts.get.Data;

%Heading control input u
ctrl_input_heading = select(experimentbag, "Topic", '/u_heading')
ctrl_input_heading_ts = timeseries(ctrl_input_heading, 'Data');
ctrl_input_heading_data = ctrl_input_heading_ts.get.Data;

%Output heading y
heading = select(experimentbag, "Topic", '/y_heading');
heading_ts = timeseries(heading, 'Data');
heading_data = heading_ts.get.Data;

%Reference heading r
reference_heading = select(experimentbag, "Topic", '/r_heading');
reference_heading_ts = timeseries(reference_heading, 'Data');
reference_heading_data = reference_heading_ts.get.Data;

%Heading racking error e
error_heading = select(experimentbag, "Topic", '/e_heading');
error_heading_ts = timeseries(error_heading, 'Data');
error_heading_data = error_heading_ts.get.Data;

%Visualize
%Velocity reference vs response
figure
start_time = reference_speed_ts.get.TimeInfo.Start;
t = reference_speed_ts.get.Time - start_time;
plot(t,reference_speed_data)
hold on
plot(t,velocity_data)
%Velocity control input
figure
plot(t,ctrl_input_speed_data)
%Velocity racking error
figure
plot(t,error_speed_data)
%Heading reference vs response
figure
plot(t,reference_heading_data)
hold on
plot(t,heading_data)
%Heading control input
figure
plot(t,ctrl_input_heading_data)
%Heading racking error
figure
plot(t,error_heading_data)

%XY
xytheta = select(experimentbag, "Topic", 'position');
NEDxy = readMessages(xytheta,'DataFormat','struct');
NEDxPoints = cellfun(@(m) double(m.X),NEDxy);
NEDyPoints = cellfun(@(m) double(m.Y),NEDxy);
figure
plot(NEDyPoints,NEDxPoints)