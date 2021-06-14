%Declare name of the bag
experimentbag = rosbag('trial1.bag')

%Control input u
ctrl_input = select(experimentbag, "Topic", '/u'); %Variable reading from rosbag
ctrl_input_ts = timeseries(ctrl_input, 'Data'); %Time series creation
ctrl_input_data = ctrl_input_ts.get.Data; %Data gathering

%Output velocity y
velocity = select(experimentbag, "Topic", '/y');
velocity_ts = timeseries(velocity, 'Data');
velocity_data = velocity_ts.get.Data;

%Reference velocity r
reference = select(experimentbag, "Topic", '/r');
reference_ts = timeseries(reference, 'Data');
reference_data = reference_ts.get.Data;

%Tracking error e
error = select(experimentbag, "Topic", '/e');
error_ts = timeseries(error, 'Data');
error_data = error_ts.get.Data;

%Visualize
%Reference vs response
figure
start_time = reference_ts.get.TimeInfo.Start;
t = reference_ts.get.Time - start_time;
plot(t,reference_data)
hold on
plot(t,velocity_data)
%Control input
figure
plot(t,ctrl_input_data)
%Tracking error
figure
plot(t,error_data)