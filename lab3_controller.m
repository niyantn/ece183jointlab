clc; close all; clear all;

data1 = importdata('trajectory6_AR.txt');
leftV1 = data1.data(:,2);
rightV1 = data1.data(:,3);

% Get simulation time step;
TIME_STEP = wb_robot_get_basic_time_step();
% Create instances of motors sensors, nodes;
motor_R = wb_robot_get_device('motor_R');
motor_L = wb_robot_get_device('motor_L'); % Device names have to be char, not string;
compass = wb_robot_get_device('compass');
gyro = wb_robot_get_device('gyro');
lidar_F = wb_robot_get_device('lidar_F');
lidar_R = wb_robot_get_device('lidar_R');
robot = wb_supervisor_node_get_from_def('robot'); % For ground truth access;
rotation = wb_supervisor_node_get_field(robot,'rotation');
gyro2 = wb_robot_get_device('gyro2');
% Enable sensors (instance, sampling period[ms]);
wb_compass_enable(compass, TIME_STEP);
wb_gyro_enable(gyro, TIME_STEP);
wb_distance_sensor_enable(lidar_F, TIME_STEP);
wb_distance_sensor_enable(lidar_R, TIME_STEP);
wb_gyro_enable(gyro2, TIME_STEP);
% Make the motors non-position control mode;
wb_motor_set_position(motor_R, inf);
wb_motor_set_position(motor_L, inf);
% Need to call wb robot step periodically to communicate to the simulator;
i = 1;
compass_dataX = zeros(5001,1);
compass_dataY = zeros(5001,1);
compass_dataZ = zeros(5001,1);
gyro_dataX = zeros(5001,1);
gyro_dataY = zeros(5001,1);
gyro_dataZ = zeros(5001,1);
lidar_F_data = zeros(5001,1);
lidar_R_data = zeros(5001,1);
positionX = zeros(5001,1);
positionY = zeros(5001,1);
positionZ = zeros(5001,1);
angleX = zeros(5001,1);
angleY = zeros(5001,1);
angleZ = zeros(5001,1);
angleangle = zeros(5001,1);
velocityX = zeros(5001,1);
velocityY = zeros(5001,1);
velocityZ = zeros(5001,1);
gyro2_dataX = zeros(5001,1);
gyro2_dataY = zeros(5001,1);
gyro2_dataZ = zeros(5001,1);

while wb_robot_step(TIME_STEP) ~= -1
  % Run a motor by velocity rad/sec;
  wb_motor_set_velocity(motor_R,rightV1(i));
  wb_motor_set_velocity(motor_L,leftV1(i));

  % Read sensor data;
  compass_data = wb_compass_get_values(compass);
  compass_dataX(i) = compass_data(1);
  compass_dataY(i) = compass_data(2);
  compass_dataZ(i) = compass_data(3);
  
  gyro_data = wb_gyro_get_values(gyro);
  gyro_dataX(i) = gyro_data(1);
  gyro_dataY(i) = gyro_data(2);
  gyro_dataZ(i) = gyro_data(3);
  
  lidar_F_data(i) = wb_distance_sensor_get_value(lidar_F);
  lidar_R_data(i) = wb_distance_sensor_get_value(lidar_R);
  
  % Get ground truth data;
  position = wb_supervisor_node_get_position(robot);
  positionX(i) = position(1);
  positionY(i) = position(2);
  positionZ(i) = position(3);
  
  angle = wb_supervisor_field_get_sf_rotation(rotation);
  angleX(i) = angle(1);
  angleY(i) = angle(2);
  angleZ(i) = angle(3);
  angleangle(i) = angle(4);
  
  velocity = wb_supervisor_node_get_velocity(robot);
  velocityX(i) = velocity(1);
  velocityY(i) = velocity(2);
  velocityZ(i) = velocity(3);
  
  gyro2_data = wb_gyro_get_values(gyro2);
  gyro2_dataX(i) = gyro2_data(1);
  gyro2_dataY(i) = gyro2_data(2);
  gyro2_dataZ(i) = gyro2_data(3);
  
  i = i + 1;
  if i == 5002
    wb_robot_step(TIME_STEP) = -1;
  end

end
output_data = table(leftV1, rightV1, compass_dataX, compass_dataY, compass_dataZ,...
              gyro_dataX, gyro_dataY, gyro_dataZ, lidar_F_data, lidar_R_data,...
              positionX, positionY, positionZ, angleX, angleY, angleZ, angleangle,...
              velocityX, velocityY, velocityZ, gyro2_dataX, gyro2_dataY, gyro2_dataZ);
              
writetable(output_data, 'SOutput6AR.txt');       

% Close your controller;
wb_robot_cleanup();                                   