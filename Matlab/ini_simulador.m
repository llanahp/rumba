
% Declaracion de subscribers
%Odometria
%sub_odom = rossubscriber('/robot0/odom','nav_msgs/Odometry');
sub_odom = rossubscriber('robot0/local_odom','nav_msgs/Odometry');

%Laser

sub_laser = rossubscriber('/robot0/laser_1','sensor_msgs/LaserScan');

%Sonars

sub_sonar0 = rossubscriber('/robot0/sonar_0','sensor_msgs/Range');
sub_sonar1 = rossubscriber('/robot0/sonar_1','sensor_msgs/Range');
sub_sonar2 = rossubscriber('/robot0/sonar_2','sensor_msgs/Range');
sub_sonar3 = rossubscriber('/robot0/sonar_3','sensor_msgs/Range');
sub_sonar4 = rossubscriber('/robot0/sonar_4','sensor_msgs/Range');
sub_sonar5 = rossubscriber('/robot0/sonar_5','sensor_msgs/Range');
sub_sonar6 = rossubscriber('/robot0/sonar_6','sensor_msgs/Range');
sub_sonar7 = rossubscriber('/robot0/sonar_7','sensor_msgs/Range');

%Declaracion de Publishers

%Velocidad
pub_vel = rospublisher('/robot0/cmd_vel','geometry_msgs/Twist');

%Generacion de Mensajes

msg_vel = rosmessage(pub_vel);

%Definimos la periodicidad del bucle

r = rateControl(10); % 10 son 10 Hz -> Leer cada 100 ms


%Nos aseguramos de recibir el mensaje relacinado

% while(strcmp(sub_odom.LatesMessage.ChildFrameId,'robot0')~=1)
%     sub_odom.LatestMessage
% end

disp("Inicialización finalizada correctamente")
