
%Figuras 
close all
fig_laser = figure; title("Laser");
fig_vfh = figure; title("VFH");

%Cargar el mapa
%%%%%%%%%%%%%%%
%load mapa_con_OnlineSLAM_Simulator.mat

%Crear el objeto VFH…y ajustar sus propiedades
vfh = controllerVFH;
vfh.UseLidarScan = true;
vfh.DistanceLimits = [0.05 0.5];
vfh.RobotRadius = 0.1;
vfh.MinTurningRadius = 0.2;
vfh.SafetyDistance = 0.1;
vfh.NumAngularSectors = 180;
vfh.TargetDirectionWeight = 5;
vfh.CurrentDirectionWeight = 2;
vfh.PreviousDirectionWeight = 2;
vfh.HistogramThresholds = [3 10];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Inicializar el localizador AMCL (práctica 1)
AMCL_Localization;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Rellenamos los campos por defecto de la velocidad del robot, para que la lineal
%sea siempre 0.1 m/s
K = 0.1;
targetDir = 0;

%Bucle de control infinito
i = 1;
while(1)
    %Leer y dibujar los datos del láser en la figura ‘fig_laser’
    lee_sensores;
    scans= lidarScan(msg_laser);
    figure(fig_laser);
    plot(msg_laser,'MaximumRange',8);

    %Leer la odometría
    odompose = sub_odom.LatestMessage;
    
    %Obtener la posición pose=[x,y,yaw] a partir de la odometría anterior
    odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
                odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
    odomRotation = quat2eul(odomQuat);
    pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y odomRotation(1)];

    %Ejecutar amcl para obtener la posición estimada estimatedPose y la
    %covarianza estimatedCovariance (mostrar la última por pantalla para
    %facilitar la búsqueda de un umbral)
    [isUpdated, estimatedPose, estimatedCovariance] = amcl(pose, scans);

    %Si la covarianza está por debajo de un umbral, el robot está localizado y
    %finaliza el programa
    if (estimatedCovariance(1,1)<umbralx && estimatedCovariance(2,2)<umbraly && estimatedCovariance(3,3)<umbralyaw)
        disp("Robot Localizado");
        break;
    end

    %Dibujar los resultados del localizador con el visualizationHelper
    if isUpdated
       i = i + 1;
       plotStep(visualizationHelper, amcl, estimatedPose, scans, i);
    end
    %Llamar al objeto VFH para obtener la dirección a seguir por el robot para
    %evitar los obstáculos. Mostrar los resultados del algoritmo (histogramas)
    %en la figura .fig_vfh’.
    steeringDir = vfh(scans,targetDir);
    figure(fig_vfh);
    show(vfh);

    %Rellenar el campo de la velocidad angular del mensaje de velocidad con un
    %valor proporcional a la dirección anterior (K=0.1)
    V_ang = K * steeringDir;
    pub_vel=rospublisher('/robot0/cmd_vel','geometry_msgs/Twist');

    msg_vel=rosmessage(pub_vel);
    msg_vel.Linear.X=0.01;
    msg_vel.Linear.Y=0;
    msg_vel.Linear.Z=0;
    msg_vel.Angular.X=0;
    msg_vel.Angular.Y=0;
    msg_vel.Angular.Z=V_ang;

    %Publicar el mensaje de velocidad
    send(pub_vel,msg_vel);

    %Esperar al siguiente periodo de muestreo
    send(pub_vel,msg_vel);
end