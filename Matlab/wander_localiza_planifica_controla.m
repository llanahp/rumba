close all

x = 15;
y = 10;
endLocation = [x y];

fig_laser=figure; title('LASER')
set(fig_laser,'Position',[50 50 800 400])
fig_vfh=figure; title('VFH')


%Crear el objeto VFH…
%%%%%%%%%%%%%%%%%%%%%%
VFH=controllerVFH;
%y ajustamos sus propiedades
%%%%%%%%%%%%%%%%%%%%%%%%%%
VFH.NumAngularSectors=180;
VFH.DistanceLimits=[0.05, 2];
VFH.RobotRadius=0.1;
VFH.SafetyDistance=0.1;
VFH.MinTurningRadius=0.1;
VFH.TargetDirectionWeight=5;
VFH.CurrentDirectionWeight=2;
VFH.PreviousDirectionWeight=2;
VFH.HistogramThresholds=[3, 10];
VFH.UseLidarScan=true; %para permitir utilizar la notación del scanç


%fichero AMCL_Localization.m
load mat/mi_mapa.mat
show(map);


odometryModel = odometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];

rangeFinderModel = likelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0 8];
rangeFinderModel.Map = map;

tftree = rostf;
%Obtener transformada entre los frames del robot y del sensor_laser
waitForTransform(tftree,'/robot0','robot0_laser_1');
sensorTransform = getTransform(tftree,'/robot0', 'robot0_laser_1');

% Get the euler rotation angles.
laserQuat = [sensorTransform.Transform.Rotation.W sensorTransform.Transform.Rotation.X ...
    sensorTransform.Transform.Rotation.Y sensorTransform.Transform.Rotation.Z];
laserRotation = quat2eul(laserQuat, 'ZYX');

% Setup the |SensorPose|, which includes the translation along base_link's
% +X, +Y direction in meters and rotation angle along base_link's +Z axis
% in radians.
rangeFinderModel.SensorPose = ...
    [sensorTransform.Transform.Translation.X sensorTransform.Transform.Translation.Y laserRotation(1)];

amcl = monteCarloLocalization;
amcl.UseLidarScan = true;
%% 
% Assign the |MotionModel| and |SensorModel| properties in the |amcl| object.

amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;

amcl.UpdateThresholds = [0.2,0.2,0.2];
amcl.ResamplingInterval = 1;


amcl.ParticleLimits =  [500 50000];           % Minimum and maximum number of particles
amcl.GlobalLocalization = false;      % global = true      local=false
amcl.InitialPose = [0 0 0];              % Initial pose of vehicle   
amcl.InitialCovariance = eye(3)*0.5; % Covariance of initial pose

visualizationHelper = ExampleHelperAMCLVisualization(map);

%TODO Crear el objeto PurePursuit y ajustar sus propiedades
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
controller=controllerPurePursuit;
controller.LookaheadDistance = 0.1;
controller.DesiredLinearVelocity=3;
controller.MaxAngularVelocity =0.5;



%Rellenamos los campos por defecto de la velocidad del robot, para que la lineal
%sea siempre 0.1 m/s
 msg_vel.Linear.X=0.1;
 msg_vel.Linear.Y=0;
 msg_vel.Linear.Z=0;
 msg_vel.Angular.X=0;
 msg_vel.Angular.Y=0;

while(1)
    
    scan = receive(sub_laser);
    scans = lidarScan(scan);

    figure(fig_laser);
    plot(scans);


    odompose = sub_odom.LatestMessage;
    odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
    odomRotation = quat2eul(odomQuat);
    pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y odomRotation(1)];

    [isUpdated,estimatedPose, estimatedCovariance] = amcl(pose, scans);

    umbralx = 0.01;
    umbraly = 0.01;
    umbralyaw = 0.01;

    if (estimatedCovariance(1,1)<umbralx && estimatedCovariance(2,2)<umbraly && estimatedCovariance(3,3)<umbralyaw)
        disp(estimatedCovariance)
        disp('Robot Localizado');
        break;
    end

    if isUpdated
        i = i + 1
        plotStep(visualizationHelper, amcl, estimatedPose, scans, i)
    end


    %Llamar al objeto VFH para obtener la dirección a seguir por el robot para
    %evitar los obstáculos. Mostrar los resultados del algoritmo (histogramas)
    %en la figura ‘fig_vfh’
    targetDir = 0;
    steeringDir = VFH(scans,targetDir);

    figure(fig_vfh);
    show(VFH);
    %Rellenar el campo de la velocidad angular del mensaje de velocidad con un
    %valor proporcional a la dirección anterior (K=0.1) 
    K=0.5;
    V_ang = K* steeringDir;

    %Publicar el mensaje de velocidad
    msg_vel.Angular.Z=V_ang;
    send(pub_vel,msg_vel);

    %Esperar al siguiente periodo de muestreo
    waitfor(r);

end

%Paramos el robot, para que no avance mientras planificamos
msg_vel.Linear.X=0;
msg_vel.Linear.Y=0;
msg_vel.Linear.Z=0;
msg_vel.Angular.X=0;
msg_vel.Angular.Y=0;
msg_vel.Angular.Z=0;
send(pub_vel,msg_vel);


%Hacemos una copia del mapa, para “inflarlo” antes de planificar
cpMap = copy(map);
inflate(cpMap,0.25);

%Crear el objeto PRM y ajustar sus parámetros
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
planner = mobileRobotPRM;
planner.Map = cpMap
planner.NumNodes = 50; 
planner.ConnectionDistance = 2; 

%Obtener la ruta hacia el destino desde la posición actual del robot y mostrarla
%en una figura
ruta = findpath(planner,startLocation,endLocation);
figure; show(plan_nodos);

% Indicamos al controlador la lista de waypoints a recorrer (ruta)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
controller.Waypoints = ruta;

while(1)

    % Leer el láser y la odometría
    scan = receive(sub_laser);
    scans = lidarScan(scan);

    %Obtener la posición pose=[x,y,yaw] a partir de la odometría anterior
    odompose = sub_odom.LatestMessage;
    odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
    odomRotation = quat2eul(odomQuat);
    pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y odomRotation(1)];

    % Ejecutar amcl para obtener la posición estimada estimatedPose
    [isUpdated,estimatedPose, estimatedCovariance] = amcl(pose, scans);

    %Dibujar los resultados del localizador con el visualizationHelper
    if isUpdated
        i = i + 1
        plotStep(visualizationHelper, amcl, estimatedPose, scans, i)
    end

    %TODO Ejecutar el controlador PurePursuit para obtener las velocidades lineal y angular
    [lin_vel,ang_vel]=CONTROLLER(estimatedPose);


    %Rellenar los campos del mensaje de velocidad
    msg_vel.Linear.X=lin_vel;
    msg_vel.Angular.Z=ang_vel;

    %Publicar el mensaje de velocidad
    send(pub_vel,msg_vel);

    %Comprobar si hemos llegado al destino, calculando la distancia euclidea
    % y estableciendo un umbral

    Umbral_X = 0.01;
    Umbral_Y = 0.01;
    if (estimatedPose.x<Umbral_X && estimatedPose.y<Umbral_Y)
        break;
    end

    %Esperar al siguiente periodo de muestreo
    waitfor(r);
end