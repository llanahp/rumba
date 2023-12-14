close all
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
load mat/map_modified_pasillo_lab.mat
show(map_modified);


odometryModel = odometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];

rangeFinderModel = likelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0 8];
rangeFinderModel.Map = map_modified;

tftree = rostf;
%Obtener transformada entre los frames del robot y del sensor_laser
waitForTransform(tftree,'/base_link','/laser_frame');
sensorTransform = getTransform(tftree,'/base_link', '/laser_frame');

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

visualizationHelper = ExampleHelperAMCLVisualization(map_modified);





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