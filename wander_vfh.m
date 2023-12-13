%Crear figuras distintas para el láser y el visualizador del VFH
% NOTA: para dibujar sobre una figura concreta, antes de llamar a la
% correspondiente función de dibujo debe convertirse en la figura activa
% utilizando figure(fig_laser) o figure(fig_vfh) respectivamente.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
fig_laser = figure; title("Laser");
fig_vfh = figure; title("VFH");

%Crear el objeto VFH…
%%%%%%%%%%%%%%%%%%%%%%
vfh = controllerVFH;
%y ajustamos sus propiedades
%%%%%%%%%%%%%%%%%%%%%%%%%%

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

%para permitir utilizar la notación del scan
%Rellenamos los campos por defecto de la velocidad del robot, para que la lineal
%sea siempre 0.1 m/s
K = 0.1;
targetDir = 0;

%Bucle de control infinito
while(1)
    %Leer y dibujar los datos del láser en la figura ‘fig_laser’
    lee_sensores;
    scans= lidarScan(msg_laser);
    figure(fig_laser);
    plot(msg_laser,'MaximumRange',8);

    %Llamar al objeto VFH para obtener la dirección a seguir por el robot para
    %evitar los obstáculos. Mostrar los resultados del algoritmo (histogramas)
    %en la figura fig_vfh
    steeringDir = vfh(scans,targetDir);
    steeringDir
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
    
    V_ang

    %Publicar el mensaje de velocidad
    send(pub_vel,msg_vel);

    %Esperar al siguiente periodo de muestreo
    waitfor(r);
end