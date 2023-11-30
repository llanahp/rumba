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



%Rellenamos los campos por defecto de la velocidad del robot, para que la lineal
%sea siempre 0.1 m/s
%Bucle de control infinito
while(1)

   
    %lee_sensores;
 
    
    %Leer y dibujar los datos del láser en la figura ‘fig_laser’
    scan = receive(sub_laser);
    scans = lidarScan(scan);

    figure(fig_laser);
    plot(scans);


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
    msg_vel.Linear.X=0.1;
    msg_vel.Linear.Y=0;
    msg_vel.Linear.Z=0;
    msg_vel.Angular.X=0;
    msg_vel.Angular.Y=0;
    msg_vel.Angular.Z=V_ang;

    send(pub_vel,msg_vel);

    %Esperar al siguiente periodo de muestreo
    waitfor(r);

end