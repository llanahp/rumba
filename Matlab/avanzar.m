distancia=1;

%Rellenamos los campos del mensaje para que el robot avance 0.1 metros
%hacia delante

msg_vel.Linear.X= 0.1;
msg_vel.Linear.Y=0;
msg_vel.Linear.Z=0;
msg_vel.Angular.X=0;
msg_vel.Angular.Y=0;
msg_vel.Angular.Z=0;

%Activar motores enviando enable_motor = 1 - 
%msg_enable_motor.Data = 1;
% send(pub_enable,msg_enable_motor);

%Leemos primera posicion
initpos= sub_odom.LatestMessage.Pose.Pose.Position;

%Bucle de control infinito
while(1)
    %Obtenemos posicion actual
    pos= sub_odom.LatestMessage.Pose.Pose.Position;
    disp(sprintf('\nPosicion actual: X=%f , Y=%f',pos.X,pos.Y));
    
    %Calculamos la distancia euclidea que se ha desplazado
    dist = sqrt((initpos.X-pos.X)^2 + (initpos.Y-pos.Y)^2);
    disp((sprintf('\tDistancia avanzada %f',dist)));


    %Si hemos avanzado dos metros detenemos e robot y salimos
    if(dist>distancia)
        msg_vel.Linear.X=0;
        send(pub_vel,msg_vel);
        break;
    else
        send(pub_vel,msg_vel);
    end
    lee_sensores;
    %vfh_dir;
    waitfor(r);
end


%Desactivar algo que no he leido
