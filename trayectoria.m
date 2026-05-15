%Limpieza de pantalla
clear all
close all
clc

%1 TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tf=31.5;             % Tiempo de simulación en segundos (s)
ts=0.005;            % Tiempo de muestreo en segundos (s)
t=0:ts:tf;         % Vector de tiempo
N= length(t);      % Muestras

%2 CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Damos valores a nuestro punto inicial de posición y orientación (t = 0)
x1(1) = 16 * (sin(0)^3);  
y1(1) = 13 * cos(0) - 5 * cos(2*0) - 2 * cos(3*0) - cos(4*0);  

% Orientar el robot usando las derivadas evaluadas en t = 0
v_x0 = 48 * (sin(0)^2) * cos(0);
v_y0 = -13 * sin(0) + 10 * sin(2*0) + 6 * sin(3*0) + 4 * sin(4*0);
phi(1) = atan2(v_y0, v_x0); % Orientación inicial del robot

%x1(1)=0;  %Posición inicial eje x
%y1(1)=0;  %Posición inicial eje y
%phi(1)=0; %Orientación inicial del robot 
% Distancia al punto desplazadoo
a = 0.5; % Distancia en metros delante del centro del robot

he_int = [0; 0];        % Inicialización de la integral del error
he_prev = [0; 0];       % Inicialización del error previo

% Ahora hx y hy desplazados hacia adelante
hx(1) = x1(1) + a*cos(phi(1));       
hy(1) = y1(1) + a*sin(phi(1));
%3 TRAYECTORIA DESEADA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%PID
Kp = [15 0; 0 15];    % Ganancia Proporcional 
Ki = [0.1 0; 0 0.1];  % Ganancia Integral
Kd = [0.5 0; 0 0.5]; % Ganancia Derivativa
%Ecuaciones paramétricas de la trayectoria deseada

%
theta4 = 0 : pi/3 : 2*pi;  
r4 = ones(1, length(theta4));
x4 = r4 .* cos(theta4);
y4 = r4 .* sin(theta4);
t_nodos = linspace(0, tf, length(theta4));

% Interpolación lineal
% Ecuaciones paramétricas de la flor de 9 pétalos
hxd = 16 * (sin(t).^3);
hyd = 13 * cos(t) - 5 * cos(2*t) - 2 * cos(3*t) - cos(4*t);

% Velocidades de la trayectoria deseada (Derivadas exactas)
hxdp = 48 * (sin(t).^2) .* cos(t);
hydp = -13 * sin(t) + 10 * sin(2*t) + 6 * sin(3*t) + 4 * sin(4*t);

%4 CONTROL, BUCLE DE SIMULACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for k=1:N 
    hxe(k)=hxd(k)-hx(k);
    hye(k)=hyd(k)-hy(k);
    
    %Matriz de error
    he= [hxe(k);hye(k)];
    %valores para calcular el PID
    he_int = he_int + (he * ts);         % Cálculo de término Integral
    he_der = (he - he_prev) / ts;        % Cálculo de término Derivativo
    he_prev = he;                        % Se guarda el error para la siguiente iteración
    
    %Magnitud del error de posición
    Error(k)= sqrt(hxe(k)^2 +hye(k)^2);

    %b)Matriz Jacobiana
    J = [cos(phi(k))   -a*sin(phi(k));
         sin(phi(k))    a*cos(phi(k))];

    %c)Matriz de Ganancias
    %c)Matriz de Ganancias
    %K=[20 0;...
    %   0 20];
    
    %d)Velocidades deseadas
    hdp=[hxdp(k);hydp(k)];

    %e)Ley de Control:Agregamos las velocidades deseadas + PID
    qpRef= pinv(J)*(hdp + Kp*he + Ki*he_int + Kd*he_der);

    v(k)= qpRef(1);   %Velocidad lineal de entrada al robot 
    w(k)= qpRef(2);   %Velocidad angular de entrada al robot 


%5 APLICACIÓN DE CONTROL AL ROBOT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %Aplico la integral a la velocidad angular para obtener el angulo "phi" de la orientación
    phi(k+1)=phi(k)+w(k)*ts; % Integral numérica (método de Euler)
           
   %%%%%%%%%%%%%%%%%%%%% MODELO CINEMATICO %%%%%%%%%%%%%%%%%%%%%%%%%
    
    xp1=v(k)*cos(phi(k)); 
    yp1=v(k)*sin(phi(k));
 
    %Aplico la integral a la velocidad lineal para obtener las cordenadas
    %"x1" y "y1" de la posición
    x1(k+1)=x1(k)+ ts*xp1; % Integral numérica (método de Euler)
    y1(k+1)=y1(k)+ ts*yp1; % Integral numérica (método de Euler)
    
    % --Posicion del punto desplazado ---
    hx(k+1) = x1(k+1) + a*cos(phi(k+1)); 
    hy(k+1) = y1(k+1) + a*sin(phi(k+1));
     

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULACION VIRTUAL 3D %%%%%%%%%%%%%%%%%%%%%%%%%%%%

% a) Configuracion de escena

scene=figure;  % Crear figura (Escena)
set(scene,'Color','white'); % Color del fondo de la escena
set(gca,'FontWeight','bold') ;% Negrilla en los ejes y etiquetas
sizeScreen=get(0,'ScreenSize'); % Retorna el tamaño de la pantalla del computador
set(scene,'position',sizeScreen); % Configurar tamaño de la figura
camlight('headlight'); % Luz para la escena
axis equal; % Establece la relación de aspecto para que las unidades de datos sean las mismas en todas las direcciones.
grid on; % Mostrar líneas de cuadrícula en los ejes
box on; % Mostrar contorno de ejes
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)'); % Etiqueta de los eje

view([-0.1 90]); % Orientacion de la figura
axis([-20 20 -20 20 0 1]);
%axis([-10 10 -10 10 0 1]); % Ingresar limites minimos y maximos en los ejes x y z [minX maxX minY maxY minZ maxZ]

% b) Graficar robots en la posicion inicial
scale = 2;
MobileRobot_5;
H1=MobilePlot_4(x1(1),y1(1),phi(1),scale);hold on;

% c) Graficar Trayectorias
H2=plot3(hx(1),hy(1),0,'r','lineWidth',2);
H3=plot3(hxd,hyd,zeros(1,N),'g','lineWidth',2); %Grafico circulo en posición deseada
%H4=plot3(hx(1),hy(1),0,'go','lineWidth',2);%Grafico circulo en posición inicial
% d) Bucle de simulacion de movimiento del robot

step= 40; % pasos para simulacion

for k=1:step:N

    delete(H1);    
    delete(H2);
    
    H1=MobilePlot_4(x1(k),y1(k),phi(k),scale);
    H2=plot3(hx(1:k),hy(1:k),zeros(1,k),'r','lineWidth',2);
    
    pause(ts);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Graficas %%%%%%%%%%%%%%%%%%%%%%%%%%%%
graph=figure;  % Crear figura (Escena)
set(graph,'position',sizeScreen); % Congigurar tamaño de la figura
subplot(311)
plot(t,v,'b','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('m/s'),legend('Velocidad Lineal (v)');
subplot(312)
plot(t,w,'g','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('[rad/s]'),legend('Velocidad Angular (w)');
subplot(313)
plot(t,Error,'r','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('[metros]'),legend('Error de posición (m)');

