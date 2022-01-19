 clear all
 close all
 clc
 
 
%%Simulação do Modelo Dinamico de 4 Barras
 %Inicialização das variaveis
 theta_servo=[];
 theta_tilt=[];
 %Definição das dimensoes de cada barra do mecanismo
 %Tilt motor 1 M1 valores em metros 
 
 s1 = 1.6e-2;
 w1 = 3.8e-2;
 u1 = 1.8e-2;
 v1 = 1.25e-2;
 t1 = sqrt(u1^2+v1^2);
 y1 = 1.95e-2;
 st1 = sqrt(w1^2+t1^2);
 x1 = sqrt(st1^2-s1^2);
 
 param=[s1 w1 u1 v1 t1 y1 x1]*1000;
 %Tilt motor 2 M2
 
 s2 = 1.6e-2;
 w2 = 3.75e-2;
 u2 = 1.8e-2;
 v2 = 1.25e-2;
 t2 = sqrt(u2^2+v2^2);
 y2 = 1.95e-2;
 st2 = sqrt(w2^2+t2^2);
 x2 = sqrt(st2^2-s2^2);
 
 param2=[s2 w2 u2 v2 t2 y2 x2]*1000;
 
 % Definição dos Angulos Limites do Servo Motor [theta_min,theta_max] (Atuador do mecanismo Tilt)
 theta_max= pi; % em radianos pi[rad] = 180 [degrees]
 theta_min= 0;  % em radianos
 max_I=1;      %Numero de repetiçoes da simulação

 for I=0 : 1 : max_I;
     if (rem(I,2)==0)
         for theta = theta_min:0.005: theta_max;
              th_tilt = plot4bar(param, theta,I,theta_max,max_I); %%Plota a
              % simulacao do mecanismo de quatro barras
%               th_tilt = sim4bar(param,theta);
%               theta_servo = [theta_servo theta];
%               theta_tilt=[theta_tilt th_tilt];
              %prt_map(theta_servo,theta_tilt); %%Plot o ponto
              %correspondente da simulacao no grafico abaixo
         end
%      else
%          for theta = theta_max :-0.005:theta_min ;
%              %th_tilt=plot4bar(theta,I,theta_max,max_I);
%              th_tilt = sim4bar(param,theta);
%              theta_servo = [theta_servo theta];
%              theta_tilt=[theta_tilt th_tilt];  
%              %prt_map(theta_servo,theta_tilt);
%          end
     end
 end

 
  
 %%Figura da relação entre o Angulo ThetaServo e o Angulo Tilt
 %Resultado Theta_tilt = -0.8649*Theta_servo-131.7590;
%  figure
%  plot(rad2deg(theta_servo),rad2deg(theta_tilt));
%  xlabel('\theta_{servo} [\circ]')
%  ylabel('\theta_{tilt} [\circ]')
%  hold on 
%  coefs1 = polyfit( rad2deg(theta_servo(197:630)), rad2deg(theta_tilt(197:630)),1 )
 %coefs1 = polyfit( rad2deg(theta_servo), rad2deg(theta_tilt),1 )
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
%  
%  
%  %%Figura da relação entre o Angulo ThetaServo e o Angulo Tilt
%  %Resultado Theta_tilt = -0.8649*Theta_servo-131.7590;
%  figure
% %  subplot(2,1,1)
% %  plot(theta_servo,theta_tilt);
% %  xlabel('\theta_{servo} [radians]')
% %  ylabel('\theta_{tilt} [radians]')
% %  subplot(2,1,2)
%  plot(rad2deg(theta_servo),rad2deg(theta_tilt));
%  xlabel('\theta_{servo} [\circ]')
%  ylabel('\theta_{tilt} [\circ]')
%  hold on 
%  coefs1 = polyfit( rad2deg(theta_servo(197:630)), rad2deg(theta_tilt(197:630)),1 )
%  %coefs1 = polyfit( rad2deg(theta_servo), rad2deg(theta_tilt),1 )
%  
%  plot(rad2deg(theta_servo),polyval(coefs1,rad2deg(theta_servo)),'g');
%  %H=gca
%  %xticks(H,[95:5:155])
%  %yticks(H,[-10:5:50])
%  hold off 
%  
%  %Experimento de Mapeamento do Tilt_angle com Adc_read
%  %Resultado Theta_tilt = 0.0705*ADC_read-24.6582;
%  figure
%  AngleTilt = [-30 -30 -10 0 0 0 0 10 10 30 30 45 45 45 60 60 70 80 80 90 90];
%  Adc_read  = [20 59 127 287 359 275 281 526 434 831 806 1011 1062 1012 1112 1179 1359 1356 1490 1709 1614];
% 
%  plot(Adc_read,AngleTilt,'ro');
%  title({'{\bf\fontsize{14} Mapeamento do Theta Tilt}'; 'ThetaTilt-ADCread mapping'},'FontWeight','Normal');
%  xlabel('ADC_{read} [int]')
%  ylabel('\theta_{tilt} [\circ]')
%  hold on
%  coefs = polyfit( Adc_read, AngleTilt,1 );
%  plot( Adc_read,polyval(coefs, Adc_read),'g');
%  hold off
% 
% %Experimento de Mapeamento do Theta_Servo com PWM
% %Resultado PWM = 10351.2 * Theta_servo + 667258.6;
% 
% figure
% theta_serv = [0 10 20 30 45 60 80 90 100 120 135 150 160 170 180];
% pwm_servo = [0.693 0.785 0.865 0.98 1.1 1.3 1.47 1.57 1.69 1.92 2.08 2.28 2.33 2.43 2.49]* 10^6;
% 
% x = theta_serv;
% y = pwm_servo;
% p1 = plot(x,y,'ro');
% 
% title({'{\bf\fontsize{14} Mapeamento do Servo HIMG65+}'; 'PWM-Angle mapping'},'FontWeight','Normal');
% ylabel('PWM [ns]');
% xlabel('\theta_{servo} [\circ]');
% 
% hold on 
% 
% [P,S] = polyfit( x , y , 1 );
% p2 = plot(x,polyval(P,x),'g');
% 
% hold off;
% 
% 
%  
%  %%Leitura dos dados Experimentais
%  filename='teste6.txt';
%  
%  formatSpec = '%f %d %f %f';
%  sizeA= [4 Inf];
%  fileID = fopen(filename,'r');
%  A = fscanf(fileID,formatSpec,sizeA);
%  fclose(fileID);
%  A=A';
%  t=1:size(A,1); %Numero de amostras
% 
%  %1 Figura do Experimento 
%  figure
%  
%  x=t;
%  y1=A(:,2);     %Adc_read
%  y2=A(:,1);     %Tilt_Angle(Adc_read)
%  y3=A(:,3);     %Theta_servo
%  
%  ylabels{1}='Adc_{read}';
%  ylabels{2}='Tilt_Angle(Adc_read) [\circ]';
%  ylabels{3}='\theta_{servo} [\circ]';
%  xlabel('N amostra');
%  [ax,hlines] = plotyyy(x,y1,x,y2,x,y3,ylabels);
%  title({'{\bf\fontsize{14} Mapeamento Tilt Rotor}'; 'ADC-TiltAngle-ServoAngle mapping'},'FontWeight','Normal');
%  %Definindo Limites do Eixo Theta Servo
%  yLimits = get(ax(3),'YLim');
%  yTicks = get(ax(3),'YTick');
%  
%  %Definindo Limites do Eixo Theta Servo
%  ylim(ax(2),[-10 95])
%  yticks(ax(2),[-10:10:95])
%  
% 
%  %Definindo Limites do Eixo ADC_read 
%  ylim(ax(1),[200 1600])
%  yticks(ax(1),[200:200:1600])
%  
%  legend('hide') 
% 
% 
%  %%2 Figura do Experimento 
%  figure
%  
%  x=t;
%  y1=A(:,3);     %Theta_servo
%  y2=A(:,1);     %Tilt_Angle(Adc_read)
%  y3=A(:,4);     %Tilt_Angle(Theta_servo)
%  
%  ylabels{1}='\theta_{servo} [\circ]';
%  ylabels{2}='Tilt_Angle(Adc_read) [\circ]';
%  ylabels{3}='Tilt_Angle(\theta_{servo}) [\circ]';
%  
%  [ax,hlines] = plotyyy(x,y1,x,y2,x,y3,ylabels);
%  title({'{\bf\fontsize{14} Mapeamento Tilt Rotor}'; 'ThetaServo-TiltAngle mapping'},'FontWeight','Normal');
%  %Definindo Limites do Eixo Tilt_Angle(Adc_read)
%  ylim(ax(2),[-10 100])
%  yticks(ax(2),[-10:10:100])
%  
%  %Definindo Limites do Eixo Tilt_Angle(Theta_servo)
%  ylim(ax(3),[-10 100])
%  yticks(ax(3),[-10:10:100])
%  
% 
%  %Definindo Limites do Eixo Theta_servo
%  ylim(ax(1),[40 160])
%  yticks(ax(1),[40:10:160])
%  
%  legend('hide') 
%  