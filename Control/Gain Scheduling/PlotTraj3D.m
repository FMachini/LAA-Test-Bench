% close all
eps = 0.03;
vx_arm_x = [param.l_b param.l_b -param.l_b -param.l_b];
vy_arm_x = [eps -eps -eps eps];
    
vx_arm_y = [eps -eps -eps eps];
vy_arm_y = [param.l_b param.l_b -param.l_b -param.l_b];
    
v_quad_z = [0 0 0 0];
gamma = [0 pi/2 pi 3*pi/2];
color_arrow = {'c','m','f'};

coord_x_seta = [0.3, 0, 0];
coord_y_seta = [0, 0.3, 0];
coord_z_seta = [0, 0, 0.3];

%% Criando arquivo de vídeo .
% animacao_quad = 'video.mp4'; % nome do arquivo
% F1 = VideoWriter(animacao_quad); 
% set(F1,'Quality',100); % Configura a qualidade em 100%
% open(F1)    % Abre o arquivo de vídeo

% Criando a Figura base
figura1 = figure;
%alpha = alpha*0 + 45*pi/180*ones(size(alpha_m));
for k = 250
    
    axis tight;
    plot3(xp(10,1:k),xp(11,1:k),xp(12,1:k),'bx-') 
    ylim([-2 1.5]); xlim([-2 1.5]); zlim([-2 1.5])
    grid on, hold on
    set(gca,'FontSize',20)
    xlabel('x','FontSize',20),ylabel('y','FontSize',20),zlabel('z','FontSize',20)
    
    % leitura de dados
    x = xp(10,k)*1;
    y = xp(11,k)*1;
    z = xp(12,k)*1;
    phi = xp(7,k)*1;
    theta = xp(8,k)*1;
    psi = xp(9,k)*1;
    
    L1 = [cos(psi),   -sin(psi),  0;
          sin(psi),   cos(psi),   0;
           0,         0,          1];  % k
    L2 = [cos(theta),   0,   sin(theta);
          0,            1,   0;
         -sin(theta),   0,   cos(theta)];  % k
    L3 = [1,   0,         0;
          0,   cos(phi), -sin(phi);
          0,   sin(phi),  cos(phi)];  % k

    LEB = L1*L2*L3;

    for i = 1:4
        V_arm_x(:,i) = LEB\[vx_arm_x(i); vy_arm_x(i); v_quad_z(i)];
        V_arm_y(:,i) = LEB\[vx_arm_y(i); vy_arm_y(i); v_quad_z(i)];
    end
%     hold on
    arm_x = patch(V_arm_x(1,:)+x, V_arm_x(2,:)+y, V_arm_x(3,:)+z,[1 0 0],'Facecolor','interp','FaceVertexCData',[[1 0 0];[1 0 0];[0 1 0];[0 1 0]]);
    arm_y = patch(V_arm_y(1,:)+x, V_arm_y(2,:)+y, V_arm_y(3,:)+z,[1 0 0],'Facecolor','interp','FaceVertexCData',[[0.5 0.5 0.5];[1 1 0];[0.5 0 1];[0.5 0 1]]);
    
%     alpha(i,k) = 45*pi/180;
    for i = [1,3]
        alpha_m = alpha(i, k)*1;
    
        R_M2B = [cos(alpha_m) 0 sin(alpha_m);
                 0 1 0;
                 -sin(alpha_m) 0 cos(alpha_m)];
       
        R_gamma = [cos(gamma(i)) -sin(gamma(i)) 0;
                   sin(gamma(i)) cos(gamma(i)) 0;
                   0 0 1];
        R_tilt = R_gamma*R_M2B;
        for j = 1:3
            S = R_tilt\LEB\[coord_x_seta(j); coord_y_seta(j);coord_z_seta(j)];
            
            arrow3([V_arm_x(1,i)+x, V_arm_x(2,i)+y, V_arm_x(3,i)+z],[V_arm_x(1,i)+S(1)+x, V_arm_x(2,i)+S(2)+y, V_arm_x(3,i)+S(3)+z],color_arrow{j},0.3) 
            
        end      
    end

     
    for i = [2, 4]    
        alpha_m = alpha(i, k)*1;
%     
        R_M2B = [cos(alpha_m) 0 sin(alpha_m);
                 0 1 0;
                 -sin(alpha_m) 0 cos(alpha_m)];
       
        R_gamma = [cos(gamma(i)) -sin(gamma(i)) 0;
                   sin(gamma(i)) cos(gamma(i)) 0;
                   0 0 1];
        R_tilt = R_gamma*R_M2B;
%                
        for j = 1:3
            S = R_tilt\LEB\[coord_x_seta(j); coord_y_seta(j);coord_z_seta(j)];
            
            arrow3([V_arm_y(1,i)+x, V_arm_y(2,i)+y, V_arm_y(3,i)+z],[V_arm_y(1,i)+S(1)+x, V_arm_y(2,i)+S(2)+y, V_arm_y(3,i)+S(3)+z],color_arrow{j},0.3) 
        end 
    end

    pause(0.01)
    hold off
    
%      FrameAtual = getframe(figura1); %capturando o frame atual da figura1
%      writeVideo(F1,FrameAtual); %Inserindo o frame atual no arquivo de vídeo F1

end

% close(F1) %Fecha o arquivo de vídeo
% close(figura1) %Fecha a figura1