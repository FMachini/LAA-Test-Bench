% close all
% xplanta = [u v w P Q R phi theta psi X Y Z]
% xlqr = [P Q phi theta]
%% Plotando resultados
% Posi��es
figure
subplot(2,2,1)
plot([1:(kmax+1)].*Ts, xp(1,:), 'LineWidth', 2)
ylabel('u (m/s)'); xlabel('Tempo (s)'); grid on
set(gca,'fontsize',20)
xlim([0 kmax*Ts])


subplot(2,2,2)
plot([1:(kmax+1)].*Ts, xp(2,:), 'LineWidth', 2)
ylabel('v (m/s)'); xlabel('Tempo (s)'); grid on
set(gca,'fontsize',20)
xlim([0 kmax*Ts])

subplot(2,2,3)
plot([1:(kmax+1)].*Ts, xp(10,:), 'LineWidth', 2)
ylabel('X (m)'); xlabel('Tempo (s)'); grid on
set(gca,'fontsize',20)
xlim([0 kmax*Ts])

subplot(2,2,4)
plot([1:(kmax+1)].*Ts, xp(11,:), 'LineWidth', 2)
ylabel('Y (m)'); xlabel('Tempo (s)'); grid on
set(gca,'fontsize',20)
xlim([0 kmax*Ts])

% Movimento em z
figure
subplot(2,2,1)
plot([1:(kmax+1)].*Ts,xp(3,:), 'LineWidth', 2)
ylabel('w (m/s)'); xlabel('Tempo (s)'); grid on
set(gca,'fontsize',20)
xlim([0 kmax*Ts])

subplot(2,2,2)
plot([1:(kmax+1)].*Ts,xp(12,:), 'LineWidth', 2)
ylabel('Z (m)'); xlabel('Tempo (s)'); grid on
set(gca,'fontsize',20)
xlim([0 kmax*Ts])

subplot(2,2,3)
plot([1:(kmax+1)].*Ts, xp(6,:).*180/pi, 'LineWidth', 2)
ylabel('R (deg/s)'); xlabel('Tempo (s)'); grid on
set(gca,'fontsize',20)
xlim([0 kmax*Ts])

subplot(2,2,4)
plot([1:(kmax+1)].*Ts, xp(9,:).*180/pi, 'LineWidth', 2)
ylabel('\psi (deg)'); xlabel('Tempo (s)'); grid on
set(gca,'fontsize',20)
xlim([0 kmax*Ts])

%% Estados do LQR - em torno de Z
figure
subplot(2,2,1)
plot([1:(kmax+1)].*Ts, xp(4,:).*180/pi, 'LineWidth', 2)
ylabel('P (deg/s)'); xlabel('Tempo (s)'); grid on
set(gca,'fontsize',20)
xlim([0 kmax*Ts])

subplot(2,2,2)
plot([1:(kmax+1)].*Ts, xp(5,:).*180/pi, 'LineWidth', 2)
ylabel('Q (deg/s)'); xlabel('Tempo (s)'); grid on
set(gca,'fontsize',20)
xlim([0 kmax*Ts])

subplot(2,2,3)
plot([1:(kmax+1)].*Ts,(xp(7,:)).*180/pi, 'LineWidth', 2)
ylabel('\phi (deg)'); xlabel('Tempo (s)'); grid on
set(gca,'fontsize',20)
xlim([0 kmax*Ts])

subplot(2,2,4)
plot([1:(kmax+1)].*Ts, (xp(8,:)).*180/pi, 'LineWidth', 2)
ylabel('\theta (deg)'); xlabel('Tempo (s)'); grid on
set(gca,'fontsize',20)
xlim([0 kmax*Ts])

%% Controle - Omega
figure
subplot(2,2,1)
plot([1:kmax].*Ts, omg(1,:), 'LineWidth', 2)
ylabel('\Omega_1 (rad/s)');xlabel('Tempo (s)'); grid on
set(gca,'fontsize',20)
% ylim([200 300])
xlim([0 kmax*Ts])

subplot(2,2,2)
plot([1:kmax].*Ts, omg(2,:), 'LineWidth', 2)
ylabel('\Omega_2 (rad/s)'); xlabel('Tempo (s)'); grid on
set(gca,'fontsize',20)
% ylim([-300 -200])
xlim([0 kmax*Ts])

subplot(2,2,3)
plot([1:kmax].*Ts,omg(3,:), 'LineWidth', 2)
ylabel('\Omega_3 (rad/s)'); xlabel('Tempo (s)'); grid on
set(gca,'fontsize',20)
% ylim([200 300])
xlim([0 kmax*Ts])

subplot(2,2,4)
plot([1:kmax].*Ts, omg(4,:), 'LineWidth', 2)
ylabel('\Omega_4 (rad/s)'); xlabel('Tempo (s)'); grid on
set(gca,'fontsize',20)
% ylim([-300 -200])
xlim([0 kmax*Ts])

%% Controle - Alpha
figure
subplot(2,2,1)
plot([1:kmax].*Ts, alpha(1,:).*180/pi, 'LineWidth', 2)
ylabel('\alpha_1 (deg)');xlabel('Tempo (s)'); grid on
set(gca,'fontsize',20)
% ylim([200 300])
xlim([0 kmax*Ts])

subplot(2,2,2)
plot([1:kmax].*Ts, alpha(2,:).*180/pi, 'LineWidth', 2)
ylabel('\alpha_2 (deg)'); xlabel('Tempo (s)'); grid on
set(gca,'fontsize',20)
% ylim([-300 -200])
xlim([0 kmax*Ts])

subplot(2,2,3)
plot([1:kmax].*Ts, alpha(3,:).*180/pi, 'LineWidth', 2)
ylabel('\alpha_3 (deg)'); xlabel('Tempo (s)'); grid on
set(gca,'fontsize',20)
% ylim([200 300])
xlim([0 kmax*Ts])

subplot(2,2,4)
plot([1:kmax].*Ts, alpha(4,:).*180/pi, 'LineWidth', 2)
ylabel('\alpha_4 (deg)'); xlabel('Tempo (s)'); grid on
set(gca,'fontsize',20)
% ylim([-300 -200])
xlim([0 kmax*Ts])

%% Analise de suavidade dos parametros
%{
for z = 1:n_row_K
    for w = 1:n_col_K
        for i = 1:max(size(theta_0))
            for j = 1:max(size(phi_0))
                dummy_alpha(i,j) = alpha_0{i,j}(z);
             end
        end
    end
end

% coeficiente de K
for z = 1:n_row_K
    figure
    for w = 1:n_col_K
        subplot(2,n_col_K/2,w)
        for i = 1:max(size(theta_0))
          for j = 1:max(size(phi_0))
                dummy_K(i,j) = Klqr{i,j}(z,w);
          end
        end
       
        surf(theta_grid.*180/pi, phi_grid.*180/pi, dummy_K,'MarkerSize',12,'Marker','+','MarkerEdgeColor',[1 0 0])
        xlabel('\theta (deg)')
        ylabel('\phi (deg)')
        zlabel(strcat('K',num2str(i), num2str(j)))
        set(gca,'fontsize',20)
    end
end

%% Omega
% close all
figure
for z = 1:4
    subplot(2,2,z)
    
    for i = 1:max(size(theta_0))
        for j = 1:max(size(phi_0))
            dummy_omg(i,j) = omg_0{i,j}(z);
        end
    end
    
    surf(theta_grid.*180/pi, phi_grid.*180/pi, dummy_omg,'MarkerSize',12,'Marker','+','MarkerEdgeColor',[1 0 0])
    xlabel('\theta (deg)')
    ylabel('\phi (deg)')
    zlabel(strcat('\Omega_{', num2str(i),'_0}',' (rad/s)'))
    set(gca,'fontsize',20)
end

%% alpha
figure
for z = 1:4

    subplot(2,2,z)

    for i = 1:max(size(theta_0))
        for j = 1:max(size(phi_0))
            dummy_alpha(i,j) = alpha_0{i,j}(z);
        end
    end

    surf(theta_grid.*180/pi, phi_grid.*180/pi, dummy_alpha.*180/pi,'MarkerSize',12,'Marker','+','MarkerEdgeColor',[1 0 0])
    xlabel('\theta (deg)'); ylabel('\phi (deg)');
    zlabel(strcat('\alpha_{', num2str(i),'_0}',' (rad/s)'))
    set(gca,'fontsize',20)
end

%}

