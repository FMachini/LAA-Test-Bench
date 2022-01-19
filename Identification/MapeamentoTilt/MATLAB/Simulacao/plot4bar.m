function theta_tilt = plot4bar(parameters,theta_servo, I,theta_max,max_I)
    %clear plot;
    %clear lines;
    %subplot(2,1,1)
    
    s = parameters(1); 
    w = parameters(2);
    u = parameters(3);
    v = parameters(4);
    t = parameters(5);
    y = parameters(6);
    x = parameters(7);
   
    alpha = asin(y/w);
    beta = (pi+alpha)-theta_servo;

    ST = sqrt(s^2+w^2-2*s*w*cos(beta));
    phi_1 = acos((ST^2+w^2-(s^2))/(2*ST*w));
    phi_2 = acos((t^2+ST^2-(x^2))/(2*t*ST));
    phi_3 = acos((t^2+v^2-(u^2))/(2*t*v));

    theta_tilt = pi - (phi_1 + phi_2 + phi_3 + alpha);
    
    points = linspace(0, theta_tilt);% phi_1 + phi_2 + phi_3 + alpha);    % 100 points from 0 to theta
    xCurveTilt = -(v/2).*cos(points);  % X coordinates of curve
    yCurveTilt = -(v/2).*sin(points);  % Y coordinates of curve

    points = linspace(0, theta_servo);    % 100 points from 0 to theta
    xCurveServo = (s/2).*cos(points);  % X coordinates of curve
    yCurveServo = -(s/2).*sin(points);  % Y coordinates of curve

    O1 = [0 0];
    O2 = [w*cos(alpha) -y];

    plot(O1(1)+[-20 20], [0 0], '-k');   % Plot dashed line
    hold on
    plot(O2(1)+[-20 20], [-y -y], '-k');   % Plot dashed line
    
    str1 = {'\theta_{tilt} ='} ;
    str3 = {'\circ '};
    str2 = sprintf('%0.2f',rad2deg(theta_tilt));
    str = strcat(str1,str2,str3);
    
    text(O1(1)+2,O1(2)+2.5,str);
    str1 = {'\theta_{servo} ='} ;
    str3 = {'\circ '};
    str2 = sprintf('%0.2f',rad2deg(theta_servo));
    str = strcat(str1,str2,str3);
    text(O2(1)+2,O2(2)+2.5,str)
    
    thetasaved = theta_servo;
    tiltsaved  = theta_tilt;
    
    str1 = {'\theta_{servo} ='} ;
    str3 = {'\circ '};
    str2 = sprintf('%0.2f',rad2deg(thetasaved));
    str = strcat(str1,str2,str3);
    str1 = {'   \theta_{tilt} ='} ;
    str3 = {'\circ '};
    str2 = sprintf('%0.2f',rad2deg(tiltsaved));
    str = strcat(str,str1,str2,str3);
    text(O1(1)+20,O1(2)+10,str)
    plot(O1(1)+xCurveTilt,O1(2)+ yCurveTilt, '--b');     % Plot curve
    plot(O2(1)+xCurveServo, O2(2)+ yCurveServo, '--b');     % Plot curve
    hold off
    H=gca;
    xlim(H,[-40 70]);
    ylim(H,[-40 50]);
    
    T  = [t*cos(phi_1+phi_2+alpha) -t*sin(phi_1+phi_2+alpha)];
    U  = [-v*cos(theta_tilt) -v*sin(theta_tilt)];             
    S  = [O2(1)+s*cos(theta_servo) -y-s*sin(theta_servo)];

    %Bolinhas sobre cada ponto
    O1_circle = viscircles(O1,0.05);
    O2_circle = viscircles(O2,0.05);

    T_circle = viscircles(real(T),0.05);
    U_circle = viscircles(real(U),0.05); 
    S_circle = viscircles(real(S),0.05); 
    T_bar = line([O1(1) T(1)],[O1(2) T(2)],'Color','red');
    U_bar = line([T(1) U(1)],[T(2) U(2)],'Color','red');
    V_bar = line([O1(1) U(1)],[O1(2) U(2)],'Color','red');

    S_bar = line([O2(1) S(1)],[O2(2) S(2)],'Color','green');
    W_bar = line([O1(1) O2(1)],[O1(2) O2(2)],'Color','black');
    X_bar = line([S(1) T(1)],[S(2) T(2)],'Color','black');
    xlabel('x [mm]')
    ylabel('y [mm]')
    pause(0.005);
    if ( (I ~= max_I) || (theta_servo <(theta_max-0.005)) )
        delete(O1_circle);
        delete(O2_circle);
        delete(T_circle);
        delete(U_circle);
        delete(S_circle);
        
        delete(T_bar);
        delete(U_bar);
        delete(V_bar);
        delete(S_bar);
        delete(W_bar);
        delete(X_bar);
    end
end