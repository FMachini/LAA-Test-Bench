function theta_tilt = sim4bar(parameters, theta_servo)

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
end