function dydt = theta4_din(t,y)

  global J;

  global b;

  global c;

  dydt = zeros (2,1);

  dydt(1) = y(2);

  dydt(2) = -c/J .* y(1) - b/J .* y(2);

end