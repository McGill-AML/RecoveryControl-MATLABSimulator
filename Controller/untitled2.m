line=[1,-1,0];
beforeRef = [2;-2;1];
afterRef  = [0;0;0];
afterRef(1)  = +beforeRef(1) - (2*line(1)/norm(line(1:2))^2)*(dot(line,beforeRef));
afterRef(2)  = +beforeRef(2) - (2*line(2)/norm(line(1:2))^2)*(dot(line,beforeRef));
afterRef