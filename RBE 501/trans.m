function T=trans(theta,l)
T=[cos(theta) -1*sin(theta) 0 l*cos(theta);
    sin(theta) cos(theta) 0 l*sin(theta);
    0 0 1 0;
    0 0 0 1];
end 