function T = dh2mat(d, theta, a, alpha)
c = @(x) cos(x);
s = @(x) sin(x);

T = [c(theta),    -s(theta)*c(alpha),    s(theta)*s(alpha),     a*c(theta);
     s(theta),     c(theta)*c(alpha),   -c(theta)*s(alpha),     a*s(theta);
     0,            s(alpha),             c(alpha),              d;
     0,            0,                    0,                     1];
end