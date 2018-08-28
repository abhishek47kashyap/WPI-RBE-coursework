function [] = Walkingtest()
x = linspace(1,-360);
y = linspace(1,360);
n = 1;
while n<numel(x)
    WalkingSim(x(n),y(n));
    n = n+1;
    
end

end