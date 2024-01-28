function [ex,ey]=ellipse(a,b,x0,y0,n)
    

t=linspace(-pi,pi,n);
ex=x0+a*cos(t);
ey=y0+b*sin(t);

end
