clc; clear all; close all;
t=0;
 
%Variables de la ecuación
velmotor2=0.3;
r=0.5;
velmotor1= 0.5;
velangular=25;


%Ecuaciones 

%Condiciones iniciales
theta=1.9;
x=3;
y=2;
l=1;
MR=transl(x,y,0)*trotz(theta);


xi=0;
yi=0;
plotvol([5 5 5])


trplot(MR,'frame','MR','view',[0 90],'color','b')
hold on

limitex= (x/(r/2*(velmotor1+velmotor2)))
limitey=(y/(r/2*(velmotor1+velmotor2)))
thetar=(r/(2*l))*(velmotor1-velmotor2)*t
limiteangulo1= 2*l*1.57/(r*(velmotor1-velmotor2));
limiteangulo2= 2*l*theta/(r*(velmotor1-velmotor2));
xr = (r/2)*(velmotor1+velmotor2)*t;
limitex1= (3/(r/2*(velmotor1+velmotor2)))

for t=0:1:limitex1

xr = (r/2)*(velmotor1+velmotor2)*t;    
M0=transl(xr,0,0)*trotz(0);
trplot(M0,'frame','M0','view',[0 90],'color','k')
pause(0.05)
   
end

t=0;
for t=0:1 :limiteangulo1

thetar=(r/(2*l))*(velmotor1-velmotor2)*t
    
M0=transl(xr,0,0)*trotz(thetar);
trplot(M0,'frame','M0','view',[0 90],'color','k')
pause(0.05)
   
end

t=0;



for t=0:1:limitey

yr = (r/2)*(velmotor1+velmotor2)*t,    
M0=transl(xr,yr,0)*trotz(thetar);
trplot(M0,'frame','M0','view',[0 90],'color','k')
pause(0.05)
   
end



for t=0:0.1:100

thetar= thetar + (r/(2*l))*(velmotor1-velmotor2)*t
    
M0=transl(xr,yr,0)*trotz(thetar);
trplot(M0,'frame','M0','view',[0 90],'color','k')
pause(0.05)

if(thetar>=theta)
break
end
   
end



