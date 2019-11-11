function p = parameters() 
thh0=deg2rad(15);
thh=deg2rad(45);
m1=.5;
I1= 50*10^-6;
m2=.1;
I2= 10*10^-6;
m3=.1;
I3= 10*10^-6;
m4=.05;
I4= 5*10^-6;

l1=.05;
l2=.15;
l3=.15 ;
l4=.075;
lc2=l2/2;
lc3=l3/2;
lc4=l4/2;
lh0=.03;
g =9.81;
    thki=deg2rad(90);
    thai=deg2rad(75);
  p=[thh0;thh; m1; I1; m2 ;I2; m3 ;I3; m4; I4; l1; l2; l3 ;l4;lc2;lc3;lc4;lh0; g ;thki;thai];
end