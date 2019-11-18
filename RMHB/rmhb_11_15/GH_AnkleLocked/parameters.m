function p = parameters() 
thh0=deg2rad(15);
thh=deg2rad(45);
m1=0;
I1= 50*10^-6;
m2=.08;
I2= 10*10^-6;
m3=.28;
I3= (1/3)*(.28)*((160/1000))^2;
m4=.071;
I4= (1/3)*(.071)*((94.33/1000))^2;

l1=0;
l2=160/1000;
l3=160/1000 ;
l4=94.33/1000;
lc2=l2/2;
lc3=l3/4;
lc4=l4/2;
lh0=.03;
g =9.81;
    thki=deg2rad(75);
    thai=deg2rad(75);
  p=[thh0;thh; m1; I1; m2 ;I2; m3 ;I3; m4; I4; l1; l2; l3 ;l4;lc2;lc3;lc4;lh0; g ;thki;thai];
end