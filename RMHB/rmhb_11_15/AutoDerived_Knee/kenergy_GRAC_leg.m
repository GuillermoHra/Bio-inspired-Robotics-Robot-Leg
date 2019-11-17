function E = kenergy_GRAC_leg(in1,in2)
%KENERGY_GRAC_LEG
%    E = KENERGY_GRAC_LEG(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    17-Nov-2019 15:51:28

I3 = in2(8,:);
dthk = in1(4,:);
dy = in1(3,:);
g = in2(19,:);
l2 = in2(12,:);
lc2 = in2(15,:);
lc3 = in2(16,:);
lh0 = in2(18,:);
m1 = in2(3,:);
m2 = in2(5,:);
m3 = in2(7,:);
thh = in2(2,:);
thh0 = in2(1,:);
thk = in1(2,:);
y = in1(1,:);
t2 = cos(thh);
t3 = cos(thh0);
t4 = dthk.^2;
t5 = dy.^2;
t7 = -thk;
t8 = -y;
t6 = lh0.*t3;
t9 = t7+thh;
t10 = cos(t9);
E = (I3.*t4)./2.0+(m1.*t5)./2.0+(m2.*t5)./2.0+(m3.*((dy-dthk.*lc3.*sin(t9)).^2+lc3.^2.*t4.*t10.^2))./2.0+g.*m1.*y-g.*m2.*(t6+t8+lc2.*t2)-g.*m3.*(t6+t8+l2.*t2+lc3.*t10);
