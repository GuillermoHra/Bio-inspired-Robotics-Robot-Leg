function b = b_GRAC_leg(in1,in2,Fy,Fx,in5)
%B_GRAC_LEG
%    B = B_GRAC_LEG(IN1,IN2,FY,FX,IN5)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    18-Nov-2019 13:49:20

dtha = in1(6,:);
dthk = in1(5,:);
dy = in1(4,:);
g = in5(19,:);
l3 = in5(13,:);
l4 = in5(14,:);
lc3 = in5(16,:);
lc4 = in5(17,:);
m1 = in5(3,:);
m2 = in5(5,:);
m3 = in5(7,:);
m4 = in5(9,:);
taua = in2(2,:);
tauk = in2(1,:);
tha = in1(3,:);
thh = in5(2,:);
thk = in1(2,:);
t2 = lc3.^2;
t3 = -thk;
t4 = t3+thh;
t5 = cos(t4);
t6 = sin(t4);
t7 = t4+tha;
t8 = cos(t7);
t9 = sin(t7);
t10 = l3.*t5;
t11 = l3.*t6;
t13 = dthk.*lc3.*t6;
t12 = lc4.*t8;
t14 = lc4.*t9;
t19 = -t13;
t15 = dtha.*t12;
t16 = dthk.*t12;
t17 = dtha.*t14;
t18 = dthk.*t14;
t25 = dy+t19;
t26 = t10+t12;
t27 = t11+t14;
t20 = t15.*2.0;
t21 = -t15;
t22 = -t16;
t23 = -t17;
t24 = -t18;
t28 = dthk.*t26;
t29 = dthk.*t27;
t30 = -t29;
t31 = t15+t22;
t32 = t17+t24;
t33 = t21+t28;
t34 = t23+t29;
t36 = t14.*(t15-t28).*-2.0;
t35 = dy+t17+t30;
t37 = t12.*t35.*2.0;
b = [Fy-g.*m1-g.*m2-g.*m3-g.*m4+dthk.*((m4.*(t20-t28.*2.0))./2.0-dthk.*lc3.*m3.*t5)+(dtha.*m4.*(t16.*2.0-t20))./2.0;tauk+dthk.*((m4.*(t26.*(t17+t30).*2.0-t26.*t35.*2.0))./2.0-(m3.*(lc3.*t5.*t25.*2.0+dthk.*t2.*t5.*t6.*2.0))./2.0)-Fx.*(t10+l4.*t8)-Fy.*(t11+l4.*t9)+(m3.*(dthk.*lc3.*t5.*t25.*2.0+dthk.^2.*t2.*t5.*t6.*2.0))./2.0+(m4.*((t17+t30).*(t15-t28).*2.0-t35.*(t15-t28).*2.0))./2.0+(dtha.*m4.*(t36+t37-t26.*t32.*2.0+t27.*t31.*2.0))./2.0+g.*m4.*t27+g.*lc3.*m3.*t6;taua+(m4.*(t31.*t35.*2.0-t32.*(t15-t28).*2.0))./2.0-(dtha.*m4.*(t37-t12.*t32.*2.0+t14.*t31.*2.0-t14.*(t15-t28).*2.0))./2.0+(dthk.*m4.*(t37-t12.*(t17+t30).*2.0))./2.0+Fx.*l4.*t8+Fy.*l4.*t9-g.*m4.*t14];
