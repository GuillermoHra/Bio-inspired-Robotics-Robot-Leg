function keypoints = keypoints_GRAC_leg(in1,in2)
%KEYPOINTS_GRAC_LEG
%    KEYPOINTS = KEYPOINTS_GRAC_LEG(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    14-Nov-2019 13:23:18

l2 = in2(12,:);
l3 = in2(13,:);
lh0 = in2(18,:);
thh = in2(2,:);
thh0 = in2(1,:);
thk = in1(2,:);
y = in1(1,:);
t2 = cos(thh);
t3 = cos(thh0);
t4 = sin(thh);
t5 = sin(thh0);
t10 = -thk;
t6 = l2.*t2;
t7 = lh0.*t3;
t8 = l2.*t4;
t9 = lh0.*t5;
t13 = t10+thh;
t11 = -t6;
t12 = -t7;
keypoints = reshape([0.0,y,0.0,t9,t12+y,0.0,t8+t9,t11+t12+y,0.0,t8+t9+l3.*sin(t13),t11+t12+y-l3.*cos(t13),0.0],[3,4]);