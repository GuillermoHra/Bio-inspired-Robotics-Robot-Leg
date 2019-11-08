function keypoints = keypoints_GRAC_leg(in1,in2)
%KEYPOINTS_GRAC_LEG
%    KEYPOINTS = KEYPOINTS_GRAC_LEG(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    01-Nov-2019 13:01:31

l2 = in2(12,:);
l3 = in2(13,:);
l4 = in2(14,:);
lh0 = in2(18,:);
tha = in1(3,:);
thh = in2(2,:);
thh0 = in2(1,:);
thk = in1(2,:);
y = in1(1,:);
t2 = sin(thh0);
t3 = lh0.*t2;
t4 = sin(thh);
t5 = l2.*t4;
t6 = thh-thk;
t7 = sin(t6);
t8 = l3.*t7;
t9 = cos(thh0);
t10 = cos(thh);
t11 = tha+thh-thk;
t12 = cos(t6);
keypoints = reshape([0.0,y,0.0,t3,y-lh0.*t9,0.0,t3+t5,y-l2.*t10-lh0.*t9,0.0,t3+t5+t8,y-l2.*t10-l3.*t12-lh0.*t9,0.0,t3+t5+t8+l4.*sin(t11),y-l2.*t10-l3.*t12-lh0.*t9-l4.*cos(t11),0.0],[3,5]);