function re = position_foot(in1,in2)
%POSITION_FOOT
%    RE = POSITION_FOOT(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    17-Nov-2019 15:56:28

l2 = in2(12,:);
l3 = in2(13,:);
l4 = in2(14,:);
lh0 = in2(18,:);
tha = in1(3,:);
thh = in2(2,:);
thh0 = in2(1,:);
thk = in1(2,:);
y = in1(1,:);
t2 = -thk;
t3 = t2+thh;
t4 = t3+tha;
re = [l3.*sin(t3)+l4.*sin(t4)+l2.*sin(thh)+lh0.*sin(thh0);y-l3.*cos(t3)-l4.*cos(t4)-l2.*cos(thh)-lh0.*cos(thh0);0.0];
