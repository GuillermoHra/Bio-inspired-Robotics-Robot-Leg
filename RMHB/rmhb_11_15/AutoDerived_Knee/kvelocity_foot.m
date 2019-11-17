function dra = kvelocity_foot(in1,in2)
%KVELOCITY_FOOT
%    DRA = KVELOCITY_FOOT(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    17-Nov-2019 15:13:55

dthk = in1(4,:);
dy = in1(3,:);
l3 = in2(13,:);
thh = in2(2,:);
thk = in1(2,:);
t2 = -thk;
t3 = t2+thh;
dra = [-dthk.*l3.*cos(t3);dy-dthk.*l3.*sin(t3);0.0];
