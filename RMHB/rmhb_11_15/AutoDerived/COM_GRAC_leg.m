function COM = COM_GRAC_leg(in1,in2)
%COM_GRAC_LEG
%    COM = COM_GRAC_LEG(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    17-Nov-2019 15:56:28

dy = in1(4,:);
y = in1(1,:);
COM = [y;dy];
