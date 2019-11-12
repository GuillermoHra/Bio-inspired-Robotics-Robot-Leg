function [flag] = check_constraints_rmhb(sol,p)

 z=sol.y;% [y, thk,tha,dy,dthk,dtha]

th_h=p(2);
th_k=z(2,:);
th_a=z(3,:);
sums=th_h-th_k+th_a;

if min(sums)>0 && max(sums)<pi/2 && min(th_k)>0 && max(th_k)<3*pi/2
    flag=1;
else
    flag=0;
end
    
                     
end