function [flag] = check_constraints_rmhb1(sol,p, uout)

 z=sol.y;% [y, thk,tha,dy,dthk,dtha]

th_h=p(2);
th_k=z(2,:);
th_a=z(3,:);
sums=th_h-th_k+th_a;
yaprev=100000;

 for i=1:length(z(1,:))
        ya1=position_ankle(z(:,i),p);
        ya=ya1(2);
        if ya<yaprev
            yaprev=ya;
        end
 end
    yaprev
%right here
if min(sums)>0 && max(sums)<pi/2 && min(th_k)>0 && max(th_k)<3*pi/2 && min(th_a)>-pi && max(th_a)< pi

    if yaprev <0
        flag=0;
    else         
        flag=1;
    end
    
else
    
    flag=0;
   

                     
end