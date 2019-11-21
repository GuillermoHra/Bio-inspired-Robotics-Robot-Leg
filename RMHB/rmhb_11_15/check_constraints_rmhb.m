function [flag] = check_constraints_rmhb(sol,p, uout)

 z=sol.y;% [y, thk,tha,dy,dthk,dtha]

th_h=p(2);
th_k=z(2,:);
th_a=z(3,:);
sums=th_h-th_k+th_a;
yaprev=100;

%right here
if min(sums)>0 && max(sums)<pi/2 && min(th_k)>0 && max(th_k)<3*pi/2 && min(th_a)>-pi && max(th_a)< pi
    for i=1:length(z(1,:))
        ya=position_ankle(z(:,i),p);
        if ya<yaprev
            yaprev=ya;
        end
    end
    if yaprev <0
        flag=0;
    else         
        flag=1;
    end
    
else
    
    flag=0;
   

                     
end