function animate_param_sweep_twocompare(sol1,sol2,p, speed)
    t1=sol1.x;
    z1=sol1.y;
    t2=sol2.x;
    z2=sol2.y;
    hold on
    axis([-.2 .2 -.2 1])
    axis equal % sets the X:Y aspect ratio 1:1; otherwise things will look stretched
    h_ground = plot([-1 1],[0 0],'k-','LineWidth',5);
%     h_leg    = plot([0],[0],'-o',...
%                 'LineWidth',3,...
%                 'MarkerEdgeColor','r',...
%                 'MarkerFaceColor','r',...
%                 'MarkerSize',6); 
    h1_GCOM = plot([0],[0],'LineWidth',2);
    h1_COM_H = plot([0],[0],'LineWidth',2);
    h1_HK = plot([0],[0],'LineWidth',2);
    h1_KA = plot([0],[0],'LineWidth',2);
   h1_AE = plot([0],[0],'LineWidth',2);
   h2_GCOM = plot([0],[0],'LineWidth',2);
    h2_COM_H = plot([0],[0],'LineWidth',2);
    h2_HK = plot([0],[0],'LineWidth',2);
    h2_KA = plot([0],[0],'LineWidth',2);
   h2_AE = plot([0],[0],'LineWidth',2);
   
   
    h_title = title('t=0.0s');
   
    tic                                             % start counter
    while toc < t1(end)/speed                        % while there's real time left
        tsim = toc*speed;  
        set(h_title,'String',  sprintf('t=%.2f',tsim) )% determine the simulation time to draw
        zint1 = interp1(t1',z1',tsim', 'linear')';     % interpolate to get coordinates at that time
        zint2 = interp1(t2',z2',tsim', 'linear')';     % interpolate to get coordinates at that time
        draw_lines(zint1,p,h1_GCOM,h1_COM_H,h1_HK,h1_KA,h1_AE,'green');
        draw_lines(zint2,p,h2_GCOM,h2_COM_H,h2_HK,h2_KA,h2_AE,'red');
    end
   %draw_lines(z1(:,end),p,h1_GCOM,h1_COM_H,h1_HK,h1_KA,h1_AE);
   % draw_lines(z2(:,end),p,h2_GCOM,h2_COM_H,h2_HK,h2_KA,h2_AE);
end

function draw_lines(z,p,h_GCOM3,h_COM_H3,h_HK3,h_KA3,h_AE3,col)
%     keypoints = keypoints_GRAC_leg(z,p);
%     h_leg.XData = keypoints(1,:);
%     h_leg.YData = keypoints(2,:);
keypoints = keypoints_GRAC_leg(z,p);

        r_cm1 = keypoints(:,1); % Vector to base of cart
        r_h0 = keypoints(:,2);
        r_k = keypoints(:,3); % Vector to tip of pendulum
        r_a = keypoints(:,4);
        r_e = keypoints(:,5);
        set(h_COM_H3,'XData',[r_cm1(1) r_h0(1)]);
        set(h_COM_H3,'YData',[r_cm1(2) r_h0(2)]);
         set(h_COM_H3,'color',col);
         
        set(h_HK3,'XData',[r_h0(1) r_k(1)]);
        set(h_HK3,'YData',[r_h0(2) r_k(2)]);
        set(h_HK3,'color',col);
        
        set(h_KA3,'XData',[r_k(1) r_a(1)]);
        set(h_KA3,'YData',[r_k(2) r_a(2)]);
        set(h_KA3,'color',col);
        
        set(h_AE3,'XData',[r_a(1) r_e(1)]);
        set(h_AE3,'YData',[r_a(2) r_e(2)]);
        set(h_AE3,'color',col);
    drawnow
end