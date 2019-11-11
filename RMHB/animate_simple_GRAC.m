function animate_simple(t,z,p, speed)
    hold on
    axis([-.2 .2 -.2 1])
    axis equal % sets the X:Y aspect ratio 1:1; otherwise things will look stretched
    h_ground = plot([-1 1],[0 0],'k-','LineWidth',5);
%     h_leg    = plot([0],[0],'-o',...
%                 'LineWidth',3,...
%                 'MarkerEdgeColor','r',...
%                 'MarkerFaceColor','r',...
%                 'MarkerSize',6); 
h_GCOM = plot([0],[0],'LineWidth',2);
    h_COM_H = plot([0],[0],'LineWidth',2);
    h_HK = plot([0],[0],'LineWidth',2);
    h_KA = plot([0],[0],'LineWidth',2);
   h_AE = plot([0],[0],'LineWidth',2);
    
   
    tic                                             % start counter
    while toc < t(end)/speed                        % while there's real time left
        tsim = toc*speed;                           % determine the simulation time to draw
        zint = interp1(t',z',tsim', 'linear')';     % interpolate to get coordinates at that time
        draw_lines(zint,p,h_GCOM,h_COM_H,h_HK,h_KA,h_AE);
    end
    draw_lines(z(:,end),p,h_GCOM,h_COM_H,h_HK,h_KA,h_AE);
end

function draw_lines(z,p,h_GCOM,h_COM_H,h_HK,h_KA,h_AE)
%     keypoints = keypoints_GRAC_leg(z,p);
%     h_leg.XData = keypoints(1,:);
%     h_leg.YData = keypoints(2,:);
keypoints = keypoints_GRAC_leg(z,p);

        r_cm1 = keypoints(:,1); % Vector to base of cart
        r_h0 = keypoints(:,2);
        r_k = keypoints(:,3); % Vector to tip of pendulum
        r_a = keypoints(:,4);
        r_e = keypoints(:,5);
        set(h_COM_H,'XData',[r_cm1(1) r_h0(1)]);
        set(h_COM_H,'YData',[r_cm1(2) r_h0(2)]);
        
        set(h_HK,'XData',[r_h0(1) r_k(1)]);
        set(h_HK,'YData',[r_h0(2) r_k(2)]);
        
        set(h_KA,'XData',[r_k(1) r_a(1)]);
        set(h_KA,'YData',[r_k(2) r_a(2)]);
        
        set(h_AE,'XData',[r_a(1) r_e(1)]);
        set(h_AE,'YData',[r_a(2) r_e(2)]);
    drawnow
end