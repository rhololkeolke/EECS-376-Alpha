function plotArc(rho,startAngle,ref_point,seg_length,dt)
    r = 1/abs(rho);
    if(rho >= 0)
        tanAngStart = startAngle + pi/2;
    else
        tanAngStart = startAngle - pi/2;
    end
    
    x = zeros(1,1/dt);
    y = zeros(1,1/dt);
    psi = zeros(1,1/dt);
    segDistDone = 0:dt:1;
    segDistDone = seg_length*segDistDone;
    for i = 1:length(segDistDone)
        dAng = segDistDone(i)*rho;
        arcAng = startAngle+dAng;
        x(i) = ref_point(1) + r*cos(2*pi+arcAng);
        y(i) = ref_point(2) + r*sin(2*pi+arcAng);
        psi(i)=tanAngStart+dAng;
    end
    
    figure
    hold on
    plot(x,y)
    plot(x(1),y(1),'sr')
    
    figure
    plot(segDistDone,psi)

end