% simulation of state updater for arc, given a point.  Only will work up
% until one revolution of an arc.  In practice we shouldn't ever really be
% doing more than 3/4s
function segDistDone = updateState(p,rho,startAngle,ref_point,seg_length)
    r = 1/abs(rho);
    if(rho >= 0)
        tanAngStart = startAngle - pi/2;
    else
        tanAngStart = startAngle + pi/2;
    end
    
    % sample arc for graphing
    xArc = zeros(1,1000);
    yArc = zeros(1,1000);
    
    for i=1:1000
        dAng = seg_length*i/1000*rho; 
        arcAng = startAngle+dAng;
        xArc(1,i) = ref_point(1)+r*cos(arcAng);
        yArc(1,i) = ref_point(2)+r*sin(arcAng);
    end 
    
    % get the vector from the arc's center to the robot's current position
    rVec = p-ref_point;
    % get the angle of that vector with respect to the x-axis
    theta = atan2(rVec(2),rVec(1));
    
    % get the minimum magnitude representation of the startAngle
    phi = mod(startAngle,2*pi); % this will always be between 0 and 2*pi
    if(abs(phi-2*pi) < phi) % if the manitude of the negative representation is smaller
        phi = phi - 2*pi; % use that instead
    end
            
    posPhi = mod(phi,2*pi);
    posTheta = mod(theta,2*pi);
    
    % figure out angle halway between end and start angle
    if(rho >= 0)
        halfAngle  = (seg_length)/(2*r) + posPhi;
        finAngle = seg_length/r;
    else % if the curvature is negative then go the opposite direction
        halfAngle = posPhi - (seg_length)/(2*r);
        finAngle = -seg_length/r;
    end
    
    % find theta in terms of starting angle
    if(rho >=0)
        if(posTheta > posPhi) % theta has looped around the circle
            beta = posTheta - posPhi;
        else
            beta = 2*pi-posPhi+posTheta;
        end
    else
        if(posTheta < posPhi)
            beta = posTheta - posPhi;
        else
            beta = posTheta-posPhi-2*pi;
        end
    end
    
    % figure out what region the angle is in
    if(rho >= 0)
        if(beta >= 0 && beta <= halfAngle-posPhi+pi) % beta is in the specified arc
            alpha = beta;
        else % beta is before the arc's starting point
            alpha = beta - 2*pi;
        end
    else
        if(beta >= halfAngle-pi-posPhi && beta <= 0)
            alpha = beta;
        else
            alpha = beta + 2*pi;
        end
    end
    
    s = r*alpha; % the unnormalized segDistDone
    dAng = s*abs(rho);
    arcAng = startAngle+dAng;
    x = ref_point(1)+r*cos(2*pi+arcAng);
    y = ref_point(2)+r*sin(2*pi+arcAng);
    
    figure
    hold on
    plot(ref_point(1),ref_point(2),'.') % plot the center
    plot(xArc,yArc) % plot the desired arc
    plot(xArc(1),yArc(1),'.r') % plot the starting point of the arc
    plot(x,y,'xk') % plot the point along the arc the robot is at
    plot(p(1),p(2),'xm') % plot the point the robot is actually at
    plot([ref_point(1),p(1)],[ref_point(2),p(2)],'--g') % plot the vector from the center to the robot's current position
    plot([r*cos(halfAngle+pi),ref_point(1)],[r*sin(halfAngle+pi),ref_point(2)],'-.m') % plot cutoff point between negative and positive
    plot([xArc(1),ref_point(1)],[yArc(1),ref_point(2)],'-.b')
    plot([xArc(end),ref_point(1)],[yArc(end),ref_point(2)],'-.b')
    
    if(rho >= 0)
        segDistDone = r*alpha/seg_length; 
    else
        segDistDone = -r*alpha/seg_length;
    end
end