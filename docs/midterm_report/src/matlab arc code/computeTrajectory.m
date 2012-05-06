function [sVAccel,sVDecel,sWAccel,sWDecel] = computeTrajectory(dt,currSeg,nextSeg)
    % segments will be vectors with the following information
    % [seg_type,seg_length,max_v,max_o,max_accel,max_decel,curvature,init_tan_angle]
    LINE = 0;
    ARC = 1;
    SPIN_IN_PLACE = 2;
    
    currSeg_type = round(currSeg(1));
    nextSeg_type = round(nextSeg(1));
    currSeg_length = currSeg(2);
    currMax_v = currSeg(3);
    nextMax_v = nextSeg(3);
    currMax_w = currSeg(4);
    nextMax_w = nextSeg(4);
    currMax_accel = currSeg(5);
    nextMax_accel = nextSeg(5);
    currMax_decel = currSeg(6);
    currCurv = currSeg(7);
    nextCurv = nextSeg(7);
    
    % calculate tAccel, distAccel, and sAccel based on path definitions
    if(currSeg_type == LINE)
        tVAccel = currMax_v/currMax_accel; % maximum amount of time allowed to accelerate
        distVAccel = 0.5*abs(currMax_accel)*tVAccel^2;
        sVAccel = distVAccel/currSeg_length;
        sWAccel = 0;
    elseif(currSeg_type == ARC)
        % find the maximum V and W given velocity constraints on both
        [maxVCmd,maxWCmd] = findMax_v_w(currMax_v,currMax_w,currCurv);
        
        tVAccel = maxVCmd/currMax_accel;
        distVAccel = 0.5*abs(currMax_accel)*tVAccel^2;
        sVAccel = distVAccel/currSeg_length;
        
        tWAccel = maxWCmd/currMax_accel;
        distWAccel = 0.5*abs(currMax_accel)*tWAccel^2;
        sWAccel = distWAccel/(currSeg_length/abs(currCurv));
        
        % they have to accelerate in sync to stay on the arc
        % so figure out the acceleration constraints
        if(sVAccel < sWAccel)
            sWAccel = sVAccel;
        else
            sVAccel = sWAccel;
        end
    elseif(currSeg_type == SPIN_IN_PLACE)
        sVAccel = 0;
        
        tWAccel = currMax_w/currMax_accel;
        distWAccel = 0.5*abs(currMax_accel)*tWAccel^2;
        sWAccel = distWAccel/currSeg_length;
    else % don't know what to do with this segment type
        sVAccel = 0;
        sVDecel = 0;
        sWAccel = 0;
        sWDecel = 0;
        return
    end
    
    % in the python code I will have to add a check to make sure nextSeg is
    % not None before accessing its member variables
    if(currSeg_type == LINE && nextSeg_type == LINE)
        % dV is how much velocity has to change from max to next segment
        if(currMax_v <= nextMax_v) % already doing the best possible
            dV = 0;
        else
            if(sign(nextMax_v) == sign(currMax_v)) % this isn't a 180 in velocity
                dV = currMax_v - nextMax_v;
            else % also use this if the next segment is none
                dV = currMax_v; % to do a 180 first the robot should stop at the segment's end
            end
        end
        
        tVDecel = abs(dV/currMax_decel); % times should always be positive
        distVDecel = 0.5*abs(currMax_decel)*tVDecel^2;
        sVDecel = 1 - distVDecel/currSeg_length;
        
        sWDecel = 1; % don't have to decelerate until the end of the segment
    elseif(currSeg_type == LINE && nextSeg_type == ARC)
        % figure out the maximum v so that w constraint in next segment
        % is not violated
        % currently assuming that acceleration constraint applies only to
        % v, not w
        [maxVCmd,maxWCmd] = findMax_v_w(nextMax_v,nextMax_accel*dt,nextCurv); % figure out the constrained velocity and omega
        
        if(maxWCmd > nextMax_w)
            maxWCmd = nextMax_w;
        end
        
        % dV is how much velocity has to change from max to next segment
        if(currMax_v <=maxVCmd) % already doing the best possible
            dV = 0;
        else
            if(sign(maxVCmd) == sign(currMax_v)) % this isn't a 180 in velocity
                dV = currMax_v - maxVCmd;
            else % also use this if the next segment is none
                dV = currMax_v; % to do a 180 first the robot should stop at the segment's end
            end
        end
        
        tVDecel = abs(dV/currMax_decel); % times should always be positive
        distVDecel = 0.5*abs(currMax_decel)*tVDecel^2;
        sVDecel = 1 - distVDecel/currSeg_length;
        
        sWDecel = 1;
    elseif(currSeg_type == ARC && nextSeg_type == ARC)
        [currVCmd,currWCmd] = findMax_v_w(currMax_v,currMax_w,currCurv);
        [nextVCmd,nextWCmd] = findMax_v_w(nextMax_v,nextMax_w,nextCurv);
        
        if(currVCmd <= nextVCmd)
            dV = 0;
        else
            if(sign(currVCmd) == sign(nextVCmd)) % curvature determines w
                dV = currVCmd - nextVCmd;
            else
                dV = currVCmd;
            end
        end
        
        tVDecel = abs(dV/currMax_decel);
        distVDecel = 0.5*abs(currMax_decel)*tVDecel^2;
        sVDecel = 1-distVDecel/currSeg_length;
        
        if(currWCmd <= nextWCmd)
            dW = 0;
        else
            if(sign(currCurv) == sign(nextCurv))
                dW = currWCmd - nextWCmd;
            else
                dW = currWCmd;
            end
        end
        
        tWDecel = abs(dW/currMax_decel);
        distWDecel = 0.5*abs(currMax_decel)*tWDecel^2;
        sWDecel = 1-distWDecel/(currSeg_length/abs(currCurv));
        
        if(sVDecel > sWDecel)
            sVDecel = sWDecel;
        else
            sWDecel = sVDecel;
        end
    elseif(currSeg_type == ARC && nextSeg_type == LINE)
        [currVCmd,currWCmd] = findMax_v_w(currMax_v,currMax_w,currCurv);
        
        if(currVCmd <= nextMax_v)
            dV = 0;
        else
            if(sign(currVCmd) == sign(nextMax_v))
                dV = currVCmd - nextMax_v;
            else
                dV = currVCmd;
            end
        end
        
        tVDecel = abs(dV/currMax_decel);
        distVDecel = 0.5*abs(currMax_decel)*tVDecel^2;
        sVDecel = 1-distVDecel/currSeg_length;
        
        tWDecel = abs(currWCmd/currMax_decel);
        distWDecel = 0.5*abs(currMax_decel)*tWDecel^2;
        sWDecel = 1-distWDecel/(currSeg_length/abs(currCurv));
        
        if(sVDecel > sWDecel)
            sVDecel = sWDecel;
        else
            sWDecel = sVDecel;
        end
    elseif(currSeg_type == SPIN_IN_PLACE && nextSeg_type == SPIN_IN_PLACE)
        if(currMax_w <= nextMax_w)
            dW = 0;
        else
            if(sign(currCurv) == sign(nextCurv))
                dW = currMax_w - nextMax_w;
            else
                dW = currMax_w;
            end
        end
        
        tWDecel = abs(dW/currMax_decel);
        distWDecel = 0.5*abs(currMax_decel)*tWDecel^2;
        sWDecel = 1-distWDecel/currSeg_length;
        
        sVDecel = 1;
    else
        if(currSeg_type == LINE)
            tVDecel = abs(currMax_v/currMax_decel);
            distVDecel = 0.5*abs(currMax_decel)*tVDecel^2;
            sVDecel = 1-distVDecel/currSeg_length;
            
            sWDecel = 1;
        elseif(currSeg_type == ARC)
            [maxVCmd,maxWCmd] = findMax_v_w(currMax_v,currMax_w,currCurv);
            
            tVDecel = abs(maxVCmd/currMax_decel);
            distVDecel = 0.5*abs(currMax_decel)*tVDecel^2;
            sVDecel = 1-distVDecel/currSeg_length;
            
            tWDecel = abs(maxWCmd/currMax_decel);
            distWDecel = 0.5*abs(currMax_decel)*tWDecel^2;
            sWDecel = 1-distWDecel/(currSeg_length/abs(currCurv));
        elseif(currSeg_type == SPIN_IN_PLACE)
            tWDecel = abs(currMax_w/currMax_decel);
            distWDecel = 0.5*abs(currMax_decel)*tWDecel^2;
            sWDecel = 1-distWDecel/currSeg_length;
            
            sVDecel = 1;
        end
    end
end