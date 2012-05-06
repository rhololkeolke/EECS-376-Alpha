function [v_cmd,w_cmd] = findMax_v_w(maxV,maxW,rho)
    v_cmd = maxV;
    w_cmd = rho*v_cmd;

    if(w_cmd > maxW) % w is binding
        w_cmd = maxW;
        v_cmd = w_cmd/rho;
    end
end