function test_computeTrajectory(pathSegs,dt)
    pathSegs(size(pathSegs,1)+1,:) = [pathSegs(size(pathSegs,1),1),0,0,0,0,0,0];
    lastSegV = 0;
    nextSegV = 0;
    lastSegW = 0;
    nextSegW = 0;
    vPlot = figure;
    wPlot = figure;
    for i=1:size(pathSegs)-1
       [sVAccel,sVDecel,sWAccel,sWDecel] = computeTrajectory(dt,pathSegs(i,:),pathSegs(i+1,:));
       figure(vPlot);
       hold on
       plot([i,sVAccel+i],[lastSegV,pathSegs(i,3)],'--b');
       plot([sVAccel+i,sVDecel+i],[pathSegs(i,3),pathSegs(i,3)],'--g');
       if(sign(pathSegs(i,3)) == sign(pathSegs(i+1,3)))
           nextSegV = max([0,pathSegs(i,3) - pathSegs(i+1,3)]);
           if(nextSegV == 0)
               lastSegV = pathSegs(i,3);
               plotDecel = 0;
           else
               lastSegV = nextSegV;
               plotDecel = 1;
           end
       else
           plotDecel = 1;
           nextSegV = 0;
           lastSegV = 0;
       end
       if(plotDecel == 1)
           plot([sVDecel+i,i+1],[pathSegs(i,3),nextSegV],'--r');
       end
       
       figure(wPlot);
       hold on
       plot([i,sWAccel+i],[lastSegW,pathSegs(i,4)],'--b');
       plot([sWAccel+i,sVDecel+i],[pathSegs(i,4),pathSegs(i,4)],'--g');
       plot([sWDecel+i,i+1],[pathSegs(i,4),nextSegW],'--r');
    end
end