function [x_reachest,x_preIndex,p_reachest,p_preIndex,x_cost,p_cost] = findreachest(nearCoor,newCoor,preIndex,T,startPoint,mapInfo,temp_cost)
calcuDis = @(x,y)  sqrt((x(1)-y(1))^2+(x(2)-y(2))^2);
    x_reachest = nearCoor;
    x_preIndex = preIndex;
    x_cost = temp_cost;
    
    p_reachest(1) = T.prex(preIndex);
    p_reachest(2) = T.prey(preIndex);
    p_preIndex = T.pre(preIndex);
    if p_preIndex
        p_cost = T.totalcost(p_preIndex);
    else
        p_cost = 0;
    end
    
    while x_reachest ~= startPoint
        contactFlag = iscontact(mapInfo,newCoor,p_reachest,calcuDis(newCoor,p_reachest));
        if contactFlag %若其中一个返回值不为0 则发生碰撞，结束，重新循环        
            break;
        end
        x_reachest = p_reachest;
        x_preIndex = p_preIndex;
        x_cost = p_cost + calcuDis(newCoor,p_reachest);
        
        p_reachest(1) = T.prex(p_preIndex);
        p_reachest(2) = T.prey(p_preIndex);
        p_preIndex = T.pre(p_preIndex);
        if p_preIndex
            p_cost = T.totalcost(p_preIndex);
        else
            p_cost = 0;
        end
    end
end

