function [x_reachest,x_preIndex,p_reachest,p_preIndex,x_cost,p_cost] = findreachest_C(nearCoor,newCoor,preIndex,T,mapInfo)
calcuDis = @(x,y)  sqrt((x(1)-y(1))^2+(x(2)-y(2))^2);
    x_reachest = nearCoor;
    x_preIndex = preIndex;
    x_cost = T.totalcost(preIndex);
    
    p_reachest(1) = T.prex(preIndex);
    p_reachest(2) = T.prey(preIndex);
    p_preIndex = T.pre(preIndex);
    if p_preIndex
        p_cost = T.totalcost(p_preIndex);
    else
        p_cost = 0;
    end
    checkPoint(1) = T.x(1);
    checkPoint(2) = T.y(1);
    while x_reachest ~= checkPoint
        contactFlag = iscontact(mapInfo,newCoor,p_reachest,calcuDis(newCoor,p_reachest));
        if contactFlag %若其中一个返回值不为0 则发生碰撞，结束，重新循环        
            break;
        end
        x_reachest = p_reachest;
        x_preIndex = p_preIndex;
        x_cost = p_cost;
        
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

