function [x_create,x_cost] = createnode(x_reachest,startPoint,p_reachest,newCoor,mapInfo,D_dich,x_cost,p_cost)
calcuDis = @(x,y)  sqrt((x(1)-y(1))^2+(x(2)-y(2))^2);
    x_allow = x_reachest;
    if x_reachest ~= startPoint
        
        x_forbid = p_reachest;
        while calcuDis(x_allow,x_forbid) > D_dich
            x_mid = (x_allow + x_forbid)/2;
            contactFlag = iscontact(mapInfo,x_mid,newCoor,calcuDis(newCoor,x_mid));
            if contactFlag %若其中一个返回值不为0 则发生碰撞       
                x_forbid = x_mid;            
            else
                x_allow = x_mid;
            end
        end
        
        x_forbid = newCoor;
        while calcuDis(x_allow,x_forbid) > D_dich
            x_mid = (x_allow + x_forbid)/2;
            contactFlag = iscontact(mapInfo,p_reachest,x_mid,calcuDis(p_reachest,x_mid));
            if contactFlag %若其中一个返回值不为0 则发生碰撞       
                x_forbid = x_mid;            
            else
                x_allow = x_mid;
            end 
        end
    end
    

    if x_allow ~= x_reachest
        x_create = x_allow;
        x_cost = calcuDis(p_reachest,x_create) + p_cost;
    else
        x_create = [];
    end
end

