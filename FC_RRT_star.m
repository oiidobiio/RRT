function Path = FC_RRT_star(startPoint,axisStart,axisLWH,goalPoint,mapInfo)
%% 变量定义
calcuDis = @(x,y)  sqrt((x(1)-y(1))^2+(x(2)-y(2))^2); %计算两点间的距离
iterMax = 2000;   %最大迭代次数
iter = 0;   %当前迭代次数
step = 2;  %步长
count = 1;  %计数器
count_ = 1;  %计数器
Thr = 5;   %阈值
RtoNeib = 20;
findpath = 0; 
pathfind = 0;
l_draw_newlist = [];
pt_draw_newlist = [];
l_draw_createlist = [];
pt_draw_createlist = [];
goal_index = count;
D_dich = 1;
new_totalcost = 1000;


%构建树
S.x(1) = startPoint(1);
S.y(1) = startPoint(2);
S.prex(1) = startPoint(1);
S.prey(1) = startPoint(2);
S.totalcost(1) = 0;
S.pre(1) = 0;


E.x(1) = goalPoint(1);
E.y(1) = goalPoint(2);
E.prex(1) = goalPoint(1);
E.prey(1) = goalPoint(2);
E.totalcost(1) = 0;
E.pre(1) = 0;


while iter < iterMax
    iter = iter + 1;
    findpath = 0;

    %% 在空间中随机采样
    newCoor = samplePoint(axisStart,axisLWH,goalPoint,mapInfo);
      
    %% 寻找树上最近点
    [nearCoor,preIndex] = findNearPoint(newCoor,S);
    
     %% 按照指定步长生成新的扩展点
%    newCoor = expandPoint(nearCoor,randCoor,step);

     %% 得到临时父节点的路径代价
%     temp_parent = preIndex;
%     temp_cost = calcuDis(newCoor,nearCoor) + S.totalcost(temp_parent);

    %% 碰撞检测    
    contactFlag = iscontact(mapInfo,nearCoor,newCoor,calcuDis(newCoor,nearCoor));
%     %% 计时
%     if pathfind
%         t = t + toc(t1);
%         if t >= 40
%             disp(t)
%             break;
%         end
%     end    
    if contactFlag %若其中一个返回值不为0 则发生碰撞，结束，重新循环        
        continue;
    end
    %% 找到障碍物最近点
    [x_reachest,x_preIndex,p_reachest,p_preIndex,x_cost,p_cost] = findreachest_C(nearCoor,newCoor,preIndex,S,mapInfo);
    
    %% 利用二分法创建靠近障碍物节点
    [x_create,x_cost] = createnode_C(x_reachest,S,p_reachest,newCoor,mapInfo,D_dich,x_cost,p_cost);
    
    %% 判断是否将创建的节点 x_create 纳入树中
    if x_create
%         count = count + 1;
%         S.x(count) = newCoor(1);
%         S.y(count) = newCoor(2);
%         S.prex(count) = x_create(1);
%         S.prey(count) = x_create(2);
%         S.totalcost(count) = x_cost + calcuDis(newCoor,x_create);
%         S.pre(count) = x_preIndex;
%         
%         S.x(x_preIndex) = x_create(1);
%         S.y(x_preIndex) = x_create(2);
%         S.prex(x_preIndex) = p_reachest(1);
%         S.prey(x_preIndex) = p_reachest(2);
%         S.totalcost(x_preIndex) = x_cost;
%         S.pre(x_preIndex) = p_preIndex;

        count = count + 1;
        S.x(count) = x_create(1);
        S.y(count) = x_create(2);
        S.prex(count) = p_reachest(1);
        S.prey(count) = p_reachest(2);
        S.totalcost(count) = x_cost;
        S.pre(count) = p_preIndex;

        count = count + 1;
        S.x(count) = newCoor(1);
        S.y(count) = newCoor(2);
        S.prex(count) = x_create(1);
        S.prey(count) = x_create(2);
        S.totalcost(count) = x_cost + calcuDis(newCoor,x_create);
        S.pre(count) = count - 1;  

        l_draw_new = plot([S.prex(count), newCoor(1)], [S.prey(count), newCoor(2)],'b','Linewidth',0.5);
        l_draw_create = plot([S.prex(x_preIndex), x_create(1)], [S.prey(x_preIndex), x_create(2)],'b','Linewidth',0.5);

        pt_draw_new = plot(newCoor(1),newCoor(2),'ko','MarkerSize', 2, 'MarkerFaceColor','k');
        pt_draw_create = plot(x_create(1),x_create(2),'ko','MarkerSize', 2, 'MarkerFaceColor','k');
    
        l_draw_newlist = [l_draw_newlist l_draw_new];
        pt_draw_newlist = [pt_draw_newlist pt_draw_new]; %生成新点点集   

        
        l_draw_createlist = [l_draw_createlist l_draw_create];
        pt_draw_createlist = [pt_draw_createlist pt_draw_create]; %生成新点点集 
        
%         delete(pt_draw_newlist(x_preIndex));
%         pt_draw_newlist(x_preIndex) = pt_draw_create;
%         
%         delete(l_draw_newlist(x_preIndex));
%         l_draw_newlist(x_preIndex) = l_draw_create;
        
    else
        count = count + 1;
        S.x(count) = newCoor(1);
        S.y(count) = newCoor(2);
        S.prex(count) = x_reachest(1);
        S.prey(count) = x_reachest(2);
        S.totalcost(count) = calcuDis(newCoor,x_reachest) + S.totalcost(x_preIndex);
        S.pre(count) = x_preIndex;
         
        l_draw_new = plot([S.prex(count), newCoor(1)], [S.prey(count), newCoor(2)],'b','Linewidth',0.5);
        pt_draw_new = plot(newCoor(1),newCoor(2),'ko','MarkerSize', 2, 'MarkerFaceColor','k');
        
        l_draw_newlist = [l_draw_newlist l_draw_new];
        pt_draw_newlist = [pt_draw_newlist pt_draw_new];
    end    
    %% 寻找树上最近点
    [nearCoor_E,preIndex_E] = findNearPoint(newCoor,E);

    %% 按照指定步长生成新的扩展点
%     newCoor_E = expandPoint(nearCoor_E,newCoor,step);
    newCoor_E = newCoor;
%     temp_parent = preIndex_E;
%     temp_cost_E = calcuDis(newCoor_E,nearCoor_E) + S.totalcost(temp_parent);
    %% 碰撞检测    
    contactFlag = iscontact(mapInfo,newCoor_E,nearCoor_E,calcuDis(nearCoor_E,newCoor_E));
    if contactFlag %若其中一个返回值不为0 则发生碰撞，结束，重新循环        
        if count_ <count
            [S,E,count,count_] = swapp(S,E,count,count_);
        end 
        continue;
    end
    %% 找到障碍物最近点
    [x_reachest_E,x_preIndex_E,p_reachest_E,p_preIndex_E,x_cost,p_cost] = findreachest_C(nearCoor_E,newCoor_E,preIndex_E,E,mapInfo);
    
    %% 利用二分法创建靠近障碍物节点
    [x_create_E,x_cost] = createnode_C(x_reachest_E,E,p_reachest_E,newCoor_E,mapInfo,D_dich,x_cost,p_cost);

    %% 判断是否将创建的节点 x_create 纳入树中
    if x_create_E
%         count_ = count_ + 1;
%         E.x(count_) = newCoor_E(1);
%         E.y(count_) = newCoor_E(2);
%         E.prex(count_) = x_create_E(1);
%         E.prey(count_) = x_create_E(2);
%         E.totalcost(count_) = x_cost + calcuDis(newCoor,x_create_E);
%         E.pre(count_) = x_preIndex_E;
%         
%         E.x(x_preIndex_E) = x_create_E(1);
%         E.y(x_preIndex_E) = x_create_E(2);
%         E.prex(x_preIndex_E) = p_reachest_E(1);
%         E.prey(x_preIndex_E) = p_reachest_E(2);
%         E.totalcost(x_preIndex_E) = x_cost;
%         E.pre(x_preIndex_E) = p_preIndex_E;

        count_ = count_ + 1;
        E.x(count_) = x_create_E(1);
        E.y(count_) = x_create_E(2);
        E.prex(count_) = p_reachest_E(1);
        E.prey(count_) = p_reachest_E(2);
        E.totalcost(count_) = x_cost;
        E.pre(count_) = p_preIndex_E;

        count_ = count_ + 1;
        E.x(count_) = newCoor_E(1);
        E.y(count_) = newCoor_E(2);
        E.prex(count_) = x_create_E(1);
        E.prey(count_) = x_create_E(2);
        E.totalcost(count_) = x_cost + calcuDis(newCoor,x_create_E);
        E.pre(count_) = count_ - 1;  
        
        l_draw_new = plot([E.prex(count_), newCoor_E(1)], [E.prey(count_), newCoor_E(2)],'b','Linewidth',0.5);
        l_draw_create = plot([E.prex(x_preIndex_E), x_create_E(1)], [E.prey(x_preIndex_E), x_create_E(2)],'b','Linewidth',0.5);

        pt_draw_new = plot(newCoor_E(1),newCoor_E(2),'ko','MarkerSize', 2, 'MarkerFaceColor','k');
        pt_draw_create = plot(x_create_E(1),x_create_E(2),'ko','MarkerSize', 2, 'MarkerFaceColor','k');
    
        l_draw_newlist = [l_draw_newlist l_draw_new];
        pt_draw_newlist = [pt_draw_newlist pt_draw_new]; %生成新点点集   

        
        l_draw_createlist = [l_draw_createlist l_draw_create];
        pt_draw_createlist = [pt_draw_createlist pt_draw_create]; %生成新点点集 
        
%         delete(pt_draw_newlist(x_preIndex));
%         pt_draw_newlist(x_preIndex) = pt_draw_create;
%         
%         delete(l_draw_newlist(x_preIndex));
%         l_draw_newlist(x_preIndex) = l_draw_create;
        
    else
        count_ = count_ + 1;
        E.x(count_) = newCoor_E(1);
        E.y(count_) = newCoor_E(2);
        E.prex(count_) = x_reachest_E(1);
        E.prey(count_) = x_reachest_E(2);
        E.totalcost(count_) = calcuDis(newCoor_E,x_reachest_E) + E.totalcost(x_preIndex_E);
        E.pre(count_) = x_preIndex_E;
         
        l_draw_new = plot([E.prex(count_), newCoor_E(1)], [E.prey(count_), newCoor_E(2)],'b','Linewidth',0.5);
        pt_draw_new = plot(newCoor_E(1),newCoor_E(2),'ko','MarkerSize', 2, 'MarkerFaceColor','k');
        
        l_draw_newlist = [l_draw_newlist l_draw_new];
        pt_draw_newlist = [pt_draw_newlist pt_draw_new];
    end
    

    %% 判断是否到达终点，构成路径
    size_S = size(S.x,2);
    size_E = size(E.x,2);
    if S.x(size_S) == E.x(size_E) && S.y(size_S) == E.y(size_E)
        findpath = 1;
        clear path;
        if S.x(1) == startPoint(1)
            E_index = count_;
            S_index = count;
            index = count_;
            j = 1;
            while 1
                path.x(j) = E.x(index);
                path.y(j) = E.y(index);
                index = E.pre(index);
                j = j + 1;
                if index == 0
                    break;
                end
            end
            index = count;
            path.x = flip(path.x);
            path.y = flip(path.y);
            j = j - 1;
            while 1
                path.x(j) = S.x(index);
                path.y(j) = S.y(index);
                index = S.pre(index);
                if index == 0
                    break;
                end
                j = j + 1;
            end
            
        else
            index = count;
            j = 1;
            while 1
                path.x(j) = S.x(index);
                path.y(j) = S.y(index);
                index = S.pre(index);
                j = j + 1;
                if index == 0
                    break;
                end
            end
            index = count_;
            path.x = flip(path.x);
            path.y = flip(path.y);
            j = j - 1;
            while 1
                path.x(j) = E.x(index);
                path.y(j) = E.y(index);
                index = E.pre(index);
                if index == 0
                    break;
                end
                j = j + 1;
            end
 
        end
        
    end

    %% 剪枝 重新划线
    if findpath
   %% 以newCoor为圆心，以RtoNeib为半径，作圆，搜索节点
    S_distoNewlist = []; 
    S_nearIndexlist = [];
    E_distoNewlist = []; 
    E_nearIndexlist = [];   
    S_createIndex = S.pre(count);
    E_createIndex = E.pre(count_);
    
    for i = 1:count
        distoNew = sqrt((newCoor(1) - S.x(i))^2 + (newCoor(2) - S.y(i))^2 );
        if(distoNew < RtoNeib && i~= preIndex && i ~= x_preIndex && i ~= count)
            S_distoNewlist = [S_distoNewlist distoNew];
            S_nearIndexlist = [S_nearIndexlist i];
        end
    end
    
    for i = 1:count_
        distoNew = sqrt((newCoor_E(1) - E.x(i))^2 + (newCoor_E(2) - E.y(i))^2 );
        if(distoNew < RtoNeib && i~= preIndex_E && i ~= x_preIndex_E && i ~= count_)
            E_distoNewlist = [E_distoNewlist distoNew];
            E_nearIndexlist = [E_nearIndexlist i];
        end
    end   

    for rewire_index = 1 : length(S_nearIndexlist)
        S_newcost = S.totalcost(count) + S_distoNewlist(rewire_index);
        if x_create
            S_createcost = S.totalcost(S_createIndex) + sqrt((x_create(1) - S.x(rewire_index))^2 + (x_create(2) - S.y(rewire_index))^2 );
            if(S_createcost < S.totalcost(S_nearIndexlist(rewire_index)))
            neib(1) = S.x(S_nearIndexlist(rewire_index));
            neib(2) = S.y(S_nearIndexlist(rewire_index));
            contactFlag = iscontact(mapInfo,neib,x_create,calcuDis(neib,x_create));
            if contactFlag %若其中一个返回值不为0 则发生碰撞，结束，重新循环 
                if(S_newcost < S.totalcost(S_nearIndexlist(rewire_index)))
                    neib(1) = S.x(S_nearIndexlist(rewire_index));
                    neib(2) = S.y(S_nearIndexlist(rewire_index));
                    contactFlag = iscontact(mapInfo,neib,newCoor,calcuDis(neib,newCoor));
                    if contactFlag %若其中一个返回值不为0 则发生碰撞，结束，重新循环 
                        continue;
                    end 
                    S.totalcost(S_nearIndexlist(rewire_index)) = S_newcost;
                    S.pre(S_nearIndexlist(rewire_index)) = count;
                    S.prex(S_nearIndexlist(rewire_index)) = newCoor(1);
                    S.prey(S_nearIndexlist(rewire_index)) = newCoor(2); 
                    l_draw_newlist(S_nearIndexlist(rewire_index)) = plot([S.x(S_nearIndexlist(rewire_index)), newCoor(1)], [S.y(S_nearIndexlist(rewire_index)), newCoor(2)],'r','Linewidth',2);
                end
                continue;
            end 
            S.totalcost(S_nearIndexlist(rewire_index)) = S_createcost;
            S.pre(S_nearIndexlist(rewire_index)) = S_createIndex;
            S.prex(S_nearIndexlist(rewire_index)) = x_create(1);
            S.prey(S_nearIndexlist(rewire_index)) = x_create(2); 
            l_draw_newlist(S_nearIndexlist(rewire_index)) = plot([S.x(S_nearIndexlist(rewire_index)), x_create(1)], [S.y(S_nearIndexlist(rewire_index)), x_create(2)],'b','Linewidth',2);
            end
        elseif(S_newcost < S.totalcost(S_nearIndexlist(rewire_index)))
            neib(1) = S.x(S_nearIndexlist(rewire_index));
            neib(2) = S.y(S_nearIndexlist(rewire_index));
            contactFlag = iscontact(mapInfo,neib,newCoor,calcuDis(neib,newCoor));
            if contactFlag %若其中一个返回值不为0 则发生碰撞，结束，重新循环 
                continue;
            end 
            S.totalcost(S_nearIndexlist(rewire_index)) = S_newcost;
            S.pre(S_nearIndexlist(rewire_index)) = count;
            S.prex(S_nearIndexlist(rewire_index)) = newCoor(1);
            S.prey(S_nearIndexlist(rewire_index)) = newCoor(2); 
            l_draw_newlist(S_nearIndexlist(rewire_index)) = plot([S.x(S_nearIndexlist(rewire_index)), newCoor(1)], [S.y(S_nearIndexlist(rewire_index)), newCoor(2)],'r','Linewidth',2);
        end
    end
 
    
    for rewire_index = 1 : length(E_nearIndexlist)
        E_newcost = E.totalcost(count_) + E_distoNewlist(rewire_index);
        if x_create_E
            E_createcost = E.totalcost(E_createIndex) + sqrt((x_create_E(1) - E.x(rewire_index))^2 + (x_create_E(2) - E.y(rewire_index))^2 );
            if(E_createcost < E.totalcost(E_nearIndexlist(rewire_index)))
            neib(1) = E.x(E_nearIndexlist(rewire_index));
            neib(2) = E.y(E_nearIndexlist(rewire_index));
            contactFlag = iscontact(mapInfo,neib,x_create_E,calcuDis(neib,x_create_E));
            if contactFlag %若其中一个返回值不为0 则发生碰撞，结束，重新循环 
                if(E_newcost < E.totalcost(E_nearIndexlist(rewire_index)))
                    neib(1) = E.x(E_nearIndexlist(rewire_index));
                    neib(2) = E.y(E_nearIndexlist(rewire_index));
                    contactFlag = iscontact(mapInfo,neib,newCoor,calcuDis(neib,newCoor));
                    if contactFlag %若其中一个返回值不为0 则发生碰撞，结束，重新循环 
                        continue;
                    end 
                    E.totalcost(E_nearIndexlist(rewire_index)) = E_newcost;
                    E.pre(E_nearIndexlist(rewire_index)) = count_;
                    E.prex(E_nearIndexlist(rewire_index)) = newCoor_E(1);
                    E.prey(E_nearIndexlist(rewire_index)) = newCoor_E(2); 
                    l_draw_newlist(E_nearIndexlist(rewire_index)) = plot([E.x(E_nearIndexlist(rewire_index)), newCoor(1)], [E.y(E_nearIndexlist(rewire_index)), newCoor(2)],'r','Linewidth',2);
                end
                continue;
            end 
            E.totalcost(E_nearIndexlist(rewire_index)) = E_createcost;
            E.pre(E_nearIndexlist(rewire_index)) = E_createIndex;
            E.prex(E_nearIndexlist(rewire_index)) = x_create_E(1);
            E.prey(E_nearIndexlist(rewire_index)) = x_create_E(2); 
            l_draw_newlist(E_nearIndexlist(rewire_index)) = plot([E.x(E_nearIndexlist(rewire_index)), x_create_E(1)], [E.y(E_nearIndexlist(rewire_index)), x_create_E(2)],'b','Linewidth',2);
            end
        elseif(E_newcost < E.totalcost(E_nearIndexlist(rewire_index)))
            neib(1) = E.x(E_nearIndexlist(rewire_index));
            neib(2) = E.y(E_nearIndexlist(rewire_index));
            contactFlag = iscontact(mapInfo,neib,newCoor,calcuDis(neib,newCoor));
            if contactFlag %若其中一个返回值不为0 则发生碰撞，结束，重新循环 
                continue;
            end 
            E.totalcost(E_nearIndexlist(rewire_index)) = E_newcost;
            E.pre(E_nearIndexlist(rewire_index)) = count_;
            E.prex(E_nearIndexlist(rewire_index)) = newCoor(1);
            E.prey(E_nearIndexlist(rewire_index)) = newCoor(2); 
            l_draw_newlist(E_nearIndexlist(rewire_index)) = plot([E.x(E_nearIndexlist(rewire_index)), newCoor(1)], [E.y(E_nearIndexlist(rewire_index)), newCoor(2)],'r','Linewidth',2);
        end
    end
    
    end
    
    %% rewire   
%     pause(0.1); 
    flag = 0;
    if findpath && pathfind 
        totalcost = 0;
        for j = 2 : size(path.x,2)
            cur_dis = sqrt((path.x(j)-path.x(j-1))^2+(path.y(j)-path.y(j-1))^2);
            totalcost = totalcost + cur_dis;
        end
        
        if new_totalcost > totalcost
            flag = 1;
            new_path = path;
            new_totalcost = totalcost;
            for delete_index = 1 : length(pt_draw_newlist)%这个地方，对每一个元素进行遍历之后都删除是什么意思？
                delete(pt_draw_newlist(delete_index));
            end
            
            
            for j = 2 : size(new_path.x,2)
                pt_draw_new = plot([new_path.x(j);new_path.x(j-1);],[new_path.y(j);new_path.y(j-1);],'g', 'Linewidth', 0.5);
                pt_draw_newlist = [pt_draw_newlist pt_draw_new];
            end
        end
        
    end
    

    %% 画出路径
    if findpath && pathfind 
         for j = 2 : size(new_path.x,2)
            pt_draw_new = plot([new_path.x(j);new_path.x(j-1);],[new_path.y(j);new_path.y(j-1);],'g', 'Linewidth', 0.5);
            pt_draw_newlist = [pt_draw_newlist pt_draw_new];
        end 
    end
   
    if findpath
        pathfind = 1;
        totalfai = 0;
        Q.x = flip(path.x);
        Q.y = flip(path.y);
        for i = 2 : size(Q.x,2) - 1
            a = [Q.x(i) - Q.x(i - 1) Q.y(i) - Q.y(i - 1)];
            b = [Q.x(i + 1) - Q.x(i) Q.y(i + 1) - Q.y(i)];
            ab = dot(a,b);
            g = a.*a;
            gh = sum(g);
            aa = sqrt(gh);
            g = b.*b;
            gh = sum(g);
            bb = sqrt(gh);
            fai = acos(ab/(aa*bb));
            totalfai = totalfai + fai;
        end 
        totalcost = 0;
        for j = 2 : size(path.x,2)
            cur_dis = sqrt((path.x(j)-path.x(j-1))^2+(path.y(j)-path.y(j-1))^2);
            totalcost = totalcost + cur_dis;
        end
        for j = 2 : size(path.x,2)
            pt_draw_new = plot([path.x(j);path.x(j-1);],[path.y(j);path.y(j-1);],'b', 'Linewidth', 0.5);
            pt_draw_newlist = [pt_draw_newlist pt_draw_new];
        end 
    end
 

%     if findpath
%         pathfind = 1;
%         for j = 2 : size(path.x,2)
%             pt_draw_new = plot([path.x(j);path.x(j-1);],[path.y(j);path.y(j-1);],'g', 'Linewidth', 0.5);
%             pt_draw_newlist = [pt_draw_newlist pt_draw_new];
%         end
%         for delete_index = 1 : length(pt_draw_newlist)%这个地方，对每一个元素进行遍历之后都删除是什么意思？
%              delete(pt_draw_newlist(delete_index));
%         end
%     end    
    
    if flag
        settimer = toc;
        disp("start")
        disp(settimer)
        disp(new_totalcost)
        disp(totalfai)
    end

end

Path(1,3) = totalcost;
Path(1,4) = count + count_;

for i = 1 : size(new_path.x,2)
    Path(i,1) = new_path.x(i);
    Path(i,2) = new_path.y(i);
end
for j = 2 : size(new_path.x,2)
    pt_draw_new = plot([new_path.x(j);new_path.x(j-1);],[new_path.y(j);new_path.y(j-1);],'k', 'Linewidth', 0.5);
    pt_draw_newlist = [pt_draw_newlist pt_draw_new];
end
end

