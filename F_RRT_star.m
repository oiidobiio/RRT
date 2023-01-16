function Path = F_RRT_star(startPoint,axisStart,axisLWH,goalPoint,mapInfo)
%% F_RRT_star算法寻找路径点
 
%% 变量定义
calcuDis = @(x,y)  sqrt((x(1)-y(1))^2+(x(2)-y(2))^2); %计算两点间的距离
iterMax = 5000;   %最大迭代次数
iter = 0;   %当前迭代次数
step = 2;  %步长
count = 1;  %计数器
Thr = 5;   %阈值
RtoNeib = 20;
findpath = 0; 
l_draw_newlist = [];
pt_draw_newlist = [];
l_draw_createlist = [];
pt_draw_createlist = [];
goal_index = count;
D_dich = 2;
new_totalcost = 1000;

% time1 = 0;
% time2 = 0;
% time3 = 0;
% time4 = 0;
%  time5 = 0;
%构建树
T.x(1) = startPoint(1);
T.y(1) = startPoint(2);
T.prex(1) = startPoint(1);
T.prey(1) = startPoint(2);
T.totalcost(1) = 0;
T.pre(1) = 0;

while iter < iterMax
%     t1 = tc;i
    iter = iter + 1;
    %% 在空间中随机采样
    newCoor = samplePoint(axisStart,axisLWH,goalPoint,mapInfo);
    
    %% 寻找树上最近点
    [nearCoor,preIndex] = findNearPoint(newCoor,T);
    
     %% 按照指定步长生成新的扩展点
    %newCoor = expandPoint(nearCoor,randCoor,step);

     %% 得到临时父节点的路径代价
    temp_parent = preIndex;
    temp_cost = calcuDis(newCoor,nearCoor) + T.totalcost(temp_parent);
    %% 碰撞检测    
    contactFlag = iscontact(mapInfo,nearCoor,newCoor,calcuDis(newCoor,nearCoor));
%     time1 = time1 + toc(t1);
    if contactFlag %若其中一个返回值不为0 则发生碰撞，结束，重新循环        
        continue;
    end
    

    %% 找到障碍物最近点
%     t2 = tic;
    x_reachest = nearCoor;
    x_preIndex = preIndex;
    x_cost = T.totalcost(preIndex);  %障碍物最近点消耗
    
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
%     time2 = time2 + toc(t2);    
    %% 利用二分法创建靠近障碍物节点
%     t5 = tic;
    x_allow = x_reachest;
    if x_reachest ~= startPoint
        
        x_forbid = p_reachest;
        while calcuDis(x_allow,x_forbid) > D_dich
            x_mid = (x_allow + x_forbid)/2;
            contactFlag = iscontact(mapInfo,newCoor,x_mid,calcuDis(newCoor,x_mid));
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
%     time5 = time5 + toc(t5);

    %% 判断是否将创建的节点 x_create 纳入树中
%     t3 = tic;
    if x_create
%         count = count + 1;
%         T.x(count) = newCoor(1);
%         T.y(count) = newCoor(2);
%         T.prex(count) = x_create(1);
%         T.prey(count) = x_create(2);
%         T.totalcost(count) = x_cost + calcuDis(newCoor,x_mid);
%         T.pre(count) = x_preIndex;
%         
%         T.x(x_preIndex) = x_create(1);
%         T.y(x_preIndex) = x_create(2);
%         T.prex(x_preIndex) = p_reachest(1);
%         T.prey(x_preIndex) = p_reachest(2);
%         T.totalcost(x_preIndex) = x_cost;
%         T.pre(x_preIndex) = p_preIndex;

        count = count + 1;
        T.x(count) = x_create(1);
        T.y(count) = x_create(2);
        T.prex(count) = p_reachest(1);
        T.prey(count) = p_reachest(2);
        T.totalcost(count) = x_cost;
        T.pre(count) = p_preIndex;

        count = count + 1;
        T.x(count) = newCoor(1);
        T.y(count) = newCoor(2);
        T.prex(count) = x_create(1);
        T.prey(count) = x_create(2);
        T.totalcost(count) = x_cost + calcuDis(newCoor,x_create);
        T.pre(count) = count - 1;        

        l_draw_new = plot([T.prex(count), newCoor(1)], [T.prey(count), newCoor(2)],'b','Linewidth',0.5);
        l_draw_create = plot([T.prex(x_preIndex), x_create(1)], [T.prey(x_preIndex), x_create(2)],'b','Linewidth',0.5);

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
        T.x(count) = newCoor(1);
        T.y(count) = newCoor(2);
        T.prex(count) = x_reachest(1);
        T.prey(count) = x_reachest(2);
        T.totalcost(count) = calcuDis(newCoor,x_reachest) + T.totalcost(x_preIndex);
        T.pre(count) = x_preIndex;
         
        l_draw_new = plot([T.prex(count), newCoor(1)], [T.prey(count), newCoor(2)],'b','Linewidth',0.5);
        pt_draw_new = plot(newCoor(1),newCoor(2),'ko','MarkerSize', 2, 'MarkerFaceColor','k');
        
        l_draw_newlist = [l_draw_newlist l_draw_new];
        pt_draw_newlist = [pt_draw_newlist pt_draw_new];
    end
%     time3 = time3 + toc(t3);
    %% 剪枝 重新划线
    if findpath
   %% 以newCoor为圆心，以RtoNeib为半径，作圆，搜索节点
    distoNewlist = [];    % 每次循环要把队列清空
    nearIndexlist = [];
    for i = 1:count
        distoNew = sqrt((newCoor(1) - T.x(i))^2 + (newCoor(2) - T.y(i))^2 );
        if(distoNew < RtoNeib)
            distoNewlist = [distoNewlist distoNew];
            nearIndexlist = [nearIndexlist i];
        end
    end
        
        
    for rewire_index = 1 : length(nearIndexlist)
        if(nearIndexlist(rewire_index) ~= preIndex && nearIndexlist(rewire_index) ~= x_preIndex)
            new_cost = temp_cost + distoNewlist(rewire_index);  %计算neib经过newCoor的到起点的路径代价
            if(new_cost < T.totalcost(nearIndexlist(rewire_index))) %判断其与之前树上的路径代价大小，判断是否需要剪枝
                neib(1) = T.x(nearIndexlist(rewire_index));
                neib(2) = T.y(nearIndexlist(rewire_index));
                contactFlag = iscontact(mapInfo,neib,newCoor,calcuDis(neib,newCoor));
                if contactFlag %若其中一个返回值不为0 则发生碰撞，结束，重新循环 
                    continue;
                end 
                T.totalcost(nearIndexlist(rewire_index)) = new_cost;
                T.pre(nearIndexlist(rewire_index)) = count;
                T.prex(nearIndexlist(rewire_index)) = newCoor(1);
                T.prey(nearIndexlist(rewire_index)) = newCoor(2);    
        
                l_draw_newlist(nearIndexlist(rewire_index)) = plot([T.x(nearIndexlist(rewire_index)), newCoor(1)], [T.y(nearIndexlist(rewire_index)), newCoor(2)],'b','Linewidth',0.5);                
            end
        end
    end
    end
    %% 判断是否到达终点，构成路径
%     t4 = tic;
%   [goalflag,t,s,bef] = isgoal(newCoor,x_reachest,x_create,Thr,goalPoint);
    distoG = calcuDis(goalPoint,newCoor);

    
    if distoG <= Thr
        findpath = 1;
        count = count + 1;
        goal_index = count;
        T.x(count) = goalPoint(1);
        T.y(count) = goalPoint(2);
        T.prex(count) = newCoor(1);
        T.prey(count) = newCoor(2);
        T.pre(count) = count - 1;
        T.totalcost(count) = T.totalcost(count - 1) + distoG;  
    end
    
    

    if(findpath == 1)
        clear path
        path.x(1) = goalPoint(1);
        path.y(1) = goalPoint(2);
        index = T.pre(goal_index);
        j = 2;
        while 1
            path.x(j) = T.x(index);
            path.y(j) = T.y(index);
            index = T.pre(index);
            
            if(index == 0)
                break; 
            end
            j = j + 1;
        end
        for delete_index = 1 : length(pt_draw_newlist)%这个地方，对每一个元素进行遍历之后都删除是什么意思？
             delete(pt_draw_newlist(delete_index));
        end
        
        totalcost = 0;
        
        for j = 2 : size(path.x,2)
            cur_dis = sqrt((path.x(j)-path.x(j-1))^2+(path.y(j)-path.y(j-1))^2);
            totalcost = totalcost + cur_dis;
        end
                
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
 
        if new_totalcost > totalcost
            new_totalcost = totalcost;
            new_path = path;
            settimer = toc;
            disp("start")
            disp(settimer)
            disp(new_totalcost)
            disp(totalfai)
        end
        
        for j = 2 : size(new_path.x,2)
            pt_draw_new = plot([new_path.x(j);new_path.x(j-1);],[new_path.y(j);new_path.y(j-1);],'g', 'Linewidth', 0.5);
            pt_draw_newlist = [pt_draw_newlist pt_draw_new];
        end

    end
%     time4 = time4 + toc(t4);
end

    Path(1,1) = goalPoint(1); 
    Path(1,2) = goalPoint(2);  
    index = T.pre(goal_index);
    Path(1,3) = totalcost;
    Path(1,4) = goal_index;
    j = 2;    
    while 1        
        Path(j,1) = T.x(index);
        Path(j,2) = T.y(index);
        index = T.pre(index);
            
        if(index == 0)
            break; 
        end
        j = j + 1;
    end
    
    for delete_index = 1 : length(pt_draw_newlist)%这个地方，对每一个元素进行遍历之后都删除是什么意思？
         delete(pt_draw_newlist(delete_index));
    end
        
    for j = 2 : size(Path,1)
        pt_draw_new = plot([path.x(j);path.x(j-1);],[path.y(j);path.y(j-1);],'g', 'Linewidth', 0.5);
        pt_draw_newlist = [pt_draw_newlist pt_draw_new];
    end
end

