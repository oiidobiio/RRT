function Path = RRT_C(startPoint,axisStart,axisLWH,goalPoint,mapInfo)
%% RRT�㷨Ѱ��·����
 
%% ��������
calcuDis = @(x,y)  sqrt((x(1)-y(1))^2+(x(2)-y(2))^2); %���������ľ���
iterMax = 1000;   %����������
iter = 0;   %��ǰ��������
step = 5;  %����
count = 1;  %������
count_ = 1;
Thr = 5;   %��ֵ
findpath = 0;
l_draw_newlist = [];
pt_draw_newlist = []; 

%������
S.x(1) = startPoint(1);
S.y(1) = startPoint(2);
S.prex(1) = startPoint(1);
S.prey(1) = startPoint(2);
S.pre(1) = 0;

E.x(1) = goalPoint(1);
E.y(1) = goalPoint(2);
E.prex(1) = goalPoint(1);
E.prey(1) = goalPoint(2);
E.pre(1) = 0;

while iter < iterMax
    iter = iter + 1;
    
    %% �ڿռ����������
    randCoor = samplePoint(axisStart,axisLWH,goalPoint,mapInfo);
    
    %% Ѱ��������ϵ������
    [nearCoor,preIndex] = findNearPoint(randCoor,S);

    %% ����ָ�����������µ���չ��
    newCoor = expandPoint(nearCoor,randCoor,step);
    
    %% ��ײ���    
    contactFlag = iscontact(mapInfo,nearCoor,newCoor,step);
    if contactFlag %������һ������ֵ��Ϊ0 ������ײ������������ѭ�� 
        %�˴������߼����󣬵����޷����е����潻����������ѭ��������
        if count_ <count
            [S,E,count,count_] = swapp_c(S,E,count,count_);
        end
        continue;
    end
    %% �����µ㵽�������
    count = count+1;
    S.x(count) = newCoor(1);
    S.y(count) = newCoor(2);
    S.prex(count) = nearCoor(1);
    S.prey(count) = nearCoor(2);
    S.pre(count) = preIndex;
    
    %���������µ�����
    line([nearCoor(1) newCoor(1)],[nearCoor(2) newCoor(2)],'Color','b','LineWidth',1);  %����ÿһ���µ�
%     pause(0.01);
    %% Ѱ���յ����ϵ������    
    [nearCoor1,preIndex1] = findNearPoint(newCoor,E);
    
    %% ��չ�յ���
    newCoor1 = expandPoint(nearCoor1,newCoor,step);
    
    %% ��ײ���    
    contactFlag = iscontact(mapInfo,nearCoor1,newCoor1,step);
    if contactFlag %������һ������ֵ��Ϊ0 ������ײ������������ѭ��
        %�˴������߼����󣬵����޷����е����潻����������ѭ��������
        if count_ <count
            [S,E,count,count_] = swapp_c(S,E,count,count_);
        end
        continue;
    end
    
    %% �����µ㵽�յ�����
    count_ = count_ + 1;
    E.x(count_) = newCoor1(1);
    E.y(count_) = newCoor1(2);
    E.prex(count_) = nearCoor1(1);
    E.prey(count_) = nearCoor1(2);
    E.pre(count_) = preIndex1;
    
    %���������µ�����
    line([nearCoor1(1) newCoor1(1)],[nearCoor1(2) newCoor1(2)],'Color','b','LineWidth',1);  %����ÿһ���µ�
%     pause(0.01);


while calcuDis(newCoor,newCoor1)>Thr
    
    %% �õ��յ����µ���������µ����µ�
    newCoor2 = expandPoint(newCoor1,newCoor,step);
    contactFlag = iscontact(mapInfo,newCoor1,newCoor2,step);
    if contactFlag %������һ������ֵ��Ϊ0 ������ײ������������ѭ��
        break;
    end
    
    %% �����µ㵽�յ�����
    count_ = count_ + 1;
    E.x(count_) = newCoor2(1);
    E.y(count_) = newCoor2(2);
    E.prex(count_) = newCoor1(1);
    E.prey(count_) = newCoor1(2);
    E.pre(count_) = count_ - 1;
    
    %���������µ�����
    line([newCoor1(1) newCoor2(1)],[newCoor1(2) newCoor2(2)],'Color','b','LineWidth',1);  %����ÿһ���µ�
%     pause(0.01);
    newCoor1 = newCoor2;   
end

% if calcuDis(newCoor1,newCoor)<Thr
%     line([newCoor1(1) newCoor(1)],[newCoor1(2) newCoor(2)],'LineWidth',1);
%     break;
% end

    %% �ж��Ƿ񵽴��յ㣬����·��
%     size_S = size(S.x,2);
%     size_E = size(E.x,2);
    if calcuDis(newCoor1,newCoor)<Thr
        findpath = 1;
        clear path;
        if S.x(1) == startPoint(1)
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

    %% ����·��    
    if findpath
        for j = 2 : size(path.x,2)
            pt_draw_new = plot([path.x(j);path.x(j-1);],[path.y(j);path.y(j-1);],'g', 'Linewidth', 0.5);
            pt_draw_newlist = [pt_draw_newlist pt_draw_new];
        end
        totalcost = 0;
        for j = 2 : size(path.x,2)
            cur_dis = sqrt((path.x(j)-path.x(j-1))^2+(path.y(j)-path.y(j-1))^2);
            totalcost = totalcost + cur_dis;
        end
        break;
    end
end
    
if iter==iterMax
    Path = [];
    disp('·���滮ʧ��');
    return;
end

%% Ѱ��·��
Path(1,3) = totalcost;
disp(totalcost)
Path(1,4) = count + count_;

for i = 1 : size(path.x,2)
    Path(i,1) = path.x(i);
    Path(i,2) = path.y(i);
end
for j = 2 : size(path.x,2)
    pt_draw_new = plot([path.x(j);path.x(j-1);],[path.y(j);path.y(j-1);],'g', 'Linewidth', 0.5);
    pt_draw_newlist = [pt_draw_newlist pt_draw_new];
end

% index = size(S.x,2);
% count = 1;
% 
% index_ = size(E.x,2);
% count_ = 1;
% 
% while index ~= 0
%     Path2(count,1) = S.x(index);
%     Path2(count,2) = S.y(index);
%     Path2(count,3) = S.z(index);
%     index = index - 1;
%     count = count + 1;      
% end
% 
% Path2(count,1) = startPoint(1);
% Path2(count,2) = startPoint(2);
% Path2(count,3) = startPoint(3);
% 
% Path2 = flipud(Path2);
% 
% while index_ ~= 0
%     Path2(count_+count,1) = E.x(index_);
%     Path2(count_+count,2) = E.y(index_);
%     Path2(count_+count,3) = E.z(index_);
%     index_ = index_ - 1;
%     count_ = count_+1;      
% end
% 
% Path2(count_+count+1,1) = goalPoint(1);
% Path2(count_+count+1,2) = goalPoint(2);
% Path2(count_+count+1,3) = goalPoint(3);
end