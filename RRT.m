function Path = RRT(startPoint,axisStart,axisLWH,goalPoint,mapInfo)
%% RRT�㷨Ѱ��·����
 
%% ��������
calcuDis = @(x,y)  sqrt((x(1)-y(1))^2+(x(2)-y(2))^2); %���������ľ���
iterMax = 5000;   %����������
iter = 0;   %��ǰ��������
step = 2;  %����
count = 1;  %������
Thr = 5;   %��ֵ
 
%������
T.x(1) = startPoint(1);
T.y(1) = startPoint(2);
T.pre(1) = 0;
 
while iter < iterMax  %ѭ������
    
    iter = iter+1;
    
    %% �ڿռ����������
    randCoor = samplePoint(axisStart,axisLWH,goalPoint);
    
    %% Ѱ�����������
    [nearCoor,preIndex] = findNearPoint(randCoor,T);
    
    %% ����ָ�����������µ���չ��
    newCoor = expandPoint(nearCoor,randCoor,step);
    
    %% ��ײ��� �ж��Ƿ��볤���塢Բ���塢���巢����ײ
    contactFlag = iscontact(mapInfo,nearCoor,newCoor,step);

    if  contactFlag%������һ������ֵ��Ϊ0 ������ײ������������ѭ��
        continue;
    end
    
    %% ���µ��������
    count = count+1;
    T.x(count) = newCoor(1);
    T.y(count) = newCoor(2);
    T.pre(count) = preIndex;
    %���������µ�����
    line([nearCoor(1) newCoor(1)],[nearCoor(2) newCoor(2)],'LineWidth',1);  %����ÿһ���µ�
%     pause(0.01);

    
    if calcuDis(newCoor,goalPoint)<Thr %�����µ���Ŀ���ľ����Ƿ�ﵽ��ֵ
        line([newCoor(1) goalPoint(1)],[newCoor(2) goalPoint(2)],'LineWidth',1);
        break;
    end 
    
end
 
if iter==iterMax
    Path = [];
    disp('·���滮ʧ��');
    return;
end
 
%% Ѱ��·��
index = T.pre(end);
count = 1;
 
while T.pre(index)~=0
    Path(count,1) = T.x(index);
    Path(count,2) = T.y(index);
    index = T.pre(index);
    count = count+1;
end
 
%����ʼ����ӵ�Path��
Path(count,1) = startPoint(1);
Path(count,2) = startPoint(2);
 
%��Ŀ�����ӵ�Path��
Path = flipud(Path);
count = count+1;
Path(count,1) = goalPoint(1);
Path(count,2) = goalPoint(2);
 
 
 
end
 
