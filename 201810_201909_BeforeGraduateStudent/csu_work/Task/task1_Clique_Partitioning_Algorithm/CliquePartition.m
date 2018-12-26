clc;
close all;
clear;
p = rand(100,2);
q = ones(2);
filename = 'C:\Users\Aikun_Xu\Desktop\TheEndPoint.mat';
save(filename, 'p', 'q')
Node=cell2mat(struct2cell(load('C:\Users\Aikun_Xu\Desktop\TheEndPoint.mat')))
%考虑到充电桩的覆盖范围为50倍根号2，因此电单车的所属范围也为50倍根号2
%以50倍根号2构建图G(V,E),如果两个电单车存在所属范围存在交集，则相连一条边
for i=1:length(Node)
    %scatter(Node(i,1),Node(i,2),50,'o','r');%画图的时候再取消注释
    hold on;
end
%判断任意两个节点是否有连线，需要判断两个点之间的距离是否小于100倍根号2
Thr=50*sqrt(2);%阈值（可修改）
for i=1:length(Node)
    for j=1:length(Node)
         Distance_Node=sqrt((Node(i,1)-Node(j,1))^2+(Node(i,2)-Node(j,2))^2);
         ResultDis(i,j)=Distance_Node;
    end
end
k=0;%用以统计小于阈值的有多少条连线
s=0;
for i1=1:length(Node)
    for j1=i1+1:length(Node)
      if ResultDis(i1,j1)<=Thr
          %画图的时候再取消注释
             %plot([Node(i1,1),Node(j1,1)],[Node(i1,2),Node(j1,2)],'b');
             k=k+1;
             Edge(i1,j1)=1;
             Edge(j1,i1)=1;
             hold on;
      else
          %画图的时候再取消注释
             %plot([Node(i1,1),Node(j1,1)],[Node(i1,2),Node(j1,2)],'r.');
             s=s+1;
             Edge(i1,j1)=0;
             Edge(j1,i1)=0;
             hold on;
     end
    end
end
grid on;
axis([1000 2000 1280 1700]);

%以上构造了无向图G，下面在无向图G上应用最小团划分算法
%首先找到节点度最大的两个顶点
%当还有边存在的时候，就要继续进行
oo=1;%用以判断出现了几个团
SubNode=Node;%用以在删减后的Edge中找到原始的节点
while(sum(abs(Edge(:)))~=0)
    SumEdge=sum(Edge,2);%所有节点的度
    %对所有节点的度进行排序
    for q=1:length(Node)
        SumEdge(q,2)=q;%为使排序后仍知道是哪一个节点,第二列表示的是节点的序号
    end
    SumEdge_sort=sortrows(SumEdge,-1);%按第一列进行降序排序，其它列随之改变
    %求每条边对应的Common neighbors的个数
    for p=1:length(Edge)%控制行变化
        First=Edge(p,:);
        for w=1:length(First)
            if First(1,w)==0
                First(1,w)=2;
            end
        end
        for l=1:length(Edge)%控制列变化
            if Edge(p,l)==1
               %提取第l行的元素
                Second=Edge(l,:);
                Diff=0;
                for u=1:length(Edge)
                    if First(1,u)==Second(1,u)
                        Diff=Diff+1;
                    end
                end
                Com_Neighbors(p,l)=Diff;
             else
                Com_Neighbors(p,l)=0;
            end
        end
    end
    Com_Neighbors=triu(Com_Neighbors);%得到上三角形矩阵
    %Com_Neighbors中存储的是不同边的Com_Neighbors个数
    %先找到具有最大Com_Neighbors所对应的顶点
    Com_Neighbors_new=unique(Com_Neighbors);
    Com_Neighbors_new=sort(Com_Neighbors_new(:),'descend');
    [x1,y1]=find(Com_Neighbors==Com_Neighbors_new(1,1));
    %判断坐标是否对应只有两个点，如果对应的不是两个点，选择度之和最大的两个点
    Du_Result=[];
    if length(x1)~=1
        for ww=1:length(x1)
                TheSumDu=SumEdge(x1(ww,1),1)+SumEdge(y1(ww,1),1);
                Du_Result(1,ww)=TheSumDu;
        end
        Node_result_coor=find(Du_Result==max(Du_Result));
        if length(Node_result_coor)~=1
            x1=x1(Node_result_coor(1,1),1);
            y1=y1(Node_result_coor(1,1),1);
        else
            x1=x1(Node_result_coor,1);
            y1=y1(Node_result_coor,1);
        end
    end
    ii=1;
    for uu=1:length(Node)
        if Node(uu,1)==SubNode(x1,1)&&Node(uu,2)==SubNode(x1,2)
            cell{1,oo}(1,ii)=uu;
        end
    end
    ii=ii+1;
    for uu=1:length(Node)
        if Node(uu,1)==SubNode(y1,1)&&Node(uu,2)==SubNode(y1,2)
            cell{1,oo}(1,ii)=uu;
        end
    end
    ii=ii+1;
    %删除x1,y1中未和common neighbors相连的边
    for pp=1:length(x1)
        while(sum(abs(Edge(x1(pp,1),:)~=0)))%判断合并后的节点是否为孤立节点
            for kk=1:length(Edge)
                for ee=1:length(x1)
                    if Edge(x1(ee,1),kk)~=0||Edge(y1(ee,1),kk)~=0
                        if Edge(x1(ee,1),kk)~=Edge(y1(ee,1),kk)
                            Edge(x1(ee,1),kk)=0;
                            Edge(y1(ee,1),kk)=0;
                        end
                    end
                    if Edge(kk,x1(ee,1))~=0||Edge(kk,y1(ee,1))~=0
                        if Edge(kk,x1(ee,1))~=Edge(kk,y1(ee,1))
                            Edge(kk,x1(ee,1))=0;
                            Edge(kk,y1(ee,1))=0;
                        end
                    end
                end
            end
         %合并x1,y1为一个点r,如果r为孤立点，再去寻找common neighbors大的点
         %如果r不是孤立点，则寻找包含r的具有最大common neighbors的边，另一个顶点为r1，将r和r1再次进行上述过程
         %结束上述过程直到图中不存在边
         SubNode(y1,:)=[];
         %删除的应为放在cell中的x1
            Edge(y1,:)=[];
            Edge(:,y1)=[];
            %更新合并后的Edge的Com_Neighbors
            Com_Neighbors=[];
        for p=1:length(Edge)%控制行变化
            First=Edge(p,:);
            for w=1:length(First)
                if First(1,w)==0
                    First(1,w)=2;
                end
            end
            for l=1:length(Edge)%控制列变化
                if Edge(p,l)==1
                   %提取第l行的元素
                    Second=Edge(l,:);
                    Diff=0;
                    for u=1:length(Edge)
                        if First(1,u)==Second(1,u)
                            Diff=Diff+1;
                        end
                    end
                    Com_Neighbors(p,l)=Diff;
                else
                    Com_Neighbors(p,l)=0;
                end
            end
        end
        Com_Neighbors=triu(Com_Neighbors);%得到上三角形矩阵
        %找到与x1相连的具有最多的common neighbors的点
        Max_x1=find(Com_Neighbors(x1,:)==max(Com_Neighbors(x1,:)));
        y1=Max_x1;
        %对更新过的Edge重新计算节点的度
        SumEdge=[];
        SumEdge=sum(Edge,2);%所有节点的度
        %对所有节点的度进行排序
        % for q=1:length(Node)
        %     SumEdge(q,2)=q;%为使排序后仍知道是哪一个节点,第二列表示的是节点的序号
        % end
        Du_Result=[];%Du_Result在每次进行时需要初始化，否则会导致索引超过矩阵维度
        if length(y1)~=1
            for ww=1:length(y1)
                    TheSumDu=SumEdge(x1(1,1),1)+SumEdge(y1(1,ww),1);
                    Du_Result(1,ww)=TheSumDu;
            end
            %可能存在两条边得到的度是相等的，这种情况下取任意一个即可
            Node_result_coor=find(Du_Result==max(Du_Result));
             if length(Node_result_coor)~=1
                %x1=x1(Node_result_coor(1,1),1);
                y1=y1(1,Node_result_coor(1,1));
            else
                %x1=x1(Node_result_coor,1);
                y1=y1(1,Node_result_coor);
            end
        end
        for uu=1:length(Node)
            if Node(uu,1)==SubNode(x1,1)&&Node(uu,2)==SubNode(x1,2)
                cell{1,oo}(1,ii)=uu;
            end
        end
        ii=ii+1;
        for uu=1:length(Node)
            if Node(uu,1)==SubNode(y1,1)&&Node(uu,2)==SubNode(y1,2)
                cell{1,oo}(1,ii)=uu;
            end
        end
        ii=ii+1;
        end
    end
    oo=oo+1;
end
for jj=1:length(cell)
    cell{1,jj}=unique(cell{1,jj},'stable');
end
oo





