clc;
close all;
clear;
p = rand(100,2);
q = ones(2);
filename = 'C:\Users\Aikun_Xu\Desktop\TheEndPoint.mat';
save(filename, 'p', 'q')
Node=cell2mat(struct2cell(load('C:\Users\Aikun_Xu\Desktop\TheEndPoint.mat')))
%���ǵ����׮�ĸ��Ƿ�ΧΪ50������2����˵絥����������ΧҲΪ50������2
%��50������2����ͼG(V,E),��������絥������������Χ���ڽ�����������һ����
for i=1:length(Node)
    %scatter(Node(i,1),Node(i,2),50,'o','r');%��ͼ��ʱ����ȡ��ע��
    hold on;
end
%�ж����������ڵ��Ƿ������ߣ���Ҫ�ж�������֮��ľ����Ƿ�С��100������2
Thr=50*sqrt(2);%��ֵ�����޸ģ�
for i=1:length(Node)
    for j=1:length(Node)
         Distance_Node=sqrt((Node(i,1)-Node(j,1))^2+(Node(i,2)-Node(j,2))^2);
         ResultDis(i,j)=Distance_Node;
    end
end
k=0;%����ͳ��С����ֵ���ж���������
s=0;
for i1=1:length(Node)
    for j1=i1+1:length(Node)
      if ResultDis(i1,j1)<=Thr
          %��ͼ��ʱ����ȡ��ע��
             %plot([Node(i1,1),Node(j1,1)],[Node(i1,2),Node(j1,2)],'b');
             k=k+1;
             Edge(i1,j1)=1;
             Edge(j1,i1)=1;
             hold on;
      else
          %��ͼ��ʱ����ȡ��ע��
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

%���Ϲ���������ͼG������������ͼG��Ӧ����С�Ż����㷨
%�����ҵ��ڵ��������������
%�����бߴ��ڵ�ʱ�򣬾�Ҫ��������
oo=1;%�����жϳ����˼�����
SubNode=Node;%������ɾ�����Edge���ҵ�ԭʼ�Ľڵ�
while(sum(abs(Edge(:)))~=0)
    SumEdge=sum(Edge,2);%���нڵ�Ķ�
    %�����нڵ�ĶȽ�������
    for q=1:length(Node)
        SumEdge(q,2)=q;%Ϊʹ�������֪������һ���ڵ�,�ڶ��б�ʾ���ǽڵ�����
    end
    SumEdge_sort=sortrows(SumEdge,-1);%����һ�н��н���������������֮�ı�
    %��ÿ���߶�Ӧ��Common neighbors�ĸ���
    for p=1:length(Edge)%�����б仯
        First=Edge(p,:);
        for w=1:length(First)
            if First(1,w)==0
                First(1,w)=2;
            end
        end
        for l=1:length(Edge)%�����б仯
            if Edge(p,l)==1
               %��ȡ��l�е�Ԫ��
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
    Com_Neighbors=triu(Com_Neighbors);%�õ��������ξ���
    %Com_Neighbors�д洢���ǲ�ͬ�ߵ�Com_Neighbors����
    %���ҵ��������Com_Neighbors����Ӧ�Ķ���
    Com_Neighbors_new=unique(Com_Neighbors);
    Com_Neighbors_new=sort(Com_Neighbors_new(:),'descend');
    [x1,y1]=find(Com_Neighbors==Com_Neighbors_new(1,1));
    %�ж������Ƿ��Ӧֻ�������㣬�����Ӧ�Ĳ��������㣬ѡ���֮������������
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
    %ɾ��x1,y1��δ��common neighbors�����ı�
    for pp=1:length(x1)
        while(sum(abs(Edge(x1(pp,1),:)~=0)))%�жϺϲ���Ľڵ��Ƿ�Ϊ�����ڵ�
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
         %�ϲ�x1,y1Ϊһ����r,���rΪ�����㣬��ȥѰ��common neighbors��ĵ�
         %���r���ǹ����㣬��Ѱ�Ұ���r�ľ������common neighbors�ıߣ���һ������Ϊr1����r��r1�ٴν�����������
         %������������ֱ��ͼ�в����ڱ�
         SubNode(y1,:)=[];
         %ɾ����ӦΪ����cell�е�x1
            Edge(y1,:)=[];
            Edge(:,y1)=[];
            %���ºϲ����Edge��Com_Neighbors
            Com_Neighbors=[];
        for p=1:length(Edge)%�����б仯
            First=Edge(p,:);
            for w=1:length(First)
                if First(1,w)==0
                    First(1,w)=2;
                end
            end
            for l=1:length(Edge)%�����б仯
                if Edge(p,l)==1
                   %��ȡ��l�е�Ԫ��
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
        Com_Neighbors=triu(Com_Neighbors);%�õ��������ξ���
        %�ҵ���x1�����ľ�������common neighbors�ĵ�
        Max_x1=find(Com_Neighbors(x1,:)==max(Com_Neighbors(x1,:)));
        y1=Max_x1;
        %�Ը��¹���Edge���¼���ڵ�Ķ�
        SumEdge=[];
        SumEdge=sum(Edge,2);%���нڵ�Ķ�
        %�����нڵ�ĶȽ�������
        % for q=1:length(Node)
        %     SumEdge(q,2)=q;%Ϊʹ�������֪������һ���ڵ�,�ڶ��б�ʾ���ǽڵ�����
        % end
        Du_Result=[];%Du_Result��ÿ�ν���ʱ��Ҫ��ʼ��������ᵼ��������������ά��
        if length(y1)~=1
            for ww=1:length(y1)
                    TheSumDu=SumEdge(x1(1,1),1)+SumEdge(y1(1,ww),1);
                    Du_Result(1,ww)=TheSumDu;
            end
            %���ܴ��������ߵõ��Ķ�����ȵģ����������ȡ����һ������
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





