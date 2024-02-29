%----------定义结构--------%
% function Truss %平面桁架
%结点坐标
gxy = [0,3;0,0;4,3;4,0];
%单元信息,例如第一个单元两边结点为1，3
ndel=[1,3;2,3;4,3;2,4];
%每个单元的抗拉刚度
EA = 3e8*ones(4,1);
%结点总数
nd = size(gxy,1);
%单元总数
ne = size(ndel,1);
%初始化结点力,2D的每个节点两个方向的力
F = zeros(2*nd,1);
%节点外力
F([6,7]) = [-2e4;3e4];
%约束位移对应的自由度编号
dofix=1:4;
%无约束位移对应的自由度编号
dofree = setdiff(1:2*nd,dofix);%setdiff 两个数组的差集

%--------形成刚度矩阵-------
%总刚度矩阵维数，因为每个节点2个自由度，所以总刚度矩阵是这样的维数
K = zeros(2*nd,2*nd);
%单元坐标下的单元刚度矩阵,每个单元的单元刚度矩阵是相同的(在没有坐标变化下)
K0=zeros(4,4); K0([1,3],[1,3]) = [1,-1;-1,1];
%返回单元左边转换矩阵和杆长,gxy(ndel(el,:),:)表示取出每个单元包含节点的坐标
for el = 1:ne
    [T,L] = TrusRota(gxy(ndel(el,:),:));
    N(2:2:4) = 2*ndel(el,:);N(1:2:4)=N(2:2:4)-1; %单元自由度，这里乘2再相减是对的，矩阵先扩展
    K(N,N)=K(N,N)+EA(el)/L*T'*K0*T;%由单元刚度矩阵安装总刚度矩阵 
end

%--------求解位移--------
U = zeros(2*nd,1); %节点位移列向量初始化
U(dofix) = 0; %约束位移
%公式：U_a = k_{aa}\(F_a-k_{ab}U_b) U_a是未知结点位移 U_b是已知
U(dofree) = K(dofree,dofree)\(F(dofree)-K(dofree,dofix)*U(dofix)); %求解位移向量的公式

%------计算结果输出------%
fprintf('\n%4s%7s%7s%12s%12s\n','结点','X坐标','Y坐标','u位移','v位移')%输出标题
for i = 1:nd
    fprintf('%4i%10.4f%10.4f%14.4g%14.4g\n',i,gxy(i,:),U(2*i+(-1:0)));%输出结点编号坐标和位移
end
fprintf('\n%4s%4s%12s%12s\n','单元','结点','面积','轴力')
for el = 1:ne
    [T,L] = TrusRota(gxy(ndel(el,:),:));
    N(2:2:4) = 2*ndel(el,:);N(1:2:4)=N(2:2:4)-1;
    Fe = EA(el)/L*K0*T*U(N); %杆端点力 单元坐标下就是沿着X方向上的力，是在EA(el)/L*T'*K0*T基础上左乘T
    fprintf('%4i%4i%4i%14.4g%14.4g\n',el,ndel(el,:),EA(el),Fe(3))
end
DrawFrame(gxy,ndel,dofix,U);
%-------坐标转换矩阵函数--------
function [T,L] = TrusRota(xy)%xy总共就两行，代表一个单元的两个节点坐标，下面这样一减就能算长度
dl = xy(2,:)-xy(1,:);
%杆长L,就是计算长度的公式
L = sqrt(dl*dl');
cs = dl/L;%算角度的cs里面是(cos(alpha),sin(alpha))
T0 = [cs;-cs(2),cs(1)];%T0就是坐标转换矩阵(未扩展)
T = [T0,zeros(2,2);zeros(2,2),T0];%扩展后的坐标转换矩阵
end

function DrawFrame(gxy,ndel,dofix,U,~)
clf;axis equal;%设置轴比例相等
s = 2;
if nargin>4,s=3;end%桁架为2；刚架为3
gm = max(abs(max(gxy)-min(gxy)));
U = 2e-2*U*gm/max(abs(U));%将位移放大一点
G = gxy+[U(1:s:end),U(2:s:end)];
for el = 1:length(ndel)
    N = ndel(el,:);%单元自由度
    line(gxy(N,1),gxy(N,2),'color','k','LineWidth',2);%画结构
    line(G(N,1),G(N,2),'color','r','LineStyle','--');%画变形
    text(mean(gxy(N,1)),mean(gxy(N,2)),sprintf('%3d',el),'color','r');%写单元号
end
for j =1:size(gxy,1)
    text(gxy(j,1),gxy(j,2),sprintf('%3d',j),'color','m');%写结点号
end
DrawSupport(dofix,gxy,0.01*gm,s);%绘制支座程序
end

function DrawSupport(dofix,gxy,r,m)
n = length(dofix);
if nargin>3
    s=m;
else
    s=2;
end
for i =1:n
    j=fix((dofix(i)+s-1)/s);%计算约束自由度对应的结点号
    dir=dofix(i)-s*j+s;%计算约束自由度对应的方向
    if dir==1%画不同方向的支座
        X=[0,-r,-r,0];Y=[-r,-r,2*r,2*r];
    elseif dir==2
        X=[-r,-r,2*r,2*r];Y=[0,-r,-r,0];
    else
        X=[0,3*r,2*r];Y=[0,2*r,3*r];
    end
    X=gxy(j,1)+X;Y=gxy(j,2)+Y;
    patch(X,Y,'k','EdgeColor','none');
end
end

