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
