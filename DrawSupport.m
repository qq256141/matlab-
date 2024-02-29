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


