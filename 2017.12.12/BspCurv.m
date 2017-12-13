% calculate the discnet points curvature;

function result = BspCurv( C,U,t,k)
%CURVATURE B样条曲线在t点曲率
%C控制点,U节点序列,U(k)<=t

dev1=0;
dev2=0;
%%计算一阶导数
for i=k-2:k
    dev1=dev1+DeBoorDv(C,U,t,i,1)*Bbase(i,2,U,t);
end

%计算二阶导数
for i=k-1:k
    dev2=dev2+DeBoorDv(C,U,t,i,2)*Bbase(i,1,U,t);
end

%计算曲率

temp=norm(dev1)^3;
if temp==0
    result=NaN;
else
    result=det( cat(1,dev1,dev2))/temp;
end

%=====================================

%************递归求导数控制点,德布尔递推算法************
function y=DeBoorDv(d,u,kn,i,L)
%d控制点序列,u节点序列,kn插入的节点,u(i)
% k B样条次数
k=3;
if L==0
     y=d(i,:);
     return;
end

temp=(u(i+k+1-L)-u(i));
if temp==0 
     y=0;
else
     y=(k+1-L)*(DeBoorDv(d,u,kn,i,L-1)-DeBoorDv(d,u,kn,i-1,L-1))/temp;
end