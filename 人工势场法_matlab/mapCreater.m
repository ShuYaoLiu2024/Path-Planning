function map=mapCreater(obstacles,area)
%优雅
s=size(obstacles);%获取障碍的个数，保存在s(1)中
map=ones(area(1),area(2));%创建空白地图，即全是1，即没有障碍
max_R=max([max(obstacles(:,1)) max(area)])+max(area);%计算掩膜的宽度，要比地图大，也要比所有障碍都大
mask0=ones(max_R*2+1,max_R*2+1);%创建基础掩膜0

X0=-max_R:max_R;
Y0=X0;
[X,Y]=meshgrid(X0,Y0);
R2=X.^2+Y.^2;%以上四行用来得到一个矩阵，每一个元素的值等于该点位置到中心的距离的平方

for i=1:s(1)%对每个障碍
    R=obstacles(i,1);
    Oy=obstacles(i,2);
    Ox=obstacles(i,3);%以上三行读取每个障碍的半径与圆心位置

    mask1=mask0(-Oy+max_R+2:-Oy+max_R+area(1)+1,-Ox+max_R+2:-Ox+max_R+area(2)+1);%裁剪基础掩膜0使其中心位置与地图上圆心位置对应，得到掩膜1
    R21=R2(-Oy+max_R+2:-Oy+max_R+area(1)+1,-Ox+max_R+2:-Ox+max_R+area(2)+1);%与上一行同理裁剪R2矩阵
    mask1(find(R21<=R*R))=0;%根据R21矩阵中已经计算好的信息将掩膜1中在障碍圆内的元素值改为0，即表示此位置有障碍
    map=map.*mask1;%矩阵对应元素相乘
end
%{
%无趣但稳定
s=size(obstacles);
map=ones(area(1),area(2));
for x=1:area(2)
    for y=1:area(1)%以上两个for实现对每个点的遍历
        for i=1:s(1)
            R=obstacles(i,1);
            Oy=obstacles(i,2);
            Ox=obstacles(i,3);
            r2=(Oy-y)^2+(Ox-x)^2;
            if r2<=R^2
                map(y,x)=0;%如果到圆心距离小于障碍圆的半径，则该点为障碍
            end
        end
    end
end
%}


