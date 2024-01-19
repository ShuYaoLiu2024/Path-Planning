function [obstacles,source,goal,area]=Inputinfo()

%先输入障碍
obstacles=Inputobstacles();

%{
%推荐地图大小,不需要就删了
area_x=max(obstacles(:,3))+2*max(obstacles(:,1));
area_y=max(obstacles(:,2))+2*max(obstacles(:,1));
definput={'','','','',num2str(area_y),num2str(area_x)};
%}

%再输入起始点，目标点以及地图大小
prompt={'Enter source position Y:','Enter source position X:', ...
    'Enter goal position Y:','Enter goal position X:', ...
    'Enter map size Y:','Enter map size X:'};%输入提示
dlgtitle='Input';%弹窗标题
dims=[1 40;1 40;1 40;1 40;1 40;1 40];%输入框大小
%info=str2double(inputdlg(prompt,dlgtitle,dims,definput));%弹出窗口
info=str2double(inputdlg(prompt,dlgtitle,dims));%不需要推荐地图大小就用这个，不用上面那条

source=[info(1) info(2)];
goal=[info(3) info(4)];
area=[info(5) info(6)];