function obstacles=Inputobstacles()
obstacles=[];%预设存储变量
over=0;%用以判断是否结束
prompt={'Enter r:','Enter Oy:','Enter Ox:'};%输入提示
dlgtitle='Input';%弹窗标题
dims=[1 40;1 40;1 40];%输入框大小
while over==0
    obstacle=str2double(inputdlg(prompt,dlgtitle,dims));%弹出窗口
    if isempty(obstacle)
        over=1;%没输入表示结束输入
    else
        obstacles=[obstacles;obstacle'];%加一行
    end
end