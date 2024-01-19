import os #文件
import shutil
#序号_A用时_B用时_b距离_B速度
path0="路径图像_Q2"
path2="路径图像_输出"
fileList=os.listdir(path2)

n=0
maxnumber=0
for name in fileList:
    maxnumber=max(int(name[:name.find("_",0)]),maxnumber)
    n+=1

'''
for i in range(6):
	path=path0+"_"+str(i)
	fileList=os.listdir(path)

	for name in fileList:
		p1=name.find("_",0)
		p2=name.find("_",p1+1)
		p3=name.find("_",p2+1)
		p4=name.find("_",p3+1)
		p5=name.find(".",p3+1)

		Atime=int(name[p1+1:p2])
		Btime=int(name[p2+1:p3])
		bdistance=int(name[p3+1:p4])
		Bspeed=int(name[p4+1:p5])

		if bdistance ==3500 and Bspeed ==10:
			maxnumber+=1
			inf=str(maxnumber)+"_"+ str(Atime)+"_"+ str(Btime)+"_" +str(bdistance) +"_"+ str(Bspeed)+".png"
			shutil.copy(path+os.sep+name,path2+os.sep+inf)

'''
path=path0
fileList=os.listdir(path)


min1=10000
for name in fileList:
	p1=name.find("_",0)
	p2=name.find("_",p1+1)
	p3=name.find("_",p2+1)
	p4=name.find("_",p3+1)
	p5=name.find(".",p3+1)

	Atime=float(name[p1+1:p2])
	Btime=float(name[p2+1:p3])
	bdistance=int(name[p3+1:p4])
	Bspeed=int(name[p4+1:p5])

	if bdistance==3500 and Bspeed==10:
		
		if min1>=max(Atime,Btime):
			min1=max(Atime,Btime)
			maxnumber+=1
			inf=str(maxnumber)+"_"+ str(Atime)+"_"+ str(Btime)+"_" +str(bdistance) +"_"+ str(Bspeed)+".png"
			shutil.copy(path+os.sep+name,path2+os.sep+inf)
print(min1)