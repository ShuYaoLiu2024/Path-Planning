import math #角度
import random #随机数
import numpy as np #数组
import matplotlib.pyplot as plt #画图
import matplotlib.style as mplstyle
import sys #递归深度
sys.setrecursionlimit(3000)
import copy #深浅复制

mplstyle.use('fast')


#----------------------------------模拟参数
dt=2#时间精度
er=20#结束范围
randomtargetprobability=0.5#随机采样的概率

#----------------------------------题目参数
v=10#速度
r_min=30#最小转弯半径

#----------------------------------输入
Start=np.array([3500,0])#起始位置
End=np.array([-1000,0])#结束位置
sircs=[[500,[0,200]],[600,[2000,-200]],[200,[1000,300]],[100,[1500,500]],]#障碍圆们 [半径,[圆心x,圆心y]]

area=np.array([[-2000,-1000],[4500,1500]])#范围

#----------------------------------计算参数
radian_max=math.pi-2*math.atan(2*r_min/dt/v)#转角取值范围

#----------------------------------节点
class Point:
    def __init__(self):
        self.xy = None
        self.dad = None
        self.sel = None
        self.son = []
        self.gen = None
        self.direction = None
        self.exi = 1


#----------------------------------位置更新
#沿子节点删除
def delson(pointall,point):
	delcount=0
	for sonnum in point.son:
		pointall[sonnum].exi=0

		delcount=delcount+1
		
		delcount=delcount+delson(pointall,pointall[sonnum])
	return delcount

#剪枝
def lopcut(pointall,point):
	maxcount=100
	poicount=0
	for i in range(2):

		while len(pointall[point.dad].son)==1 and poicount<maxcount:
			poicount+=1
			point=pointall[point.dad]

	delcount=delson(pointall,point)
	return delcount

#计算下一节点
def getnextpoint(point,direction):

	nextpoint=Point()
	nextpoint.xy=point.xy+np.array([v*dt*math.cos(direction),v*dt*math.sin(direction)])
	nextpoint.dad=point.sel
	nextpoint.gen=point.gen+1
	nextpoint.direction=direction
	
	return nextpoint

#随机采样
def gettarget(pointall):
	if random.uniform(0,1)>randomtargetprobability:
		target=End
	else:
		success=0
		while success==0:
			
			
			target=[random.uniform(area[0][0],area[1][0]),random.uniform(area[0][1],area[1][1])]
			target=np.array(target)
			
			success=1
	return target

#分支节点和方向
def getnear(pointall,target,point_all_gen):
	
	lasts=0
	success=0
	maxst=10
	st=0
	r_all=[]
	
	for point in pointall:
		if point.exi==1:
			
				
			r_nt=math.sqrt(sum((point.xy-target)*(point.xy-target)))
			r_all.append(np.array([r_nt,point.sel]))
	r_all=np.array(r_all)
	
	if len(r_all)==0:
		return -1,0
	r_all_t=r_all[np.argsort(r_all[:,0])]
	

	while success==0:
		
		
		near=pointall[int(r_all_t[min(st,len(r_all_t)-1)][1])-1]
		
			
		direction=math.atan2(target[1]-near.xy[1],target[0]-near.xy[0])
	
		if near.direction+radian_max>direction and near.direction-radian_max<direction:
			success=1
		if near.direction+radian_max>direction+2*math.pi and near.direction-radian_max<direction+2*math.pi:
			success=1
		if near.direction+radian_max>direction-2*math.pi and near.direction-radian_max<direction-2*math.pi:
			success=1
		if success==0:
			st=st+1
			if st>maxst:
				return -1,0
			
			if near.dad==-1:
				return -1,0



	return near,direction



#----------------------------------约束条件
#不进入障碍圆
def isobsirc(poi,sirc):
	R=sirc[0]
	O=np.array(sirc[1])
	r=math.sqrt(sum((poi-O)*(poi-O)))
	
	if r>R :
		return 1
	else:
		return 0

#不进入障碍圆们
def isobsircs(poi):
	for sirc in sircs:
		if isobsirc(poi,sirc)==0:
			return 0
	return 1

#在区域内

def isinarea(poi_A):
	
	if poi_A[0]<area[0][0] or poi_A[1]>area[1][0]:
		return 0
	if poi_A[1]<area[0][1] or poi_A[1]>area[1][1]:
		return 0

	return 1

#----------------------------------终止条件
def isend(poi):
	
	r=math.sqrt(sum((poi-End)*(poi-End)))
	
	if r<er:
		return 1
	
	return 0

#----------------------------------画个图
def draw(path,pointall,time):
	
	plt.close()
	plt.ion()
	fig, ax = plt.subplots()


	#计算最大代数以以灰度显示方向
	gen=0
	for point in path[1:]:
		if point.exi==1:
			if point.gen>gen:
				gen=point.gen
	#画圆
	for sirc in sircs:
		R=sirc[0]
		O=np.array(sirc[1])
		
		ax.add_artist(plt.Circle((O[0],O[1]),R,color='black',fill='True'))

	#被剪部分
	for point in pointall[1:]:
		if point.exi==0 or 1:
			plt.plot([point.xy[0],pointall[point.dad].xy[0]],
					 [point.xy[1],pointall[point.dad].xy[1]],
					 color="red")

	#路径
	for point in path[1:]:
		if point.exi==1:
			plt.plot([point.xy[0],path[point.dad].xy[0]],
					 [point.xy[1],path[point.dad].xy[1]],
					 color=str(1-point.gen/gen))
			
	
	
	
	#ax = plt.gca()
	ax.set_aspect(1)
	plt.pause(time)
	


#----------------------------------生成路径


#B初始节点
def getgeo_inista():
	
	s=math.atan2(End[1]-Start[1],End[0]-Start[0])

	geo_inista=Point()
	geo_inista.xy=Start
	geo_inista.dad=0
	
	geo_inista.gen=0
	
	geo_inista.direction=s
	return geo_inista


def getpath():
	success=0
	while success==0:
		ini=Point()
		ini.xy=Start
		ini.dad=-1
		ini.sel=0
		ini.gen=0
		point_all=[]
		point_all.append(ini)
		point_num=0
		point_gen=0
		point_num_real=0
		failcount=0
		path=[]
		while success==0 and point_num<2000:
			near=-1
			if len(point_all)==1:
				nextpoint=getgeo_inista()
				near=ini
			else:
				while near==-1:
					tar=gettarget(point_all)
					near,direction=getnear(point_all[1:],tar,point_gen)
				nextpoint=getnextpoint(near,direction)
			obsirc=isobsircs(nextpoint.xy)
			inarea=isinarea(nextpoint.xy)
			if obsirc and inarea:
				failcount=0
				point_num=point_num+1
				point_num_real=point_num_real+1
				point_all[near.sel].son.append(point_num)
				nextpoint.sel=point_num
				point_all.append(nextpoint)
				if nextpoint.gen>point_gen:
					point_gen=nextpoint.gen
				success = isend(nextpoint.xy)
				if point_num%100==0 :
					print("|",end="")
			else:
				failcount+=1
				point_num_real=point_num_real-lopcut(point_all,near)
		if point_num<=2000 :
			
			path_t=[]
			startgen=point_all[1].gen
			while nextpoint.dad != -1:
				geo_point=copy.copy(nextpoint)
				path_t.append(geo_point)
				nextpoint=point_all[nextpoint.dad]
			
			endgen=0
			for i in range(len(path_t)-1,0-1,-1):
				
				endgen=path_t[i].gen
				path_t[i].sel=len(path_t)-1-i
				path_t[i].son=len(path_t)-1-i+1
				path_t[i].dad=len(path_t)-1-i-1
				path.append(path_t[i])
	return path,point_all,startgen,endgen



#----------------------------------主函数


while 1:

	path,point_all,startgen,endgen=getpath()
	print(len(path))
	draw(path,point_all,1)
	
	
	
	


