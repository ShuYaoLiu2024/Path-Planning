import math #角度
import random #随机数
import numpy as np #数组
import matplotlib.pyplot as plt #画图

#----------------------------------节点
class Point:
    def __init__(self):
        self.xy = None
        self.dad = None
        self.sel = None
        self.son = []
        self.gen = None
        self.direction = None

#----------------------------------模拟参数
dt=1#时间精度
dradian=0.1#角度精度
er=10#结束范围
area=np.array([[-2000,-1000],[4500,1000]])#范围
randomtargetprobability=0.9#随机采样的概率

#----------------------------------题目参数
v=10#速率
r_min=30#最小转弯半径
R=500#障碍圆半径
sta_a=np.array([-1000,0])#站点A位置
sta_b=np.array([3500,0])#站点B位置

#----------------------------------计算参数
radian_max=math.pi-2*math.atan(2*r_min/dt/v)#转角取值范围




#----------------------------------位置更新
#计算下一节点
def getnextpoint(point,direction):

	nextpoint=Point()
	nextpoint.xy=point.xy+np.array([v*dt*math.cos(direction),v*dt*math.sin(direction)])
	nextpoint.dad=point.sel
	nextpoint.gen=point.gen+1
	nextpoint.direction=direction
	
	return nextpoint

#随机采样
def gettarget(isA):
	if random.uniform(0,1)>randomtargetprobability:
		target=np.multiply([isA], sta_b)+np.multiply([1-isA], sta_a)
	else:
		success=0
		while success==0:
			target=[random.uniform(area[0][0],area[1][0]),random.uniform(area[0][1],area[1][1])]
			target=np.array(target)
			if math.sqrt(sum(target*target))>R:
				success=1
	return target

#分支节点和方向
def getnear(pointall,target,gen):
	
	lasts=0
	success=0
	maxst=100
	st=0
	r_all=[]
	
	for point in pointall:
		r_nt=math.sqrt(sum((point.xy-target)*(point.xy-target)))+(-5*(gen-point.gen))
		r_all.append(np.array([r_nt,point.sel]))
	r_all=np.array(r_all)
	

	r_all_t=r_all[np.argsort(r_all[:,0])]

	while success==0:
		near=pointall[int(r_all_t[st][1])]
		
			
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
'''
#采样点位置评价
def targetvalue(pointall,newpoint):
	if random.uniform(0,1)>0.9:
		return 1
	acceptvalue=0.45
	value=0
	point_simgen=[]
	for point in pointall:
		if newpoint.gen==point.gen:
			point_simgen.append(point)
			r_OP=math.sqrt(sum(point.xy*point.xy))
			c1=(math.pi-math.asin(R/r_OP)+math.atan2(point.xy[1],point.xy[0]))%(2*math.pi)
			c2=(-math.pi+math.asin(R/r_OP)+math.atan2(point.xy[1],point.xy[0]))%(2*math.pi)
			d_N=math.atan2(newpoint.xy[1],newpoint.xy[0])
			r_ON=math.sqrt(sum(newpoint.xy*newpoint.xy))
			t=min(R+r_ON*math.tan(c1-d_N),R+r_ON*math.tan(-d_N+c2))
			value=value+t/100
	if value>=acceptvalue*len(point_simgen):
		return 1
	else:
		print("disaccess",value,acceptvalue*len(point_simgen))
		return 0

'''
#采样点位置评价
def targetvalue(pointall,newpoint):
	if random.uniform(0,1)>0.94:
		return 1
	acceptvalue=0.8
	value=0
	num=0
	
	for point in pointall:
		if abs(newpoint.gen-point.gen)<10:
			num=num+1
			
			value=value+dismeet(newpoint.xy,point.xy)
	if value>=(1-acceptvalue)*num:
		return 1
	else:
		print("disaccess",value,(1-acceptvalue)*num)
		return 0
'''
#计算下一方向
def getnextdirection(direction,radian):
	nextdirection=direction+radian
	nextdirection_t=nextdirection%(2*math.pi)
	return nextdirection_t

#计算转角
def getnextradian():
	nextradian=random.uniform(-radian_max,radian_max)
	nextradian_t=round(nextradian,1)
	return nextradian_t
'''
#----------------------------------约束条件
#不进入障碍圆
def obsirc(poi_A,poi_B):
	r_A=math.sqrt(sum(poi_A*poi_A))
	r_B=math.sqrt(sum(poi_B*poi_B))
	if r_A>R and r_B>R:
		return 1
	else:
		return 0

#不相见
def dismeet(poi_A,poi_B):
	r_A=math.sqrt(sum(poi_A*poi_A))
	d_A=math.atan2(poi_A[1],poi_A[0])
	d_AB=math.atan2(poi_A[1]-poi_B[1],poi_A[0]-poi_B[0])
	s_A=math.asin(R/r_A)
	r_AB=math.sqrt(sum((poi_A-poi_B)*(poi_A-poi_B)))
	if r_AB<R:
		return 0
	if d_A+s_A>d_AB and d_A-s_A<d_AB:
		return 1
	if d_A+s_A>d_AB+2*math.pi and d_A-s_A<d_AB+2*math.pi:
		return 1
	if d_A+s_A>d_AB-2*math.pi and d_A-s_A<d_AB-2*math.pi:
		return 1
	return 0

#在区域内
def inarea(poi_A,poi_B):
	
	if poi_A[0]<area[0][0] or poi_A[1]>area[1][0]:
		return 0
	if poi_A[1]<area[0][1] or poi_A[1]>area[1][1]:
		return 0
	if poi_B[0]<area[0][0] or poi_B[1]>area[1][0]:
		return 0
	if poi_B[1]<area[0][1] or poi_B[1]>area[1][1]:
		return 0
	return 1

#----------------------------------终止条件
def end(poi_A,poi_B):
	end_A=0
	end_B=0
	r_Ab=math.sqrt(sum((poi_A-sta_b)*(poi_A-sta_b)))
	r_Ba=math.sqrt(sum((poi_B-sta_a)*(poi_B-sta_a)))
	if r_Ab<er:
		end_A=1
		return 1
	else:
		return 0#qqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqq
	if r_Ba<er:
		end_B=1
	return end_A,end_B

#----------------------------------画个图
def draw(pointall_A,pointall_B,time):
	
	
	plt.ion()
	gen=max(point_A_gen,point_B_gen)
	for point in pointall_A[1:]:
		plt.plot([point.xy[0],pointall_A[point.dad].xy[0]],
				 [point.xy[1],pointall_A[point.dad].xy[1]],
				 color=str(1-point.gen/gen))
	
	for point in pointall_B[1:]:
		plt.plot([point.xy[0],pointall_B[point.dad].xy[0]],
				 [point.xy[1],pointall_B[point.dad].xy[1]],
				 color=str(1-point.gen/gen))
	
	ax = plt.gca()
	ax.set_aspect(1)
	plt.pause(time)
	

#----------------------------------主函数

ini_A=Point()
ini_A.xy=sta_a
ini_A.dad=-1
ini_A.sel=0
ini_A.gen=0
ini_A.direction=round(random.uniform(-0.5*math.pi,0.5*math.pi),1)
point_A_all=[]
point_A_all.append(ini_A)
point_A_num=0
point_A_gen=0

success_A=0

ini_B=Point()
ini_B.xy=sta_b
ini_B.dad=-1
ini_B.sel=0
ini_B.gen=0
ini_B.direction=round(random.uniform(-0.5*math.pi,1.5*math.pi),1)
point_B_all=[]
point_B_all.append(ini_B)
point_B_num=0
point_B_gen=0

success_B=0

while success_A==0:
	point_A_all[0].direction=round(random.uniform(-0.5*math.pi,0.5*math.pi),1)
	point_B_all[0].direction=round(random.uniform(-0.5*math.pi,1.5*math.pi),1)
	
	#print(point_A_all)
	closegen=0
	
	while closegen==0:
		if random.uniform(0,1)>0.5:
			closegen=1
		near_A=-1
		while near_A==-1:
			tar=gettarget(1)
			near_A,direction=getnear(point_A_all,tar,point_A_gen)
			
		nextpoint_A=getnextpoint(near_A,direction)
		near_B=-1
		while near_B==-1:
			tar=gettarget(0)
			near_B,direction=getnear(point_B_all,tar,point_B_gen)
			
		nextpoint_B=getnextpoint(near_B,direction)
		if abs(max(point_A_gen,nextpoint_A.gen)-max(point_B_gen,nextpoint_B.gen))<10:
			closegen=1
	if obsirc(nextpoint_A.xy,nextpoint_B.xy) and inarea(nextpoint_A.xy,nextpoint_B.xy):
		if targetvalue(point_B_all,nextpoint_A) and targetvalue(point_A_all,nextpoint_B):
			point_A_num=point_A_num+1
			point_A_all[near_A.sel].son.append(point_A_num)
			nextpoint_A.sel=point_A_num
			point_A_all.append(nextpoint_A)
			if nextpoint_A.gen>point_A_gen:
				point_A_gen=nextpoint_A.gen

			point_B_num=point_B_num+1
			point_B_all[near_B.sel].son.append(point_B_num)
			nextpoint_B.sel=point_B_num
			point_B_all.append(nextpoint_B)
			if nextpoint_B.gen>point_B_gen:
				point_B_gen=nextpoint_B.gen

			print(point_A_gen)


			if point_A_gen%60==0:
				draw(point_A_all,point_B_all,0.001)

			if end(nextpoint_A.xy,nextpoint_A.xy):
				success_A=1
print("用时：",point_A_gen*dt)
draw(point_A_all,point_B_all,2000)
