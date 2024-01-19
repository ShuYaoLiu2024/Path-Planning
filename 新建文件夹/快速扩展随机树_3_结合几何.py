import math #角度
import random #随机数
import numpy as np #数组
import matplotlib.pyplot as plt #画图
import matplotlib.style as mplstyle
import sys #递归深度
sys.setrecursionlimit(3000)

mplstyle.use('fast')
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

#----------------------------------模拟参数
dt=1#时间精度
dradian=0.1#角度精度
er=10#结束范围
area=np.array([[-2000,-1500],[4500,1500]])#范围
randomtargetprobability=0.8#随机采样的概率

#----------------------------------题目参数
v=5#速度
r_min=30#最小转弯半径
R=500#障碍圆半径
sta_a=np.array([-1000,0])#站点A位置
sta_b=np.array([3500,0])#站点B位置

#----------------------------------计算参数
radian_max=math.pi-2*math.atan(2*r_min/dt/v)#转角取值范围




#----------------------------------位置更新
#沿子节点删除
def delson(pointall,point):
	delcount=0
	for sonnum in point.son:
		pointall[sonnum].exi=0
		delcount=delcount+1
		print("del ",pointall[sonnum].xy)
		delcount=delcount+delson(pointall,pointall[sonnum])
	return delcount

#剪枝
def lopcut(pointall,point):
	maxcount=5
	poicount=0
	for i in range(1):

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
def getnear(pointall,target,point_all_gen):
	if random.uniform(0,1)>0.9:
		point=pointall[len(pointall)-1-int(random.uniform(0,1)*len(pointall)/1000)]
		if point.exi==1:
			
			direction=point.direction+radian_max*random.uniform(-1,1)
			return point,direction
	lasts=0
	success=0
	maxst=10
	st=0
	r_all=[]
	
	for point in pointall:
		if point.exi==1:
			if point.gen>=random.uniform(0,1)*point_all_gen*0.6-2:
				
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
	if random.uniform(0,1)>0.9:
		return 1
	acceptvalue=0.2
	value=0
	num=0
	
	for point in pointall:
		if point.exi==1:
			if abs(newpoint.gen-point.gen)<5:
				num=num+1
				
				value=value+dismeet(newpoint.xy,point.xy)
	if value>=(1-acceptvalue)*num:
		return 1
	else:
		#print("disaccess",value,(1-acceptvalue)*num)
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
def obsirc(poi_A):
	r_A=math.sqrt(sum(poi_A*poi_A))
	
	if r_A>R :
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
	
	if r_AB<r_A:
		return 0
	if d_A+s_A>d_AB and d_A-s_A<d_AB:
		return 1
	if d_A+s_A>d_AB+2*math.pi and d_A-s_A<d_AB+2*math.pi:
		return 1
	if d_A+s_A>d_AB-2*math.pi and d_A-s_A<d_AB-2*math.pi:
		return 1
	return 0

#在区域内

def inarea(poi_A):
	
	if poi_A[0]<area[0][0] or poi_A[1]>area[1][0]:
		return 0
	if poi_A[1]<area[0][1] or poi_A[1]>area[1][1]:
		return 0
	
	'''
	if poi_A[1]>300 and poi_A[0]<0:
		return 0
	if poi_A[1]<-300  and dd==0:
		return 0
	'''
	return 1

#----------------------------------终止条件
def end(poi,isA):
	end=0
	if isA:
		r=math.sqrt(sum((poi-sta_b)*(poi-sta_b)))
	else:
		r=math.sqrt(sum((poi-sta_a)*(poi-sta_a)))
	
	if r<er:
		end=1
	
	return end

#----------------------------------画个图
def draw(pointall_A,pointall_B,time):
	
	plt.close()
	plt.ion()
	
	gen=max(point_A_gen,point_B_gen)
	for point in pointall_A[1:]:
		if point.exi==1 :
			plt.plot([point.xy[0],pointall_A[point.dad].xy[0]],
					 [point.xy[1],pointall_A[point.dad].xy[1]],
					 color=str(1-point.gen/gen))
		else:
			plt.plot([point.xy[0],pointall_A[point.dad].xy[0]],
					 [point.xy[1],pointall_A[point.dad].xy[1]],
					 color="red")
	for point in pointall_B[1:]:
		if point.exi==1:
			plt.plot([point.xy[0],pointall_B[point.dad].xy[0]],
					 [point.xy[1],pointall_B[point.dad].xy[1]],
					 color=str(1-point.gen/gen))
		else:
			plt.plot([point.xy[0],pointall_B[point.dad].xy[0]],
					 [point.xy[1],pointall_B[point.dad].xy[1]],
					 color="red")
	
	ax = plt.gca()
	ax.set_aspect(1)
	plt.pause(time)
	
#----------------------------------几何代替
createnewinistaprobability=0.0000001#生成新初始点概率
#初始时刻 几何代替时间
geo_initime_min=math.sqrt(sum(sta_a*sta_a)-R*R)/v+R*math.asin(R/math.sqrt(sum(sta_a*sta_a)))/v
geo_initime_max=(math.sqrt(sum(sta_b*sta_b))-R)/v
#A初始节点
def getgeo_inista_A(geo_initime):
	geo_inista_A=Point()
	geo_inista_A.xy=np.array([0,-R-0.02])
	geo_inista_A.dad=0
	
	geo_inista_A.gen=int(geo_initime/dt)
	s_A_max=0.5*math.pi-math.asin(R/math.sqrt(sum(geo_inista_A.xy*geo_inista_A.xy)))
	geo_inista_A.direction=round(random.uniform(-s_A_max,s_A_max),1)
	return geo_inista_A
#B初始节点
def getgeo_inista_B(geo_initime):
	r_Bb=geo_initime*v
	s_B_max=math.asin(R/math.sqrt(sum(sta_b*sta_b)))*2
	s_B=math.pi-random.uniform(0,s_B_max)

	geo_inista_B=Point()
	geo_inista_B.xy=sta_b+np.array([r_Bb*math.cos(s_B),r_Bb*math.sin(s_B)])
	geo_inista_B.dad=0
	
	geo_inista_B.gen=int(geo_initime/dt)
	
	geo_inista_B.direction=s_B
	return geo_inista_B



#----------------------------------主函数

ini_A=Point()
ini_A.xy=sta_a
ini_A.dad=-1
ini_A.sel=0
ini_A.gen=0

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

point_B_all=[]
point_B_all.append(ini_B)
point_B_num=0
point_B_gen=0

success_B=0

act_A=1
act_B=1

point_A_num_real=0
point_B_num_real=0

while success_B==0  or success_A==0:
	
	
	#print(point_A_all)
	

	if (point_A_gen<=point_B_gen or 0) and success_A==0:
		act_A=1
		near_A=-1
		if random.uniform(0,1)<createnewinistaprobability or len(point_A_all)<4:
			nextpoint_A=getgeo_inista_A(random.uniform(geo_initime_min,geo_initime_max))
			near_A=ini_A

		else:
			while near_A==-1:
				tar=gettarget(1)
				near_A,direction=getnear(point_A_all[1:],tar,point_A_gen)
				
			nextpoint_A=getnextpoint(near_A,direction)
		
		obsirc_A=obsirc(nextpoint_A.xy)
		inarea_A=inarea(nextpoint_A.xy)

		if obsirc_A and inarea_A:
			if targetvalue(point_B_all,nextpoint_A):

				point_A_num=point_A_num+1

				point_A_num_real=point_A_num_real+1
				point_A_all[near_A.sel].son.append(point_A_num)
				nextpoint_A.sel=point_A_num
				point_A_all.append(nextpoint_A)
				if nextpoint_A.gen>point_A_gen:
					point_A_gen=nextpoint_A.gen
				success_A = end(nextpoint_A.xy,1)



				if point_A_num%100==0:
					print(point_A_num_real,point_A_num,point_A_gen,point_B_num_real,point_B_num,point_B_gen)

				if point_A_num%100==0:
					draw(point_A_all,point_B_all,0.01)

			else:
				point_A_num_real=point_A_num_real-lopcut(point_A_all,near_A)
		else:
			point_A_num_real=point_A_num_real-lopcut(point_A_all,near_A)
	else:
		act_A=0
	
	point_A_all_toB=point_A_all
	point_A_gen_toB=point_A_gen

	if (point_A_gen_toB>=point_B_gen or 0) and success_B==0:
		act_B=1
		near_B=-1
		if random.uniform(0,1)<createnewinistaprobability or len(point_B_all)<4:
			nextpoint_B=getgeo_inista_B(random.uniform(geo_initime_min,geo_initime_max))
			near_B=ini_B
			
		else:
			while near_B==-1:
				tar=gettarget(0)
				near_B,direction=getnear(point_B_all[1:],tar,point_B_gen)
				
			nextpoint_B=getnextpoint(near_B,direction)
		obsirc_B=obsirc(nextpoint_B.xy)
	
		inarea_B=inarea(nextpoint_B.xy)

		if obsirc_B and inarea_B:
			if targetvalue(point_A_all_toB,nextpoint_B):
				point_B_num=point_B_num+1
				point_B_num_real=point_B_num_real+1
				point_B_all[near_B.sel].son.append(point_B_num)
				nextpoint_B.sel=point_B_num
				point_B_all.append(nextpoint_B)
				if nextpoint_B.gen>point_B_gen:
					point_B_gen=nextpoint_B.gen
				success_B = end(nextpoint_B.xy,0)
			else:
				point_B_num_real=point_B_num_real-lopcut(point_B_all,near_B)
		else:
			point_B_num_real=point_B_num_real-lopcut(point_B_all,near_B)
	else:
		act_B=0



	
	
	
print("用时：",point_A_gen*dt)
draw(point_A_all,point_B_all,2000000)
