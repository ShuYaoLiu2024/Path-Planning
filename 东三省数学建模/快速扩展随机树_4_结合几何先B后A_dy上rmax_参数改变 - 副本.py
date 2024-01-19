import math #角度
import random #随机数
import numpy as np #数组
import matplotlib.pyplot as plt #画图
import matplotlib.style as mplstyle
import sys #递归深度
sys.setrecursionlimit(3000)
import copy #深浅复制
import os #文件
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
er=20#结束范围
area=np.array([[-2000,-1000],[4500,1500]])#范围
randomtargetprobability=0.8#随机采样的概率

#----------------------------------题目参数
v=10#速度
v_B=10
r_min=30#最小转弯半径
R=500#障碍圆半径
sta_a=np.array([-1000,0])#站点A位置
sta_b=np.array([3500,0])#站点B位置



#----------------------------------位置更新
#沿子节点删除
def delson(pointall,point):
	delcount=0
	for sonnum in point.son:
		pointall[sonnum].exi=0
		delcount=delcount+1
		#print("del ",pointall[sonnum].xy)
		delcount=delcount+delson(pointall,pointall[sonnum])
	return delcount

#剪枝
def lopcut(pointall,point):
	maxcount=100
	poicount=0
	for i in range(3):

		while len(pointall[point.dad].son)==1 and poicount<maxcount:
			poicount+=1
			point=pointall[point.dad]

	delcount=delson(pointall,point)
	return delcount

#计算下一节点
def getnextpoint(point,direction):

	nextpoint=Point()
	nextpoint.xy=point.xy+np.array([v_B*dt*math.cos(direction),v_B*dt*math.sin(direction)])
	nextpoint.dad=point.sel
	nextpoint.gen=point.gen+1
	nextpoint.direction=direction
	
	return nextpoint

#随机采样
def gettarget(isA,pointall):
	
	if random.uniform(0,1)>randomtargetprobability:
		target=np.multiply([isA], sta_b)+np.multiply([1-isA], sta_a)
	else:
		success=0
		while success==0:
			if isA==1:
				target=[random.choice(np.arange(pointall[1].xy[0],area[1][0],dxy).tolist()),random.choice(np.arange(area[0][1],0,dxy).tolist())]
			if isA==0:
				target=[random.choice(np.arange(area[0][0],pointall[1].xy[0],dxy).tolist()),random.choice(np.arange(0,area[1][1],dxy).tolist())]
			target=np.array(target)
			'''
			if math.sqrt(sum(target*target))>R:
				success=1
			'''
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
		
		radian_max=math.pi-2*math.atan(2*r_min/dt/v_B)
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

def inarea(poi_A,isA):
	
	if poi_A[0]<area[0][0] or poi_A[1]>area[1][0]:
		return 0
	if poi_A[1]<area[0][1] or poi_A[1]>area[1][1]:
		return 0
	if isA==0:
		if poi_A[1]<-2:
			return 0

	if isA==1:
		if poi_A[1]>2:
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
def draw(pointall_A,pointall_B,time,):
	
	plt.close()
	plt.ion()
	
	gen=0
	

	for point in pointall_A[1:]:
		
		if point.gen*dt_A/dt>gen:
			gen=point.gen*dt_A/dt
	
	for point in pointall_B[1:]:
		if point.exi==1:
			if point.gen>gen:
				gen=point.gen

	
	for point in pointall_B[1:]:
		if point.exi==1:
			plt.plot([point.xy[0],pointall_B[point.dad].xy[0]],
					 [point.xy[1],pointall_B[point.dad].xy[1]],
					 color=str(1-point.gen/gen))
			#print([point.xy[0],pointall_B[point.dad].xy[0]],[point.xy[1],pointall_B[point.dad].xy[1]])
		else:
			plt.plot([point.xy[0],pointall_B[point.dad].xy[0]],
					 [point.xy[1],pointall_B[point.dad].xy[1]],
					 color="red")
		
	for point in pointall_A[1:]:
		
		plt.plot([point.xy[0],pointall_A[point.dad].xy[0]],
				 [point.xy[1],pointall_A[point.dad].xy[1]],
				 color=str(1-point.gen*dt_A/dt/gen))
		
	
	ax = plt.gca()
	ax.set_aspect(1)
	plt.pause(time)



def drawpoints(pointall_t,pointb,time):
	
	plt.close()
	plt.ion()
	figure, axes = plt.subplots(figsize = (10, 4))
	x=pointall_t[-1][0].gen
	i=0
	for pointall in pointall_t[:-1]:
		i+=1
		if i%5==0 :

			for point in pointall:
				plt.plot([point.xy[0]],[point.xy[1]],',',color=str((x-point.gen)/x))

	for point in pointall_t[-1]:
		plt.plot([point.xy[0]],[point.xy[1]],',',color="red")
	plt.plot([sta_a[0],sta_b[0]],[sta_a[1],sta_b[1]],'.',color="red")
	plt.plot([pointb[0]],[pointb[1]],'.',color="blue")
	draw_circle =plt.Circle((0, 0), R,fill=False)
	ax = plt.gca()
	plt.gcf().gca().add_artist(draw_circle)
	ax.set_aspect(1)
	plt.pause(time)
	



def draw_save(pointall_A,pointall_B,time,save,inf):
	if save >=1:
		plt.close()
	if save ==2:
		plt.ion()
	
	gen=0
	figure, axes = plt.subplots(figsize = (10, 4))

	for point in pointall_A[1:]:
		
		if point.gen*dt_A/dt>gen:
			gen=point.gen*dt_A/dt
	
	for point in pointall_B[1:]:
		if point.exi==1:
			if point.gen>gen:
				gen=point.gen

	
	for point in pointall_B[1:]:
		
		if point.exi==1  :
			plt.plot([point.xy[0],pointall_B[point.dad].xy[0]],
					 [point.xy[1],pointall_B[point.dad].xy[1]],
					 color=str(1-point.gen/gen))
			#print([point.xy[0],pointall_B[point.dad].xy[0]],[point.xy[1],pointall_B[point.dad].xy[1]])
		else:
			plt.plot([point.xy[0],pointall_B[point.dad].xy[0]],
					 [point.xy[1],pointall_B[point.dad].xy[1]],
					 color="red")
		
	for point in pointall_A[1:]:
		
		plt.plot([point.xy[0],pointall_A[point.dad].xy[0]],
				 [point.xy[1],pointall_A[point.dad].xy[1]],
				 color=str(1-point.gen*dt_A/dt/gen))
		
	draw_circle =plt.Circle((0, 0), R,fill=False)
	ax = plt.gca()
	plt.gcf().gca().add_artist(draw_circle)
	
	ax.set_aspect(1)
	if save >=1:

		plt.savefig(inf+".png")
	if save ==2:
		plt.pause(time)


#----------------------------------几何代替
createnewinistaprobability=0.0000001#生成新初始点概率
#初始时刻 几何代替时间

#A初始节点
def getgeo_inista_A(geo_initime):
	geo_inista_A=Point()
	geo_inista_A.xy=np.array([0,-R-0.01])
	geo_inista_A.dad=0
	
	geo_inista_A.gen=int(geo_initime/dt_A)
	s_A_max=0.5*math.pi-math.asin(R/math.sqrt(sum(geo_inista_A.xy*geo_inista_A.xy)))
	geo_inista_A.direction=0
	return geo_inista_A
#B初始节点
def getgeo_inista_B(geo_initime):
	r_Bb=geo_initime*v_B
	s_B_max=math.asin(R/math.sqrt(sum(sta_b*sta_b)))
	s_B=math.pi-random.uniform(0,s_B_max)

	geo_inista_B=Point()
	geo_inista_B.xy=sta_b+np.array([r_Bb*math.cos(s_B),r_Bb*math.sin(s_B)])
	geo_inista_B.dad=0
	
	geo_inista_B.gen=int(geo_initime/dt)
	
	geo_inista_B.direction=s_B
	return geo_inista_B


#----------------------------------生成路径
def getpath_B():
	geo_initime_min=math.sqrt(sum(sta_a*sta_a)-R*R)/v+R*math.asin(R/math.sqrt(sum(sta_a*sta_a)))/v
	geo_initime_max=(math.sqrt(sum(sta_b*sta_b))-R-100)/v_B
	if geo_initime_min>=geo_initime_max:
		return 1,[],0,0
	success_B=0
	count2=0
	while success_B==0:
		count2+=1
		if count2>20:
			print("2")
			return 1,[],0,0
		ini_B=Point()
		ini_B.xy=sta_b
		ini_B.dad=-1
		ini_B.sel=0
		ini_B.gen=0
		point_B_all=[]
		point_B_all.append(ini_B)
		point_B_num=0
		point_B_gen=0
		point_B_num_real=0
		failcount=0
		path_B=[]
		count=0
		while success_B==0 and point_B_num<2000 :
			if count>2000:
				print("0")
				return 1,[],0,0
			count+=1
			near_B=-1
			if len(point_B_all)==1:
				nextpoint_B=getgeo_inista_B(random.uniform(geo_initime_min,geo_initime_max))
				near_B=ini_B
			else:
				count1=0
				while near_B==-1 :
					tar=gettarget(0,point_B_all)
					near_B,direction=getnear(point_B_all[1:],tar,point_B_gen)
					count1+=1

					if count1==2000:
						print("1")
						return 1,[],0,0
				nextpoint_B=getnextpoint(near_B,direction)
			obsirc_B=obsirc(nextpoint_B.xy)
			inarea_B=inarea(nextpoint_B.xy,0)
			if obsirc_B and inarea_B:
				failcount=0
				point_B_num=point_B_num+1
				point_B_num_real=point_B_num_real+1
				point_B_all[near_B.sel].son.append(point_B_num)
				nextpoint_B.sel=point_B_num
				point_B_all.append(nextpoint_B)
				if nextpoint_B.gen>point_B_gen:
					point_B_gen=nextpoint_B.gen
				success_B = end(nextpoint_B.xy,0)
				if point_B_num%100==0 :
					print("|",end="")
				'''
				if point_B_num==1:
					draw([],point_B_all,0.5)
				'''
			else:
				failcount+=1
				point_B_num_real=point_B_num_real-lopcut(point_B_all,near_B)
		if success_B==1 :
			
			path_B_t=[]
			startgen=point_B_all[1].gen
			while nextpoint_B.dad != -1:
				geo_point=copy.copy(nextpoint_B)
				path_B_t.append(geo_point)
				nextpoint_B=point_B_all[nextpoint_B.dad]
			for i in range(point_B_all[1].gen-1,0-1,-1):
				geo_point=Point()
				geo_point.direction=point_B_all[1].direction
				geo_point.xy=sta_b+np.array([v_B*dt*i*math.cos(geo_point.direction),v_B*dt*i*math.sin(geo_point.direction)])

				geo_point.gen=i
				path_B_t.append(geo_point)
			endgen=0
			for i in range(len(path_B_t)-1,0-1,-1):
				if path_B_t[i].xy[0]<0 and endgen==0:
					endgen=path_B_t[i].gen
				path_B_t[i].sel=len(path_B_t)-1-i
				path_B_t[i].son=len(path_B_t)-1-i+1
				path_B_t[i].dad=len(path_B_t)-1-i-1
				path_B.append(path_B_t[i])
	return 0,path_B,startgen,endgen
'''
def getpath_A(path_B,startgen_B,endgen_B):
	success_A=0
	while success_A==0:
		ini_A=Point()
		ini_A.xy=sta_a
		ini_A.dad=-1
		ini_A.sel=0
		ini_A.gen=0
		point_A_all=[]
		point_A_all.append(ini_A)
		point_A_num=0
		point_A_gen=0
		point_A_num_real=0
		failcount=0
		path_A=[]
		while success_A==0 and point_A_num<3000:
			near_A=-1
			if len(point_A_all)==1:
				nextpoint_A=getgeo_inista_A(int(startgen_B*dt))
				near_A=ini_A
			else:
				while near_A==-1:
					tar=gettarget(1,point_A_all)
					near_A,direction=getnear(point_A_all[1:],tar,point_A_gen)
				nextpoint_A=getnextpoint(near_A,direction)
			obsirc_A=obsirc(nextpoint_A.xy)
			inarea_A=inarea(nextpoint_A.xy,1)
			if obsirc_A and inarea_A:
				failcount=0
				point_A_num=point_A_num+1
				point_A_num_real=point_A_num_real+1
				point_A_all[near_A.sel].son.append(point_A_num)
				nextpoint_A.sel=point_A_num
				point_A_all.append(nextpoint_A)
				if nextpoint_A.gen>point_A_gen:
					point_A_gen=nextpoint_A.gen
				success_A = end(nextpoint_A.xy,0)
				if point_A_num%100==0 :
					print("|",end="")
			else:
				failcount+=1
				point_A_num_real=point_A_num_real-lopcut(point_A_all,near_A)
		if point_A_num<=2000 :
			print(" ")
			path_A_t=[]
			startgen=point_A_all[1].gen
			while nextpoint_A.dad != -1:
				geo_point=copy.copy(nextpoint_A)
				path_A_t.append(geo_point)
				nextpoint_A=point_A_all[nextpoint_A.dad]
			for i in range(point_A_all[1].gen-1,0-1,-1):
				geo_point=Point()
				geo_point.direction=point_A_all[1].direction
				geo_point.xy=sta_a+np.array([v*dt*i*math.cos(geo_point.direction),v*dt*i*math.sin(geo_point.direction)])
				geo_point.gen=i
				path_A_t.append(geo_point)
			endgen=0
			for i in range(len(path_A_t)-1,0-1,-1):
				if path_A_t[i].xy[0]>0 and endgen==0:
					endgen=path_A_t[i].gen
				path_A_t[i].sel=len(path_A_t)-1-i
				path_A_t[i].son=len(path_A_t)-1-i+1
				path_A_t[i].dad=len(path_A_t)-1-i-1
				path_A.append(path_A_t[i])
	return path_A
'''


#----------------------------------几何方法计算A

dt_A=2#时间精度


dr=1#长度精度
radian_max_A=math.pi-2*math.atan(2*r_min/dt_A/v)#转角取值范围


#边界点
class Boundarypoint:
    def __init__(self):
    	self.xy = None
    	self.dad = None
    	self.sel = None
    	self.son = []
    	self.gen = None
    	self.direction = None
    	self.exi=1
    	self.r_PB=0
    	self.r_PD=0


#计算切线
def gettangentline(point):
	poi=point.xy
	r_p=math.sqrt(sum(poi*poi))
	c=1.5*math.pi-math.acos(R/r_p)+math.atan2(poi[1],poi[0])
	c_t=c%(2*math.pi)
	sinc=math.sin(c_t%(math.pi))
	if sinc==0:
		return 0
	f_x=R/sinc
	return c_t,f_x,point.gen

#形状发展
def getnextboundarypoints(pointall,path_B):
	nextboundarypoints_may=[]
	dy=10
	
	dradian_A=radian_max_A/(-1.2*area[0][1]/len(pointall)/dy+10)
	
	
	y_max=area[0][1]
	y_min=0
	for point in pointall:

		for directions in np.arange(point.direction-radian_max_A,point.direction+radian_max_A,dradian_A):
			newpoint=Boundarypoint()
			newpoint.xy=point.xy+np.array([v*dt_A*math.cos(directions),v*dt_A*math.sin(directions)])
			newpoint.gen=point.gen+1
			newpoint.direction=directions
			if obsirc(newpoint.xy) and inarea(newpoint.xy,1) :
				count=0
				while obsirc(newpoint.xy) and dismeet(newpoint.xy,path_B[min(int(newpoint.gen*dt_A/dt),len(path_B)-1)].xy)==0 and count<=v*dt_A/dr+200:
					newpoint.xy[0]-=dr*math.cos(directions)
					newpoint.xy[1]-=dr*math.sin(directions)
					
					count+=1
				
				if count<v*dt_A/dr and obsirc(newpoint.xy):
					newpoint.dad=point.sel
					newpoint.r_PD=math.sqrt(sum((newpoint.xy-np.array([0,-R]))*(newpoint.xy-np.array([0,-R]))))
					newpoint.r_PB=math.sqrt(sum((newpoint.xy-sta_b)*(newpoint.xy-sta_b)))
					nextboundarypoints_may.append(newpoint)
					y_max=max(y_max,newpoint.xy[1])
					y_min=min(y_min,newpoint.xy[1])
	nextboundarypoints=[]
	
	dy=(y_max-y_min+10)/80
	
	for y in np.arange(y_max,y_min,-dy):
		max_r=-10000
		find=0

		for point_may in nextboundarypoints_may:
			
			if point_may.xy[1] > y-dy and point_may.xy[1] <= y:
				r=point_may.r_PD-point_may.r_PB
				r=point_may.xy[0]
				if max_r<r:
					find=1
					max_r=r
					point=point_may
					
		if find !=0:
			
			point.sel=len(nextboundarypoints)
			
			pointall[point.dad].son.append(point.sel)
			nextboundarypoints.append(point)
	return nextboundarypoints,pointall

#判断结束
def end_A(pointall):
	for point in pointall:
		if end(point.xy,1):
			return 1,point.sel
	return 0,0


#生成A路径
def getpath_A(path_B,startgen_B,endgen_B):
	success=0
	
	ini_A=Point()
	ini_A.xy=sta_a
	ini_A.dad=-1
	ini_A.sel=0
	ini_A.gen=0
	point_A_all=[]
	point_A_all.append([ini_A])
	ini_A_geo=getgeo_inista_A(startgen_B*dt)
	ini_A_geo.sel=0
	
	point_A_all.append([ini_A_geo])

	
	gen_geoini=ini_A_geo.gen
	endgen=int(endgen_B*dt/dt_A)-gen_geoini
	gen=1
	print(int(endgen),end="")
	while success==0 and gen<endgen and error==0:
		
		nextpoints,point_A_all[gen]=getnextboundarypoints(point_A_all[gen],path_B)
		if len(nextpoints)==0:
			
			return 1,[]
		'''
		if gen%20==0:
			drawpoints(point_A_all,path_B[int((gen+gen_geoini-1)*dt_A/dt)].xy,1)
		'''
		point_A_all.append(nextpoints)
		print("|",end="")
		gen+=1
		end,endpointsel=end_A(nextpoints)
		if end:
			success=1


	if success==0:
		r_min=100000
		for point in point_A_all[gen]:
			r=math.sqrt(sum((point.xy-sta_b)*(point.xy-sta_b)))
			if r<r_min:
				r_min=r
				endpointsel=point
		point_F=copy.copy(endpointsel)

	path_A_t=[]
	


	while endpointsel.gen != 0:
		point=copy.copy(endpointsel)
		path_A_t.append(point)
		endpointsel=point_A_all[endpointsel.gen-1-gen_geoini+1][endpointsel.dad]
	path_A=[]
	
	for i in range(len(path_A_t)-1,0-1,-1):
		
		path_A_t[i].sel=len(path_A_t)-1-i
		path_A_t[i].son=len(path_A_t)-1-i+1
		path_A_t[i].dad=len(path_A_t)-1-i-1
		path_A.append(path_A_t[i])

	if success==0:
		for i in range(1,int(r_min/dt_A/v)):
			point=Boundarypoint()
			point.xy=point_F.xy+np.multiply([i*dt_A*v/r_min],sta_b-point_F.xy)
			point.sel=len(path_A)
			point.son=len(path_A)+1
			point.dad=len(path_A)-1
			point.direction=math.atan2(sta_b[1]-point_F.xy[1],sta_b[0]-point_F.xy[0])
			point.gen=i+point_F.gen
			path_A.append(point)

	return 0,path_A



#----------------------------------主函数
#序号_A用时_B用时_b距离_B速度_dt
path="路径图像_参数改变_1"
fileList=os.listdir(path)

n=0
maxnumber=0
for name in fileList:
    maxnumber=max(int(name[:name.find("_",0)]),maxnumber)
    n+=1

mintimeall=[]

for css in np.arange(0.98,1.03,0.01):

	sta_b[0]=3500

	
	v_B=10
	
	dxy=css

	mintime=10000

	for i in range(20):
	


		error,path_B,startgen,endgen=getpath_B()
		if error ==0:
			print(len(path_B))
			#draw([],path_B,1)
			error = 0
			error,path_A=getpath_A(path_B,startgen,endgen)
			if error ==0:
				maxnumber+=1
				inf=path+os.sep+str(maxnumber)+"_"+ str(path_A[-1].gen*dt_A)+"_"+ str(path_B[-1].gen*dt)+"_" +str(sta_b[0]) +"_"+ str(v_B)
				draw_save(path_A,path_B,1,1,inf)
				print(path_A[-1].gen*dt_A)



				if sta_b[0]==3500 and v_B==10:
					mintime=min(path_A[-1].gen*dt_A,mintime)

				print("Q2 min:",mintime)


			else:
				print("error")
		else:
			print("error")
	
	mintimeall.append([dxy,mintime])
	

print(mintimeall)
