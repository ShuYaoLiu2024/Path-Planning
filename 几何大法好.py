import math #角度
import numpy as np #数组
import matplotlib.pyplot as plt #画图
import matplotlib.style as mplstyle
import time
import copy #深浅复制
import os #文件
mplstyle.use('fast')

class Node:
    def __init__(self):
        self.xy = None
        self.number = None
        self.R=None

def get_r(point1,point2):
	R=point1-point2
	r=math.sqrt(sum(np.multiply(R,R)))
	return r

def get_direction(point1,point2):
	r=point2-point1
	direction=math.atan2(r[1],r[0])
	return direction

def get_tpoints(point,node):
	r=get_r(point,node.xy)
	R=node.R
	if R>=r:
		return []
	s=math.sqrt(r*r-R*R)
	direction=get_direction(point,node.xy)
	angle=math.atan2(R,s)
	tpoints=[np.multiply([s],np.array([math.cos(direction+angle),math.sin(direction+angle)]))+point]
	tpoints.append(np.multiply([s],np.array([math.cos(direction-angle),math.sin(direction-angle)]))+point)
	return tpoints

def get_tpoints44(node_f,node_t):
	r=get_r(node_f.xy,node_t.xy)
	R_f=node_f.R
	R_t=node_t.R
	
	direction=get_direction(node_f.xy,node_t.xy)

	d_R=R_t-R_f
	if abs(d_R)>=r:
		return [],[]

	s=math.sqrt(r*r-d_R*d_R)
	angle=math.atan2(d_R,s)
	direction1=direction+angle+math.pi/2
	tpoints_f=[np.multiply([R_f],np.array([math.cos(direction1),math.sin(direction1)]))+node_f.xy]
	tpoints_t=[np.multiply([R_t],np.array([math.cos(direction1),math.sin(direction1)]))+node_t.xy]
	direction1=direction-angle-math.pi/2
	tpoints_f.append(np.multiply([R_f],np.array([math.cos(direction1),math.sin(direction1)]))+node_f.xy)
	tpoints_t.append(np.multiply([R_t],np.array([math.cos(direction1),math.sin(direction1)]))+node_t.xy)

	a_R=R_t+R_f
	if a_R>=r:
		return tpoints_f,tpoints_t

	s=math.sqrt(r*r-a_R*a_R)
	angle=math.atan2(a_R,s)
	direction1=direction-angle+math.pi/2
	tpoints_f.append(np.multiply([R_f],np.array([math.cos(direction1),math.sin(direction1)]))+node_f.xy)
	tpoints_t.append(np.multiply([R_t],np.array([math.cos(direction1+math.pi),math.sin(direction1+math.pi)]))+node_t.xy)
	direction1=direction+angle-math.pi/2
	tpoints_f.append(np.multiply([R_f],np.array([math.cos(direction1),math.sin(direction1)]))+node_f.xy)
	tpoints_t.append(np.multiply([R_t],np.array([math.cos(direction1+math.pi),math.sin(direction1+math.pi)]))+node_t.xy)

	return tpoints_f,tpoints_t

def is_ok_connection(obstacles,connection,node_f,node_t):
	tpoints_f,tpoints_t=connection[0],connection[1]
	ok=1
	for obstacle in obstacles:
		if not obstacle.number==node_f.number and not obstacle.number==node_t.number:
			R=obstacle.R
			r=get_r(tpoints_f,tpoints_t)
			d=100#-----------------------------------------------------------检测精度
			
			for i in range(d):
				point=tpoints_f+np.multiply([i/d],tpoints_t-tpoints_f) 
				if get_r(point,obstacle.xy)<=R:
					ok=0
	return ok 

def get_connection(node_f,node_t,obstacles):
	if node_f.number==-2:
		if node_t.number==-1:
			#Begin2End
			connection_co=[[node_f.xy,node_t.xy]]
		else:
			#Begin2Obstacle
			tpoints=get_tpoints(node_f.xy,node_t)
			connection_co=[]
			for tpoint in tpoints:
				connection_co.append(copy.copy([node_f.xy,tpoint]))

	else:
		if node_t.number==-1:
			#Obstacle2End
			tpoints=get_tpoints(node_t.xy,node_f)
			connection_co=[]
			for tpoint in tpoints:
				connection_co.append(copy.copy([tpoint,node_t.xy]))

		else:
			#Obstacle2Obstacle
			tpoints_f,tpoints_t=get_tpoints44(node_f,node_t)
			connection_co=[]
			
			for i in range(len(tpoints_f)):
				connection_co.append(copy.copy([tpoints_f[i],tpoints_t[i]]))

	connection_co_true=[]
	for connection in connection_co:
		if is_ok_connection(obstacles,connection,node_f,node_t):
			connection_co_true.append(connection)

	return connection_co_true


def get_connections(nodes):#nodes:Begin,Obstacles,End -2,0~N,-1
	connections=[]
	connection_number=-1
	for number_f in range(len(nodes)):
		for number_t in range(number_f+1,len(nodes)):
			node_f=nodes[number_f]
			node_t=nodes[number_t]
			connection_co=get_connection(node_f,node_t,nodes[1:-1])
			for connection in connection_co:
				connection_number+=1
				connection_sim=[connection_number,[node_f.number,node_t.number],connection]
				connections.append(copy.copy(connection_sim))
				connection_number+=1
				connection_sim=[connection_number,[node_t.number,node_f.number],[connection[1],connection[0]]]
				connections.append(copy.copy(connection_sim))
	return connections

def search_connection(connections,node_f_number):
	connection_numbers=[]
	for connection in connections:
		if connection[1][0]==node_f_number:
			connection_numbers.append(connection[0])
	return connection_numbers

def nodes_number2nodes_index(nodes,nodes_number):

	if nodes_number==-2:
		nodes_index=0
	elif nodes_number==-1:
		nodes_index=len(nodes)-1
	else:
		nodes_index=nodes_number+1
	return nodes_index

def check_not_err_detour(nodes,connection_t,connection_f):

	Obstacles=nodes[1:-1]
	Obstacle=Obstacles[connection_t[1][1]]

	O=Obstacle.xy
	R=Obstacle.R

	point_t=connection_t[2][1]
	point_f=connection_f[2][0]

	detourdirection=get_detourdirection(nodes,connection_t,1)

	radian=2*math.asin(get_r(point_t,point_f)/R/2)
	tf2tt=connection_t[2][1]-connection_t[2][0]
	t2f=point_f-point_t
	if t2f[0]*tf2tt[0]+t2f[1]*tf2tt[1]<0:
		radian=math.pi*2-radian

	points=[]
	for d in range(100):
		diraction=get_direction(O,point_t)+d/100*radian*(detourdirection*2-1)
		points.append(O+np.multiply([R],np.array([math.cos(diraction),math.sin(diraction)])))


	for Obstacle_other in Obstacles:
		O_other=Obstacle_other.xy
		R_other=Obstacle_other.R
		if get_r(O_other,O)<=R_other+R and Obstacle_other.number!=Obstacle.number:
			for point in points:
				if R_other>=get_r(point,O_other):
					
					return 0
	return 1

def check_not_err_detours(nodes,connections,path):
	noerr=1
	for i in range(len(path[:-1])):
		connection_number=path[i]
		
		noerr*=check_not_err_detour(nodes,connections[connection_number],connections[path[i+1]])

	return noerr



def check_not_err(nodes,connections,path):


	for i in range(len(path)-1):
		connection_i_number=path[i]
		if len(path)>1:
			
			j=len(path)-1
			connection_j_number=path[j]
			if connection_i_number==connection_j_number:
				return 0
			if abs(connection_i_number-connection_j_number)==1 and min(connection_i_number,connection_j_number)%2==0:
				return 0
			if connections[connection_i_number][1][0]==connections[connection_j_number][1][1]:
				return 0
			if j-i==1:
				if not get_detourdirection(nodes,connections[connection_i_number],1)==get_detourdirection(nodes,connections[connection_j_number],0):
					return 0
			node_t_i_O=nodes[nodes_number2nodes_index(nodes,connections[connection_i_number][1][1])].xy
			node_t_j_O=nodes[nodes_number2nodes_index(nodes,connections[connection_j_number][1][1])].xy
			Begin_O=nodes[0].xy
			r_i=get_r(node_t_i_O,Begin_O)
			r_j=get_r(node_t_j_O,Begin_O)
			if r_i>r_j:
				return 0
	if check_not_err_detours(nodes,connections,path)==0:
		
		return 0
	
	return 1


def get_paths(nodes,connections):
	paths=[]
	Start_connections=search_connection(connections,-2)
	for Start_connection in Start_connections:
		paths.append([Start_connection])
	
	success=0
	
	while success==0:
		
		paths_new=[]
		success=1
		for path in paths:
			#draw(nodes,connections,path)
			if not connections[path[-1]][1][1]==-1:#最后一个连接的终点不是End时
				
				next_connections=search_connection(connections,connections[path[-1]][1][1])
				
				for next_connection in next_connections:
					path_new=copy.copy(path)
					path_new.append(next_connection)

					if check_not_err(nodes,connections,path_new):
						success=0
						paths_new.append(copy.copy(path_new))
			else:
				paths_new.append(copy.copy(path))

		paths=paths_new
	return paths

def check_equal_direction(direction_1,direction_2,d):
	ddirection=abs(direction_1-direction_2)
	

	if ddirection%(math.pi*2)<d:
		
		return 1
	
	return 0

def get_detourdirection(nodes,connection,m):#逆时针为1
	
	Obstacles=nodes[1:-1]
	if m==1:#向后
		O=Obstacles[connection[1][1]].xy
		point_t=connection[2][1]
		point_f=connection[2][0]
	else:
		O=Obstacles[connection[1][0]].xy
		point_t=connection[2][0]
		point_f=connection[2][1]
	direction_t=get_direction(point_f,point_t)
	direction_r=get_direction(O,point_t)
	direction_1=direction_r+math.pi/2
	
	return -m+1+(2*m-1)*check_equal_direction(direction_1,direction_t,0.1)

def get_cost_detour(nodes,connection_t,connection_f):
	Obstacles=nodes[1:-1]
	Obstacle=Obstacles[connection_t[1][1]]

	O=Obstacle.xy
	R=Obstacle.R

	point_t=connection_t[2][1]
	point_f=connection_f[2][0]
	detourdirection=get_detourdirection(nodes,connection_t,1)

	radian=2*math.asin(get_r(point_t,point_f)/R/2)
	tf2tt=connection_t[2][1]-connection_t[2][0]
	t2f=point_f-point_t
	if t2f[0]*tf2tt[0]+t2f[1]*tf2tt[1]<0:
		radian=math.pi*2-radian

	return radian*R

def get_cost(nodes,connections,path):
	cost=0
	for i in range(len(path[:-1])):
		connection_number=path[i]
		cost+=get_r(connections[connection_number][2][0],connections[connection_number][2][1])
		cost+=get_cost_detour(nodes,connections[connection_number],connections[path[i+1]])

	connection_number=path[-1]
	cost+=get_r(connections[connection_number][2][0],connections[connection_number][2][1])
	return cost

def get_costs(nodes,connections,paths):
	costs=[]
	for path in paths:
		costs.append(get_cost(nodes,connections,path))
	return costs

def search_bestpath(nodes,connections,paths):
	costs=get_costs(nodes,connections,paths)
	min_cost=costs[0]
	bestpath=paths[0]
	for i in range(len(costs)):
		if min_cost>costs[i]:
			min_cost=costs[i]
			bestpath=paths[i]
	return bestpath

def search_node_under(nodes,point):
	a=[]
	R_min=nodes[1].R
	for node in nodes[1:-1]:
		if R_min>node.R:
			R_min=node.R

	for node in nodes:
		if node.number<0:
			R=R_min
		else:
			R=node.R
		O=node.xy
		r=get_r(O,point)
		if r<=R:
			a.append(node.number)
	return a

def app_node(nodes,point):
	R_average=0
	for node in nodes[1:-1]:
		R_average+=node.R/len(nodes[1:-1])

	nodes_new=copy.copy(nodes[0:-1])
	newnode=Node()
	newnode.number=len(nodes[0:-1])-1
	newnode.xy=point
	newnode.R=R_average
	nodes_new.append(newnode)
	nodes_new.append(nodes[-1])
	return nodes_new


def draw_detour(nodes,connection_t,connection_f):
	detour_line_xs=[]
	detour_line_ys=[]
	Obstacles=nodes[1:-1]
	Obstacle=Obstacles[connection_t[1][1]]

	O=Obstacle.xy
	R=Obstacle.R

	point_t=connection_t[2][1]
	point_f=connection_f[2][0]

	detourdirection=get_detourdirection(nodes,connection_t,1)

	radian=2*math.asin(get_r(point_t,point_f)/R/2)
	tf2tt=connection_t[2][1]-connection_t[2][0]
	t2f=point_f-point_t
	if t2f[0]*tf2tt[0]+t2f[1]*tf2tt[1]<0:
		radian=math.pi*2-radian

	for d in range(100):
		diraction=get_direction(O,point_t)+d/100*radian*(detourdirection*2-1)
		point=O+np.multiply([R],np.array([math.cos(diraction),math.sin(diraction)]))
		detour_line_xs.append(point[0])
		detour_line_ys.append(point[1])
				
	return [detour_line_xs,detour_line_ys]

def draw_detours(nodes,connections,path):
	detour_lines=[]
	for i in range(len(path[:-1])):
		connection_number=path[i]
		
		detour_lines.append(draw_detour(nodes,connections[connection_number],connections[path[i+1]]))

	return detour_lines


def draw(nodes,connections,paths,path,fig):
	

	Start=nodes[0]
	End=nodes[-1]
	Obstacles=nodes[1:-1]


	plt.clf()
	plt.ion()
	ax = plt.gca()

	Xlim=[min(Start.xy[0],End.xy[0])-50,max(Start.xy[0],End.xy[0])+50]
	Ylim=[min(Start.xy[1],End.xy[1])-50,max(Start.xy[1],End.xy[1])+50]
	for Obstacle in Obstacles:
		Xlim=[min(Xlim[0],Obstacle.xy[0]-Obstacle.R-50),max(Xlim[1],Obstacle.xy[0]+Obstacle.R+50)]
		Ylim=[min(Ylim[0],Obstacle.xy[1]-Obstacle.R-50),max(Ylim[1],Obstacle.xy[1]+Obstacle.R+50)]
		
		O=Obstacle.xy
		R=Obstacle.R
		ax.add_artist(plt.Circle((O[0],O[1]),R,color='gray',fill='True'))
	plt.xlim((Xlim[0], Xlim[1]))
	plt.ylim((Ylim[0], Ylim[1]))
	'''
	for connection in connections:
		point_f=connection[2][0]
		point_t=connection[2][1]
		plt.plot([point_f[0],point_t[0]],[point_f[1],point_t[1]],color="gray")
		if connection[0]%2==0:
			plt.text((point_f[0]+point_t[0])/2,(point_f[1]+point_t[1])/2,str(connection[0]),
				fontsize=9,verticalalignment="top",horizontalalignment="left")
	'''
	'''
	for path1 in paths:
		for i in range(len(path1[:-1])):
			connection_number=path1[i]
			point_f=connections[path1[i+1]][2][0]
			point_t=connections[connection_number][2][1]

			cost=get_cost_detour(nodes,connections[connection_number],connections[path1[i+1]])
			plt.text((point_f[0]+point_t[0])/2,(point_f[1]+point_t[1])/2,str(cost),
					fontsize=9,verticalalignment="top",horizontalalignment="left")
	'''
	detour_lines=draw_detours(nodes,connections,path)
	for detour_line in detour_lines:
		plt.plot(detour_line[0],detour_line[1],color="blue")


	for connection_number in path:
		point_f=connections[connection_number][2][0]
		point_t=connections[connection_number][2][1]
		plt.plot([point_f[0],point_t[0]],[point_f[1],point_t[1]],color="blue")

	ax.set_aspect(1)
	plt.pause(0.001)
	plt.ioff()

if __name__=='__main__':
	Begin=Node()
	Begin.number=-2
	End=Node()
	End.number=-1

	Obstacles=[[90,[200,0]],]#,[100,[400,400]]
	Begin.xy=np.array([0,0])
	End.xy=np.array([800,0])

	nodes=[Begin]
	for i in range(len(Obstacles)):
		Obstacle=Node()
		Obstacle.number=i
		Obstacle.R=Obstacles[i][0]
		Obstacle.xy=np.array(Obstacles[i][1])
		nodes.append(copy.copy(Obstacle))
	nodes.append(End)


	#交互
	Mouse_Down=0
	fig = plt.figure()
	def mouse_move(event):
		global Mouse_Down
		global nodes
		
		point=np.array([event.xdata,event.ydata])
		if Mouse_Down==1:
			nodes_under_numbers=search_node_under(nodes,point)
			if len(nodes_under_numbers)==1:
				nodes_under_number=nodes_under_numbers[0]
				nodes[nodes_number2nodes_index(nodes,nodes_under_number)].xy=point
				
	def mouse_down(event):
		global Mouse_Down
		global nodes
		if event.button==1:
			Mouse_Down=1
			point=np.array([event.xdata,event.ydata])
			nodes_under_numbers=search_node_under(nodes,point)
			if len(nodes_under_numbers)==0:
				nodes=app_node(nodes,point)

		if event.button==3:
			
		
			point=np.array([event.xdata,event.ydata])
			nodes_under_numbers=search_node_under(nodes,point)
			if len(nodes_under_numbers)==1:
				
				nodes_under_number=nodes_under_numbers[0]
				nodes_new=[]
				node_cut_index=nodes_number2nodes_index(nodes,nodes_under_number)
				
				for i in range(len(nodes)):
					if node_cut_index!=i:
						if node_cut_index<i and i!=len(nodes)-1:
							nodes[i].number-=1
							
						nodes_new.append(copy.copy(nodes[i]))
				nodes=nodes_new
	
	def mouse_up(event):
		global Mouse_Down
		
		if event.button==1:
			Mouse_Down=0
	
	def scroll(event):
		global nodes
		
		point=np.array([event.xdata,event.ydata])
		nodes_under_numbers=search_node_under(nodes,point)
		if len(nodes_under_numbers)==1:
			if event.button=='up':
				nodes_under_number=nodes_under_numbers[0]
				nodes[nodes_number2nodes_index(nodes,nodes_under_number)].R*=1.05
			if event.button=='down':
				nodes_under_number=nodes_under_numbers[0]
				nodes[nodes_number2nodes_index(nodes,nodes_under_number)].R*=0.95

	fig.canvas.mpl_connect('motion_notify_event', mouse_move)
	fig.canvas.mpl_connect('button_press_event', mouse_down)
	fig.canvas.mpl_connect('button_release_event', mouse_up)
	fig.canvas.mpl_connect('scroll_event', scroll)

	while 1:
		time0=time.time()
		connections=get_connections(nodes)
		time1=time.time()
		print(time1-time0,end=" ")
		time0=time1
		paths=get_paths(nodes,connections)
		time1=time.time()
		print(time1-time0,end=" ")
		time0=time1
		bestpath=search_bestpath(nodes,connections,paths)
		time1=time.time()
		print(time1-time0,end=" ")
		time0=time1
		draw(nodes,connections,paths,bestpath,fig)
		time1=time.time()
		print(time1-time0)
		time0=time1
	