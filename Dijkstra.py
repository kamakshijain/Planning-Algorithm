
import math
import numpy as np
import cv2
import matplotlib.pyplot as plt
import time
import heapq
GUI=True


#Define Node
class Node:
	def __init__(self,x,y,c2c,parentId):
		self.x = x
		self.y = y
		self.c2c = c2c
		self.parentId = parentId
	def __str__(self):
		return str(self.x)+","+str(self.y)+","+str(self.c2c)+","+str(self.parentId)

#Define costs for each movement
def movement():
	# dx, dy, cost

	motion = [[1,0,1],                #Move Right
			  [1,1,math.sqrt(2)],     #Move Right Up
			  [0,1,1],                #Move Up
			  [-1,1,math.sqrt(2)],    #Move Left Up
			  [-1,0,1],               #Move Left
			  [-1,-1,math.sqrt(2)],   #Move Left Down
			  [0,-1,1],               #Move Down
			  [1,-1,math.sqrt(2)]]    #Move Left Down
	return motion

def gen_index(node):
	return node.x*250+node.y


def obstacle(node, grid,r):

	if grid[int(node.y)][int(node.x)] ==False:
		return False
	elif node.y>150/r:
		return False
	elif node.x < 0:
		return False
	elif node.x >250/r:
		return False
	elif node.y< 0:
		return False
	else:
		return True



def Dijkstra(nStart,nGoal,rr,grid,ox,oy,r):
	
	motion = movement()
	cost = []
	surr, explored = dict(),dict()
	surr = dict()
	
	surr[gen_index(nStart)] = nStart
	# open.add(gen_index(nStart))
	heapq.heappush(cost,[nStart.c2c,nStart])

	#cache the background for plotting
	fig,ax = plt.subplots(figsize = (25,15))
	canvas = ax.figure.canvas
	ax.grid(True)
	plt.xlim(0,270/r)
	plt.ylim(0,160/r)
	# ax.hold(True)
	x = np.linspace(1,150/r,150/r)
	y = np.linspace(1,250/r,250/r)
	for i in x:
		plt.axhline(y=i,color='snow')
	for j in y:
		plt.axvline(x=j,color='snow')
	p1 = ax.plot(ox,oy,"c.")
	p2 = ax.plot(nStart.x,nStart.y,"go")
	p3 = ax.plot(nGoal.x,nGoal.y,"ro")
	ax.grid(True)

	plt.show(False)
	ax.hold(True)
	canvas.draw()

	background = fig.canvas.copy_from_bbox(ax.bbox)
	pts = ax.plot(nStart.x,nStart.y,"y*")[0]
	j = 0
	k = 100/r
	pointsx = []
	pointsy = []
	while 1:
		if len(cost)==0:
			print("Goal cannot be found")
			break

		curr = heapq.heappop(cost)[1]
		# print("test",curr)
		curr_id = gen_index(curr)
		pointsx.append(curr.x)
		pointsy.append(curr.y)
		#Draw Graph to show node exploration
		if (j%k == 0) or (curr.x == nGoal.x and curr.y == nGoal.y):
			
			pts.set_data(pointsx,pointsy)
			# fig.canvas.restore_region(background)
			ax.draw_artist(pts)
			fig.canvas.blit(ax.bbox)
			# if len(dead)%10 == 0:
				# plt.pause(0.00000000001)
		j = j+1
		# print(j)


		if curr.x == nGoal.x and curr.y == nGoal.y:
			print("Target Found")
			nGoal.parentId = curr.parentId
			nGoal.c2c = curr.c2c
			break


		#Pop the visited nodes from the surr set
		# open.remove(curr_id)
		#Add the visited node to the dead set
		# closed.add(curr_id)
		explored[curr_id] = curr
		#Search the adjacent nodes
		for i,_ in enumerate(motion):
			new_node = Node(curr.x + motion[i][0],curr.y + motion[i][1],curr.c2c + motion[i][2],curr_id)

			#Calculate new ID for each new nodes
			new_id = gen_index(new_node)

			#Checks for unique and feasible node
			if obstacle(new_node,grid,r)==False:
				continue

			if new_id in explored:
				continue

			if new_id in surr:
				if surr[new_id].c2c > new_node.c2c:
					surr[new_id].c2c = new_node.c2c
					surr[new_id].parentId = new_node.parentId
					heapq.heappush(cost,[surr[new_id].c2c,surr[new_id]])
			else:
				surr[new_id] = new_node
				heapq.heappush(cost,[surr[new_id].c2c,surr[new_id]])
				# open.add(new_id)
	px,py = findPath(nGoal,explored)

	return px,py,explored

#Trace Back the final path
def findPath(nGoal,explored):
	px,py = [nGoal.x],[nGoal.y]
	parentId = nGoal.parentId

	while parentId !=-1:
		visited = explored[parentId]
		px.append(visited.x)
		py.append(visited.y)
		parentId = visited.parentId

	return px,py



#Define required Functions
def map(rr,r,clearance):
	ox=[]
	oy = [] #Row wise list of True or False based on if there is an obstacle or not
	mx,my = np.mgrid[:int(150/r)+2,:int(250/r)+2]
	mx=r*mx
	my=r*my
	rr=(clearance+rr)
	#Grid Boundary
	o1 = mx
	o2 = my
	o3 = mx
	o4 = my


	circle = (mx-130)**2+(my-190)**2
	ellipse = (6*(my-140))**2 + (15*(mx-120))**2
	#polygon
	l4 = 38*my+23*mx-(8530+(rr)*(math.sqrt(38**2 + 23**2)))
	l3 = -20*mx+37*my-(6101+(rr)*(math.sqrt(20**2 + 37**2)))
	l2 = -mx+15-(rr)
	l1 = 41*my+25*mx-(6525-(rr)*(math.sqrt(41**2 + 25**2)))
	l6 = 4*my+38*mx-(2628+(rr)*(math.sqrt(4**2 + 38**2)))
	l5 = 38*my-7*mx-(5830-(rr)*(math.sqrt(38**2 + 7**2)))

	#rect
	s1 = mx
	s2 = my
	s3 = mx
	s4 = my

	# grid = np.full((151, 251), True, dtype=bool)
	grid = np.full(((150/r) +2, (250/r) +2), True, dtype=bool)
	grid[o1<0+(rr)]=False
	grid[o2<0+(rr)]=False
	grid[o3>150-(rr)]=False
	grid[o4>250-(rr)]=False
	grid[(circle<=(15+(rr))**2) | (ellipse<=(90+(rr))**2) |((s1<=112.5+(rr))&(s2<=100+(rr))&(s3>=67.5-(rr))&(s4>=50-(rr))) ]=False  #(l5>0) | (l6<0)))
	grid[(l1>=0)&(l2<=0)&(l3<=0)&(l4<=0)&((l5>=0)|(l6<=0))] = False
	for i in range((150/r)+2):
		for j in range((250/r)+2):
			if grid[i][j]==False:
				ox.append(j)
				oy.append(i)

	return grid,ox,oy


def main():
	
	while True:
	# rr = 5.0
	# c=1

		r=input("Enter resolution(preferred minimum 1 ):") 
		rr= input("Enter robot radius ")
		clearance=input("Enter clearance")
		grid,ox,oy = map(rr,r,clearance)


		# sx = 1.0
		# sy = 1.0

		sx,sy=input("Enter start node x axis and y axis")

		nStart=Node((sx/r),(sy/r),0.0,-1)  #coordinates,cost,parentid

		#collision check for start node
		if not obstacle(nStart,grid,r):
			print("Start node invalid")
			return False
			

		# gx = 249.0
		# gy = 149.0
		gx,gy=input("Enter goal node x axis and y axis")
		nGoal = Node((gx/r),(gy/r),0.0,-1)

		#Check for collision of goal node
		if not obstacle(nGoal,grid,r):
			print("Goal Node invalid")
			return False
			

		start = time.time()
		px,py,explored = Dijkstra(nStart,nGoal,rr=rr,grid=grid,ox=ox,oy=oy,r=r)
		end = time.time()
		print("Time taken:",(end-start))
		path=explored.values()
		p1=[]
		p2=[]
		for i in range(len(path)):
			p1.append(path[i].x)

			p2.append(path[i].y)


		if GUI:
			plt.plot(p1,p2,"y*")
			plt.plot(px,py,"k")
			plt.plot(nStart.x,nStart.y,"go")
			plt.plot(nGoal.x,nGoal.y,"ro")
			
			
			plt.pause(3)
			plt.close()
		repeat=raw_input("If you want to enter new values type 1: ")
	
		if repeat=="1":
			continue
		else:
			print("Thank you,bye")
			return False

		

if __name__ == '__main__':
	main()
