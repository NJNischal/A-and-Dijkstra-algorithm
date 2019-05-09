#!/usr/bin/env python
# coding: utf-8

# In[1]:


import math
import numpy as np
import cv2 as cv


# In[2]:


rad=int(input('Please enter radius of the robot:'))
clearance=int(input('Please enter the clearance:'))

tot=rad+clearance
#print(tot)


# In[3]:


def boundary_check(i,j):
    if (i<tot or j>(149-tot) or j<tot or i>(249-tot)):
        return 0
    else:
        return 1


# In[4]:


def obs_map(x,y):
    polygon1 = (41*x-25*y-(2775-(tot*48.02))>=0) and (y-(135+tot)<=0) and (37*x-10*y-(5051-(tot)*38.32)<=0) and (2*x-19*y+1536+tot*19.1<=0)
    polygon2 = (37*x-10*y-(5051-(tot)*38.32)>0) and (38*x+7*y-(6880-tot*38.64)>0) and (38*x-23*y-(5080+tot*44.42)<0) and (37*x+20*y-(9101+tot*42.05)<0)
    circle = ((np.square(x-190))+ (np.square(y-20)) <=np.square(15)+tot)
    rectangle = (x>=50-tot) and (x<= 100+tot) and (y<=82.5+tot) and (y>= 37.5-tot)  
    ellipse = (((np.square(x-140))/np.square(15+tot))+((np.square(y-30))/np.square(6+tot)) -1 <=0)
    if circle or rectangle or polygon1 or polygon2 or ellipse:
        obj_val = 0
    else:
        obj_val = 1
    
    return obj_val


# In[5]:


parent_list=[]

for j in range (250):
    column=[]
    for i in range (150):
        column.append(0)
    parent_list.append(column)
    


# In[6]:


x_start=int(input("Please enter start point x coordinate"))
y_start=int(input("Please enter start point y coordinate"))

start_obs=obs_map(x_start,y_start)
start_boundary=boundary_check(x_start,y_start)


while(start_obs and start_boundary!=1):
    print("Incorrect start point! Please enter a valid start point")
    x_start=int(input("Please enter start point x coordinate"))
    y_start=int(input("Please enter start point y coordinate"))
    start_obs=obs_map(x_start,y_start)
    start_boundary=boundary_check(x_start,y_start)


start=[x_start,y_start]


# In[7]:


x_goal=int(input("Please enter goal point x coordinate"))
y_goal=int(input("Please enter goal point y coordinate"))

goal_obs=obs_map(x_goal,y_goal)
goal_boundary=boundary_check(x_goal,y_goal)


while(goal_obs and goal_boundary!=1):
    print("Incorrect goal point! Please enter a valid goal point")
    x_goal=int(input("Please enter another goal point x coordinate"))
    y_goal=int(input("Please enter another goal point y coordinate"))
    goal_obs=obs_map(x_goal,y_goal)
    goal_boundary=boundary_check(x_goal,y_goal)
    

goal=[x_goal,y_goal]

resolution=input("Please enter the resolution")


# In[8]:


cost_array=np.array(np.ones((250,150)) * np.inf)
visited=np.array(np.zeros((250,150)))

size=(150,250)
mapy=np.ones((size),np.uint8)*255


# In[9]:


Q=[]

# append start point and initialize it's cost to zero

Q.append([x_start,y_start])
cost_array[x_start][y_start]=0

# Priority Queue

def pop(Q):
    minimum_index=0
    minimum_X = Q[0][0] 
    minimum_Y = Q[0][1]
    for i in range(len(Q)):
        x = Q[i][0]
        y = Q[i][1]
        if cost_array[x,y] < cost_array[minimum_X,minimum_Y]:
            minimum_index = i
            minimum_X = x 
            minimum_Y= y
    print(Q[minimum_index])
    current_node = Q[minimum_index]
    Q.remove(Q[minimum_index])
    return current_node


# In[10]:


# All movements possible

def north(i,j):
    new_node=[i,j+1]     
    return new_node

def south(i,j):
    new_node=[i,j-1]
    return new_node

def east(i,j):
    new_node=[i+1,j]
    return new_node

def west(i,j):
    new_node=[i-1,j]
    return new_node

def NE(i,j):
    new_node=[i+1,j+1]
    return new_node

def SE(i,j):
    new_node=[i+1,j-1]
    return new_node

def NW(i,j):
    new_node=[i-1,j+1]
    return new_node
def SW(i,j):
    new_node=[i-1,j-1]
    return new_node


# In[11]:


visited_node=[]
current_node=[x_start,y_start]
while current_node!=goal:
    current_node=pop(Q)
    
    new_north=north(current_node[0],current_node[1])
    status=boundary_check(new_north[0],new_north[1])
    flag=obs_map(new_north[0],new_north[1])
    if (status and flag == 1):
        if visited[new_north[0],new_north[1]]==0:
            visited[new_north[0],new_north[1]]=1
            visited_node.append(new_north)
            Q.append(new_north)
            parent_list[new_north[0]][new_north[1]]=current_node
            cost_array[new_north[0],new_north[1]]=(cost_array[current_node[0],current_node[1]]+1)
        else:
            if cost_array[new_north[0],new_north[1]]>(cost_array[current_node[0],current_node[1]]+1):
                cost_array[new_north[0],new_north[1]]=(cost_array[current_node[0],current_node[1]]+1)
                parent_list[new_north[0]][new_north[1]]=current_node
    
    
    new_south=south(current_node[0],current_node[1])
    status=boundary_check(new_south[0],new_south[1])
    flag=obs_map(new_south[0],new_south[1])
    if (status and flag == 1):
        if visited[new_south[0],new_south[1]]==0:
            visited[new_south[0],new_south[1]]=1
            visited_node.append(new_south)
            Q.append(new_south)
            parent_list[new_south[0]][new_south[1]]=current_node
            cost_array[new_south[0],new_south[1]]=(cost_array[current_node[0],current_node[1]]+1)
        else:
            if cost_array[new_south[0],new_south[1]]>(cost_array[current_node[0],current_node[1]]+1):
                cost_array[new_south[0],new_south[1]]=(cost_array[current_node[0],current_node[1]]+1)
                parent_list[new_south[0]][new_south[1]]=current_node
    
    new_east=east(current_node[0],current_node[1])
    status=boundary_check(new_east[0],new_east[1])
    flag=obs_map(new_east[0],new_east[1])
    if (status and flag == 1):
        if visited[new_east[0],new_east[1]]==0:
            visited[new_east[0],new_east[1]]=1
            visited_node.append(new_east)
            Q.append(new_east)
            #print('pl',parent_list[new_east[0]][new_east[1]])
            parent_list[new_east[0]][new_east[1]]=current_node
            cost_array[new_east[0],new_east[1]]=(cost_array[current_node[0],current_node[1]]+1)
        else:
            if cost_array[new_east[0],new_east[1]]>(cost_array[current_node[0],current_node[1]]+1):
                cost_array[new_east[0],new_east[1]]=(cost_array[current_node[0],current_node[1]]+1)
                parent_list[new_east[0]][new_east[1]]=current_node
    
    
    
    new_west=west(current_node[0],current_node[1])
    status=boundary_check(new_west[0],new_west[1])
    flag=obs_map(new_west[0],new_west[1])
    if (status and flag == 1):
        if visited[new_west[0],new_west[1]]==0:
            visited[new_west[0],new_west[1]]=1
            visited_node.append(new_west)
            Q.append(new_west)
            parent_list[new_west[0]][new_west[1]]=current_node
            cost_array[new_west[0],new_west[1]]=(cost_array[current_node[0],current_node[1]]+1)
        else:
            if cost_array[new_west[0],new_west[1]]>(cost_array[current_node[0],current_node[1]]+1):
                cost_array[new_west[0],new_west[1]]=(cost_array[current_node[0],current_node[1]]+1)
                parent_list[new_west[0]][new_west[1]]=current_node
    
    
    new_NE=NE(current_node[0],current_node[1])
    status=boundary_check(new_NE[0],new_NE[1]) 
    flag=obs_map(new_NE[0],new_NE[1])
    if (status and flag == 1):    
        if visited[new_NE[0],new_NE[1]]==0:
            visited[new_NE[0],new_NE[1]]=1
            visited_node.append(new_NE)
            Q.append(new_NE)
            parent_list[new_NE[0]][new_NE[1]]=current_node
            cost_array[new_NE[0],new_NE[1]]=(cost_array[current_node[0],current_node[1]]+math.sqrt(2))
        else:
            if cost_array[new_NE[0],new_NE[1]]>(cost_array[current_node[0],current_node[1]]+math.sqrt(2)):
                cost_array[new_NE[0],new_NE[1]]=(cost_array[current_node[0],current_node[1]]+math.sqrt(2))
                parent_list[new_NE[0]][new_NE[1]]=current_node
    
    
    new_SE=SE(current_node[0],current_node[1])
    status=boundary_check(new_SE[0],new_SE[1])
    flag=obs_map(new_SE[0],new_SE[1])
    if (status and flag == 1):   
        if visited[new_SE[0],new_SE[1]]==0:
            visited[new_SE[0],new_SE[1]]=1
            visited_node.append(new_SE)
            Q.append(new_SE)
            parent_list[new_SE[0]][new_SE[1]]=current_node
            cost_array[new_SE[0],new_SE[1]]=(cost_array[current_node[0],current_node[1]]+math.sqrt(2))
        else:
            if cost_array[new_SE[0],new_SE[1]]>(cost_array[current_node[0],current_node[1]]+math.sqrt(2)):
                cost_array[new_SE[0],new_SE[1]]=(cost_array[current_node[0],current_node[1]]+math.sqrt(2))
                parent_list[new_SE[0]][new_SE[1]]=current_node
            
    new_NW=NW(current_node[0],current_node[1])
    status=boundary_check(new_NW[0],new_NW[1])
    flag=obs_map(new_NW[0],new_NW[1])
    if (status and flag == 1):
        if visited[new_NW[0],new_NW[1]]==0:
            visited[new_NW[0],new_NW[1]]=1
            visited_node.append(new_NW)
            Q.append(new_NW)
            parent_list[new_NW[0]][new_NW[1]]=current_node
            cost_array[new_NW[0],new_NW[1]]=(cost_array[current_node[0],current_node[1]]+math.sqrt(2))
        else:
            if cost_array[new_NW[0],new_NW[1]]>(cost_array[current_node[0],current_node[1]]+math.sqrt(2)):
                cost_array[new_NW[0],new_NW[1]]=(cost_array[current_node[0],current_node[1]]+math.sqrt(2))
                parent_list[new_NW[0]][new_NW[1]]=current_node
    
    new_SW=SW(current_node[0],current_node[1])
    status=boundary_check(new_SW[0],new_SW[1])
    flag=obs_map(new_SW[0],new_SW[1])
    if (status and flag == 1):   
        if visited[new_SW[0],new_SW[1]]==0:
            visited[new_SW[0],new_SW[1]]=1
            visited_node.append(new_SW)
            Q.append(new_SW)
            parent_list[new_SW[0]][new_SW[1]]=current_node
            cost_array[new_SW[0],new_SW[1]]=(cost_array[current_node[0],current_node[1]]+math.sqrt(2))
        else:
            if cost_array[new_SW[0],new_SW[1]]>(cost_array[current_node[0],current_node[1]]+math.sqrt(2)):
                cost_array[new_SW[0],new_SW[1]]=(cost_array[current_node[0],current_node[1]]+math.sqrt(2))
                parent_list[new_SW[0]][new_SW[1]]=current_node

print("Goal reached")


# In[12]:


visited_node


# In[13]:


goal=[x_goal,y_goal]
start=[x_start,y_start]
path=[]
def path_find(goal,start):
    GN=goal
    path.append(goal)
    while (GN!=start):
        a=parent_list[GN[0]][GN[1]]
        path.append(a)
        GN=a

path_find(goal,start)
path


# In[14]:


parent_list[30][30]


# In[15]:


print('The cost of the shortest path is',cost_array[x_goal,y_goal])


# In[16]:


x = 250
y = 150
image = np.ones((y,x,3),np.uint8)*255

#Outer edges mapping
for x in range(0,250):
    for y in range(0,150):
        if x>=tot and x<=250-tot and y>=tot and y<=150-tot:
            image[y,x] = (255,255,255)
        else:
            image[y,x] = (175,175,175)
            
# Rectangle
for x in range(45,106):
    for y in range(31,88):
        if (x>=50-tot) and (x<= 100+tot) and (y<=82.5+tot) and (y>= 37.5-tot):
            image[y,x] = (0,0,0)

            
# Circle
for x in range(170, 210):
    for y in range(0,40):
        if ((np.square(x-190))+ (np.square(y-20)) <=np.square(15)+tot):
            image[y,x] = (0,0,0)

            
# Ellipse
for x in range(120, 160):
    for y in range(19,41):
        if (((np.square(x-140))/np.square(15+tot))+((np.square(y-30))/np.square(6+tot)) -1 <=0): 
            image[y,x] = (0,0,0)

            
# Convex Polygon
for x in range(115,180):
    for y in range(88,140):
        if (41*x-25*y-(2775-(tot*48.02))>=0) and (y-(135+tot)<=0) and (37*x-10*y-(5051-(tot)*38.32)<=0) and (2*x-19*y+1536+tot*19.1<=0):
            image[y,x] = (0,0,0)


for x in range(160, 250):
    for y in range(46,140):
        if (37*x-10*y-(5051-(tot)*38.32)>0) and (38*x+7*y-(6880-tot*38.64)>0) and (38*x-23*y-(5080+tot*44.42)<0) and (37*x+20*y-(9101+tot*42.05)<0):
            image[y,x] = (0,0,0)  


# In[17]:


cv.circle(image,(int(goal[0]),int(goal[1])), (1), (0,0,255), -1);
cv.circle(image,(int(start[0]),int(start[1])), (1), (0,0,255), -1);

for i in visited_node:
    cv.circle(image,(int(i[0]),int(i[1])), (1), (255,0,0));
    pic=cv.resize(image,None,fx=3,fy=3)
    cv.imshow('map',pic)
    cv.waitKey(1)

for i in path:
    cv.circle(image,(int(i[0]),int(i[1])), (1), (153,50,204));
    pic=cv.resize(image,None,fx=3,fy=3)
    cv.imshow('map',pic)
    cv.waitKey(1)
    
    
cv.imwrite('map1.png',image)
cv.waitKey(0) 
cv.destroyAllWindows()


# In[ ]:




