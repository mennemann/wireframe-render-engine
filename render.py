import os, time
import numpy as np
import math
import cv2
import random


plane = {
"focal_length":400000,
"size":(1080, 920)
 }


class Vertex:
    vertices = []

    def __init__(self, x,y,z):
        self.x, self.y, self.z = x,y,z
        Vertex.vertices.append(self)
        
    def project(self):
        pass
        self.proj_x = int(((plane["focal_length"] * self.x) // (plane["focal_length"] + self.z) ) + plane["size"][0]//2)
        self.proj_y = int(((plane["focal_length"] * self.y) // (plane["focal_length"] + self.z) ) + plane["size"][1]//2)
        
    def __str__(self):
        return f"x:{self.x}, y:{self.y}, z:{self.z}"


draw_type = "cv2"

edges = []

i=0
a = Vertex(-250,-250,50)
b = Vertex(-250,250,50)
c = Vertex(250,-250,50)
d = Vertex(250,250,50)
e = Vertex(-250,-250,550)
f = Vertex(-250,250,550)
g = Vertex(250,-250,550)
h = Vertex(250,250,550)


edges = [
(a,c),(b,d),(a,b),(c,d),
(e,g),(f,h),(e,f),(g,h),
(a,e),(c,g),(b,f),(d,h)
]



def draw(img,x,y,val=1):
    try:
        img[y][x] = [255]*3
        return True
    except:
        return False
	
    
def gen_img():
    img = np.zeros((plane["size"][1], plane["size"][0], 3), dtype=np.uint8)
    
    for vertex in Vertex.vertices:
        vertex.project()
        draw(img, vertex.proj_x, vertex.proj_y)
    
    for edge in edges:
        flipped = False
        m=0
        try:
            m = (edge[1].proj_y - edge[0].proj_y) / (edge[1].proj_x - edge[0].proj_x)
            if not(-1 <= m <= 1):
                raise ZeroDivisionError
        except ZeroDivisionError:
            flipped = True
            try:
                m = (edge[1].proj_x - edge[0].proj_x) / (edge[1].proj_y - edge[0].proj_y)
            except:
                pass
            if not(-1 <= m <= 1):
                raise Exception


        x_width = (edge[0].proj_x, edge[1].proj_x)
        y_width = (edge[0].proj_y, edge[1].proj_y)
        
        
        if not(flipped):
            height_b = edge[1].proj_y - m * edge[1].proj_x
            for x in range(min(x_width), max(x_width)):
                draw(img, x, round(m*x+height_b))
        else:
            height_b = edge[1].proj_x - m * edge[1].proj_y
            for y in range(min(y_width), max(y_width)):
                draw(img, round(m*y+height_b), y)
                

    return img
    


def draw_cv2(img, delay = 0):
    cv2.imshow("Hee", img[::-1])
    time.sleep(delay)
      
        
def draw_terminal(img, delay = 0):
    os.system("cls")
    for row in img[::-1]:
        
        print(''.join(["@" if _[0] == 255 else " " for _ in row]))
    time.sleep(delay)
    
    
if __name__ == "__main__":

    alpha = 0.01
    beta = 0.01
    gamma = 0.01
    
    sin, cos = math.sin, math.cos
    
    
    rot_matrix2 = np.array([[cos(alpha), -sin(alpha), 0],
                           [sin(alpha), cos(alpha), 0],
                           [0,0,1]])
    
    out = None
    if draw_type == "render":
    
    
        out = cv2.VideoWriter('outpy.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 20, plane["size"])
    frames = 0


    s = False
    while True:
        #alpha = random.random() * 0.02 - 0.01
        #beta = random.random() * 0.02 - 0.01
        #gamma = random.random() * 0.02 - 0.01
         
        
        rot_matrix = np.array([[cos(beta) * cos(gamma), sin(alpha) * sin(beta) * cos(gamma) - cos(alpha) * sin(gamma), cos(alpha) * sin(beta) * cos(gamma) + sin(alpha) * sin(gamma)],
                           [cos(beta) * sin(gamma), sin(alpha) * sin(beta) * sin(gamma) + cos(alpha) * cos(gamma), cos(alpha) * sin(beta) * sin(gamma) - sin(alpha) * cos(gamma)],
                           [-sin(beta)            , sin(alpha) * cos(beta)                                       , cos(alpha) * cos(beta)]])
    
        img = gen_img()
        frames+=1
        if draw_type == "terminal":
            draw_terminal(img)
        elif draw_type == "cv2":    
            draw_cv2(img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        elif draw_type == "render":
            #cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
            out.write(img)
            print(frames)
            if frames > 1000:
                out.release()
                exit()

        for vertex in Vertex.vertices:
            vertex.z -= 250
            (vertex.x, vertex.y, vertex.z) = rot_matrix @ (vertex.x, vertex.y, vertex.z)
            vertex.z += 250