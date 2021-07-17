#!/usr/bin/env python3
from typing import Tuple
import cv2
import time
from heapq import heappop, heappush
import numpy 
from PIL import Image
import cv_bridge
from std_msgs import msg
global pub,  width, height, PROB_FREE, PROB_OCC, START, GOAL, gridmap, heuristic, graph, graph_tools, path
from std_msgs.msg import String
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from geometry_msgs.msg import PoseArray
import roslib
import rospy
import os
roslib.load_manifest('rospy')
from std_msgs.msg import MultiArrayDimension
from rospy_tutorials.msg import Floats
#messages for image publisher
from sensor_msgs.msg import Image as Image2
from cv_bridge import CvBridge, CvBridgeError

def configuration_method(inicial_m,final_m):
	scriptDir = os.path.dirname(__file__)
	#ruta = scriptDir + "/mapa_erc.png"
	ruta = scriptDir + "/Mapa.jpg"
	#ruta = scriptDir + "/imagen2.png"

	#img = Image.open(ruta).convert('RGB')
	pixel = cv2.imread(ruta,1) 
	print(pixel.shape)
	#cv2.imshow("antes", pixel)
	#cv2.waitKey(0)
	# DO NOT CHANGE
	# load gridmap
	gridmap = cv2.imread(ruta,0) 
	print(gridmap.shape)
	print(type(gridmap))
	# define a threshold, 128 is the middle of black and white in grey scale
	thresh = 100

	# threshold the image
	ignorar,gridmap = cv2.threshold(gridmap, thresh, 255, cv2.THRESH_BINARY)

	#img = gridmap.convert('RGB')
	#pixel = img.load()

	#print(gridmap.shape)

	#cv2.imshow("antes", gridmap)
	#cv2.waitKey(0)
	#gridmap = gridmap/100
	#print(gridmap.shape)
	#cv2.imshow("despues", gridmap)
	#cv2.waitKey(0)
	PROB_FREE = 0.3
	PROB_OCC = 0.6
	
	# DO NOT CHANGE
	# size of canvas (variables used in Tools methods)
	height, width = gridmap.shape
	x_inicial_m = float(inicial_m[0])
	y_inicial_m = float(inicial_m[1])
	x_final_m  = float (final_m[0])
	y_final_m   = float (final_m[1])
	x_inicial = pixels([x_inicial_m,y_inicial_m],height, width) [0]
	y_inicial = pixels([x_inicial_m,y_inicial_m],height, width) [1]
	x_final = pixels([x_final_m,y_final_m],height, width) [0]
	y_final = pixels([x_final_m,y_final_m],height, width) [1]
	
	x_inicial,y_inicial,x_final,y_final=es_obstaculo(pixel,x_inicial,y_inicial,x_final,y_final,width,height)
	#declaración coordenadas iniciales y finales
	
	coordenates_array = []
	coordenates_array.append([x_inicial,y_inicial])

	START  =  tuple([y_inicial,x_inicial])
	GOAL   = tuple([y_final,x_final])

	# types of heuristic function
	Manhattan_heuristic = 0 # DO NOT CHANGE
	Eclidian_heuristic = 1 # DO NOT CHANGE
	heuristica_propia = 2 # DO NOT CHANGE
	heuristica = Eclidian_heuristic #seleccion heuristica 

	graph = gridmap2graph(gridmap,width,height,PROB_FREE)
	print("* gridmap2graph")

	start_time = time.time()

	path = Astar(graph, heuristica,GOAL,START,gridmap)
	print("* Astar")
	print("--- %s segundos ---" % (time.time() - start_time))

	pos_actualx = x_inicial
	pos_actualy = y_inicial

	pos_actualx = x_inicial
	pos_actualy = y_inicial
	for direccion in path:
		if direccion == "N":
			pos_actualx = pos_actualx 
			pos_actualy = pos_actualy -1
			coordenates_array.append([pos_actualx,pos_actualy])

		elif direccion == "W":
			pos_actualx = pos_actualx -1
			pos_actualy = pos_actualy 
			coordenates_array.append([pos_actualx,pos_actualy])
		elif direccion == "S":
			pos_actualx = pos_actualx 
			pos_actualy = pos_actualy + 1
			coordenates_array.append([pos_actualx,pos_actualy])
			
		elif direccion == "E":
			pos_actualx = pos_actualx + 1
			pos_actualy = pos_actualy 
			coordenates_array.append([pos_actualx,pos_actualy])

	esquinas =coordenates(coordenates_array)
	dibujo_ruta2(pixel,coordenates_array,esquinas,width,height)
	print (f"* {len (esquinas)} Coordenates.")
	finales = convertir(esquinas,width,height)
	return finales

def gridmap2graph(gridmap,width,height,PROB_FREE):
	# DO NOT CHANGE
	graph = {(i, j): [] for j in range(width) for i in range(height) if gridmap[i][j]>(1-PROB_FREE)}
	for row, col in graph.keys():
		if row < height - 1 and gridmap[row + 1][col]>(1-PROB_FREE):
			graph[(row, col)].append(("S", (row + 1, col)))
			graph[(row + 1, col)].append(("N", (row, col)))
		if col < width - 1 and gridmap[row][col + 1]>(1-PROB_FREE):
			graph[(row, col)].append(("E", (row, col + 1)))
			graph[(row, col + 1)].append(("W", (row, col)))
	return graph

def heuristic(cell, goal, type,graph):
	if type == 0:
		#Manhattan distance
		# TODO: write manhattan distance heuristic equation 
		x1, y1 = cell
		x2, y2 = goal
		return abs(x2-x1) + abs(y2-y1)
		pass
		# return 0
	elif type == 1:
		#Euclidian distance
		x1, y1 = cell
		x2, y2 = goal
		return numpy.sqrt((x2-x1)**2 + (y2-y1)**2)
		# TODO: write euclidian dimustance heuristic equation 
		pass
	elif type == 2:
		x1,y1 = cell
		x2,y2 = goal
		mu = 27
		puntos = 30
		penalizacion = 0
		for i in range(mu):
			if not (x1+i,y1) in graph:
				penalizacion = penalizacion + puntos
		for i in range(mu):
			if not (x1-i,y1) in graph:
				penalizacion = penalizacion + puntos
		for i in range(mu):
			if not (x1,y1+i) in graph:
				penalizacion = penalizacion + puntos
		for i in range(mu):
			if not (x1,y1-i) in graph:
				penalizacion = penalizacion + puntos
				
		for i in range(mu):
			if not (x1+i,y1+i) in graph:
				penalizacion = penalizacion + puntos
		for i in range(mu):
			if not (x1-i,y1-i) in graph:
				penalizacion = penalizacion + puntos
		for i in range(mu):
			if not (x1-i,y1+i) in graph:
				penalizacion = penalizacion + puntos
		for i in range(mu):
			if not (x1+i,y1-i) in graph:
				penalizacion = penalizacion + puntos


		return numpy.sqrt((x2-x1)**2 + (y2-y1)**2) + penalizacion

def Astar(graph, type_h,GOAL,START,gridmap):
	# TODO: 

	encontrado = False
	colaVacia = False

	celda = START
	cola = []
	visitados = set()

	costo = gridmap[celda]
	costo_acumulado = 0
	heur = heuristic(celda,GOAL,type_h,graph)
	tupla = (costo+heur,costo_acumulado,celda,"")
	heappush(cola,tupla)

	while encontrado == False and colaVacia == False:

		menor = heappop(cola)
		costo = menor[0]
		costo_acumulado = menor[1]
		celda = menor[2]
		path = menor[3]


		if celda == GOAL:
			encontrado  = True
		else:
			if celda not in visitados:

				visitados.add(celda)
				
				for i,j in graph[celda]:

					costo2 = gridmap[j]
					heur = heuristic(j,GOAL,type_h,graph)

					tupla = (costo_acumulado+heur,costo_acumulado+costo2,j,path+i)

					heappush(cola,tupla)

		if not cola:
			colaVacia = True
	
	if encontrado == True:
		
		return path
	else:
		print("No se encontró un camino directo.")

def coordenates(route):  
	c = []  
	for i in range (2,len (route) -2):     
		if (route[i][0]-route[i-2][0]) !=0 and abs((route[i][1]-route[i-2][1])/(route[i][0]-route[i-2][0])) == 1:
			if (route[i][0]-route[i+2][0]) != 0 and  abs((route[i][1]-route[i+2][1])/(route[i][0]-route[i+2][0])) != 1:
				new_p = route [i]
				c.append(new_p)
		elif route [i][0] == route [i-1][0] :
			if route [i][0] != route [i+1][0] :
				new_p = route [i]
				c.append(new_p)
		elif route [i][1] == route [i-1][1] :
			if route [i][1] != route [i+1][1] :
				new_p = route [i]            
				c.append(new_p)  
		elif route [i][0]-1 == route [i-1][0] and route [i][1]-1 == route [i-1][1]:  
			if route [i][0]+1 != route [i+1][0] or  route [i][1]+1 != route [i+1][1]:
				new_p = route [i]
				c.append(new_p)
	c.append(route [len (route)-1])
	c2=menos_p(c)
	return  c2

def menos_p (esq):
	c2=esq[:]
	menor = 10
	for i in range(1,len(esq)-1):
		if esq[i-1][0] ==esq[i][0] and esq[i][1] ==esq[i+1][1]:
			if  numpy.sqrt((esq[i-1][0]-esq[i+1][0])**2 + (esq[i-1][1]-esq[i+1][1])**2) <menor:
				c2.remove(esq[i])
		elif esq[i-1][1] ==esq[i][1] and esq[i][0] ==esq[i+1][0]:
			if  numpy.sqrt((esq[i-1][0]-esq[i+1][0])**2 + (esq[i-1][1]-esq[i+1][1])**2) <menor:
				c2.remove(esq[i])
		elif esq[i+1][1] ==esq[i][1] or esq[i+1][0] ==esq[i][0]:
			if  numpy.sqrt((esq[i+1][0]-esq[i][0])**2 + (esq[i+1][1]-esq[i][1])**2) <2:
				c2.remove(esq[i])
	c2 = depurar_coord(c2)
	return c2

def depurar_coord(esq):
	c3 = esq[:]
	menor = 10
	for i in range(1,len(esq)-1):
		if  numpy.sqrt((esq[i][0]-esq[i+1][0])**2 + (esq[i][1]-esq[i+1][1])**2) <menor:
			sentinela = numpy.random.randint(1,4)
			if sentinela %2 !=0:
				c3.remove (esq[i])
	return c3

def convertir (route, width,height):
	x_center = (width / 2)
	y_center = (height / 2)
	x_len = 30
	y_len = 40
	x_scale = round (x_len  / width , 3)
	y_scale = round (y_len  / height , 3)
	final_coordenates=[] 
	for i in range(len(route)):
		new_coord = []
		new_coord.append ( ( (route [i][0] - x_center) * x_scale ) + 0)
		new_coord.append ( ( (route [i][1] - y_center) * y_scale ) - 0)
		final_coordenates.append (new_coord)
	return final_coordenates

def es_obstaculo(pixel,x_inicial,y_inicial,x_final,y_final,width,height):
	#print("inicial",x_inicial,y_inicial)
	#print("final",x_final,y_final)
	#print("-------------------")
	if pixel[x_inicial,y_inicial][0]!=255 and pixel[x_inicial,y_inicial][1]!=255 and pixel[x_inicial,y_inicial][2]!=255:
		print("La posicion inicial es un obstaculo")
		izquierda = x_inicial
		derecha = x_inicial
		arriba = y_inicial
		abajo = y_inicial
		aux=True
		while (aux):
			# Mirar izquierda
			entro = True
			if aux and pixel[izquierda,y_inicial]!=None and pixel[izquierda,y_inicial][0]!=255 and pixel[izquierda,y_inicial][1]!=255 and pixel[izquierda,y_inicial][2]!=255:
				izquierda=izquierda-1
			elif entro:
				aux=False
				entro=False
				x_inicial=izquierda
				y_inicial=y_inicial
			# Mirar derecha
			if aux and pixel[derecha,y_inicial]!=None and pixel[derecha,y_inicial][0]!=255 and pixel[derecha,y_inicial][1]!=255 and pixel[derecha,y_inicial][2]!=255:
				derecha=derecha+1
			elif entro:
				aux=False
				entro=False
				x_inicial=derecha
				y_inicial=y_inicial
			# Mirar arriba
			if aux and pixel[x_inicial,arriba]!=None and pixel[x_inicial,arriba][0]!=255 and pixel[x_inicial,arriba][1]!=255 and pixel[x_inicial,arriba][2]!=255:
				arriba=arriba+1
			elif entro:
				aux=False
				entro=False
				y_inicial=arriba
				x_inicial=x_inicial
			# Mirar abajo
			if aux and pixel[x_inicial,abajo]!=None and pixel[x_inicial,abajo][0]!=255 and pixel[x_inicial,abajo][1]!=255 and pixel[x_inicial,abajo][2]!=255:
				abajo=abajo-1
			elif entro:
				aux=False
				entro=False
				y_inicial=abajo
				x_inicial=x_inicial
			if aux and pixel[izquierda,y_inicial]==None and pixel[derecha,y_inicial]==None and pixel[x_inicial,arriba]==None and pixel[x_inicial,abajo]==None:
				aux=False
				print("No se encontro otro punto que no sea obstaculo")

	if pixel[x_final,y_final][0]!=255 and pixel[x_final,y_final][1]!=255 and pixel[x_final,y_final][2]!=255:
		print("La posicion final es un obstaculo")
		izquierda = x_final
		derecha = x_final
		arriba = y_final
		abajo = y_final
		aux=True
		while (aux):
			# Mirar izquierda
			entro = True
			if aux and pixel[izquierda,y_final]!=None and pixel[izquierda,y_final][0]!=255 and pixel[izquierda,y_final][1]!=255 and pixel[izquierda,y_final][2]!=255:
				izquierda=izquierda-1
			elif entro:
				aux=False
				entro=False
				x_final=izquierda
				y_final=y_final
			# Mirar derecha
			if aux and pixel[derecha,y_final]!=None and pixel[derecha,y_final][0]!=255 and pixel[derecha,y_final][1]!=255 and pixel[derecha,y_final][2]!=255:
				derecha=derecha+1
			elif entro:
				aux=False
				entro=False
				x_final=derecha
				y_final=y_final
			# Mirar arriba
			if aux and pixel[x_final,arriba]!=None and pixel[x_final,arriba][0]!=255 and pixel[x_final,arriba][1]!=255 and pixel[x_final,arriba][2]!=255:
				arriba=arriba+1
			elif entro:
				aux=False
				entro=False
				y_final=arriba
				x_final=x_final
			# Mirar abajo
			if aux and pixel[x_final,abajo]!=None and pixel[x_final,abajo][0]!=255 and pixel[x_final,abajo][1]!=255 and pixel[x_final,abajo][2]!=255:
				abajo=abajo-1
			elif entro:
				aux=False
				entro=False
				y_final=abajo
				x_final=x_final
			if aux and pixel[izquierda,y_final]==None and pixel[derecha,y_final]==None and pixel[x_final,arriba]==None and pixel[x_final,abajo]==None:
				aux=False
				print("No se encontro otro punto que no sea obstaculo")

	#print("inicial",x_inicial,y_inicial)
	#print("final",x_final,y_final)
	return x_inicial,y_inicial,x_final,y_final

def dibujo_ruta2(pixel,array_pos,esquinas,height,width):
	matrixMap=numpy.ones([width,height])
	RGBMap=[]
	for i in range(width):
		for j in range(height):
			if pixel[j,i][0]==255 and pixel[j,i][1]==255 and pixel[j,i][2]==255:
				matrixMap[i,j]=0
			else:
				matrixMap[i,j]=1
	for i in range(len(array_pos)):
		matrixMap[array_pos[i][1],array_pos[i][0]]=2
	for i in range(len(esquinas)):
		matrixMap[esquinas[i][1],esquinas[i][0]]=3
	for i in range(width):
		file=[]
		for j in range(height):
			if matrixMap[i,j]==1:
				RGB=[255,255,255,255] #white
			elif matrixMap[i,j]==0:
				RGB = [0,0,0,255]   #Black
			elif matrixMap[i,j]==2:
				RGB = [255,0,0,255] #Red
			elif matrixMap[i,j]==3: 
				RGB = [0,255,0,255] #Green
			elif matrixMap[i,j]==4: 
				RGB = [0,0,255,255] #Blue
			
			file.append(RGB)
		RGBMap.append(file)
	RGBMap=numpy.array(RGBMap).astype(numpy.uint8)
	routeMap=Image.fromarray(RGBMap,"RGBA")
	scriptDir = os.path.dirname(__file__)
	ruta = scriptDir + "/mapafinal.png"
	routeMap.save(ruta)

	print("*Se guardo el mapa")
	return routeMap

def pixels (coord,height, width) :
	x_center = (width / 2)
	y_center = (height / 2)
	x_len = 30
	y_len = 40
	x_scale = x_len  / width
	y_scale = y_len  / height
	x =  round(((coord[0] + 0) / x_scale) +  x_center) 
	y =  round(((coord [1] - 0) / y_scale ) + y_center)   
	return [x,y]

def inicio_fin(coordenadas):
	print("-----------------------------------------------------")
	global pub
	global img_pub
	scriptDir = os.path.dirname(__file__)
	ruta = scriptDir + "/Mapa.jpg"
	#ruta = scriptDir + "/imagen2.png"
	#ruta = scriptDir + "/mapa_erc.png"
	gridmap = cv2.imread(ruta,0) 
	gridmap = gridmap/100
	height, width = gridmap.shape
	
	x_ini = coordenadas.poses[0].position.x    #toca crear el mensaje
	y_ini = - coordenadas.poses[0].position.y    #toca crear el mensaje
	x_fin = coordenadas.poses[1].position.x     #toca crear el mensaje
	y_fin = - coordenadas.poses[1].position.y     #toca crear el mensaje
	inicial_m=(x_ini,y_ini)
	final_m=(x_fin,y_fin)

	x_inicial_m = float(inicial_m[0])
	y_inicial_m = float(inicial_m[1])
	x_final_m  = float (final_m[0])
	y_final_m   = float (final_m[1])
	x_inicial = pixels([x_inicial_m,y_inicial_m],height, width) [0]
	y_inicial = pixels([x_inicial_m,y_inicial_m],height, width) [1]
	x_final = pixels([x_final_m,y_final_m],height, width) [0]
	y_final = pixels([x_final_m,y_final_m],height, width) [1]

	# coordenadas

	# Revisar si estan fuera de rango
	if not fuera_rango(x_inicial,y_inicial,x_final,y_final,width,height):
    	#cambiar las coordenadas de metros a pixeles   
		
		print(f"Inicio:({x_ini},{-y_ini})\nFin:({x_fin},{-y_fin})")
		ruta=configuration_method(inicial_m,final_m)
		print("*Finalizado")
		ruta2 = []
		for i in range (len(ruta)):
			ruta2.append(ruta[i][0])
			ruta2.append(-ruta[i][1])
		r = numpy.array (ruta2,dtype = numpy.float32)
		pub.publish(r)
		print ("*Route published")
		scriptDir = os.path.dirname(__file__)
		ruta = scriptDir + "/mapafinal.png"
		image = cv2.imread(ruta)
		window_name = 'image'
		resized = cv2.resize(image,(300,500),interpolation = cv2.INTER_AREA)
		# cv2.imshow(window_name, resized)
		# cv2.waitKey(0)  
		# cv2.destroyAllWindows()

		# publicar imagen a topico para visualizar en rviz
		bridge = CvBridge()
		try:
			image_message = bridge.cv2_to_imgmsg(image, encoding= 'passthrough')
			img_pub.publish(image_message)

		except CvBridgeError as e:
			print(e)
			
		print("*Waiting for new coordenates")

	else:
		print()
		print("------------------------")
		print("Las coordenadas estan fuera de rango. Por favor ingrese nuevas coordenadas")
		print("------------------------")

def fuera_rango(x_inicial,y_inicial,x_final,y_final,width,height):
	if x_inicial>width or x_inicial<0 or x_final>width or x_final<0 or y_inicial>height or y_inicial<0 or y_final>height or y_final<0:
		return True
	else:
		return False

def planeacion_nodo():
	#declaración coordenadas iniciales y finales 
	global pub
	global img_pub
	print ("Esperando coordenadas.")
	rospy.init_node('Planeacion', anonymous=True)  # Inicia el nodo teleop
	#pub = rospy.Publisher('/robocol/ruta', numpy_nd_msg(Float32MultiArray), queue_size=1)    #toca modificar los mensajes 
	pub = rospy.Publisher('Robocol/MotionControl/ruta', numpy_msg(Floats),queue_size=10)
	img_pub = rospy.Publisher('Robocol/MotionControl/imagen', Image2, queue_size=1)
	rospy.Subscriber('/Robocol/Inicio_fin', PoseArray, inicio_fin)
	rate = rospy.Rate(10)
	rospy.spin()
	
if __name__ == '__main__':
	
    try:
        planeacion_nodo()
    except rospy.ROSInterrupyException:
        rospy.loginfo("node terminated")

