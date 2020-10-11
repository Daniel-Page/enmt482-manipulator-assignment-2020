import plotly

import numpy as np
from stl import mesh  # pip install numpy-stl
import plotly.graph_objects as go

def cylinder(r, h, a =0, nt=100, nv =50):
	"""
	parametrize the cylinder of radius r, height h, base point a
	"""
	theta = np.linspace(0, 2*np.pi, nt)
	v = np.linspace(a, a+h, nv )
	theta, v = np.meshgrid(theta, v)
	x = r*np.cos(theta)
	y = r*np.sin(theta)
	z = v
	return x, y, z

def boundary_circle(r, h, nt=100):
	"""
	r - boundary circle radius
	h - height above xOy-plane where the circle is included
	returns the circle parameterization
	"""
	theta = np.linspace(0, 2*np.pi, nt)
	x = r*np.cos(theta)
	y = r*np.sin(theta)
	z = h*np.ones(theta.shape)
	return x, y, z


r2 = 4
a2 = 0
h2 = 100

x2, y2, z2 = cylinder(r2, h2, a=a2)

colorscale_x = [[0, 'red'],[1, 'red']]
colorscale_y = [[0, 'green'],[1, 'green']]
colorscale_z = [[0, 'blue'],[1, 'blue']]

def axesrgb(x=0,y=0,z=0):
	cyl_x = go.Surface(x=x+z2, y=y+y2, z=z+x2,
					 colorscale = colorscale_x,
					 showscale=False,
					 opacity=1)

	cyl_y = go.Surface(x=x+x2, y=y+z2, z=z+y2,
					 colorscale = colorscale_y,
					 showscale=False,
					 opacity=1)

	cyl_z = go.Surface(x=x+x2, y=y+y2, z=z+z2,
					 colorscale = colorscale_z,
					 showscale=False,
					 opacity=1)

	#layout = go.Layout(scene_xaxis_visible=True, scene_yaxis_visible=True, scene_zaxis_visible=True)

	cone_x = go.Cone(x=[x+100], y=[y+0], z=[z+0], u=[70], v=[0], w=[0],opacity=1,colorscale = colorscale_x,showscale=False)
	cone_y = go.Cone(x=[x+0], y=[y+100], z=[z+0], u=[0], v=[70], w=[0],opacity=1,colorscale = colorscale_y,showscale=False)
	cone_z = go.Cone(x=[x+0], y=[y+0], z=[z+100], u=[0], v=[0], w=[70],opacity=1,colorscale = colorscale_z,showscale=False)
	return cyl_x, cyl_y, cyl_z, cone_x, cone_y, cone_z


def stl2mesh3d(stl_mesh):
	# stl_mesh is read by nympy-stl from a stl file; it is  an array of faces/triangles (i.e. three 3d points) 
	# this function extracts the unique vertices and the lists I, J, K to define a Plotly mesh3d
	p, q, r = stl_mesh.vectors.shape #(p, 3, 3)
	# the array stl_mesh.vectors.reshape(p*q, r) can contain multiple copies of the same vertex;
	# extract unique vertices from all mesh triangles
	vertices, ixr = np.unique(stl_mesh.vectors.reshape(p*q, r), return_inverse=True, axis=0)
	I = np.take(ixr, [3*k for k in range(p)])
	J = np.take(ixr, [3*k+1 for k in range(p)])
	K = np.take(ixr, [3*k+2 for k in range(p)])
	
	return vertices, I, J, K


def meshfile(filename,ox=0,oy=0,oz=0,theta=0):
	my_mesh = mesh.Mesh.from_file(filename)
	my_mesh.vectors.shape


	vertices, I, J, K = stl2mesh3d(my_mesh)
	x, y, z = vertices.T

	vertices.shape

	colorscale= [[0, '#e5dee5'], [1, '#e5dee5']]   


	xr = []
	yr = []
	zr = []

	theta = np.radians(theta)
	c, s = np.cos(theta), np.sin(theta)
	R = np.array(((c, -s), (s, c)))

	for i in range(len(x)):
		res = np.matmul(R,np.array([[x[i]],[y[i]]]))
		xr.append(list(res[0])[0]+ox)
		yr.append(list(res[1])[0]+oy)

	mesh3D = go.Mesh3d(
				x=xr,
				y=yr,
				z=z+oz, 
				i=I, 
				j=J, 
				k=K, 
				flatshading=True,
				colorscale=colorscale, 
				intensity=z, 
				#name='AT&T',
				showscale=False,
				opacity=0.3)
	return mesh3D


one = meshfile('grinder.stl',484.51,-426.6,0,120)
two = meshfile('plate.stl',0,0,20)
three = meshfile('grinder_stand.stl',370,-220,15,120)

layout = go.Layout(
			width=1920,
			height=1080,
			scene_camera=dict(eye=dict(x=1.5, y=1.5, z=0.7)),
			scene_xaxis_visible=True,
			scene_yaxis_visible=True,
			scene_zaxis_visible=True)

graphs = [one,two,three]

ax1 = axesrgb(0,0,20)
for i in range(len(ax1)):
	graphs.append(ax1[i])

ax2 = axesrgb(450,-316,334)
for i in range(len(ax2)):
	graphs.append(ax2[i])


fig = go.Figure(data=graphs, layout=layout)

fig.data[0].update(lighting=dict(ambient= 0.18,
								 diffuse= 1,
								 fresnel=  .1,
								 specular= 1,
								 roughness= .1,
								 facenormalsepsilon=0))

fig.data[0].update(lightposition=dict(x=3000,
									  y=3000,
									  z=10000));

fig.data[1].update(lighting=dict(ambient= 0.18,
								 diffuse= 1,
								 fresnel=  .1,
								 specular= 1,
								 roughness= .1,
								 facenormalsepsilon=0))

fig.data[1].update(lightposition=dict(x=3000,
									  y=3000,
									  z=10000));

fig.add_trace(go.Scatter3d(x=[0,450], y=[0,-316], z=[20,334], name = 'Low 2014',marker=dict(color='black', size=5),line=dict(color='black', width=9, dash='dash')))
fig.add_trace(go.Scatter3d(x=[0,360.1], y=[0,-204.7], z=[20,78.2], name = 'Low 2014',marker=dict(color='black', size=5),line=dict(color='black', width=9, dash='dash')))

fig.show()
