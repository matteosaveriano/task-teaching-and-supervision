define robot position (

	# actual size
  size [0.44 0.33 0.33]

  # the pioneer's center of rotation is offset from its center of area
  origin [-0.04 0.0 0 0]
# this polygon approximates the shape of a pioneer
  block (
	points 8
  point[0] [  0.23  0.05 ]
  point[1] [  0.15  0.15 ]
  point[2] [ -0.15  0.15 ]
  point[3] [ -0.23  0.05 ]
  point[4] [ -0.23 -0.05 ]
  point[5] [ -0.15 -0.15 ]
  point[6] [  0.15 -0.15 ]
  point[7] [  0.23 -0.05 ]
  z [0 1]
)
  # draw a nose on the robot so we can see which way it points
  gui_nose 1
  gui_boundary 0	

  # estimated mass in KG
  mass 15.0 
	
  
drive "diff"

eye()
sonar()
bumper()

)


define eye blobfinder(
colors_count 4
colors ["red" "orange" "yellow" "green"]
image [200 150]
range 8.00
fov 120
)


#questi sono i sonar
define sonar ranger
(

  scount 16

  # define the pose of each transducer [xpos ypos heading]
  spose[0] [ 0.075 0.130  90 ]
  spose[1] [ 0.115 0.115  50 ]
  spose[2] [ 0.150 0.080  30 ]
  spose[3] [ 0.170 0.025  10 ]
  spose[4] [ 0.170 -0.025  -10 ]
  spose[5] [ 0.150 -0.080   -30 ]
  spose[6] [ 0.115 -0.115  -50 ]
  spose[7] [ 0.075 -0.130  -90 ]
  spose[8] [ -0.155 -0.130  -90 ]
  spose[9] [ -0.195 -0.115  -130 ]
  spose[10] [ -0.230 -0.080  -150 ]
  spose[11] [ -0.250 -0.025  -170 ]
  spose[12] [ -0.250 0.025  170 ]
  spose[13] [ -0.230 0.080  150 ]
  spose[14] [ -0.195 0.115  130 ]
  spose[15] [ -0.155 0.130  90 ]	
		
  # define the field of view of each transducer [range_min range_max view_angle]
  sview [0 5.0 30]
  
  # define the size of each transducer [xsize ysize] in meters
  ssize [0.01 0.05]
  



#sview [0.3 2.0 10]
#ssize [0.01 0.05]
)


#questi sonar a ampi raggio ma con minimo range simulano un bumper
define bumper ranger
(
  scount 5

  # define the pose of each transducer [xpos ypos heading]
  spose[0] [ 0.115 -0.115  -30 ]
  spose[1] [ 0.17 -0.08  -15 ]
  spose[2] [ 0.17 0.0  0 ]
  spose[3] [ 0.17 0.08  15 ]
  spose[4] [ 0.115 0.115  30 ]		
  
  # define the field of view of each transducer [range_min range_max view_angle]
  sview [0 0.4 30]
  
  # define the size of each transducer [xsize ysize] in meters
  ssize [0.01 0.05]
  
)
