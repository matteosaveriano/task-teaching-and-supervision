define reperti model
(
  block (
	points 4
  	point[0] [0 0]
  	point[1] [1  0]
  	point[2] [1  1]
  	point[3] [0 1]
  	z [0 1]
  	)

  color "red"	
  gui_movemask 3
  gui_nose 0
  sonar_return 1
  ranger_return 1
  fiducial_return 0
  laser_return 1
  blob_return 1
  obstacle_return 1
)

