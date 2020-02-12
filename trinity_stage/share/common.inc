# https://www.desmos.com/calculator/x381kzcjkp

define sicklms ranger
(
  sensor(
    fov 360.0
    samples 180
  )
  model
  (
    size [ 0.001 0.001 0 ]
    color "black"
    #gui_grid 1
  )
)

define flame_sensor fiducial
(
	range_min 0.01
    range_max 0.9
    range_max_id 1
    fov 20
	name "flame"
    ignore_zloc 1
	
	model 
	(
		size [ 0.01 0.01 0.01]
		color "green"
	)
)

define robot position
(
  size [ 0.3 0.3 0.15]
  localization_origin [ 0.0 0 0 0]
  origin [ 0 0 0 0]
  gui_nose 1
  gui_grid 0
  gui_outline 1
  gui_move 1
  drive "omni"
  name "robot" 
  bitmap "t20_2.png"
  sicklms(pose [ 0 0 -0.15 0.000 ])
  flame_sensor(pose [0.087 -0.05 0.01 330])
  velocity_bounds [-5.0 5.0 -5.0 5.0 0 0 -90.0 90.0 ]					
  acceleration_bounds [-5.0 5.0 -5.0 5.0 0 0 -90 90.0 ]
)

define floorplan model
(
  color "gray30"
  boundary 1

  gui_nose 1
  gui_grid 0

  gui_outline 0
  gripper_return 0
  fiducial_return 0
  ranger_return 1

  gui_move 0
)

define candle model
(
  size [0.02 0.02 0.20]
  color "red"
  fiducial_return 1
  obstacle_return 1
  gui_move 1
)

resolution 0.02 
interval_sim 100

