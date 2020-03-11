# https://www.desmos.com/calculator/x381kzcjkp

define sicklms ranger
(
  sensor(
    fov 360.0
    samples 360
  )
  model
  (
    size [ 0.001 0.001 0 ]
    color "black"
    #gui_grid 1
  )
)

define reflectance blobfinder
(
  colors_count 5
  colors [ "red" "blue" "green" "orange" "yellow" ]
  range 0.03
  fov 250
  image [50 50]
  model
  (
    size [0.0 0.0 0.0]
  )
)

define cam camera
(
  resolution [50 50]
  range [0.02 0.1]
  fov [ 10 5 ]
  pantilt [ 0 90 ]

  size [0.02 0.02 0.02]
  color "blue"
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
  reflectance( pose [0.0 0.0 -0.15 0.0 ])
  cam ( pose [0.0 0.0 -0.12 0.0 ])
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

define roomr model(
  size [0.46 0.02 0.0]
  color "red"
  fiducial_return 0
  obstacle_return 0
  gui_move 1
  blob_return 1
)

define roomb model(
  size [0.46 0.02 0.0]
  color "blue"
  fiducial_return 0
  obstacle_return 0
  gui_move 1
  blob_return 1
)

define roomg model(
  size [0.46 0.02 0.0]
  color "green"
  fiducial_return 0
  obstacle_return 0
  gui_move 1
  blob_return 1
)

define roomo model(
  size [0.46 0.02 0.0]
  color "orange"
  fiducial_return 0
  obstacle_return 0
  gui_move 1
  blob_return 1
)

define roomy model(
  size [0.46 0.02 0.0]
  color "yellow"
  fiducial_return 0
  obstacle_return 0
  gui_move 1
  blob_return 1
)

resolution 0.02 
interval_sim 100

