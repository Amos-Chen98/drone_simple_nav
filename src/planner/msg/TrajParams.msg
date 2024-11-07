int16 drone_id
time start_time # the ros time when the trajectory starts
float32[] init_state # drone state at the start of the local trajectory, row major, [x, y, vx, vy]
float32[] target_state # drone state at the end of the local trajectory, row major, [x, y, vx, vy]
float32[] int_wpts_x # intermediate waypoints x, M-1 length
float32[] int_wpts_y # intermediate waypoints y, M-1 length
float32[] ts # time of each seg, M length