from controllers import PID, MPC
from utils import *
from parameters import *
from car import Car
from draw import Draw
import HC05

def motor_movements(path_input, car_x, car_y, robot_wheelbase, VIEW_W, VIEW_H, orientation):

	path = []

	for point in reversed(path_input):
		x = round(point[0])
		y = round(point[1])
		point = [x, y]
		path.append(point)

	controller_name = "MPC"

	draw = Draw(VIEW_W, VIEW_H, window_name = "Canvas")

	car = Car(car_x, car_y, orientation)

	if controller_name == "PID":
		controller = PID(kp_linear = 0.5, kd_linear = 0.1, ki_linear = 0,
								kp_angular = 3, kd_angular = 0.1, ki_angular = 0)
	if controller_name == "MPC":
		controller = MPC(horizon = MPC_HORIZON)

	class Robot:
		def __init__(self, wheelbase):
			self.wheelbase = wheelbase  # Distance between the wheels

		def calculate_wheel_speeds(self, linear_velocity, angular_velocity):
			v_l = linear_velocity - (self.wheelbase / 2) * angular_velocity
			v_r = linear_velocity + (self.wheelbase / 2) * angular_velocity
			return v_l, v_r

	current_idx = 0
	linear_v = 0
	angular_v = 0
	max_velocity = abs(TECHNICAL_MAX_LINEAR_VELOCITY + (robot_wheelbase / 2) * (TECHNICAL_MAX_LINEAR_VELOCITY * 2 / robot_wheelbase))
	car_path_points = []
	robot = Robot(wheelbase=robot_wheelbase)

	stop = False
	force_stop = False

	while not stop and not force_stop: 
		draw.clear()
		if len(path)>0:
			draw.draw_path(path, color = (200, 200, 200), thickness = 1)

		if len(car_path_points)>0:
			draw.draw_path(car_path_points, color = (255, 0, 0), thickness = 1, dotted = True)

		draw.draw(car.get_points(), color = (255, 0, 0), thickness = 1)
		

		k = draw.show()

		x, _ = car.get_state()

		if len(path)>0 and current_idx != len(path):
			car_path_points.append([int(x[0, 0]), int(x[1, 0])])
			goal_pt = path[current_idx]

			if controller_name == "PID":
				linear_v, angular_v = controller.get_control_inputs(x, goal_pt, car.get_points()[2], current_idx)

				left_speed, right_speed = robot.calculate_wheel_speeds(linear_v, angular_v)

				speed_l_output = left_speed / max_velocity
				speed_r_output = right_speed / max_velocity
			
			if controller_name == "MPC":
				linear_v, angular_v = controller.optimize(car = car, goal_x = goal_pt)

				left_speed, right_speed = robot.calculate_wheel_speeds(linear_v, angular_v)
			
			dist = get_distance(x[0, 0], x[1, 0], goal_pt[0], goal_pt[1])
			if dist<10:
				current_idx+= 1

		else:
			linear_v = 0
			angular_v = 0

			speed_l_output = 0
			speed_r_output = 0

			stop = True

		car.set_robot_velocity(linear_v, angular_v)
		car.update(DELTA_T)

		# print(f"L: {speed_l_output} R: {speed_r_output}")

		print(f"Left: {left_speed} Right: {right_speed}")

		# Assumes MPC Controller
  		# The code below is not a mistype, my coding skills are just trash and they ended up like that in relation to the real robot
		HC05.save_movements(left_speed, right_speed, DELTA_T)
		
		if k == ord("a"):
			force_stop = True
			HC05.move_motors(0, 0)
	
	if not force_stop:

		HC05.start_movements()

		if k == ord("a"):
			HC05.move_motors(0, 0)
	