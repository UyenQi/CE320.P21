from controller import Robot
import skfuzzy as fuzz
import skfuzzy.control as ctrl
import numpy as np

robot = Robot()
timestep = int(robot.getBasicTimeStep())

sensors = []
sensor_names = ['ps0', 'ps1', 'ps6', 'ps7']
for name in sensor_names:
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    sensors.append(sensor)

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

left_sensor = ctrl.Antecedent(np.arange(0, 2000, 1), 'left')
right_sensor = ctrl.Antecedent(np.arange(0, 2000, 1), 'right')
turn = ctrl.Consequent(np.arange(-3.0, 3, 0.1), 'turn')
go = ctrl.Consequent(np.arange(-3.0, 3, 0.1), 'go')

left_sensor['far'] = fuzz.trapmf(left_sensor.universe, [0, 20, 60, 80])
left_sensor['near'] = fuzz.trapmf(left_sensor.universe, [80, 200, 800, 1000])
right_sensor['far'] = fuzz.trapmf(right_sensor.universe, [0, 20, 60, 80])
right_sensor['near'] = fuzz.trapmf(right_sensor.universe, [80, 200, 800, 1000])

turn['left'] = fuzz.trapmf(turn.universe, [-2.8, -2.2, -1.8, -1.2])
turn['straight'] = fuzz.trapmf(turn.universe, [-0.1, 0, 0, 0.1])
turn['right'] = fuzz.trapmf(turn.universe, [1.2, 1.8, 2.2, 2.8])

go['stop'] = fuzz.trimf(go.universe, [0, 0, 0.2])
go['forward_slow'] = fuzz.trimf(go.universe, [0.0, 0.2, 0.5])
go['forward_fast'] = fuzz.trapmf(go.universe, [0.3, 0.7, 1.0, 1.0])

rule1 = ctrl.Rule(left_sensor['far'] & right_sensor['far'], (turn['straight'], go['forward_fast']))
rule2 = ctrl.Rule(left_sensor['near'] & right_sensor['far'], (turn['right'], go['forward_slow']))
rule3 = ctrl.Rule(left_sensor['far'] & right_sensor['near'], (turn['left'], go['forward_slow']))
rule4 = ctrl.Rule(left_sensor['near'] & right_sensor['near'], (turn['left'], go['stop']))

turning_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4])
turning = ctrl.ControlSystemSimulation(turning_ctrl)

MAX_SPEED = 6.28
base_speed = 5.0

while robot.step(timestep) != -1:
    left_val = (sensors[0].getValue() + sensors[1].getValue()) / 2
    right_val = (sensors[2].getValue() + sensors[3].getValue()) / 2

    turning.input['left'] = left_val
    turning.input['right'] = right_val
    turning.compute()

    direction = turning.output['turn']
    speed_factor = turning.output['go']

    if speed_factor < -0.1: 
        left_motor_speed = speed_factor * base_speed
        right_motor_speed = speed_factor * base_speed
    else:
        left_motor_speed = speed_factor * base_speed * (1 - direction)
        right_motor_speed = speed_factor * base_speed * (1 + direction)

    left_motor_speed = max(min(left_motor_speed, MAX_SPEED), -MAX_SPEED)
    right_motor_speed = max(min(right_motor_speed, MAX_SPEED), -MAX_SPEED)

    left_motor.setVelocity(left_motor_speed)
    right_motor.setVelocity(right_motor_speed)
    
    print(f"Left_Sensor: {left_val:.2f}, Right_Sensor: {right_val:.2f}, Turn_Output: {direction:.2f}, Go_Output: {speed_factor:.2f}")