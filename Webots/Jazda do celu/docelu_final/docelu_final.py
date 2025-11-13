import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from math import pi, sin, cos, atan2, sqrt
from controller import Robot, Supervisor

# Inicjalizacja robota
robot = Robot()
#robot = Supervisor()

# Pobranie kroku czasowego symulacji
timestep = int(robot.getBasicTimeStep())

gps = robot.getDevice("gps")

# Inicjalizacja IMU
imu = robot.getDevice("inertial unit") 
imu.enable(timestep)

gps.enable(timestep)

# Inicjalizacja urządzeń
def initialize_devices():
    sensors = []
    motors = {'left': robot.getDevice('left wheel motor'),
              'right': robot.getDevice('right wheel motor')}
    motors['left'].setPosition(float('inf'))
    motors['right'].setPosition(float('inf'))
    motors['left'].setVelocity(0)
    motors['right'].setVelocity(0)
    
    for i in range(9):
        sensor = robot.getDevice("ds" + str(i))
        if sensor is not None:
            print("Sensor ds{} found! :D".format(i))
            sensor.enable(timestep) 
            sensors.append(sensor)
        else:
            print("Sensor ds{} not found.".format(i))
    
    return sensors, motors

def k3ReadProximitySensors(sensors):
    sensor_values = [sensor.getValue() for sensor in sensors]
    return sensor_values
    
def k3FuzzyGoalDef():
    #R = 44
    MaxZ = 2 * sqrt(2)
    MaxSpeed = 6.28 #rad/s
    z = ctrl.Antecedent(np.arange(0, MaxZ, 1), 'z')
    psi = ctrl.Antecedent(np.arange(-pi, pi, 1/10 * pi), 'psi')
    vl = ctrl.Consequent(np.arange(-MaxSpeed, MaxSpeed, 10), 'vl')
    vr = ctrl.Consequent(np.arange(-MaxSpeed, MaxSpeed, 10), 'vr')
    z['NR'] = fuzz.trimf(z.universe, [0, 0, MaxZ])
    z['FR'] = fuzz.trimf(z.universe, [0, MaxZ, MaxZ])
    psi['N'] = fuzz.trimf(psi.universe, [-pi, -pi, 0])
    psi['Z'] = fuzz.trimf(psi.universe, [-pi, 0, pi])
    psi['P'] = fuzz.trimf(psi.universe, [0, pi, pi])
    vl['back'] = fuzz.trimf(vl.universe, [-MaxSpeed, -MaxSpeed, 0])
    vl['stop'] = fuzz.trimf(vl.universe, [-MaxSpeed, 0, MaxSpeed])
    vl['front'] = fuzz.trimf(vl.universe, [0, MaxSpeed, MaxSpeed])
    vr['back'] = fuzz.trimf(vr.universe, [-MaxSpeed, -MaxSpeed, 0])
    vr['stop'] = fuzz.trimf(vr.universe, [-MaxSpeed, 0, MaxSpeed])
    vr['front'] = fuzz.trimf(vr.universe, [0, MaxSpeed, MaxSpeed])
    
    # Definicja reguł
    rule1 = ctrl.Rule(antecedent=(z['NR'] & psi['N']), consequent=(vl['back'], vr['front']))
    rule2 = ctrl.Rule(antecedent=(z['NR'] & psi['Z']), consequent=(vl['stop'], vr['stop']))
    rule3 = ctrl.Rule(antecedent=(z['NR'] & psi['P']), consequent=(vl['front'], vr['back']))
    rule4 = ctrl.Rule(antecedent=(z['FR'] & psi['N']), consequent=(vl['back'], vr['front']))
    rule5 = ctrl.Rule(antecedent=(z['FR'] & psi['Z']), consequent=(vl['front'], vr['front']))
    rule6 = ctrl.Rule(antecedent=(z['FR'] & psi['P']), consequent=(vl['front'], vr['back']))
    
    goal_ctr = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6])
    goal_sym = ctrl.ControlSystemSimulation(goal_ctr)
    return goal_sym

def k3FuzzyGoalEval(goal_sym, val_z, val_psi):
    goal_sym.input['z'] = val_z
    goal_sym.input['psi'] = val_psi
    goal_sym.compute()
    return goal_sym
   

def k3FuzzyGoalLoop(xg, yg, sensors, motors):
    # Inicjalizacja logiki rozmytej
    goal_sym = k3FuzzyGoalDef()

    # Parametry sterowania
    MaxSpeed = 6.28  # Maksymalna prędkość w rad/s
    EpsilonZ = 0.05  # Tolerancja dla osiągnięcia celu
    EpsilonPsi = 0.05  # Tolerancja kąta skierowania robota do celu

    while robot.step(timestep) != -1:
        # Pobranie bieżącej pozycji robota z GPS
        gps_values = gps.getValues()
        x = gps_values[0]
        y = gps_values[1]

        # Pobranie orientacji robota z IMU
        imu_values = imu.getRollPitchYaw()  # Odczyt kąta orientacji (yaw)
        theta_robot = imu_values[2]  # Yaw (obrót w płaszczyźnie poziomej)

        # Obliczanie odległości z do celu
        z = sqrt((xg - x) ** 2 + (yg - y) ** 2)

        # Obliczanie kąta do celu
        theta_to_goal = atan2(yg - y, xg - x)  # Kąt do celu
        psi = theta_to_goal - theta_robot  # Różnica między kątem celu a aktualną orientacją

        # Normalizacja kąta psi do zakresu [-pi, pi]
        if psi > pi:
            psi -= 2 * pi
        elif psi < -pi:
            psi += 2 * pi

        # Wypisywanie wartości w konsoli
        print(f"x={x:.2f}, y={y:.2f}, z={z:.2f}, theta_robot={theta_robot:.2f}, psi={psi:.2f}")

        # Wprowadzenie wartości wejściowych do logiki rozmytej
        goal_sym.input['z'] = z
        goal_sym.input['psi'] = psi
        goal_sym.compute()

        # Odczyt prędkości z logiki rozmytej
        vl = goal_sym.output['vl']
        vr = goal_sym.output['vr']

        # Ograniczenie prędkości
        vl = max(min(vl, MaxSpeed), -MaxSpeed)
        vr = max(min(vr, MaxSpeed), -MaxSpeed)

        # Ustawienie prędkości kół
        motors['left'].setVelocity(vl)
        motors['right'].setVelocity(vr)
        
        # Warunek zakończenia
        if z < EpsilonZ:
            motors['left'].setVelocity(0)
            motors['right'].setVelocity(0)
            print("Goal reached!")
            break



sensors, motors = initialize_devices()
k3FuzzyGoalLoop(-0.3, -0.7, sensors, motors)  
