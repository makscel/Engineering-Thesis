import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl 
from controller import Robot

# Inicjalizacja robota w środowisku Webots
robot = Robot()

# Pobranie kroku czasowego symulacji
timestep = int(robot.getBasicTimeStep())

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
            sensor.enable(timestep)  # Włącz czujnik z podanym krokiem czasowym
            sensors.append(sensor)
        else:
            print("Sensor ds{} not found.".format(i))
    
    return sensors, motors

# Funkcja do odczytu pomiarów czujników
def k3ReadProximitySensors(sensors):
    sensor_values = [sensor.getValue() for sensor in sensors]
    return sensor_values
    
# Definicja funkcji sterującej na podstawie systemu rozmytego Mamdaniego
def k3FuzzyAvoidEval(avoid_sym, val_left, val_front, val_right):
    avoid_sym.input['left'] = val_left
    avoid_sym.input['front'] = val_front
    avoid_sym.input['right'] = val_right
    avoid_sym.compute()
    #print('vl=',avoid_sym.output['vl'], ' vr=',avoid_sym.output['vr'])
    return avoid_sym.output['vl'], avoid_sym.output['vr']

# Funkcja symulująca pętlę sterowania
def k3FuzzyAvoidLoop(avoid_sym, sensors, motors):
    #avoid_sym = k3FuzzyAvoidDef() 
    iter = 0
    while robot.step(timestep) != -1 and iter <= 100000: 
        # Odczyt danych z czujników
        sensor_values = k3ReadProximitySensors(sensors)
        val_left = max(sensor_values[1], sensor_values[2])
        val_front = max(sensor_values[3], sensor_values[4])
        val_right = max(sensor_values[5], sensor_values[6])
        
        print(f'left sensors: {val_left}, front sensors: {val_front}, right sensors: {val_right}')
        
        # Symulacja obliczeń sterowania na podstawie systemu rozmytego
        vl, vr = k3FuzzyAvoidEval(avoid_sym, val_left, val_front, val_right)
        
        # Symulacja ustawienia prędkości kół robota
        motors['left'].setVelocity(vl)
        motors['right'].setVelocity(vr)
        
        iter += 1

# Funkcja definiująca system rozmyty Mamdaniego
def k3FuzzyAvoidDef():
    MaxProximitySignal = 4096
    MaxSpeed = 19.1
    
    left = ctrl.Antecedent(np.arange(0, MaxProximitySignal, 10), 'left')
    front = ctrl.Antecedent(np.arange(0, MaxProximitySignal, 10), 'front')
    right = ctrl.Antecedent(np.arange(0, MaxProximitySignal, 10), 'right')
    vl = ctrl.Consequent(np.arange(-MaxSpeed, MaxSpeed, 10), 'vl')
    vr = ctrl.Consequent(np.arange(-MaxSpeed, MaxSpeed, 10), 'vr')
    
    # Definicja zbiorów rozmytych
    left['S'] = fuzz.trimf(left.universe, [0, 0, MaxProximitySignal])
    left['B'] = fuzz.trimf(left.universe, [0, MaxProximitySignal, MaxProximitySignal])
    front['S'] =fuzz.trimf(front.universe, [0, 0, MaxProximitySignal])
    front['B'] =fuzz.trimf(front.universe, [0, MaxProximitySignal, MaxProximitySignal])
    right['S'] =fuzz.trimf(right.universe, [0, 0, MaxProximitySignal])
    right['B'] =fuzz.trimf(right.universe, [0, MaxProximitySignal, MaxProximitySignal])
    vl['back'] =fuzz.trimf(vl.universe, [-MaxSpeed, -MaxSpeed, 0])
    vl['front'] =fuzz.trimf(vl.universe, [0, MaxSpeed, MaxSpeed])
    vr['back'] =fuzz.trimf(vr.universe, [-MaxSpeed, -MaxSpeed, 0])
    vr['front'] =fuzz.trimf(vr.universe, [0, MaxSpeed, MaxSpeed])
    
    # Definicja reguł sterowania
    rule1 = ctrl.Rule(antecedent=(left['S'] & front['S'] & right['S']),
    consequent=(vl['front'], vr['front']) )
    rule2 =ctrl.Rule(antecedent=(left['S'] & front['S'] & right['B']),
    consequent=(vl['back'], vr['front']) )
    rule3 =ctrl.Rule(antecedent=(left['S'] & front['B'] & right['S']),
    consequent=(vl['back'], vr['front']) )
    rule4 =ctrl.Rule(antecedent=(left['S'] & front['B'] & right['B']),
    consequent=(vl['back'], vr['front']) )
    rule5 =ctrl.Rule(antecedent=(left['B'] & front['S'] & right['S']),
    consequent=(vl['front'], vr['back']) )
    rule6 =ctrl.Rule(antecedent=(left['B'] & front['B'] & right['S']),
    consequent=(vl['front'], vr['back']) )
    rule7 =ctrl.Rule(antecedent=(left['B'] & front['S'] & right['B']),
    consequent=(vl['front'], vr['front']) )
    rule8 =ctrl.Rule(antecedent=(left['B'] & front['B'] & right['B']),
    consequent=(vl['back'], vr['front']) )
    
    # Inicjalizacja i zwrócenie systemu rozmytego
    avoid_ctr = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8])
    avoid_sym = ctrl.ControlSystemSimulation(avoid_ctr)
    return avoid_sym

############################## Główna część programu #############################

sensors, motors = initialize_devices()
avoid_sym = k3FuzzyAvoidDef()
k3FuzzyAvoidLoop(avoid_sym, sensors, motors)
         