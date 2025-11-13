import pybullet as p
import pybullet_data
import numpy as np
import time

# Inicjalizacja PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()

# Wczytaj płaszczyznę oraz punkt startowy i docelowy
plane = p.loadURDF("plane.urdf")
start_pos = [0, 0, 0.1]  # Pozycja startowa robota
goal_pos = [5, 5, 0.1]  # Pozycja docelowa
p.addUserDebugText("Goal", goal_pos, textColorRGB=[1, 0, 0], textSize=2)

# Dodaj model robota (Husky)
robot = p.loadURDF("husky/husky.urdf", start_pos)

# Parametry symulacji
p.setGravity(0, 0, -9.81)
goal_tolerance = 0.2
learning_rate = 5.0  # Prędkość
turning_rate = 2.0  # Szybkość skrętu

# Zidentyfikuj silniki robota
wheel_joints = [2, 3, 4, 5]  # Husky ma cztery koła napędowe


def calculate_reward(robot_position, goal_position):
    """
    Funkcja nagrody na podstawie odległości od celu.
    Nagroda jest większa, gdy robot zbliża się do celu i mniejsza przy oddalaniu.
    """
    distance = np.linalg.norm(np.array(robot_position[:2]) - np.array(goal_position[:2]))
    reward = -distance  # Ujemna nagroda za odległość
    if distance < goal_tolerance:
        reward += 100  # Dodatkowa nagroda za osiągnięcie celu
    return reward, distance


# Pętla symulacji
for step in range(3000):  # Zwiększono liczbę kroków
    # Pobierz aktualną pozycję i orientację robota
    robot_pos, robot_orn = p.getBasePositionAndOrientation(robot)
    robot_yaw = p.getEulerFromQuaternion(robot_orn)[2]  # Kąt obrotu wokół osi Z

    # Oblicz nagrodę na podstawie odległości od celu
    reward, distance_to_goal = calculate_reward(robot_pos, goal_pos)

    # Kierunek do celu
    direction = np.array(goal_pos[:2]) - np.array(robot_pos[:2])
    target_yaw = np.arctan2(direction[1], direction[0])  # Celowy kąt do obrotu

    # Obliczenie różnicy kąta
    yaw_error = target_yaw - robot_yaw
    if yaw_error > np.pi:
        yaw_error -= 2 * np.pi
    elif yaw_error < -np.pi:
        yaw_error += 2 * np.pi

    # Ustawienie prędkości dla każdego koła z uwzględnieniem kierunku
    if abs(yaw_error) > 0.1:  # Skręt, jeśli różnica kąta jest duża
        left_velocity = -turning_rate if yaw_error > 0 else turning_rate
        right_velocity = turning_rate if yaw_error > 0 else -turning_rate
    else:  # Jazda na wprost
        left_velocity = learning_rate
        right_velocity = learning_rate

    # Sterowanie napędami
    p.setJointMotorControl2(robot, wheel_joints[0], p.VELOCITY_CONTROL, targetVelocity=left_velocity)
    p.setJointMotorControl2(robot, wheel_joints[1], p.VELOCITY_CONTROL, targetVelocity=right_velocity)
    p.setJointMotorControl2(robot, wheel_joints[2], p.VELOCITY_CONTROL, targetVelocity=left_velocity)
    p.setJointMotorControl2(robot, wheel_joints[3], p.VELOCITY_CONTROL, targetVelocity=right_velocity)

    # Wyświetlanie informacji o stanie robota
    print(f"Step: {step}, Position: {robot_pos[:2]}, Distance to goal: {distance_to_goal:.2f}, Reward: {reward:.2f}")

    # Sprawdzenie, czy robot osiągnął cel
    if distance_to_goal < goal_tolerance:
        print("Robot osiągnął cel!")
        break

    # Przeprowadzenie kroku symulacji
    p.stepSimulation()
    time.sleep(0.01)

# Zamknięcie symulacji
p.disconnect()