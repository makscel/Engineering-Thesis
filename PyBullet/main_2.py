import pybullet as p
import pybullet_data
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
import time


class MobileRobotEnv(gym.Env):

    def __init__(self):
        super(MobileRobotEnv, self).__init__()

        # Konfiguracja PyBullet
        p.connect(p.GUI)  # Uruchamiamy w trybie GUI, aby zobaczyć symulację
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Parametry środowiska
        self.goal_pos = np.array([5, 5])  # Cel robota
        self.start_pos = np.array([0, 0])  # Pozycja startowa
        self.goal_tolerance = 0.5  # Tolerancja celu
        self.previous_distance = None  # Zmienna do śledzenia poprzedniej odległości od celu

        # Przestrzeń akcji (skręt i prędkość)
        self.action_space = spaces.Box(low=np.array([-1, 0]), high=np.array([1, 1]), dtype=np.float32)

        # Przestrzeń stanów (x, y, orientacja robota)
        self.observation_space = spaces.Box(low=np.array([-10, -10, -np.pi]),
                                            high=np.array([10, 10, np.pi]), dtype=np.float32)

        # Inicjalizacja środowiska
        self.seed()  # Inicjalizacja generatora liczb losowych
        self.reset()

    def seed(self, seed=None):
        """Ustawia ziarno dla generatora liczb losowych"""
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

    def reset(self, seed=None, options=None):
        """Resetuje środowisko, przyjmuje opcjonalne ziarno (seed)"""
        # Ustawienie ziarna
        if seed is not None:
            self.seed(seed)

        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")

        # Pozycja startowa robota
        self.robot = p.loadURDF("husky/husky.urdf", [self.start_pos[0], self.start_pos[1], 0.1])
        self.wheel_joints = [2, 3, 4, 5]  # Identyfikatory napędów

        # Zaznaczenie celu na mapie
        p.addUserDebugText("Goal", [self.goal_pos[0], self.goal_pos[1], 0.2], textColorRGB=[1, 0, 0], textSize=2)
        p.addUserDebugLine(
            np.append(self.goal_pos, 0.2) - np.array([0.2, 0.2, 0]),
            np.append(self.goal_pos, 0.2) + np.array([0.2, 0.2, 0]),
            lineColorRGB=[1, 0, 0],
            lineWidth=3
        )

        # Reset pozycji i poprzedniej odległości
        self.robot_pos = self.start_pos
        self.robot_yaw = 0
        self.previous_distance = np.linalg.norm(self.robot_pos - self.goal_pos)

        return self._get_observation(), {}  # Gymnasium wymaga zwrócenia obs, info

    def _get_observation(self):
        """Zwraca obserwację (pozycja, orientacja)"""
        pos, orn = p.getBasePositionAndOrientation(self.robot)
        yaw = p.getEulerFromQuaternion(orn)[2]
        return np.array([pos[0], pos[1], yaw], dtype=np.float32)

    def _calculate_angle_to_goal(self):
        """Oblicza kąt między aktualnym kierunkiem robota a wektorem do celu"""
        robot_pos, robot_yaw = self._get_observation()[:2], self._get_observation()[2]
        direction_to_goal = np.array(self.goal_pos) - np.array(robot_pos)
        angle_to_goal = np.arctan2(direction_to_goal[1], direction_to_goal[0])

        # Różnica między aktualnym kątem a kątem do celu
        angle_difference = angle_to_goal - robot_yaw
        angle_difference = (angle_difference + np.pi) % (2 * np.pi) - np.pi  # Normalizacja do zakresu [-pi, pi]

        return angle_difference

    def step(self, action):
        """Wykonuje akcję w środowisku"""

        # Sterowanie napędem
        left_velocity = action[1] + action[0]
        right_velocity = action[1] - action[0]
        for i, wheel in enumerate(self.wheel_joints):
            p.setJointMotorControl2(self.robot, wheel, p.VELOCITY_CONTROL,
                                    targetVelocity=left_velocity if i % 2 == 0 else right_velocity)

        # Przeprowadzenie kroku symulacji
        p.stepSimulation()

        # Aktualizacja pozycji i nagroda
        obs = self._get_observation()
        self.robot_pos = obs[:2]
        distance_to_goal = np.linalg.norm(self.robot_pos - self.goal_pos)

        # Obliczanie nagrody na podstawie zmiany odległości
        reward = 0
        if self.previous_distance is not None:
            if distance_to_goal < self.previous_distance:
                reward = 10.0
            else:
                reward = -10.0

        self.previous_distance = distance_to_goal

        # Sprawdzanie, czy robot osiągnął cel
        terminated = False
        if distance_to_goal < self.goal_tolerance:
            terminated = True
            reward += 100  # Nagroda za osiągnięcie celu
            print("Robot osiągnął cel!")

        # Debugowanie, aby sprawdzić, dlaczego epizod może się kończyć
        print(
            f"Step: Position: {self.robot_pos}, Distance to Goal: {distance_to_goal:.2f}, Reward: {reward:.2f}")

        return obs, reward, terminated, False, {}  # Gymnasium: zwracamy obs, reward, terminated, truncated, info

    def close(self):
        p.disconnect()


# Inicjalizacja środowiska i sprawdzenie poprawności
env = MobileRobotEnv()
check_env(env, warn=True)

# Trenowanie modelu PPO
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=50000)

# Testowanie nauczonego modelu
try:
    obs = env.reset()[0]  # Gymnasium: reset zwraca tuple (obs, info)
    while True:  # Pętla, aby GUI pozostało otwarte
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)

        # Opóźnienie do obserwacji GUI
        time.sleep(0.01)

        if terminated:
            obs = env.reset()[0]  # Resetuje środowisko po osiągnięciu celu
finally:
    env.close()
