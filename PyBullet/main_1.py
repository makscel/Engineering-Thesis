import pybullet as p
import pybullet_data
import time
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import DummyVecEnv


class RobotEnv(gym.Env):
    def __init__(self):
        super(RobotEnv, self).__init__()

        # Inicjalizacja PyBullet z GUI
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        # Załaduj płaszczyznę i robota
        self.plane = p.loadURDF("plane.urdf")
        self.robot_id = p.loadURDF("husky/husky.urdf", [0, 0, 0.1])

        # Definicja celu i przeszkód
        self.target_position = np.array([5, 5])
        self.target_marker = p.loadURDF("sphere2.urdf", self.target_position.tolist() + [0.5], globalScaling=0.3)
        self.obstacle_ids = [p.loadURDF("cube.urdf", [2.5, 2.5, 0.5], globalScaling=1.5)]

        # Przestrzeń stanów i akcji
        self.action_space = spaces.Discrete(4)
        self.observation_space = spaces.Box(low=-10, high=10, shape=(3,), dtype=np.float32)

        # Przechowywanie poprzedniej odległości dla funkcji nagrody
        self.previous_distance = None

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")
        self.robot_id = p.loadURDF("husky/husky.urdf", [0, 0, 0.1])

        # Ustawienie celu i przeszkód
        self.target_position = np.array([5, 5])
        self.target_marker = p.loadURDF("sphere2.urdf", self.target_position.tolist() + [0.5], globalScaling=0.3)
        self.obstacle_ids = [p.loadURDF("cube.urdf", [2.5, 2.5, 0.5], globalScaling=1.5)]

        # Zresetuj poprzednią odległość
        self.previous_distance = None
        obs = self._get_observation()
        self.previous_distance = obs[2]  # Ustawienie wstępnej odległości do celu

        return obs, {}

    def _get_observation(self):
        # Pobierz pozycję robota i oblicz odległość do celu
        position, _ = p.getBasePositionAndOrientation(self.robot_id)
        distance_to_target = np.linalg.norm(np.array(position[:2]) - self.target_position)
        return np.array([position[0], position[1], distance_to_target], dtype=np.float32)

    def step(self, action):
        # Dynamika ruchu z mniejszą prędkością przy bliskim zbliżeniu do celu
        if self.previous_distance and self.previous_distance < 1.0:
            wheel_velocity = 1.0  # Zmniejszona prędkość blisko celu
        else:
            wheel_velocity = 2.0  # Standardowa prędkość

        # Sterowanie kół
        if action == 0:  # Przód
            p.setJointMotorControlArray(self.robot_id, [2, 3, 4, 5], p.VELOCITY_CONTROL,
                                        targetVelocities=[wheel_velocity] * 4)
        elif action == 1:  # Tył
            p.setJointMotorControlArray(self.robot_id, [2, 3, 4, 5], p.VELOCITY_CONTROL,
                                        targetVelocities=[-wheel_velocity] * 4)
        elif action == 2:  # Skręt w lewo
            p.setJointMotorControlArray(self.robot_id, [2, 3], p.VELOCITY_CONTROL,
                                        targetVelocities=[-wheel_velocity, wheel_velocity])
            p.setJointMotorControlArray(self.robot_id, [4, 5], p.VELOCITY_CONTROL,
                                        targetVelocities=[-wheel_velocity, wheel_velocity])
        elif action == 3:  # Skręt w prawo
            p.setJointMotorControlArray(self.robot_id, [2, 3], p.VELOCITY_CONTROL,
                                        targetVelocities=[wheel_velocity, -wheel_velocity])
            p.setJointMotorControlArray(self.robot_id, [4, 5], p.VELOCITY_CONTROL,
                                        targetVelocities=[wheel_velocity, -wheel_velocity])

        # Krok symulacji
        p.stepSimulation()
        time.sleep(1. / 240)

        # Obserwacja i nagroda
        obs = self._get_observation()
        distance_to_target = obs[2]

        # Funkcja nagrody oparta na różnicy odległości
        reward = 0
        if self.previous_distance is not None:
            reward = self.previous_distance - distance_to_target  # Nagroda za zbliżenie się do celu

            # Dodatkowa kara, jeśli robot zaczyna się oddalać, gdy jest blisko celu
            if self.previous_distance < 1.0 and distance_to_target > self.previous_distance:
                reward -= 5

        # Aktualizacja poprzedniej odległości
        self.previous_distance = distance_to_target

        # Nagroda za osiągnięcie celu
        if distance_to_target < 0.5:
            reward += 100
            done = True
        else:
            done = False

        # Kara za kolizje
        for obstacle_id in self.obstacle_ids:
            contacts = p.getContactPoints(self.robot_id, obstacle_id)
            if contacts:
                reward -= 10
                done = True

        # Debugging pozycji
        print(f"Pozycja robota: {obs[:2]}, Dystans do celu: {distance_to_target}, Nagroda: {reward}")

        return obs, reward, done, False, {}

    def render(self, mode="human"):
        pass

    def close(self):
        p.disconnect()


# Inicjalizacja środowiska i opakowanie w DummyVecEnv
env = DummyVecEnv([lambda: RobotEnv()])

# Konfiguracja modelu RL
model = PPO("MlpPolicy", env, verbose=1)

# Trening modelu
model.learn(total_timesteps=20000)

# Testowanie modelu w GUI
obs = env.reset()
for i in range(1000):
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, done, info = env.step(action)
    time.sleep(1. / 60.)
    if done:
        obs = env.reset()
