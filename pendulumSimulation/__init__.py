import random as rnd
import math

G = 9.81


class CartPendulumSimulation:
    def __init__(self, cart_mass=1.0, pendulum_mass=0.1, pendulum_length=0.5,
                 cart_friction=0.1, pendulum_friction=0.05, pendulum_theta=math.pi):

        self.thetaReset = pendulum_theta

        self.m_c = cart_mass
        self.m_p = pendulum_mass
        self.L = pendulum_length

        self.track_limit = 2.4  # maximale Auslenkung in beide Richtungen (Meter)

        self.x = 0.0
        self.x_dot = 0.0
        self.x_ddot = 0.0
        self.theta = pendulum_theta
        self.theta_dot = 0.0
        self.theta_ddot = 0.0

        # Reibungskoeffizienten
        self.b_c = cart_friction
        self.b_p = pendulum_friction

    def update(self, dt, f):
        sin_theta = math.sin(self.theta)
        cos_theta = math.cos(self.theta)

        # Reibungskraft des Wagens
        F_friction = -self.b_c * self.x_dot

        # Gesamtkraft auf den Wagen
        F_ext = f + F_friction

        denom = self.m_c + self.m_p * sin_theta ** 2
        # Beschleunigungen berechnen:
        self.x_ddot = (F_ext
                       + self.m_p * self.L * (self.theta_dot ** 2) * sin_theta
                       + self.m_p * G * sin_theta * cos_theta
                       ) / denom

        self.theta_ddot = (-F_ext * cos_theta
                           - self.m_p * self.L * (self.theta_dot ** 2) * sin_theta * cos_theta
                           - (self.m_c + self.m_p) * G * sin_theta
                           ) / (self.L * denom)

        # semi-implizites Euler-Verfahren
        self.x_dot += self.x_ddot * dt
        self.x += self.x_dot * dt

        self.theta_dot += self.theta_ddot * dt
        self.theta += self.theta_dot * dt

        self.theta %= 2 * math.pi  # clip theta

        if self.x > self.track_limit:  # track limit
            self.x = self.track_limit
            self.x_dot = 0.0
            self.x_ddot = 0.0
        elif self.x < -self.track_limit:
            self.x = -self.track_limit
            self.x_dot = 0.0
            self.x_ddot = 0.0

    def reset(self):
        self.x = 0.0
        self.x_dot = 0.0
        self.x_ddot = 0.0
        self.theta = self.thetaReset
        self.theta_dot = 0.0
        self.theta_ddot = 0.0
