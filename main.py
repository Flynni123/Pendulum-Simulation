import math

import pygame as pg

import pendulumSimulation as pSim


class Display:

    def __init__(self):

        width, height = 800, 600
        self.disp = pg.display.set_mode((width, height))
        self.clock = pg.time.Clock()

        self.sim = pSim.CartPendulumSimulation()

        scale = 100  # 1 Meter = 100 Pixel
        ground_y = height * 0.8  # Boden bei 80 %
        cart_width = 0.5 * scale  # Wagenbreite (50 cm)
        cart_height = 0.2 * scale  # Wagenhöhe   (20 cm)

        tCurr = 0
        score = 0

        run = True
        while run:

            dt = self.clock.tick(60) / 1000.0
            f = pg.key.get_pressed()[pg.K_LEFT] * -10 + pg.key.get_pressed()[pg.K_RIGHT] * 10
            self.sim.update(dt, f)
            tCurr += dt

            self.disp.fill((255, 255, 255))

            # Bodenlinie
            pg.draw.line(self.disp, (0, 0, 0), (0, ground_y), (width, ground_y), 2)

            # Schienenbegrenzung
            left_limit = width / 2 - self.sim.track_limit * scale
            right_limit = width / 2 + self.sim.track_limit * scale
            pg.draw.line(self.disp, (180, 0, 0), (left_limit, ground_y - 40), (left_limit, ground_y + 10), 4)
            pg.draw.line(self.disp, (180, 0, 0), (right_limit, ground_y - 40), (right_limit, ground_y + 10), 4)

            # Wagen
            cart_x = width / 2 + self.sim.x * scale  # Koord. x des Wagenmittelpunkts
            cart_y = ground_y - cart_height
            cart_rect = pg.Rect(cart_x - cart_width / 2, cart_y, cart_width, cart_height)
            pg.draw.rect(self.disp, (50, 50, 200), cart_rect)

            # Pendel
            pivot_x = cart_x  # Drehpunkt Mitte Oben
            pivot_y = cart_y
            # Masse
            bob_x = pivot_x + self.sim.L * scale * math.sin(self.sim.theta)
            bob_y = pivot_y + self.sim.L * scale * math.cos(self.sim.theta)

            score += self.sim.theta - math.pi * dt

            pg.draw.aaline(self.disp, (0, 0, 0), (pivot_x, pivot_y), (bob_x, bob_y), 4)
            pg.draw.aacircle(self.disp, (200, 50, 50), (int(bob_x), int(bob_y)), 10)

            for event in pg.event.get():
                if event.type == pg.QUIT:
                    run = False
                elif event.type == pg.KEYDOWN:
                    if event.key == pg.K_ESCAPE:
                        run = False

            self.clock.tick()
            pg.display.flip()


if __name__ == '__main__':
    display = Display()
