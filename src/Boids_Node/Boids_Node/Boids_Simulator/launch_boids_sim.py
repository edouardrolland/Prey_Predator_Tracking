from boids import Boid
from simulation import Simulation

import os
os.environ['SDL_AUDIODRIVER'] = 'directx'

time = 50
visual_range = 0
projected_range = 20
separation_factor = 0
alignment_factor = 0
cohesion_factor = 0
turnfactor = 0


def launch_2D_sim(x_drone, y_drone):
    
    window = (1000, 1000)
    margin =   420
    simulation = Simulation(window, margin, 50)
    simulation.graphic_interface()


    simulation.update_animation(x_drone, y_drone)


if __name__ == "__main__":

    window = (1000, 1000)
    margin =   420
    simulation = Simulation(window, margin, 50)
    simulation.graphic_interface()
    while True:
        simulation.update_animation(500,500)
