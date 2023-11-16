import pygame_widgets
import pygame
from pygame_widgets.slider import Slider
from pygame_widgets.textbox import TextBox
import numpy as np
from .boids import Boid
from scipy.spatial import KDTree
from .predator import Predator
from px4_msgs.msg import VehicleLocalPosition

time = 50

class Simulation():

    def __init__(self, window, margin, Number_of_agents):

        pygame.init()
        self.window = window
        self.margin = margin
        self.Number_of_agents = Number_of_agents
        self.screen = pygame.display.set_mode(window)
        pygame.display.set_caption("Predator Prey Simulation") 
        self.boids =  [Boid(window,margin) for _ in range(Number_of_agents)]
        #self.clock = pygame.time.Clock()  # create pygame clock object 
        self.predator = Predator(window)

    def graphic_interface(self):

        x_slider = 80
        self.separation_slider = Slider(self.screen, x_slider, 10, int(self.window[1]/8), int(self.window[1]/120), min=0, max=0.1, step=0.001, color=(0, 0, 0), handleColour=(255, 0, 0), handleRadius=5, initial=0.039, handleThickness=0)
        self.alignment_slider = Slider(self.screen, x_slider, 10 + int(self.window[1]/120) + 10, int(self.window[0]/8), int(self.window[1]/120), min=0, max=1, step=0.001, color=(0, 0, 0), handleColour=(255, 0, 0), handleRadius=5, initial=0.5, handleThickness=0)
        self.cohesion_slider = Slider(self.screen, x_slider, 10 + 2*int(self.window[1]/120) + 20, int(self.window[0]/8), int(self.window[1]/120), min=0, max=0.1, step=0.001, color=(0, 0, 0), handleColour=(255, 0, 0), handleRadius=5, initial=0.002 , handleThickness=0)
        self.turning_slider = Slider(self.screen, x_slider, 10 + 3*int(self.window[1]/120) + 30, int(self.window[0]/8), int(self.window[1]/120), min=0, max=100, step=0.01, color=(0, 0, 0), handleColour=(255, 0, 0), handleRadius=5, initial=100, handleThickness=0)
        self.visual_slider = Slider(self.screen, x_slider, 10 + 4*int(self.window[1]/120) + 40, int(self.window[0]/8), int(self.window[1]/120), min=0, max=40, step=1, color=(0, 0, 0), handleColour=(255, 0, 0), handleRadius=5, initial=25, handleThickness=0)

        self.output_separation = TextBox(self.screen, x_slider + 135, 10 - int(self.window[1]/120) + 2, 35, 20, fontSize=15, borderColour=(255, 255, 255), textColour=(0, 0, 0), radius=0, text=str(np.around(self.separation_slider.getValue(), 3)))
        self.output_alignment = TextBox(self.screen, x_slider + 135, 10 + int(self.window[1]/120) + 4, 35, 20, fontSize=15, borderColour=(255, 255, 255), textColour=(0, 0, 0), radius=0, text=str(np.around(self.alignment_slider.getValue(), 3)))
        self.output_cohesion = TextBox(self.screen, x_slider + 135, 10 + 2*int(self.window[1]/120) + 15, 35, 20, fontSize=15, borderColour=(255, 255, 255), textColour=(0, 0, 0), radius=0, text=str(np.around(self.cohesion_slider.getValue(), 3)))
        self.output_turning = TextBox(self.screen, x_slider + 135, 10 + 3*int(self.window[1]/120) + 24, 35, 20, fontSize=15, borderColour=(255, 255, 255), textColour=(0, 0, 0), radius=0, text=str(np.around(self.turning_slider.getValue(), 3)))
        self.output_visual = TextBox(self.screen, x_slider + 135, 10 + 4*int(self.window[1]/120) + 34, 35, 20, fontSize=15, borderColour=(255, 255, 255), textColour=(0, 0, 0), radius=0, text=str(np.around(self.visual_slider.getValue(), 3)))
        
        self.output_separation.disable()  
        self.output_alignment.disable()  
        self.output_cohesion.disable()  
        self.output_turning.disable()  
        self.output_visual.disable() 
    

        self.font = pygame.font.Font(None, 18)  # Create a font object

        self.kdtree = KDTree([[boid.x, boid.y] for boid in self.boids])


    def display_drone(self, x, y):
        pygame.draw.circle(self.screen, 'orange', (x,y), 5)

    def update_animation(self, x_drone, y_drone):

        events = pygame.event.get()

        self.screen.fill((255, 255, 255))
        #pygame.draw.rect(self.screen, 'black', (self.margin, self.margin, self.window[0] - 2*self.margin, self.window[1] - 2*self.margin))
        #pygame.draw.rect(self.screen, 'white', (self.margin + 2, self.margin + 2, self.window[0] - 2*self.margin - 4, self.window[1] - 2*self.margin - 4))
        
        text = self.font.render("Separation", True, (0, 0, 0))
        self.screen.blit(text, (5, 10-2))
        separation_factor = self.separation_slider.getValue()
        
        text = self.font.render("Alignment", True, (0, 0, 0))
        self.screen.blit(text, (5, 10-2 + int(self.window[1]/120) + 10))
        alignment_factor = self.alignment_slider.getValue()
        
        text = self.font.render("Cohesion", True, (0, 0, 0))
        self.screen.blit(text, (5, 10 + 2*int(self.window[1]/120) + 18))
        cohesion_factor = self.cohesion_slider.getValue()
        
        text = self.font.render("Turning", True, (0, 0, 0))
        self.screen.blit(text, (5, 10 + 2*int(self.window[1]/120) + 36))
        turnfactor = self.turning_slider.getValue()
    
        text = self.font.render("Visual", True, (0, 0, 0))
        self.screen.blit(text, (5, 10 + 2*int(self.window[1]/120) + 54))
        visual_range = self.visual_slider.getValue()

        text = self.font.render("Number of preys remaining:                   " + str(self.Number_of_agents), True, (0, 0, 0))
        self.screen.blit(text, (5, 10 + 2*int(self.window[1]/120) + 80))

        self.predator.uptate(self.window, 50, self.kdtree, self.boids)
        self.Number_of_agents = len(self.boids)
        self.kdtree = KDTree([[boid.x, boid.y] for boid in self.boids])            
        
        for boid in self.boids:
            pygame.draw.polygon(self.screen, 'red', boid.draw_triangle())
            boid.update(self.window, turnfactor, separation_factor, cohesion_factor, alignment_factor, self.kdtree, self.boids, visual_range, self.predator, self.predator.predation_detected)
        
        pygame.draw.polygon(self.screen, 'blue', self.predator.draw_triangle())
        pygame.draw.circle(self.screen, 'green', self.predator.centroid, 5)

        self.display_drone(x_drone, y_drone)


        if self.predator.eating == True:
            text_2 = self.font.render("The predator is eating his meal", True, (0, 0, 255))
            self.screen.blit(text_2, (5, 130))


        for events in pygame.event.get():  # loop through all events
            if events.type == pygame.QUIT:
                pygame.quit()
                quit()

        self.output_separation.setText(np.around(self.separation_slider.getValue(),3))
        self.output_alignment.setText(np.around(self.alignment_slider.getValue(), 3))
        self.output_cohesion.setText(np.around(self.cohesion_slider.getValue(), 3))
        self.output_turning.setText(np.around(self.turning_slider.getValue(),3))
        self.output_visual.setText(np.around(self.visual_slider.getValue(),3))

        pygame_widgets.update(events)
        pygame.display.update()
        #self.clock.tick(time)