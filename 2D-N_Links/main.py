if __name__ == "__main__":
    import pygame  
    import numpy as np


    # pygame setup
    WIDTH = 800; HEIGHT = 800;  
    BACKGROUND_COLOR = (28, 27, 24)
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("inverse-kinematics")

    LINK_COLORS = (219, 217, 211)


    # inverse-kinematics
    N_LINKS = 3
    LINK_LENGTH = 50
    angles = np.zeros(N_LINKS)
    link_lengths = np.ones(N_LINKS) * LINK_LENGTH
    
    fixed_point_position = np.array([WIDTH//2, HEIGHT//2])

    def get_positions(angles, link_lengths):
        pass

    # main loop
    running = True
      
    while running: 

        screen.fill(BACKGROUND_COLOR)

        for event in pygame.event.get(): 
          
            if event.type == pygame.QUIT: 
                running = False
