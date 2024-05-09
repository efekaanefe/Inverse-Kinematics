if __name__ == "__main__":
    

    import pygame  
    import numpy as np


    # pygame setup
    WIDTH = 800; HEIGHT = 800;  
    background_colour = (234, 212, 252)
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("inverse-kinematics")
    pygame.display.flip()

    # inverse-kinematics
    N_LINKS = 3
    LINK_LENGTH = 50
    angles = np.zeros(N_LINKS)
    link_lengths = np.ones(N_LINKS) * LINK_LENGTH


    # main loop
    running = True
      
    while running: 

        screen.fill(background_colour)

        for event in pygame.event.get(): 
          
            if event.type == pygame.QUIT: 
                running = False
