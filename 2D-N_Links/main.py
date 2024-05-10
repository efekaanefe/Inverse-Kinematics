if __name__ == "__main__":
    import pygame  
    import numpy as np


    # pygame setup
    WIDTH = 800; HEIGHT = 800;  
    BACKGROUND_COLOR = (28, 27, 24)
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("inverse-kinematics")

    LINK_COLORS = (219, 217, 211)
    JOINT_COLOR = LINK_COLORS
    FIXED_JOINT_COLOR = (230, 10, 10)


    def draw_links(positions, screen):
        for i in range(1, N_LINKS+1):
            x0, y0 = positions[i-1]
            x1, y1 = positions[i]
            pygame.draw.line(screen, LINK_COLORS, (x0, y0), (x1, y1), width=3)
            
            # joints
            pygame.draw.circle(screen, JOINT_COLOR, (x1, y1), 5)
            
            if i == 1:
                pygame.draw.circle(screen, FIXED_JOINT_COLOR, (x0, y0), 5)
            

    # inverse-kinematics
    N_LINKS = 3
    LINK_LENGTH = 100
    angles = np.zeros(N_LINKS)
    link_lengths = np.ones(N_LINKS) * LINK_LENGTH
    
    FIXED_POINT_POSITION = np.array([WIDTH//2, HEIGHT//2])

    def get_positions(angles, link_lengths):
        positions = []
        positions.append(FIXED_POINT_POSITION)
        for i in range(1, N_LINKS+1):
            prev_pos = positions[i-1]
            angle = angles[i-1]
            link_length = link_lengths[i-1]
            direction = np.array([np.cos(np.deg2rad(angle)), np.sin(np.deg2rad(angle))])

            positions.append(prev_pos + link_length * direction)
        return positions


    # main loop
    running = True
    while running: 

        screen.fill(BACKGROUND_COLOR)

        for event in pygame.event.get(): 
          
            if event.type == pygame.QUIT: 
                running = False
        
        positions = get_positions(angles, link_lengths)
        draw_links(positions, screen)
        pygame.display.update()
