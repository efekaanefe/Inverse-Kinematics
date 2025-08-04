
if __name__ == "__main__":
    import pygame  
    import numpy as np
    from scipy.optimize import minimize
    

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
    N_LINKS = 10
    LINK_LENGTH = 100//2
    angles = np.ones(N_LINKS) * 15
    link_lengths = np.ones(N_LINKS) * LINK_LENGTH
    
    FIXED_POINT_POSITION = np.array([WIDTH//2, HEIGHT//2])

    def get_optimum_angles(angles, link_lengths, target_position): 
        
        def get_distance_difference(angles, link_lengths, target_position): # angles -> x
            positions = get_positions(angles, link_lengths)
            return np.sqrt((target_position[0] - positions[-1][0])**2 + \
                    (target_position[1] - positions[-1][1])**2)

        optimized_results = minimize(get_distance_difference, angles, method='nelder-mead',
                            args = (link_lengths, target_position) ,options={'xatol': 1e-8, 'disp': True})

        return optimized_results.x

    def get_positions(angles, link_lengths):
        positions = []
        positions.append(FIXED_POINT_POSITION)
        for i in range(N_LINKS):
            prev_pos = positions[i]
            angle = angles[i]
            link_length = link_lengths[i]
            direction = np.array([np.cos(np.deg2rad(angle)), np.sin(np.deg2rad(angle))])
            positions.append(prev_pos + link_length * direction)
        return positions


    # main loop
    running = True
    target_position = None
    while running: 

        screen.fill(BACKGROUND_COLOR)

        for event in pygame.event.get(): 
          
            if event.type == pygame.QUIT: 
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                target_position = event.pos
            elif pygame.mouse.get_pressed()[0]:
                target_position = pygame.mouse.get_pos()
    
        if target_position:
            angles = get_optimum_angles(angles, link_lengths, target_position)
            target_position = None

        positions = get_positions(angles, link_lengths)
        draw_links(positions, screen)
        pygame.display.update()
