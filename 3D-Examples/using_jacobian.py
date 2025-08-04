import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
import matplotlib.animation as animation
from matplotlib.widgets import Button, Slider
import time

class FrankaPandaIK:
    def __init__(self):
        # Franka Panda Denavit-Hartenberg (DH) parameters 
        self.dh_params = np.array([
            [0,     0,           0.333,  0],      # Joint 1
            [0,     -np.pi/2,    0,      0],      # Joint 2
            [0,     np.pi/2,     0.316,  0],      # Joint 3
            [0.0825, np.pi/2,    0,      0],      # Joint 4
            [-0.0825, -np.pi/2,  0.384,  0],      # Joint 5
            [0,     np.pi/2,     0,      0],      # Joint 6
            [0.088,  np.pi/2,    0,      0],      # Joint 7"
        ])
        
        # Joint limits (radians)
        self.joint_limits = np.array([
            [-2.8973, 2.8973],   # Joint 1
            [-1.7628, 1.7628],   # Joint 2
            [-2.8973, 2.8973],   # Joint 3
            [-3.0718, -0.0698],  # Joint 4
            [-2.8973, 2.8973],   # Joint 5
            [-0.0175, 3.7525],   # Joint 6
            [-2.8973, 2.8973]    # Joint 7
        ])
        
        self.ee_offset = 0.107
        
    def dh_transform(self, a, alpha, d, theta):
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        
        T = np.array([
            [ct,    -st*ca,  st*sa,   a*ct],
            [st,     ct*ca, -ct*sa,   a*st],
            [0,      sa,     ca,      d],
            [0,      0,      0,       1]
        ])
        return T
    
    def forward_kinematics(self, joint_angles):
        T = np.eye(4)
        transforms = [T.copy()]
        
        for i, (a, alpha, d, offset) in enumerate(self.dh_params):
            theta = joint_angles[i] + offset
            T_i = self.dh_transform(a, alpha, d, theta)
            T = T @ T_i
            transforms.append(T.copy())
        
        # Add end-effector offset
        T_ee = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, self.ee_offset],
            [0, 0, 0, 1]
        ])
        T = T @ T_ee
        transforms.append(T.copy())
        
        return T, transforms
    
    def compute_jacobian(self, joint_angles):
        T_total, transforms = self.forward_kinematics(joint_angles)
        p_ee = T_total[:3, 3]
        J = np.zeros((6, 7))
        
        T_current = np.eye(4)
        
        for i in range(7):
            if i == 0:
                z_i = np.array([0, 0, 1])
                p_i = np.array([0, 0, 0])
            else:
                T_current = T_current @ self.dh_transform(
                    self.dh_params[i-1, 0], 
                    self.dh_params[i-1, 1], 
                    self.dh_params[i-1, 2], 
                    joint_angles[i-1] + self.dh_params[i-1, 3]
                )
                z_i = T_current[:3, 2]
                p_i = T_current[:3, 3]
            
            J[:3, i] = np.cross(z_i, p_ee - p_i)
            J[3:, i] = z_i
            
        return J
    
    def inverse_kinematics(self, target_pos, target_rot, initial_guess=None, 
                          max_iterations=500, tolerance=1e-2):
        """
        Robust inverse kinematics with multiple attempts and adaptive damping
        """
        best_solution = None
        best_error = float('inf')
        
        # Try multiple initial guesses and damping values
        damping_values = [0.005, 0.01, 0.02, 0.05]
        
        for damping_val in damping_values:
            # Try with provided initial guess
            initial_guesses = []
            if initial_guess is not None:
                initial_guesses.append(np.array(initial_guess))
            
            # Add some random perturbations around current pose
            if initial_guess is not None:
                for _ in range(3):
                    noise = np.random.normal(0, 0.1, 7)
                    perturbed = np.array(initial_guess) + noise
                    perturbed = np.clip(perturbed, self.joint_limits[:, 0], self.joint_limits[:, 1])
                    initial_guesses.append(perturbed)
            else:
                # Random initial guesses
                for _ in range(5):
                    q_init = np.random.uniform(self.joint_limits[:, 0], self.joint_limits[:, 1])
                    initial_guesses.append(q_init)
            
            for q_init in initial_guesses:
                q = q_init.copy()
                
                if hasattr(target_rot, 'as_matrix'):
                    target_rot_matrix = target_rot.as_matrix()
                else:
                    target_rot_matrix = np.array(target_rot)
                
                target_pos = np.array(target_pos)
                
                for iteration in range(max_iterations):
                    T_current, _ = self.forward_kinematics(q)
                    current_pos = T_current[:3, 3]
                    current_rot = T_current[:3, :3]
                    
                    pos_error = target_pos - current_pos
                    
                    R_error = target_rot_matrix @ current_rot.T
                    angle = np.arccos(np.clip((np.trace(R_error) - 1) / 2, -1, 1))
                    if angle < 1e-6:
                        rot_error = np.zeros(3)
                    else:
                        axis = np.array([
                            R_error[2, 1] - R_error[1, 2],
                            R_error[0, 2] - R_error[2, 0],
                            R_error[1, 0] - R_error[0, 1]
                        ]) / (2 * np.sin(angle))
                        rot_error = angle * axis
                    
                    error = np.concatenate([pos_error, rot_error])
                    error_norm = np.linalg.norm(error)
                    
                    # Track best solution
                    if error_norm < best_error:
                        best_error = error_norm
                        best_solution = q.copy()
                    
                    if error_norm < tolerance:
                        return q, True, iteration
                    
                    J = self.compute_jacobian(q)
                    JTJ = J.T @ J
                    
                    # Adaptive damping based on error
                    adaptive_damping = damping_val * (1 + error_norm)
                    damped_inverse = np.linalg.inv(JTJ + adaptive_damping**2 * np.eye(7)) @ J.T
                    
                    # Step size adaptation
                    step_size = min(1.0, 1.0 / (1 + error_norm))
                    dq = step_size * damped_inverse @ error
                    
                    q_new = q + dq
                    q_new = np.clip(q_new, self.joint_limits[:, 0], self.joint_limits[:, 1])
                    q = q_new
        
        # Return best solution found even if not converged
        return best_solution if best_solution is not None else q, False, max_iterations



class PandaVisualizer:
    def __init__(self):
        self.ik_solver = FrankaPandaIK()
        self.current_joints = np.zeros(7)
        self.target_pos = np.array([0.5, 0.0, 0.4])
        self.target_rot = R.from_euler('xyz', [0, np.pi, 0])
        
        # Animation parameters
        self.animation_active = False
        self.trajectory_points = []
        self.current_trajectory_idx = 0
        
        self.setup_plot()
        
    def setup_plot(self):
        self.fig = plt.figure(figsize=(16,9))
        
        # Main 3D plot
        #self.ax = self.fig.add_subplot(121, projection='3d')
        self.ax = self.fig.add_axes([0, 0.2, 0.5, 0.85], projection='3d')

        self.ax.set_xlim([-1, 1])
        self.ax.set_ylim([-1, 1])
        self.ax.set_zlim([0, 1.5])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('Franka Panda Robot Arm')
        
        # Joint angle plot
        #self.ax_joints = self.fig.add_subplot(122)
        self.ax_joints = self.fig.add_axes([0.58, 0.3, 0.38, 0.6])

        self.ax_joints.set_xlim([-1, 7])
        self.ax_joints.set_ylim([-np.pi, np.pi])
        self.ax_joints.set_xlabel('Joint Number')
        self.ax_joints.set_ylabel('Joint Angle (rad)')
        self.ax_joints.set_title('Current Joint Angles')
        self.ax_joints.grid(True)
        
        # Initialize robot visualization
        self.robot_lines = []
        self.joint_points = []
        self.target_point = None
        self.ee_point = None
        self.coordinate_frames = []
        
        self.joint_bars = self.ax_joints.bar(range(7), self.current_joints, 
                                           color='steelblue', alpha=0.7)
        
        self.status_text = self.fig.text(
            0.62,     # x (fraction of figure width)
            0.1,     # y (fraction of figure height)
            'Status: Waiting for input...',  
            fontsize=24,
            color='black'
        )
        
        self.setup_controls()
        self.update_robot_visualization()
        
    def setup_controls(self):
        button_width = 0.1
        button_height = 0.04
        button_y = 0.02
        
        # Buttons
        ax_solve = plt.axes([0.05, button_y, button_width, button_height])
        self.btn_solve = Button(ax_solve, 'Solve IK')
        self.btn_solve.on_clicked(self.solve_ik)
        
        ax_random = plt.axes([0.2, button_y, button_width, button_height])
        self.btn_random = Button(ax_random, 'Random Target')
        self.btn_random.on_clicked(self.set_random_target)
        
        ax_stop = plt.axes([0.35, button_y, button_width, button_height])
        self.btn_stop = Button(ax_stop, 'Stop')
        self.btn_stop.on_clicked(self.stop_animation)
        
        # Sliders for target position
        slider_height = 0.03
        ax_x = plt.axes([0.1, 0.15, 0.3, slider_height])
        ax_y = plt.axes([0.1, 0.11, 0.3, slider_height])
        ax_z = plt.axes([0.1, 0.07, 0.3, slider_height])
        
        self.slider_x = Slider(ax_x, 'Target X', -0.8, 0.8, valinit=self.target_pos[0])
        self.slider_y = Slider(ax_y, 'Target Y', -0.8, 0.8, valinit=self.target_pos[1])
        self.slider_z = Slider(ax_z, 'Target Z', 0.1, 1.2, valinit=self.target_pos[2])
        
        self.slider_x.on_changed(self.update_target_position)
        self.slider_y.on_changed(self.update_target_position)
        self.slider_z.on_changed(self.update_target_position)
        
    def update_target_position(self, val):
        self.target_pos = np.array([self.slider_x.val, self.slider_y.val, self.slider_z.val])
        self.update_target_visualization()
        
    def get_link_positions(self, joint_angles):
        _, transforms = self.ik_solver.forward_kinematics(joint_angles)
        positions = []
        for T in transforms:
            positions.append(T[:3, 3])
        return np.array(positions)
    
    def update_robot_visualization(self):
        # Clear previous visualization
        for line in self.robot_lines:
            line.remove()
        for point in self.joint_points:
            point.remove()
        for frame in self.coordinate_frames:
            for line in frame:
                line.remove()
        
        self.robot_lines = []
        self.joint_points = []
        self.coordinate_frames = []
        
        positions = self.get_link_positions(self.current_joints)
        
        # Draw robot links
        for i in range(len(positions) - 1):
            line = self.ax.plot([positions[i, 0], positions[i+1, 0]],
                              [positions[i, 1], positions[i+1, 1]],
                              [positions[i, 2], positions[i+1, 2]], 
                              'b-', linewidth=3, alpha=0.8)[0]
            self.robot_lines.append(line)
        
        # Draw joints
        for i, pos in enumerate(positions[:-1]):  # Exclude end-effector
            point = self.ax.scatter(pos[0], pos[1], pos[2], 
                                  c='red', s=50, alpha=0.8)
            self.joint_points.append(point)
        
        # Draw end-effector
        if self.ee_point:
            self.ee_point.remove()
        ee_pos = positions[-1]
        self.ee_point = self.ax.scatter(ee_pos[0], ee_pos[1], ee_pos[2], 
                                      c='green', s=100, marker='s', alpha=0.8)
        
        # Draw coordinate frames at joints
        _, transforms = self.ik_solver.forward_kinematics(self.current_joints)
        for i, T in enumerate(transforms[::2]):  # Show every other frame to avoid clutter
            frame_lines = self.draw_coordinate_frame(T, scale=0.05)
            self.coordinate_frames.append(frame_lines)
        
        # Update joint angle bars
        for i, (bar, angle) in enumerate(zip(self.joint_bars, self.current_joints)):
            bar.set_height(angle)
            # Color code based on joint limits
            limit_range = self.ik_solver.joint_limits[i, 1] - self.ik_solver.joint_limits[i, 0]
            normalized_angle = (angle - self.ik_solver.joint_limits[i, 0]) / limit_range
            if normalized_angle < 0.1 or normalized_angle > 0.9:
                bar.set_color('red')
            elif normalized_angle < 0.2 or normalized_angle > 0.8:
                bar.set_color('orange')
            else:
                bar.set_color('steelblue')
        
        self.update_target_visualization()
        
    def draw_coordinate_frame(self, transform, scale=0.1):
        origin = transform[:3, 3]
        x_axis = transform[:3, 0] * scale
        y_axis = transform[:3, 1] * scale
        z_axis = transform[:3, 2] * scale
        
        frame_lines = []
        
        # X-axis (red)
        line = self.ax.plot([origin[0], origin[0] + x_axis[0]],
                          [origin[1], origin[1] + x_axis[1]],
                          [origin[2], origin[2] + x_axis[2]], 
                          'r-', linewidth=2, alpha=0.6)[0]
        frame_lines.append(line)
        
        # Y-axis (green)
        line = self.ax.plot([origin[0], origin[0] + y_axis[0]],
                          [origin[1], origin[1] + y_axis[1]],
                          [origin[2], origin[2] + y_axis[2]], 
                          'g-', linewidth=2, alpha=0.6)[0]
        frame_lines.append(line)
        
        # Z-axis (blue)
        line = self.ax.plot([origin[0], origin[0] + z_axis[0]],
                          [origin[1], origin[1] + z_axis[1]],
                          [origin[2], origin[2] + z_axis[2]], 
                          'b-', linewidth=2, alpha=0.6)[0]
        frame_lines.append(line)
        
        return frame_lines
    
    def update_target_visualization(self):
        if self.target_point:
            self.target_point.remove()
        
        self.target_point = self.ax.scatter(self.target_pos[0], self.target_pos[1], 
                                          self.target_pos[2], c='red', s=100, 
                                          marker='*', alpha=0.8)
        
        # Draw target coordinate frame
        target_transform = np.eye(4)
        target_transform[:3, :3] = self.target_rot.as_matrix()
        target_transform[:3, 3] = self.target_pos
        
        if hasattr(self, 'target_frame'):
            for line in self.target_frame:
                line.remove()
        
        self.target_frame = self.draw_coordinate_frame(target_transform, scale=0.08)
        
        plt.draw()
    
    def solve_ik(self, event=None):
        solution, success, iterations = self.ik_solver.inverse_kinematics(
            self.target_pos, self.target_rot, self.current_joints
        )
        
        if success:
            self.animate_to_solution(solution)
            msg = f"IK solved in {iterations} iterations"
            print(msg)
            self.status_text.set_text(f"Status: {msg}")
            self.status_text.set_color("green")
        else:
            msg = "IK failed to converge"
            print(msg)
            self.status_text.set_text(f"Status: {msg}")
            self.status_text.set_color("red")
        
        self.fig.canvas.draw_idle()  # Redraw the updated text

    def animate_to_solution(self, target_joints, steps=50):
        start_joints = self.current_joints.copy()
        joint_diff = target_joints - start_joints
        
        def animate_step(frame):
            if frame < steps:
                alpha = frame / (steps - 1)
                # Use smooth interpolation
                smooth_alpha = 3 * alpha**2 - 2 * alpha**3
                self.current_joints = start_joints + smooth_alpha * joint_diff
                self.update_robot_visualization()
                return frame < steps - 1
            else:
                return False
        
        self.start_custom_animation(animate_step, steps)
    
    def set_random_target(self, event=None):
        self.target_pos = np.array([
            np.random.uniform(-0.6, 0.6),
            np.random.uniform(-0.6, 0.6),
            np.random.uniform(0.2, 1.0)
        ])
        
        self.target_rot = R.random()
        
        # Update sliders
        self.slider_x.set_val(self.target_pos[0])
        self.slider_y.set_val(self.target_pos[1])
        self.slider_z.set_val(self.target_pos[2])
        
        self.update_target_visualization()
    
    def start_trajectory_animation(self):
        def animate_trajectory(frame):
            if not self.animation_active or frame >= len(self.trajectory_points):
                return False
            
            target_pos, target_rot = self.trajectory_points[frame]
            solution, success, _ = self.ik_solver.inverse_kinematics(
                target_pos, target_rot, self.current_joints, max_iterations=20
            )
            
            if success:
                self.current_joints = solution
                self.target_pos = target_pos
                self.target_rot = target_rot
                self.update_robot_visualization()
            
            return True
        
        self.animation_active = True
        self.start_custom_animation(animate_trajectory, len(self.trajectory_points))
    
    def start_custom_animation(self, animate_func, total_frames):
        def animation_wrapper(frame):
            if animate_func(frame):
                return [self.ax]
            else:
                return []
        
        self.ani = animation.FuncAnimation(
            self.fig, animation_wrapper, frames=total_frames,
            interval=50, blit=False, repeat=False
        )
        plt.draw()
    
    def stop_animation(self, event=None):
        self.animation_active = False
        if hasattr(self, 'ani'):
            self.ani.event_source.stop()
    
    def show(self):
        plt.tight_layout()
        plt.show()
        plt.close('all')  # Add this line to ensure proper cleanup



# Main execution
if __name__ == "__main__":
    print("Initializing Franka Panda IK Visualizer...")
    print("Controls:")
    print("- Use sliders to set target position")
    print("- Click 'Solve IK' to move robot to target")
    print("- Click 'Random Target' for random reachable position")
    print("- Click 'Circle Path' or 'Figure-8' for animated trajectories")
    print("- Click 'Stop' to halt animations")
    print("\nStarting visualization...")
    
    visualizer = PandaVisualizer()
    visualizer.show()