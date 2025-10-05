import numpy as np

def normalize(v):
    """Normalizes a vector to unit length."""
    norm = np.linalg.norm(v)
    return v if norm == 0 else v / norm

class Aircraft:
    """
    Encapsulates the state, physics, and control logic for a simulated aircraft
    with a Region of Avoidance (ROA) system.
    """
    def __init__(self, initial_pos, initial_velocity):
        """
        Initializes the aircraft with its physical parameters and starting state.
        
        Args:
            initial_pos (list or np.ndarray): The starting [x, y, z] position.
            initial_velocity (list or np.ndarray): The starting [vx, vy, vz] velocity.
        """
        # --------------------------
        # Aircraft State
        # --------------------------
        self.pos = np.array(initial_pos, dtype=float)
        self.velocity = np.array(initial_velocity, dtype=float)

        # --------------------------
        # Physical Properties
        # --------------------------
        self.mass = 0.60  # kg
        self.thrust = 8.0  # Newtons
        self.k_lift = 0.025  # Lift constant
        self.k_drag = 0.005  # Drag constant
        self.g_vector = np.array([0, 0, -9.8])

        # --------------------------
        # Control System Parameters
        # --------------------------
        # Speed Control
        self.V_setpoint = 18.0  # m/s, desired speed ceiling
        self.Kp_thrust = 4.0    # Proportional gain for thrust control

        # Roll Control
        self.max_roll_target_deg = 90.0
        self.Kp_roll_correction = 0.60
        self.Kd_roll = 0.60

        # Pitch Control
        self.Kp_pitch = 1.85
        self.Kd_pitch = 1.50
        self.pitch_rate_scaling_max_rad_s = np.radians(120)

        # --------------------------
        # Region of Avoidance (ROA) Parameters
        # --------------------------
        self.roa_major_axis = 40.0
        self.roa_minor_axis = self.roa_major_axis / 2.0
        self.focal_dist = np.sqrt(self.roa_major_axis**2 - self.roa_minor_axis**2)
        self.Kp_roa_pitch = 150.0
        self.Kp_roa_roll = 150.0
        
        # --------------------------
        # Internal State Variables
        # --------------------------
        # Reference Frame
        v_norm_init = normalize(self.velocity)
        if np.abs(np.dot(v_norm_init, np.array([0, 0, 1]))) > 0.999:
            self.local_right = normalize(np.cross(np.array([0, 1, 0]), v_norm_init))
        else:
            self.local_right = normalize(np.cross(v_norm_init, np.array([0, 0, 1])))
        self.local_up = normalize(np.cross(self.local_right, v_norm_init))

        # Control State
        self.prev_signed_roll_rad = 0.0
        self.phi_cmd_rad = 0.0
        self.prev_pitch_rad = 0.0
        self.avoidance_vec = np.array([0.0, 0.0, 0.0])
        self.roa_active = False

    def update(self, dt, threats):
        """
        Updates the aircraft's state for a single time step.

        Args:
            dt (float): The time step duration in seconds.
            threats (list of tuples): A list of potential threats. Each threat is a
                                      tuple of (surface_point, surface_normal),
                                      e.g., [(np.array([x,y,z]), np.array([nx,ny,nz]))].
        """
        speed = np.linalg.norm(self.velocity)
        if speed == 0:
            return

        v_norm = normalize(self.velocity)

        # --- Continuous Reference Frame Calculation (Gram-Schmidt) ---
        self.local_up = self.local_up - np.dot(self.local_up, v_norm) * v_norm
        self.local_up = normalize(self.local_up)
        self.local_right = np.cross(v_norm, self.local_up)
        
        level_up_vec = self.local_up
        level_right_vec = self.local_right

        # --- Dynamic Lift/Drag based on Pitch Rate (AoA Surrogate) ---
        current_pitch_rad = np.arcsin(np.clip(v_norm[2], -1.0, 1.0))
        pitch_rate = (current_pitch_rad - self.prev_pitch_rad) / dt
        normalized_pitch_rate = np.clip(np.abs(pitch_rate) / self.pitch_rate_scaling_max_rad_s, 0.0, 1.0)
        aoa_surrogate_factor = 0.5 * normalized_pitch_rate
        k_lift_dynamic = self.k_lift * (1.0 + aoa_surrogate_factor)
        k_drag_dynamic = self.k_drag * (1.0 + aoa_surrogate_factor)

        # --- Speed & Drag Calculation ---
        thrust_cmd = self.thrust
        if speed > self.V_setpoint:
            thrust_cmd = self.thrust - self.Kp_thrust * (speed - self.V_setpoint)
        drag_force = k_drag_dynamic * speed**2
        drag_accel = drag_force / self.mass
        drag_vec = -v_norm * drag_accel

        # --- Ellipsoidal Region of Avoidance (ROA) ---
        max_penetration_factor = 0.0
        self.avoidance_vec = np.array([0.0, 0.0, 0.0])
        ellipsoid_center = self.pos + self.focal_dist * v_norm

        for surface_point, normal_vec in threats:
            vec_to_surface = surface_point - ellipsoid_center
            x_local = np.dot(vec_to_surface, v_norm)
            y_local = np.dot(vec_to_surface, level_right_vec)
            z_local = np.dot(vec_to_surface, level_up_vec)
            
            ellipsoid_eq_val = (x_local / self.roa_major_axis)**2 + \
                               (y_local / self.roa_minor_axis)**2 + \
                               (z_local / self.roa_minor_axis)**2

            if ellipsoid_eq_val < 1.0:
                penetration_factor = 1.0 - np.sqrt(ellipsoid_eq_val)
                if penetration_factor > max_penetration_factor:
                    max_penetration_factor = penetration_factor
                    self.avoidance_vec = normal_vec
        
        self.roa_active = (max_penetration_factor > 0)

        # --- Roll Controller ---
        target_roll_angle_rad = 0.0
        if self.roa_active:
            roll_component = np.dot(self.avoidance_vec, level_right_vec)
            roa_target_roll_rad = self.Kp_roa_roll * max_penetration_factor * roll_component
            target_roll_angle_rad = np.clip(roa_target_roll_rad,
                                            np.radians(-self.max_roll_target_deg),
                                            np.radians(self.max_roll_target_deg))

        error_p = target_roll_angle_rad - self.phi_cmd_rad
        roll_rate = (self.phi_cmd_rad - self.prev_signed_roll_rad) / dt
        self.prev_signed_roll_rad = self.phi_cmd_rad
        self.phi_cmd_rad += (self.Kp_roll_correction * error_p - self.Kd_roll * roll_rate) * dt
        self.phi_cmd_rad = np.clip(self.phi_cmd_rad, 
                                   np.radians(-self.max_roll_target_deg), 
                                   np.radians(self.max_roll_target_deg))

        binormal = normalize(np.cos(self.phi_cmd_rad) * level_up_vec + np.sin(self.phi_cmd_rad) * level_right_vec)

        # --- Unified Pitch Controller ---
        max_physical_lift_accel = (k_lift_dynamic * speed**2) / self.mass
        gravity_compensation_scalar = np.linalg.norm(self.g_vector)

        if self.roa_active:
            pitch_component = np.dot(self.avoidance_vec, binormal)
            roa_pitch_correction = self.Kp_roa_pitch * max_penetration_factor * pitch_component
            commanded_lift_accel_scalar = roa_pitch_correction + gravity_compensation_scalar
        else:
            pitch_error = 0.0 - current_pitch_rad
            leveling_pitch_command = self.Kp_pitch * pitch_error - self.Kd_pitch * pitch_rate + gravity_compensation_scalar
            commanded_lift_accel_scalar = leveling_pitch_command

        clamped_lift_accel_scalar = np.clip(commanded_lift_accel_scalar,
                                            -max_physical_lift_accel,
                                            max_physical_lift_accel)
        lift_vec = binormal * clamped_lift_accel_scalar
        self.prev_pitch_rad = current_pitch_rad

        # --- Orthogonality Correction & Final Physics Update ---
        lift_vec = lift_vec - np.dot(lift_vec, v_norm) * v_norm
        thrust_cmd = max(0.0, thrust_cmd)
        thrust_accel = (thrust_cmd / self.mass) * v_norm

        net_accel = thrust_accel + drag_vec + lift_vec + self.g_vector
        self.velocity += net_accel * dt
        self.pos += self.velocity * dt