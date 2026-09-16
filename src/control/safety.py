import numpy as np


class SafetySupervisor:

  def __init__(
      self,
      q_min=None,
      q_max=None,
      max_joint_vel=1.0,  # rad/s (~57 deg/s)
      buffer_zone_deg=5.0,  # Μοίρες απόστασης από το όριο
      workspace_limits=None,
  ):
    # Joint Limits (rad)
    if q_min is None:
      self.q_min = np.radians([-165, -165, -165, -165, -165, -165, -175])
    else:
      self.q_min = np.asarray(q_min)

    if q_max is None:
      self.q_max = np.radians([165, 165, 165, 165, 165, 165, 175])
    else:
      self.q_max = np.asarray(q_max)

    self.max_joint_vel = np.full(7, max_joint_vel)
    self.buffer = np.radians(buffer_zone_deg)

    # Προαιρετικό καρτεσιανό bounding box [xmin, xmax, ymin, ymax, zmin, zmax] σε μέτρα
    self.workspace_limits = (
        workspace_limits
        if workspace_limits
        else {
            "x": (-0.35, 0.35),
            "y": (-0.35, 0.35),
            "z": (0.02, 0.45),  # 2 cm πάνω από το τραπέζι
        }
    )

  def enforce_joint_limits(self, q_current, q_dot):
    """Directional Clamping:

    Μπλοκάρει μόνο τις ταχύτητες που οδηγούν προς το όριο,
    επιτρέποντας πλήρως την αντίθετη κίνηση.
    """
    q_safe_dot = np.copy(q_dot)

    for i in range(len(q_current)):
      # Έλεγχος άνω ορίου
      if q_current[i] >= (self.q_max[i] - self.buffer):
        if q_safe_dot[i] > 0:
          q_safe_dot[i] = 0.0  # Στοπ προς το όριο

      # Έλεγχος κάτω ορίου
      if q_current[i] <= (self.q_min[i] + self.buffer):
        if q_safe_dot[i] < 0:
          q_safe_dot[i] = 0.0  # Στοπ προς το όριο

    return q_safe_dot

  def scale_velocities(self, q_dot):
    """Uniform scaling:

    Διατηρεί την ακριβή κατεύθυνση της κίνησης του άκρου κλιμακώνοντας
    αναλογικά όλες τις ταχύτητες αν κάποια ξεπεράσει το επιτρεπτό όριο.
    """
    abs_ratios = np.abs(q_dot) / self.max_joint_vel
    max_ratio = np.max(abs_ratios)

    if max_ratio > 1.0:
      return q_dot / max_ratio
    return q_dot

  def check_workspace(self, pos, v_cartesian):
    """Αποτρέπει το άκρο από το να συγκρουστεί με το τραπέζι ή να βγει εκτός χώρου."""
    v_safe = np.copy(v_cartesian)

    # Έλεγχος Z (π.χ. προστασία τραπεζιού)
    if (
        pos[2] <= self.workspace_limits["z"][0] and v_safe[2] < 0
    ):  # Προσπάθεια κίνησης προς τα κάτω
      v_safe[2] = 0.0
    elif (
        pos[2] >= self.workspace_limits["z"][1] and v_safe[2] > 0
    ):  # Προσπάθεια κίνησης υπερβολικά ψηλά
      v_safe[2] = 0.0

    return v_safe

  def filter_command(self, q_current, q_dot, current_pos=None, v_cartesian=None):
    """Συνδυαστικός έλεγχος: Επιστρέφει την τελική ασφαλή ταχύτητα q_dot."""
    # 1. Έλεγχος ορίων αρθρώσεων
    q_dot_filtered = self.enforce_joint_limits(q_current, q_dot)

    # 2. Ενιαία κλιμάκωση ταχύτητας (Uniform velocity scaling)
    q_dot_safe = self.scale_velocities(q_dot_filtered)

    return q_dot_safe