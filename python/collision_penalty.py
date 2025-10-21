import numpy as np
import matplotlib.pyplot as plt

d_safe = 1.5
d_collide = 0.375
d = np.linspace(0, 2.5, 500)

# your thresholded version
penalty_unsafe = np.where((d <= d_safe) & (d > d_collide), np.exp(-0.5 * (d**2)/(d_safe**2)), 0)
penalty_collide = np.where(d <= d_collide, np.exp(-0.5 * (d**2)/(d_safe**2)), 0)

# smooth transition version
alpha = np.clip((d - d_collide) / (d_safe - d_collide), 0, 1)
smooth_unsafe = alpha * np.exp(-0.5*(d**2)/(d_safe**2))
smooth_collide = (1-alpha) * np.exp(-0.5*(d_collide**2)/(d_safe**2))

plt.plot(d, penalty_unsafe, 'b--', label="unsafe (thresholded)")
plt.plot(d, penalty_collide, 'r--', label="collided (thresholded)")
plt.plot(d, smooth_unsafe, 'b', label="unsafe (smooth)")
plt.plot(d, smooth_collide, 'r', label="collided (smooth)")
plt.axvline(d_safe, color='gray', linestyle=':')
plt.axvline(d_collide, color='gray', linestyle=':')
plt.xlabel('distance d')
plt.ylabel('penalty')
plt.legend()
plt.title('Comparison of Thresholded vs Smooth Penalty Functions')
plt.show()

# an improvement 

# Parameters
d_safe = 1.0       # "unsafe" distance threshold
d_collide = 0.25    # collision  distance threshold

# Distance values
d = np.linspace(0, 1.2 * d_safe, 500)

# α = normalized distance within unsafe zone
alpha = np.clip((d - d_collide) / (d_safe - d_collide), 0, 1)

# --- Original version (constant collision term)
collided_penalty_orig = (1 - alpha) * np.exp(-0.5 * (d_collide**2) / (d_safe**2))
unsafe_penalty_orig   = alpha * np.exp(-0.5 * (d**2) / (d_safe**2))

# --- Improved version (distance-sensitive collision term)
collided_penalty_new = (1 - alpha) * np.exp(-0.5 * (d**2) / (d_collide**2))
unsafe_penalty_new   = alpha * np.exp(-0.5 * (d**2) / (d_safe**2))

# Total combined penalty
total_orig = collided_penalty_orig + unsafe_penalty_orig
total_new  = collided_penalty_new + unsafe_penalty_new

# --- Plot ---
plt.figure(figsize=(8,5))
plt.plot(d, collided_penalty_orig, 'r--', label='Collision (orig)')
plt.plot(d, unsafe_penalty_orig, 'orange', label='Unsafe (orig)')
plt.plot(d, total_orig, 'r', label='Total (orig)', linewidth=2)

plt.plot(d, collided_penalty_new, 'b--', label='Collision (new)')
plt.plot(d, unsafe_penalty_new, 'cyan', label='Unsafe (new)')
plt.plot(d, total_new, 'b', label='Total (new)', linewidth=2)

plt.axvline(d_collide, color='k', linestyle=':', label='d_collide')
plt.axvline(d_safe, color='gray', linestyle='--', label='d_safe')

plt.xlabel("Inter-boid distance (d)")
plt.ylabel("Penalty contribution")
plt.title("Comparison of Collision Penalty Formulations")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

