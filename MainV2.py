import math, time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import matplotlib.transforms as transforms

# ------------- CONFIG -------------
AREA_W = 10.0
AREA_H = 6.0
LANE_SPACING = 1.0

WHEELBASE = 0.8       # m
BODY_LEN = 1.0
BODY_WID = 0.6
WHEEL_LEN = 0.26
WHEEL_WID = 0.10

V_BASE = 0.6          # nominal speed (m/s)
DT = 0.05             # timestep (s)
LOOKAHEAD = 0.8       # lookahead distance (m)
MAX_STEER_DEG = 60.0
MAX_STEER_RATE = math.radians(60.0)   # rad/s
SENSOR_RADIUS = 1.6

GOAL_TOL = 0.12
WAYPOINT_TOL = 0.18
MAX_SEG_LOOK = 2
# ----------------------------------

# ----- generate lawnmower path -----
def generate_lawnmower(x_min, x_max, y_min, y_max, spacing):
    pts = []
    y = y_min
    dir = 1
    while y <= y_max + 1e-9:
        if dir == 1:
            pts.append((x_min, y))
            pts.append((x_max, y))
        else:
            pts.append((x_max, y))
            pts.append((x_min, y))
        y += spacing
        dir *= -1
    out = []
    for p in pts:
        if not out or (abs(out[-1][0]-p[0])>1e-9 or abs(out[-1][1]-p[1])>1e-9):
            out.append(p)
    return np.array(out, dtype=float)

path = generate_lawnmower(0.0, AREA_W, 0.0, AREA_H, LANE_SPACING)

# ----- helper: find lookahead intersection -----
def find_lookahead_point_from(path, pos, lookahead, start_seg, heading, max_seg_look=10):
    px, py = pos
    hx, hy = heading
    n = len(path)
    candidates = []
    end_seg = min(n - 1, start_seg + max_seg_look)
    for i in range(start_seg, end_seg):
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        dx, dy = x2 - x1, y2 - y1
        a = dx * dx + dy * dy
        if a < 1e-12:
            continue
        b = 2 * (dx * (x1 - px) + dy * (y1 - py))
        c = (x1 - px) ** 2 + (y1 - py) ** 2 - lookahead ** 2
        disc = b * b - 4 * a * c
        if disc < 0:
            continue
        sqrt_disc = math.sqrt(disc)
        t_candidates = [(-b - sqrt_disc) / (2 * a), (-b + sqrt_disc) / (2 * a)]
        for t in t_candidates:
            if 0.0 <= t <= 1.0:
                lx, ly = x1 + t * dx, y1 + t * dy
                if (lx - px) * hx + (ly - py) * hy > 1e-9:
                    candidates.append((i, t, np.array([lx, ly])))
    if candidates:
        i_best, t_best, p_best = min(candidates, key=lambda k: (k[0], k[1]))
        return p_best, i_best, t_best
    if start_seg < n - 1:
        x1, y1 = path[start_seg]
        x2, y2 = path[start_seg + 1]
        dx, dy = x2 - x1, y2 - y1
        seg_len2 = dx * dx + dy * dy
        if seg_len2 < 1e-12:
            return find_lookahead_point_from(path, pos, lookahead, start_seg + 1, heading, max_seg_look)
        seg_len = math.sqrt(seg_len2)
        t0 = ((px - x1) * dx + (py - y1) * dy) / seg_len2
        t_tar = min(1.0, max(0.0, t0) + lookahead / seg_len)
        vx, vy = x1 + t_tar * dx, y1 + t_tar * dy
        if (vx - px) * hx + (vy - py) * hy <= 0.0:
            eps = 1e-3
            t_tar = min(1.0, t_tar + eps)
            vx, vy = x1 + t_tar * dx, y1 + t_tar * dy
        return np.array([vx, vy]), start_seg, t_tar
    return np.array(path[-1]), n - 2, 1.0
    
# ----- pure pursuit steering (Ackermann) -----
def pure_pursuit_steer_from(path, rear_pos, yaw, lookahead, wheelbase, max_steer_rad, start_seg):
    heading = (math.cos(yaw), math.sin(yaw))
    look_pt, seg_idx, seg_t = find_lookahead_point_from(path, rear_pos, lookahead, start_seg, heading)

    dx = look_pt[0] - rear_pos[0]
    dy = look_pt[1] - rear_pos[1]
    x_f =  math.cos(yaw)*dx + math.sin(yaw)*dy
    y_l = -math.sin(yaw)*dx + math.cos(yaw)*dy

    if x_f <= 0.0:
        look_pt, seg_idx, seg_t = find_lookahead_point_from(
            path, rear_pos, lookahead, min(start_seg+1, len(path)-2), heading)
        dx = look_pt[0] - rear_pos[0]
        dy = look_pt[1] - rear_pos[1]
        x_f =  math.cos(yaw)*dx + math.sin(yaw)*dy
        y_l = -math.sin(yaw)*dx + math.cos(yaw)*dy

    if abs(x_f) < 1e-9 and abs(y_l) < 1e-9:
        return 0.0, look_pt, seg_idx

    curvature = 2.0 * y_l / (lookahead**2)
    steer = math.atan(curvature * wheelbase)
    steer = max(-max_steer_rad, min(max_steer_rad, steer))
    return steer, look_pt, seg_idx

# ----- initial vehicle state -----
rear_x, rear_y, yaw = 0.5, 0.5, 0.0
steer = 0.0
trajectory = [(rear_x, rear_y)]
path_done = False
current_seg = 0

# ----- plotting setup -----
fig, ax = plt.subplots(figsize=(12,6))
ax.set_aspect('equal')
ax.set_xlim(-1, AREA_W + 1)
ax.set_ylim(-1, AREA_H + 1)
ax.set_title("Ackermann + CPP + Pure Pursuit (Realtime)")

for gx in np.arange(0, AREA_W+1, 1.0):
    ax.axvline(gx, color='lightgray', linewidth=0.6)
for gy in np.arange(0, AREA_H+1, 1.0):
    ax.axhline(gy, color='lightgray', linewidth=0.6)

ax.add_patch(patches.Rectangle((0,0), AREA_W, AREA_H, fill=False, linestyle='--', color='gray'))
ax.plot(path[:,0], path[:,1], 'g--', linewidth=1.5, label='CPP path')
ax.scatter(path[:,0], path[:,1], c='green', s=12)

traj_line, = ax.plot([], [], 'b-', linewidth=2, label='trajectory')
look_circle = patches.Circle((rear_x, rear_y), LOOKAHEAD, fill=False, linestyle=':', edgecolor='blue', alpha=0.6)
ax.add_patch(look_circle)
look_dot, = ax.plot([], [], 'ro', ms=6)
sensor_circle = patches.Circle((rear_x, rear_y), SENSOR_RADIUS, color='gray', alpha=0.08)
ax.add_patch(sensor_circle)
heading_line, = ax.plot([], [], 'k-', lw=2)

half_w = BODY_WID/2.0
rear_ext = 0.1
body_local = np.array([
    [-rear_ext,  half_w],
    [ BODY_LEN-rear_ext,  half_w],
    [ BODY_LEN-rear_ext, -half_w],
    [-rear_ext, -half_w]
])
body_patch = patches.Polygon(body_local, closed=True, facecolor='blue', alpha=0.4)
ax.add_patch(body_patch)

wheel_local = np.array([
    [-WHEEL_LEN/2, -WHEEL_WID/2],
    [ WHEEL_LEN/2, -WHEEL_WID/2],
    [ WHEEL_LEN/2,  WHEEL_WID/2],
    [-WHEEL_LEN/2,  WHEEL_WID/2]
])
rear_left = np.array([0.0,  half_w])
rear_right= np.array([0.0, -half_w])
front_left= np.array([WHEELBASE,  half_w])
front_right=np.array([WHEELBASE, -half_w])
wheel_centers = [rear_left, rear_right, front_left, front_right]
wheel_patches = []
for _ in wheel_centers:
    wp = patches.Polygon(wheel_local.copy(), closed=True, facecolor='black')
    ax.add_patch(wp)
    wheel_patches.append(wp)

ax.legend(loc='upper right')

# ----- terminal axes -----
term_ax = fig.add_axes([0.85, 0.05, 0.14, 0.9])
term_ax.axis('off')
terminal_text = term_ax.text(0, 1.0, "", va='top', ha='left', fontsize=10, family='monospace')

# ----- animation update -----
max_steer = math.radians(MAX_STEER_DEG)
prev_steer = 0.0

def update(frame):
    global rear_x, rear_y, yaw, steer, prev_steer, path_done, current_seg

    if path_done:
        return (traj_line, look_circle, look_dot, sensor_circle, heading_line, body_patch, *wheel_patches, terminal_text)

    desired_steer, look_pt, seg_found = pure_pursuit_steer_from(
        path, (rear_x, rear_y), yaw, LOOKAHEAD, WHEELBASE, max_steer, current_seg)

    current_seg = max(current_seg, seg_found)

    max_dsteer = MAX_STEER_RATE * DT
    dsteer = desired_steer - prev_steer
    if dsteer > max_dsteer:
        desired_steer = prev_steer + max_dsteer
    elif dsteer < -max_dsteer:
        desired_steer = prev_steer - max_dsteer
    prev_steer = desired_steer

    steer_abs = abs(desired_steer)
    v = V_BASE * max(0.35, 1.0 - (steer_abs / max_steer))
    omega = (v / WHEELBASE) * math.tan(steer)

    rear_x += v * math.cos(yaw) * DT
    rear_y += v * math.sin(yaw) * DT
    yaw += omega * DT
    steer = desired_steer
    trajectory.append((rear_x, rear_y))

    while current_seg < len(path)-1:
        nx, ny = path[current_seg+1]
        if math.hypot(nx - rear_x, ny - rear_y) < WAYPOINT_TOL:
            current_seg += 1
        else:
            break

    tr = np.array(trajectory)
    traj_line.set_data(tr[:,0], tr[:,1])
    look_circle.center = (rear_x, rear_y)
    look_dot.set_data([look_pt[0]], [look_pt[1]])
    sensor_circle.center = (rear_x, rear_y)
    head_len = 0.6
    hx = rear_x + head_len * math.cos(yaw)
    hy = rear_y + head_len * math.sin(yaw)
    heading_line.set_data([rear_x, hx], [rear_y, hy])
    t_body = transforms.Affine2D().rotate(yaw).translate(rear_x, rear_y)
    body_patch.set_transform(t_body + ax.transData)

    for idx, wpatch in enumerate(wheel_patches):
        center = wheel_centers[idx]
        cx = rear_x + math.cos(yaw)*center[0] - math.sin(yaw)*center[1]
        cy = rear_y + math.sin(yaw)*center[0] + math.cos(yaw)*center[1]
        if idx < 2:
            rot = yaw
        else:
            rot = yaw + steer
        R = np.array([[math.cos(rot), -math.sin(rot)], [math.sin(rot), math.cos(rot)]])
        corners = (wheel_local @ R.T) + np.array([cx, cy])
        wpatch.set_xy(corners)

    final = path[-1]
    if math.hypot(final[0] - rear_x, final[1] - rear_y) < GOAL_TOL and current_seg >= len(path)-2:
        path_done = True
        print("Reached final goal — stopping.")

    # ----- terminal update -----  
    dx = v * math.cos(yaw) * DT
    dy = v * math.sin(yaw) * DT
    dtheta = omega * DT
    R = (WHEELBASE / math.tan(steer)) if abs(steer) > 1e-6 else float('inf')

    terminal_str = (f"Sim:\n"
                    f"rear=({rear_x:.2f},{rear_y:.2f})\n"
                    f"yaw={math.degrees(yaw):.1f} deg\n"
                    f"steer={math.degrees(steer):.1f} deg\n"
                    f"seg={current_seg}\n"
                    f"v={v:.2f} m/s\n"
                    f"omega={math.degrees(omega):.1f} deg/s\n"
                    f"dx={dx:.3f}, dy={dy:.3f}\n"
                    f"dθ={math.degrees(dtheta):.3f} deg\n"
                    f"R={'∞' if R==float('inf') else f'{R:.2f} m'}")
    terminal_text.set_text(terminal_str)
    return (traj_line, look_circle, look_dot, sensor_circle, heading_line, body_patch, *wheel_patches, terminal_text)

ani = animation.FuncAnimation(fig, update, frames=10000, interval=int(DT*1000), blit=False, repeat=False)
plt.show()

