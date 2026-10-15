import math
import pyray as rl

SCREEN_WIDTH = 900
SCREEN_HEIGHT = 650
ORIGIN = (450, 325)


# Algorithms
def forward_kinematics(base, lengths, angles):
  x, y = base
  global_angle = 0.0

  positions = [(x,y)]

  for length, local_angle in zip(lengths, angles):
    global_angle += local_angle
    x += length * math.cos(global_angle)
    y += length * math.sin(global_angle)
    positions.append((x,y))

  return positions

def solve_ccd(base, lengths, angles, target, max_iter=20, tolerance=0.5):
  target_x, target_y = target
  EPSILON = 1e-9

  for iteration in range(max_iter):

      positions = forward_kinematics(base, lengths, angles)
      end_x, end_y = positions[-1]

      error = math.hypot(target_x - end_x, target_y - end_y)
      if error <= tolerance:
        break

      for i in range(len(angles)-1, -1, -1):
        positions = forward_kinematics(base, lengths, angles)
        pivot_x, pivot_y = positions[i]
        end_x, end_y = positions[-1]

        # Pivot -> End-effector
        ax = end_x - pivot_x
        ay = end_y - pivot_y

        # Pivot -> Target
        bx = target_x - pivot_x
        by = target_y - pivot_y

        # a zero-length vector doesn't have a defined direction. hence this :
        if math.hypot(ax, ay) < EPSILON:
           continue
        if math.hypot(bx, by) < EPSILON:
           continue

        dot = ax*bx + ay*by
        cross = ax*by - ay*bx

        correction = math.atan2(cross, dot)
        angles[i]+=correction

  return angles

# Helpers
def to_screen(point):
  x, y = point
  return (ORIGIN[0]+x, ORIGIN[1]-y)

def to_world(point):
   x, y = point
   return(x-ORIGIN[0], ORIGIN[1]-y)

def draw_arm(positions):
  screen_positions = [to_screen(point) for point in positions]

  for start, end in zip(screen_positions, screen_positions[1:]):
    rl.draw_line_ex(start, end, 8, rl.Color(73, 73, 73, 255))

  for position in  screen_positions[:-1]:
    rl.draw_circle(int(position[0]), int(position[1]), 7, rl.Color(235, 94, 40, 255))
  end_effector = screen_positions[-1]
  rl.draw_circle(int(end_effector[0]), int(end_effector[1]), 6, rl.SKYBLUE)

def main():
    tracking = False
    base = (0, 0)
    lengths = [100, 80, 60]
    angles = [0.0]*len(lengths)
    target = (sum(lengths), 0)
    rl.init_window(
        SCREEN_WIDTH,
        SCREEN_HEIGHT,
        "3 Segment Robot Arm"
    )

    rl.set_target_fps(60)

    while not rl.window_should_close():
        if rl.is_mouse_button_pressed(rl.MOUSE_BUTTON_LEFT):
           tracking = not tracking

        if tracking:
           mouse = rl.get_mouse_position()
           target = to_world((mouse.x, mouse.y))

           angles = solve_ccd(base, lengths, angles, target)

        positions = forward_kinematics(base,lengths,angles)

        target_screen = to_screen(target)

        rl.begin_drawing()
        rl.clear_background(rl.Color(37, 36, 34, 255))

        rl.draw_circle_lines(int(target_screen[0]), int(target_screen[1]), 10, rl.RED)
        draw_arm(positions)

        if tracking:
          status = "TRACKING"
          status_color = rl.GREEN
        else:
          status = "PAUSED"
          status_color = rl.GRAY

        rl.draw_text(f"Mouse tracking: {status}",20, 20, 22, status_color)
        rl.draw_text("Left-click to toggle tracking",20, 50, 18, rl.DARKGRAY)

        rl.end_drawing()

    rl.close_window()


if __name__ == "__main__":
    main()



