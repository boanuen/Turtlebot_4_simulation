# Thay cho teleop khi chụp/đo: lên lộ trình phủ các hành lang rồi bám theo, gửi /cmd_vel như teleop.
#   follow_route.py plan  <spacing_m> <route.json>   : lên lộ trình từ bản đồ chuẩn (maze.sdf)
#   follow_route.py run   <route.json> [v] [wmax]     : bám lộ trình theo /odom (pure pursuit)
# Robot đi chậm, quay mượt (không quay tại chỗ) để scan matching của SLAM theo kịp.
import heapq, json, math, os, sys, time
import numpy as np, cv2
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import truth

RES, OX, OY, N = 0.1, -11.0, -11.0, 220


def cell(x, y):
    return int(N - 1 - (y - OY) / RES), int((x - OX) / RES)          # (row, col)


def xy(r, c):
    return OX + (c + 0.5) * RES, OY + (N - 1 - r + 0.5) * RES


def safe_mask(clear=0.60):
    occ = truth.truth_mask((N, N), OX, OY, RES) > 0
    for r in range(N):
        for c in range(N):
            x, y = xy(r, c)
            if abs(x) > 9.9 or abs(y) > 9.9: occ[r, c] = True        # tường biên
    dist = cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 5) * RES
    return dist > clear


def dijkstra(safe, src):
    dist = np.full((N, N), np.inf); prev = {}
    dist[src] = 0; pq = [(0.0, src)]
    while pq:
        d, (r, c) = heapq.heappop(pq)
        if d > dist[r, c]: continue
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0: continue
                rr, cc = r + dr, c + dc
                if 0 <= rr < N and 0 <= cc < N and safe[rr, cc]:
                    nd = d + math.hypot(dr, dc)
                    if nd < dist[rr, cc]:
                        dist[rr, cc] = nd; prev[(rr, cc)] = (r, c); heapq.heappush(pq, (nd, (rr, cc)))
    return dist, prev


def path_to(prev, dst):
    p = [dst]
    while p[-1] in prev: p.append(prev[p[-1]])
    return p[::-1]


def los(safe, a, b):
    n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 1.5) + 1
    for t in np.linspace(0, 1, n):
        r = int(round(a[0] + (b[0] - a[0]) * t)); c = int(round(a[1] + (b[1] - a[1]) * t))
        if not safe[r, c]: return False
    return True


def simplify(safe, cells):
    out = [cells[0]]; i = 0
    while i < len(cells) - 1:
        j = len(cells) - 1
        while j > i + 1 and not los(safe, cells[i], cells[j]): j -= 1
        out.append(cells[j]); i = j
    return out


def plan(spacing, out):
    safe = safe_mask()
    start = cell(0.0, 0.0)
    d0, _ = dijkstra(safe, start)
    reach = np.isfinite(d0)
    wps = []
    for r in range(0, N, int(spacing / RES)):
        for c in range(0, N, int(spacing / RES)):
            if reach[r, c] and d0[r, c] > 1.5: wps.append((r, c))
    cur, poly, left = start, [start], wps[:]
    while left:                                                        # chặng nào gần nhất đi trước
        dist, prev = dijkstra(safe, cur)
        nxt = min(left, key=lambda p: dist[p])
        poly += simplify(safe, path_to(prev, nxt))[1:]; cur = nxt; left.remove(nxt)
    dist, prev = dijkstra(safe, cur)
    poly += simplify(safe, path_to(prev, start))[1:]                  # về điểm xuất phát để đóng vòng
    pts = [xy(r, c) for r, c in poly]
    L = sum(math.hypot(pts[i+1][0]-pts[i][0], pts[i+1][1]-pts[i][1]) for i in range(len(pts)-1))
    json.dump({'points': pts, 'length_m': L, 'waypoints': len(wps), 'reachable_m2': float(reach.sum() * RES * RES)}, open(out, 'w'))
    print('waypoints %d, dinh %d, dai %.1f m, vung di duoc %.0f m2' % (len(wps), len(pts), L, reach.sum() * RES * RES))


def run(route_file, v=0.22, wmax=0.30):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import Twist
    pts = json.load(open(route_file))['points']

    class F(Node):
        def __init__(s):
            super().__init__('follow_route')
            s.pub = s.create_publisher(Twist, '/cmd_vel', 10)
            s.create_subscription(Odometry, '/sim_ground_truth_pose', s.odom, qos_profile_sensor_data)   # vị trí thật trong Gazebo (thay người lái nhìn robot)
            s.create_subscription(LaserScan, '/scan', s.scan, qos_profile_sensor_data)
            s.i = 0; s.front = 9.0; s.pose = None; s.t0 = time.time(); s.log = []
            s.create_timer(0.05, s.step)

        def scan(s, m):
            f = [r for k, r in enumerate(m.ranges)
                 if abs(math.atan2(math.sin(m.angle_min + k*m.angle_increment + math.pi/2),
                                   math.cos(m.angle_min + k*m.angle_increment + math.pi/2))) < 0.35
                 and m.range_min < r < 20]
            s.front = min(f) if f else 9.0

        def odom(s, m):
            q = m.pose.pose.orientation
            s.pose = (m.pose.pose.position.x, m.pose.pose.position.y, 2 * math.atan2(q.z, q.w))

        def step(s):
            if s.pose is None: return
            x, y, th = s.pose
            s.log.append((time.time() - s.t0, x, y))
            while s.i < len(pts) - 1 and math.hypot(pts[s.i+1][0]-x, pts[s.i+1][1]-y) < 0.25: s.i += 1
            tx, ty = pts[min(s.i + 1, len(pts) - 1)]
            # điểm nhìn trước 0.4 m dọc đường (nhỏ để không cắt góc tường)
            acc, k = 0.0, s.i
            while k < len(pts) - 1 and acc < 0.4:
                acc += math.hypot(pts[k+1][0]-pts[k][0], pts[k+1][1]-pts[k][1]); k += 1
            tx, ty = pts[k]
            t = Twist()
            if s.i >= len(pts) - 1 and math.hypot(pts[-1][0]-x, pts[-1][1]-y) < 0.3:
                self_done(s); return
            a = math.atan2(ty - y, tx - x) - th
            a = math.atan2(math.sin(a), math.cos(a))
            if abs(a) > 0.5:                                           # góc lệch lớn: gần như đứng yên và xoay chậm
                t.linear.x = 0.03; t.angular.z = max(-wmax, min(wmax, 1.5 * a))
            else:
                t.linear.x = v if s.front > 0.30 else 0.0       # dừng khẩn cấp; lộ trình đã cách tường >= 0.5 m
                t.angular.z = max(-wmax, min(wmax, 1.5 * a))
            s.pub.publish(t)

    def self_done(node):
        node.pub.publish(Twist())
        json.dump(node.log[::10], open(route_file + '.trace.json', 'w'))
        print('XONG lo trinh sau %.0f s thuc' % (time.time() - node.t0), flush=True)
        os._exit(0)

    rclpy.init(); rclpy.spin(F())


if __name__ == '__main__':
    if sys.argv[1] == 'plan': plan(float(sys.argv[2]), sys.argv[3])
    else: run(sys.argv[2], *[float(a) for a in sys.argv[3:]])
