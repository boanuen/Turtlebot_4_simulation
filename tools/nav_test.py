# Phần 3 và 4: đo sai số AMCL so với vị trí thật và chạy 3 goal Nav2 (giống Nav2 Goal trên RViz2: action NavigateToPose).
#   nav_test.py <map.yaml> <out_dir>
# Ghi: p3_amcl.json (sai số AMCL), p4_results.json (kết quả từng goal), và chụp ảnh RViz2/Gazebo qua tools/shot.sh
import json, math, os, subprocess, sys, time
import numpy as np, cv2, yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
import tf2_ros

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import truth

MAP, OUT = sys.argv[1], sys.argv[2]


def pick_goals():
    y = yaml.safe_load(open(MAP)); res = y['resolution']; ox, oy = y['origin'][:2]
    m = cv2.imread(os.path.join(os.path.dirname(MAP), y['image']), 0)
    H = m.shape[0]
    free = (m >= 250).astype(np.uint8)
    r0 = H - 1 - int((0 - oy) / res); c0 = int((0 - ox) / res)
    _, lab = cv2.connectedComponents(free)
    reg = (lab == lab[r0, c0]).astype(np.uint8)
    dist = cv2.distanceTransform(reg, cv2.DIST_L2, 5) * res
    cands = []
    for r in range(0, H, 5):
        for c in range(0, m.shape[1], 5):
            if dist[r, c] > 0.7:
                x = ox + (c + 0.5) * res; yy = oy + (H - 1 - r + 0.5) * res
                d = math.hypot(x, yy)
                if 3.0 < d < 6.5: cands.append((x, yy, d, math.degrees(math.atan2(yy, x))))
    goals = []
    for a in (0, 120, 240):
        goals.append(min(cands, key=lambda t: abs(((t[3] - a + 180) % 360) - 180)))
    return goals


def shot(win, name):
    subprocess.run(['bash', os.path.join(HERE, 'shot.sh'), win, name], timeout=60)


class N(Node):
    def __init__(s):
        super().__init__('nav_test')
        s.tf_buf = tf2_ros.Buffer(); tf2_ros.TransformListener(s.tf_buf, s)
        s.create_subscription(Odometry, '/sim_ground_truth_pose', s.on_truth, qos_profile_sensor_data)
        s.create_subscription(LaserScan, '/scan', s.on_scan, qos_profile_sensor_data)
        s.create_subscription(Clock, '/clock', lambda m: setattr(s, 'sim', m.clock.sec + m.clock.nanosec * 1e-9), qos_profile_sensor_data)
        s.pose_pub = s.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        s.ac = ActionClient(s, NavigateToPose, 'navigate_to_pose')
        s.truth = None; s.sim = 0.0; s.min_scan = 99.0; s.trace = []

    def on_truth(s, m):
        q = m.pose.pose.orientation
        s.truth = (m.pose.pose.position.x, m.pose.pose.position.y, 2 * math.atan2(q.z, q.w))
        s.trace.append(s.truth[:2])

    def on_scan(s, m):
        v = [r for r in m.ranges if m.range_min < r < 20]
        if v: s.min_scan = min(s.min_scan, min(v))

    def spin_for(s, sec):
        t = time.time()
        while time.time() - t < sec: rclpy.spin_once(s, timeout_sec=0.1)

    def amcl(s):
        try:
            t = s.tf_buf.lookup_transform('map', 'base_link', rclpy.time.Time())
            q = t.transform.rotation
            return (t.transform.translation.x, t.transform.translation.y, 2 * math.atan2(q.z, q.w))
        except Exception:
            return None

    def err(s):
        a = s.amcl()
        if a is None or s.truth is None: return None
        dth = math.atan2(math.sin(a[2] - s.truth[2]), math.cos(a[2] - s.truth[2]))
        return {'amcl': a, 'truth': s.truth, 'e_xy': math.hypot(a[0] - s.truth[0], a[1] - s.truth[1]), 'e_th_deg': abs(math.degrees(dth))}

    def set_initial_pose(s):
        m = PoseWithCovarianceStamped(); m.header.frame_id = 'map'
        m.pose.pose.orientation.w = 1.0
        m.pose.covariance[0] = 0.25; m.pose.covariance[7] = 0.25; m.pose.covariance[35] = 0.0685
        for _ in range(3):
            m.header.stamp = s.get_clock().now().to_msg(); s.pose_pub.publish(m); s.spin_for(0.5)


def main():
    rclpy.init(); n = N()
    n.spin_for(5)
    res = {'sim_time_start': n.sim}
    # --- Phần 3: AMCL sau khi đặt pose ban đầu, so với vị trí thật ---
    e0 = n.err()
    n.set_initial_pose(); n.spin_for(15)
    shot('RViz', 'p3_rviz_initialpose.png')
    samples = []
    for _ in range(12):
        n.spin_for(1.0); e = n.err()
        if e: samples.append(e)
    res['amcl_rest'] = {'n': len(samples), 'mean_e_xy': float(np.mean([e['e_xy'] for e in samples])), 'max_e_xy': float(np.max([e['e_xy'] for e in samples])),
                        'mean_e_th_deg': float(np.mean([e['e_th_deg'] for e in samples])), 'last': samples[-1] if samples else None}
    json.dump(res, open(os.path.join(OUT, 'p3_amcl.json'), 'w'), indent=1)
    shot('Gazebo"', 'p3_gazebo.png'); shot('RViz', 'p3_rviz_localized.png')
    # --- Phần 4: 3 goal ---
    goals = pick_goals(); res['goals'] = []
    if not n.ac.wait_for_server(timeout_sec=120): print('KHONG CO action server'); return
    T, dist_wall = None, None
    tm = truth.truth_mask((440, 440), -11.0, -11.0, 0.05)
    dw = cv2.distanceTransform((tm == 0).astype(np.uint8), cv2.DIST_L2, 5) * 0.05
    for i, (gx, gy, gd, ga) in enumerate(goals, 1):
        goal = NavigateToPose.Goal(); goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = gx; goal.pose.pose.position.y = gy; goal.pose.pose.orientation.w = 1.0
        n.trace = []; n.min_scan = 99.0; fb = {'rec': 0, 'dist': None}
        def cb(msg):
            fb['rec'] = msg.feedback.number_of_recoveries; fb['dist'] = msg.feedback.distance_remaining
        t0 = time.time(); s0 = n.sim
        print('GOAL %d (%.2f, %.2f)' % (i, gx, gy), flush=True)
        fut = n.ac.send_goal_async(goal, feedback_callback=cb)
        while not fut.done(): rclpy.spin_once(n, timeout_sec=0.1)
        gh = fut.result()
        if not gh.accepted: res['goals'].append({'goal': [gx, gy], 'status': 'REJECTED'}); continue
        rf = gh.get_result_async(); shot_done = False
        while not rf.done():
            rclpy.spin_once(n, timeout_sec=0.1)
            if not shot_done and time.time() - t0 > 25:
                shot('RViz', 'p4_rviz_goal%d_going.png' % i); shot_done = True
            if time.time() - t0 > 900: gh.cancel_goal_async(); break
        status = rf.result().status if rf.done() else -1
        n.spin_for(2)
        tr = np.array(n.trace) if n.trace else np.zeros((1, 2))
        pathlen = float(np.sum(np.hypot(*np.diff(tr, axis=0).T))) if len(tr) > 1 else 0.0
        ix = np.clip(((tr[:, 0] + 11.0) / 0.05).astype(int), 0, 439); iy = np.clip(440 - 1 - ((tr[:, 1] + 11.0) / 0.05).astype(int), 0, 439)
        min_wall = float(dw[iy, ix].min())
        fin = n.truth
        rr = {'goal': [gx, gy], 'status': {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(status, str(status)),
              'wall_s': time.time() - t0, 'sim_s': n.sim - s0, 'path_m': pathlen, 'recoveries': fb['rec'],
              'min_clearance_m': min_wall, 'min_scan_m': n.min_scan,
              'final_err_to_goal_m': math.hypot(fin[0] - gx, fin[1] - gy) if fin else None,
              'amcl_err': n.err()}
        res['goals'].append(rr); print(json.dumps(rr), flush=True)
        shot('RViz', 'p4_rviz_goal%d_reached.png' % i); shot('Gazebo"', 'p4_gazebo_goal%d.png' % i)
        json.dump(res, open(os.path.join(OUT, 'p4_results.json'), 'w'), indent=1)
    json.dump(res, open(os.path.join(OUT, 'p4_results.json'), 'w'), indent=1)
    print('XONG', flush=True)
    os._exit(0)


main()
