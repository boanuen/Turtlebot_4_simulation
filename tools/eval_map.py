# đánh giá bản đồ SLAM so với bản đồ chuẩn (maze.sdf): sai số vị trí tường và độ phủ. In ra 1 dòng JSON.
import json, os, sys
import numpy as np, cv2, yaml
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import truth

y = yaml.safe_load(open(sys.argv[1])); res = y['resolution']; mox, moy = y['origin'][:2]
m = cv2.imread(os.path.join(os.path.dirname(sys.argv[1]), y['image']), 0)
mH, mW = m.shape
# lưới đấu trường cố định: x, y từ -10.5 đến 10.5 m
OX, OY, N = -10.5, -10.5, int(21.0 / res)
T = truth.truth_mask((N, N), OX, OY, res)
rr, cc = np.mgrid[0:N, 0:N]
X = OX + (cc + 0.5) * res; Y = OY + (N - 1 - rr + 0.5) * res
# giá trị bản đồ SLAM tại từng ô của lưới đấu trường (ngoài khung bản đồ = chưa biết 205)
mc = np.floor((X - mox) / res).astype(int); mr = (mH - 1 - np.floor((Y - moy) / res)).astype(int)
inside = (mc >= 0) & (mc < mW) & (mr >= 0) & (mr < mH)
M = np.full((N, N), 205, np.uint8); M[inside] = m[mr[inside], mc[inside]]
occ = (M == 0)
dist_wall = cv2.distanceTransform((T == 0).astype(np.uint8), cv2.DIST_L2, 5) * res
d = dist_wall[occ]
arena = (np.abs(X) < 9.95) & (np.abs(Y) < 9.95)
free_truth = (T == 0) & arena
nav_free = free_truth & (dist_wall > 0.30)
known = (M != 205)
r = {'occ_cells': int(occ.sum()), 'mean_dist_m': float(d.mean()), 'median_dist_m': float(np.median(d)),
     'within_0.10': float((d <= 0.10).mean()), 'within_0.20': float((d <= 0.20).mean()),
     'coverage_free': float((known & free_truth).sum() / free_truth.sum()),
     'coverage_navigable': float((known & nav_free).sum() / nav_free.sum()),
     'free_truth_m2': float(free_truth.sum() * res * res), 'navigable_m2': float(nav_free.sum() * res * res)}
print(json.dumps(r))
