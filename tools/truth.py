# bản đồ chuẩn từ maze.sdf, so với map.pgm: truth.py <map.yaml> [out.png]
import os, sys, math, yaml, numpy as np, cv2
import xml.etree.ElementTree as ET
SDF = '/opt/ros/humble/share/turtlebot4_ignition_bringup/worlds/maze.sdf'

def pose(e):
    p = e.find('pose')
    return [float(x) for x in p.text.split()] if p is not None and p.text else [0.0] * 6

def mat(v):
    x, y, z, r, pi, ya = v
    c, s = math.cos(ya), math.sin(ya)
    return np.array([[c, -s, x], [s, c, y], [0, 0, 1]])

boxes = []
for model in ET.parse(SDF).getroot().iter('model'):
    mm = mat(pose(model))
    for link in model.findall('link'):
        lm = mm @ mat(pose(link))
        for col in link.findall('collision'):
            b = col.find('geometry/box/size')
            if b is None: continue
            sx, sy, sz = [float(t) for t in b.text.split()]
            boxes.append((lm @ mat(pose(col)), sx, sy))

def truth_mask(shape, ox, oy, res):
    H, W = shape
    img = np.zeros((H, W), np.uint8)
    for T, sx, sy in boxes:
        if sx > 50 or sy > 50: continue
        pts = np.array([[sx/2, sy/2, 1], [-sx/2, sy/2, 1], [-sx/2, -sy/2, 1], [sx/2, -sy/2, 1]]).T
        w = T @ pts
        px = np.stack([(w[0]-ox)/res, H-1-(w[1]-oy)/res], 1).astype(np.int32)
        cv2.fillPoly(img, [px], 255)
    return img

def evaluate(yaml_path):
    y = yaml.safe_load(open(yaml_path)); res = y['resolution']; ox, oy = y['origin'][:2]
    m = cv2.imread(os.path.join(os.path.dirname(yaml_path), y['image']), 0)
    T = truth_mask(m.shape, ox, oy, res)
    occ = (m == 0)
    dt = cv2.distanceTransform((T == 0).astype(np.uint8), cv2.DIST_L2, 5) * res
    d = dt[occ]
    r = {'occ_cells': int(occ.sum()), 'mean_dist_m': float(d.mean()), 'median_dist_m': float(np.median(d)),
         'within_0.10': float((d <= 0.10).mean()), 'within_0.20': float((d <= 0.20).mean())}
    return r, m, T, res, ox, oy

if __name__ == '__main__':
    r, m, T, res, ox, oy = evaluate(sys.argv[1])
    for k, v in r.items(): print(k, round(v, 3))
    if len(sys.argv) > 2:
        vis = cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)
        sel = (T > 0) & (m != 0)
        vis[sel] = (vis[sel] * 0.5 + np.array([0, 0, 255]) * 0.5).astype(np.uint8)
        ys, xs = np.where(m != 205)
        y0, y1, x0, x1 = max(0, ys.min()-5), ys.max()+5, max(0, xs.min()-5), xs.max()+5
        cv2.imwrite(sys.argv[2], cv2.resize(vis[y0:y1, x0:x1], None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST))
