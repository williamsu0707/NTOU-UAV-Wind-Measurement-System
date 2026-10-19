# 3dAutoPath.py
# 功能：
# - STL -> voxel(.npy) 自動建圖（若不存在）
# - 快速3D膨脹 (scipy distance transform)
# - 3D A* 規劃（26鄰、避免切角、限制高度±ztol）
# - Open3D 視覺化
# - 可選：輸出 ArduPilot/QGC 任務

import os, sys, time, math, heapq, json
from pathlib import Path
import numpy as np
import open3d as o3d
from scipy.ndimage import distance_transform_edt
import pymap3d as pm

# ====================== 基本參數 ======================
STL_PATH      = "ntouCampus_fixed.stl"  # 你的校園 STL
VOXEL_SIZE    = 1.0                     # 體素解析度 (公尺/格)
CLEARANCE_M   = 25.0                    # 障礙安全半徑 (公尺)
TARGET_Z_M    = 15.0                    # 目標飛行高度 (公尺)
ZTOL_VOX      = 1                       # 允許高程偏差 (±格)，1=±1m(配合 voxel_size=1)

# 起終點：二選一 (A) 直接填 voxel 索引；(B) 用世界座標(公尺)自動換算
# ---- (A) 直接給索引（知道 grid 大小時用）----
START_IDX = (1050, 300, None)   # z 會自動用 target_z 換算
GOAL_IDX  = (100,  50,  None)

# ---- (B) 或改用世界座標（公尺，和 STL/voxel 同座標系）----
START_XYZ_M = None  # 例如 (1200.0, 300.0, 15.0)
GOAL_XYZ_M  = None  # 例如 (100.0,  50.0,  15.0)

# 是否輸出任務檔
EXPORT_MISSION = False  # True 開啟
# 原點 GPS（把 voxel 的 (0,0,0) 或 min_bound 對到真實地球）
LAT0, LON0, ALT0 = 25.150000, 121.780000, 5.0   # 請換成你的基準點

# =====================================================

# -------------- 建圖：STL -> voxel(.npy) --------------
def build_voxel_map_from_stl(stl_path:str, voxel_size:float):
    stl = Path(stl_path)
    if not stl.exists():
        raise FileNotFoundError(f"找不到 STL：{stl.resolve()}")
    print(f"[Build] 讀取 STL：{stl.name}")
    mesh = o3d.io.read_triangle_mesh(str(stl))
    if mesh.is_empty():
        raise RuntimeError("STL 讀取失敗或為空模型")
    mesh.compute_vertex_normals()

    print(f"[Build] 體素化 (voxel_size={voxel_size} m)...")
    vg = o3d.geometry.VoxelGrid.create_from_triangle_mesh(mesh, voxel_size=voxel_size)

    min_bound = vg.get_min_bound()
    max_bound = vg.get_max_bound()
    nx = int(np.ceil((max_bound[0]-min_bound[0]) / voxel_size))
    ny = int(np.ceil((max_bound[1]-min_bound[1]) / voxel_size))
    nz = int(np.ceil((max_bound[2]-min_bound[2]) / voxel_size))
    if min(nx,ny,nz) <= 0:
        raise RuntimeError("模型邊界或 voxel_size 有問題，導致格數為 0")

    print(f"[Build] 佔據矩陣大小 = ({nx}, {ny}, {nz})")
    occ = np.zeros((nx, ny, nz), dtype=bool)
    for v in vg.get_voxels():
        ix, iy, iz = v.grid_index
        if 0<=ix<nx and 0<=iy<ny and 0<=iz<nz:
            occ[ix, iy, iz] = True

    np.save("voxel_grid_map.npy", occ)
    np.save("voxel_min_bound.npy", np.array(min_bound, dtype=float))
    print("[Build] 已輸出 voxel_grid_map.npy, voxel_min_bound.npy")
    return occ, np.array(min_bound, dtype=float)

# -------------- 快速 3D 膨脹（距離變換） --------------
def dilate3d_fast(occ: np.ndarray, clearance_m: float, voxel_size: float) -> np.ndarray:
    # 對「可通行區」做距離變換，得到每格到最近障礙的距離(公尺)
    dist_free = distance_transform_edt(~occ) * voxel_size
    # 距離 < 安全半徑者視為不可通行（碰撞遮罩）
    mask = dist_free < clearance_m
    return mask

# -------------- A* 相關 --------------
def heuristic(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

def get_neighbors_3d(p, dims, mask):
    x, y, z = p
    X, Y, Z = dims
    for dx in (-1,0,1):
        for dy in (-1,0,1):
            for dz in (-1,0,1):
                if dx==dy==dz==0: continue
                nx, ny, nz = x+dx, y+dy, z+dz
                if not (0<=nx<X and 0<=ny<Y and 0<=nz<Z): 
                    continue
                # 基本切角防護：軸向鄰格只要有障礙就禁止
                if dx!=0 and mask[nx, y, z]:  continue
                if dy!=0 and mask[x, ny, z]:  continue
                if dz!=0 and mask[x, y, nz]:  continue
                # 更嚴一點：兩兩組合的面對角
                if dx!=0 and dy!=0 and mask[nx, ny, z]: continue
                if dx!=0 and dz!=0 and mask[nx, y, nz]: continue
                if dy!=0 and dz!=0 and mask[x, ny, nz]: continue
                yield (nx,ny,nz), math.sqrt(dx*dx + dy*dy + dz*dz)

def astar_3d(start, goal, mask, z_idx, ztol, dims):
    open_set = []
    heapq.heappush(open_set, (0.0, start))
    came_from = {}
    g_score = {start: 0.0}
    closed = set()

    while open_set:
        _, current = heapq.heappop(open_set)
        if current in closed:
            continue
        if current == goal:
            # reconstruct
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path
        closed.add(current)

        for nbr, cost in get_neighbors_3d(current, dims, mask):
            # 限制高度在 z_idx ± ztol
            if abs(nbr[2] - z_idx) > ztol: 
                continue
            if mask[nbr]: 
                continue
            tentative = g_score[current] + cost
            if tentative < g_score.get(nbr, float('inf')):
                g_score[nbr] = tentative
                came_from[nbr] = current
                f = tentative + heuristic(nbr, goal)
                heapq.heappush(open_set, (f, nbr))
    return None

# -------------- 輔助：座標互轉 --------------
def world_to_index(xyz, min_bound, voxel_size):
    x,y,z = xyz
    i = int(round((x - min_bound[0]) / voxel_size))
    j = int(round((y - min_bound[1]) / voxel_size))
    k = int(round((z - min_bound[2]) / voxel_size))
    return (i,j,k)

def index_to_world(ijk, min_bound, voxel_size):
    i,j,k = ijk
    x = i*voxel_size + min_bound[0]
    y = j*voxel_size + min_bound[1]
    z = k*voxel_size + min_bound[2]
    return (x,y,z)

# -------------- 輸出任務檔 --------------
def write_missionplanner_waypoints(gps_pts, filename="ntou_path.waypoints"):
    lines=["QGC WPL 110"]
    for i,(lat,lon,alt) in enumerate(gps_pts):
        seq=i; current=0; frame=3; command=16
        p1=p2=p3=p4=0; autocont=1
        lines.append(f"{seq}\t{current}\t{frame}\t{command}\t{p1}\t{p2}\t{p3}\t{p4}\t{lat:.8f}\t{lon:.8f}\t{alt:.2f}\t{autocont}")
    with open(filename,"w",encoding="utf-8") as f:
        f.write("\n".join(lines))
    print("Saved", filename)

def write_qgc_plan(gps_pts, filename="ntou_path.plan"):
    items=[]
    for i,(lat,lon,alt) in enumerate(gps_pts, start=1):
        items.append({
            "AMSLAltAboveTerrain": None, "Altitude": float(alt),
            "AltitudeMode": 1, "autoContinue": True, "command": 16,
            "doJumpId": i, "frame": 3,
            "params":[0,0,0,0, float(lon), float(lat), float(alt)],
            "type":"SimpleItem"
        })
    plan={
      "fileType":"Plan","groundStation":"QGroundControl",
      "geoFence":{"circles":[],"polygons":[],"version":2},
      "mission":{
        "cruiseSpeed":10,"hoverSpeed":5,"items":items,
        "plannedHomePosition":[LON0,LAT0,ALT0],
        "vehicleType":2,"firmwareType":3,"version":2
      },
      "rallyPoints":{"points":[],"version":2},
      "version":1
    }
    with open(filename,"w",encoding="utf-8") as f:
        json.dump(plan,f,indent=2)
    print("Saved", filename)

# ====================== 主流程 ======================
def main():
    # 1) 載入/建立 voxel 地圖
    if Path("voxel_grid_map.npy").exists() and Path("voxel_min_bound.npy").exists():
        occ = np.load("voxel_grid_map.npy").astype(bool)
        min_bound = np.load("voxel_min_bound.npy").astype(float)
        print(f"[Map] 載入現有 .npy：shape={occ.shape}, voxel_size={VOXEL_SIZE} m")
    else:
        occ, min_bound = build_voxel_map_from_stl(STL_PATH, VOXEL_SIZE)

    X,Y,Z = occ.shape
    # 2) 決定 z 層
    z_idx = int(round((TARGET_Z_M - min_bound[2]) / VOXEL_SIZE))
    if not (0<=z_idx<Z):
        raise ValueError(f"target_z 轉成索引超出範圍：{z_idx} / Z={Z}")

    # 3) 快速膨脹
    print(f"[Dilation] grid={occ.shape}, clearance={CLEARANCE_M} m ...")
    t0=time.time()
    mask = dilate3d_fast(occ, CLEARANCE_M, VOXEL_SIZE)
    print(f"[Dilation] done in {time.time()-t0:.2f}s")

    # 4) 取得 start/goal 索引
    if START_XYZ_M and GOAL_XYZ_M:
        start = world_to_index(START_XYZ_M, min_bound, VOXEL_SIZE)
        goal  = world_to_index(GOAL_XYZ_M,  min_bound, VOXEL_SIZE)
        start = (start[0], start[1], z_idx)
        goal  = (goal[0],  goal[1],  z_idx)
    else:
        sx,sy,_ = START_IDX
        gx,gy,_ = GOAL_IDX
        start = (sx, sy, z_idx)
        goal  = (gx, gy, z_idx)

    if not (0<=start[0]<X and 0<=start[1]<Y and 0<=start[2]<Z):
        raise ValueError("Start 超出地圖")
    if not (0<=goal[0]<X and 0<=goal[1]<Y and 0<=goal[2]<Z):
        raise ValueError("Goal 超出地圖")
    if mask[start] or mask[goal]:
        raise RuntimeError("Start 或 Goal 位於膨脹後障礙內，請調整座標或降低 clearance。")

    print(f"[A*] start={start}, goal={goal}, z_idx={z_idx}, ztol=±{ZTOL_VOX}")
    t0=time.time()
    path_idx = astar_3d(start, goal, mask, z_idx, ZTOL_VOX, (X,Y,Z))
    if not path_idx:
        print("[A*] No path found.")
        sys.exit(1)
    print(f"[A*] Path found: {len(path_idx)} nodes, time={time.time()-t0:.2f}s")

    # 5) 轉回世界座標
    path_world = [index_to_world(p, min_bound, VOXEL_SIZE) for p in path_idx]

    # 6) 視覺化
    print("[Viz] 準備顯示 Open3D 視窗...")
    mesh = o3d.io.read_triangle_mesh(STL_PATH)
    mesh.compute_vertex_normals()
    voxel_vis = o3d.geometry.VoxelGrid.create_from_triangle_mesh(mesh, voxel_size=VOXEL_SIZE)

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(path_world)
    lines = [[i, i+1] for i in range(len(path_world)-1)]
    line_set.lines  = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector([[0,0,1] for _ in lines])  # 藍色

    o3d.visualization.draw_geometries([voxel_vis, line_set])

    # 7) 可選：輸出任務檔
    if EXPORT_MISSION:
        gps_pts = [pm.enu2geodetic(x,y,z, LAT0,LON0,ALT0) for (x,y,z) in path_world]
        write_missionplanner_waypoints(gps_pts, "ntou_path.waypoints")
        write_qgc_plan(gps_pts, "ntou_path.plan")

if __name__ == "__main__":
    main()
