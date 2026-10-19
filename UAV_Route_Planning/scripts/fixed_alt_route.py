# fixed_alt_route.py
# 需求：
# - 不做障礙膨脹，起飛後「固定高度」飛完全程
# - 依序到你指定的大樓(XY)上方，於固定高度 LOITER 5 分鐘
# - 產生 QGC .plan 與 Mission Planner .waypoints（含 LOITER_TIME）
# - 若 .npy 體素地圖不存在，會自動由 STL 生成

import os, sys, json, math, heapq, time
from pathlib import Path
import numpy as np
import open3d as o3d
import pymap3d as pm

# ===================== 使用者參數 =====================
# 固定飛行高度（公尺）：請設為「最高屋頂 + 安全裕度」
TARGET_Z_M = 40.0

# 高度容忍(體素格數)：允許在固定高度上下微調 (voxel_size=1m 時，5 表 ±5m)。不想微調可設 0。
ZTOL_VOX = 2

# 體素解析度（公尺/格）
VOXEL_SIZE = 1.0

# STL 模型檔（和此程式放一起）
STL_PATH = "ntouCampus_fixed.stl"

# 你要去的每棟大樓「平面座標」(世界座標、公尺；與 STL 一致，X東/Y北/Z上)
# ★ 請改成你的座標清單
BUILDINGS_XY_M = [
    (1200.0, 300.0),
    (800.0,  200.0),
    (400.0,  150.0),
]

# 起點（若你知道在體素索引空間的 x,y，可用 START_IDX_XY；若有世界座標用 START_XY_M）
# 二選一：A) 體素索引 xy；B) 世界座標 xy（公尺）
START_IDX_XY = (1050, 300)   # A) 例：體素索引 (i, j)
START_XY_M   = None          # B) 例：世界座標 (x, y)。上面 A 與 B 擇一有效。

# 盤旋時間（秒）
LOITER_SECONDS = 300  # 5 分鐘

# 產出任務檔開關
EXPORT_MISSION = True

# ENU→WGS84 轉換之原點（把 STL/voxel 的 (0,0,0) 對到真實地球 GPS）
# ★★★ 必須改成你的基準點 ★★★
LAT0, LON0, ALT0 = 25.150000, 121.780000, 5.0

# =====================================================


# ------------------- 體素地圖建立/載入 -------------------
def build_voxel_map_from_stl(stl_path: str, voxel_size: float):
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


# ------------------------ A* 規劃 ------------------------
def heuristic(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

def get_neighbors_3d(p, dims, mask):
    x, y, z = p
    X, Y, Z = dims
    for dx in (-1,0,1):
        for dy in (-1,0,1):
            for dz in (-1,0,1):
                if dx==dy==dz==0: 
                    continue
                nx, ny, nz = x+dx, y+dy, z+dz
                if not (0<=nx<X and 0<=ny<Y and 0<=nz<Z):
                    continue
                # 基本切角防護：軸向鄰格若為障礙則禁止
                if dx!=0 and mask[nx, y, z]:  continue
                if dy!=0 and mask[x, ny, z]:  continue
                if dz!=0 and mask[x, y, nz]:  continue
                # 面對角更嚴一點
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
            # 限制在固定高度薄層內（±ZTOL_VOX）
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


# ----------------------- 輔助函式 -----------------------
def index_to_world(ijk, min_bound, voxel_size):
    i,j,k = ijk
    x = i*voxel_size + min_bound[0]
    y = j*voxel_size + min_bound[1]
    z = k*voxel_size + min_bound[2]
    return (x,y,z)

def world_to_index(xyz, min_bound, voxel_size):
    x,y,z = xyz
    i = int(round((x - min_bound[0]) / voxel_size))
    j = int(round((y - min_bound[1]) / voxel_size))
    k = int(round((z - min_bound[2]) / voxel_size))
    return (i,j,k)


# -------------------- 任務檔輸出（含Loiter） --------------------
def write_qgc_plan_with_loiters(gps_pts, loiter_seconds, filename="ntou_rooftops.plan"):
    items=[]; seq=1
    for (lat,lon,alt) in gps_pts:
        # 到該樓上方的航點
        items.append({
          "AMSLAltAboveTerrain": None, "Altitude": float(alt),
          "AltitudeMode": 1, "autoContinue": True,
          "command": 16, "doJumpId": seq, "frame": 3,
          "params":[0,0,0,0, float(lon), float(lat), float(alt)],
          "type":"SimpleItem"
        }); seq+=1
        # Loiter 指定秒數（MAV_CMD_NAV_LOITER_TIME=19）
        items.append({
          "AMSLAltAboveTerrain": None, "Altitude": float(alt),
          "AltitudeMode": 1, "autoContinue": True,
          "command": 19, "doJumpId": seq, "frame": 3,
          "params":[float(loiter_seconds), 0, 0, 0, float(lon), float(lat), float(alt)],
          "type":"SimpleItem"
        }); seq+=1

    plan={
      "fileType":"Plan","groundStation":"QGroundControl",
      "geoFence":{"circles":[],"polygons":[],"version":2},
      "mission":{
        "cruiseSpeed":10,"hoverSpeed":5,"items":items,
        "plannedHomePosition":[LON0, LAT0, ALT0],
        "vehicleType":2, "firmwareType":3, "version":2  # 3=ArduPilot
      },
      "rallyPoints":{"points":[],"version":2},
      "version":1
    }
    with open(filename,"w",encoding="utf-8") as f: json.dump(plan,f,indent=2)
    print("Saved", filename)

def write_mp_waypoints_with_loiters(gps_pts, loiter_seconds, filename="ntou_rooftops.waypoints"):
    lines=["QGC WPL 110"]; seq=0
    for (lat,lon,alt) in gps_pts:
        # 航點
        lines.append(f"{seq}\t0\t3\t16\t0\t0\t0\t0\t{lat:.8f}\t{lon:.8f}\t{alt:.2f}\t1"); seq+=1
        # Loiter（command=19, param1=time(s)）
        lines.append(f"{seq}\t0\t3\t19\t{int(loiter_seconds)}\t0\t0\t0\t{lat:.8f}\t{lon:.8f}\t{alt:.2f}\t1"); seq+=1
    with open(filename,"w",encoding="utf-8") as f: f.write("\n".join(lines))
    print("Saved", filename)


# =========================== 主流程 ===========================
def main():
    # 1) 載入/建立體素地圖
    if Path("voxel_grid_map.npy").exists() and Path("voxel_min_bound.npy").exists():
        occ = np.load("voxel_grid_map.npy").astype(bool)
        min_bound = np.load("voxel_min_bound.npy").astype(float)
        print(f"[Map] 載入 .npy：shape={occ.shape}, voxel_size={VOXEL_SIZE} m")
    else:
        occ, min_bound = build_voxel_map_from_stl(STL_PATH, VOXEL_SIZE)

    X,Y,Z = occ.shape

    # 2) 固定高度層的索引
    z_idx = int(round((TARGET_Z_M - float(min_bound[2])) / VOXEL_SIZE))
    if not (0 <= z_idx < Z):
        raise ValueError(f"TARGET_Z_M 對應的 z 索引超出範圍：{z_idx} / Z={Z}")

    # 3) 不做膨脹，直接以原始佔據網格當作阻擋
    mask = occ

    # 4) 起點索引
    if START_XY_M is not None:
        si, sj, sk = world_to_index((START_XY_M[0], START_XY_M[1], TARGET_Z_M), min_bound, VOXEL_SIZE)
        start_idx = (si, sj, z_idx)
    else:
        start_idx = (START_IDX_XY[0], START_IDX_XY[1], z_idx)

    # 5) 目標（每棟樓的上方，固定高度）
    targets_idx = []
    for (x,y) in BUILDINGS_XY_M:
        i = int(round((x - min_bound[0]) / VOXEL_SIZE))
        j = int(round((y - min_bound[1]) / VOXEL_SIZE))
        targets_idx.append((i, j, z_idx))

    # 邊界檢查
    for p in [start_idx] + targets_idx:
        if not (0<=p[0]<X and 0<=p[1]<Y and 0<=p[2]<Z):
            raise ValueError(f"索引 {p} 超出地圖 ({X},{Y},{Z})")
        if mask[p]:
            print(f"[Warn] 點 {p} 位於障礙（或樓內部）。建議把 TARGET_Z_M 設更高。")

    # 6) 逐段 A*：起點→樓1→樓2→…
    route_idx = [start_idx]
    curr = start_idx
    print(f"[A*] 固定高度 z_idx={z_idx}, 容忍 ±{ZTOL_VOX} 層")
    for k, goal in enumerate(targets_idx, start=1):
        t0 = time.time()
        seg = astar_3d(curr, goal, mask, z_idx, ZTOL_VOX, occ.shape)
        dt = time.time()-t0
        if not seg:
            raise RuntimeError(f"[A*] 段 {k} 找不到路，請提高 TARGET_Z_M 或稍微提高 ZTOL_VOX")
        route_idx += seg[1:]
        curr = goal
        print(f"[A*] 段 {k} OK，節點 {len(seg)}，耗時 {dt:.2f}s")

    # 7) 轉世界座標（公尺），並視覺化
    path_world = [index_to_world(p, min_bound, VOXEL_SIZE) for p in route_idx]
    print("[Viz] 顯示 Open3D 視窗…")
    mesh = o3d.io.read_triangle_mesh(STL_PATH)
    mesh.compute_vertex_normals()
    voxel_vis = o3d.geometry.VoxelGrid.create_from_triangle_mesh(mesh, voxel_size=VOXEL_SIZE)
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(path_world)
    line_set.lines  = o3d.utility.Vector2iVector([[i,i+1] for i in range(len(path_world)-1)])
    line_set.colors = o3d.utility.Vector3dVector([[0,0,1]]*(len(path_world)-1))
    o3d.visualization.draw_geometries([voxel_vis, line_set])

    # 8) 產生「每棟樓上方盤旋點」的 GPS 航點（不需要把整條折線寫進任務）
    loiter_targets_world = [(x, y, TARGET_Z_M) for (x,y) in BUILDINGS_XY_M]
    gps_loiters = [pm.enu2geodetic(x, y, z, LAT0, LON0, ALT0) for (x,y,z) in loiter_targets_world]

    if EXPORT_MISSION:
        write_qgc_plan_with_loiters(gps_loiters, LOITER_SECONDS, "ntou_rooftops.plan")
        write_mp_waypoints_with_loiters(gps_loiters, LOITER_SECONDS, "ntou_rooftops.waypoints")
        print("→ 請用 QGroundControl 匯入 .plan 或 Mission Planner 匯入 .waypoints 上傳")

if __name__ == "__main__":
    main()
