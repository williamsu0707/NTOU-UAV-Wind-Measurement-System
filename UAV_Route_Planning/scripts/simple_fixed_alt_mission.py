# simple_fixed_alt_mission.py
# 目的：
# - 不用 STL/voxel，直接依固定高度與目標點輸出任務檔
# - 每個大樓上方 Loiter 300 秒
# - 支援：經緯度清單（推薦）或 ENU(x,y)+原點 GPS
# - 顯示 2D 路線圖（經緯度平面）

import json
from typing import List, Tuple
import os

# ======================【必改參數】======================

# 固定飛行高度（相對起飛點，單位：公尺）
TARGET_ALT_M = 40.0

# 在樓上方停留秒數
LOITER_SECONDS = 300  # 5 分鐘

# ★ 方式 A：直接給每棟樓的經緯度（建議）
USE_LATLON = True
BUILDINGS_LATLON: List[Tuple[float, float]] = [
    # (lat, lon)
    (25.150800, 121.780200),
    (25.151500, 121.779600),
    (25.152100, 121.780900),
]

# ★ 方式 B：若你只有相對原點的 ENU（公尺），改成 USE_LATLON=False，並填下面三個：
#   原點 GPS（把 ENU 的 (0,0,0) 對到這裡）
LAT0, LON0, ALT0 = 25.150000, 121.780000, 5.0
#   每棟樓的 ENU XY（公尺）。Z 由 TARGET_ALT_M 決定，不用填。
BUILDINGS_ENU_M: List[Tuple[float, float]] = [
    # (x_east_m, y_north_m)
    # 例： (1200.0, 300.0),
]

# 是否在任務最前面加入 TAKEOFF 指令（建議 True）
ADD_TAKEOFF = True

# 產出檔名前綴
OUT_STEM = "ntou_rooftops"

# 畫圖設定：是否顯示與是否儲存 png
SHOW_PLOT = True
SAVE_PLOT = True

# =======================================================


# -------------------- 工具函式 --------------------
def enu_to_geodetic(points_enu_xy: List[Tuple[float, float]],
                    lat0: float, lon0: float, alt0: float,
                    alt_rel: float) -> List[Tuple[float, float, float]]:
    """將一串 ENU(x,y) 轉成 (lat,lon,alt)，alt 用相對高度 alt_rel"""
    import pymap3d as pm
    out = []
    for x, y in points_enu_xy:
        lat, lon, _ = pm.enu2geodetic(x, y, 0.0, lat0, lon0, alt0)
        out.append((lat, lon, alt_rel))
    return out


def write_qgc_plan_with_loiters(gps_pts: List[Tuple[float, float, float]],
                                loiter_seconds: int,
                                filename: str,
                                add_takeoff: bool = True):
    """輸出 QGroundControl .plan（含 Loiter 時間）"""
    items = []
    seq = 1

    if add_takeoff:
        # MAV_CMD_NAV_TAKEOFF=22, frame=3 (GLOBAL_RELATIVE_ALT)
        items.append({
            "AMSLAltAboveTerrain": None,
            "Altitude": float(TARGET_ALT_M),
            "AltitudeMode": 1,
            "autoContinue": True,
            "command": 22,
            "doJumpId": seq,
            "frame": 3,
            "params": [0, 0, 0, 0, gps_pts[0][1], gps_pts[0][0], float(TARGET_ALT_M)],
            "type": "SimpleItem"
        })
        seq += 1

    for (lat, lon, alt) in gps_pts:
        # 航點
        items.append({
            "AMSLAltAboveTerrain": None, "Altitude": float(alt),
            "AltitudeMode": 1, "autoContinue": True,
            "command": 16, "doJumpId": seq, "frame": 3,
            "params": [0, 0, 0, 0, float(lon), float(lat), float(alt)],
            "type": "SimpleItem"
        }); seq += 1
        # Loiter 時間（MAV_CMD_NAV_LOITER_TIME=19, param1=秒數）
        items.append({
            "AMSLAltAboveTerrain": None, "Altitude": float(alt),
            "AltitudeMode": 1, "autoContinue": True,
            "command": 19, "doJumpId": seq, "frame": 3,
            "params": [float(loiter_seconds), 0, 0, 0, float(lon), float(lat), float(alt)],
            "type": "SimpleItem"
        }); seq += 1

    plan = {
        "fileType": "Plan",
        "groundStation": "QGroundControl",
        "geoFence": {"circles": [], "polygons": [], "version": 2},
        "mission": {
            "cruiseSpeed": 10,
            "hoverSpeed": 5,
            "items": items,
            "plannedHomePosition": [gps_pts[0][1], gps_pts[0][0], 0],
            "vehicleType": 2,      # 2=MultiRotor
            "firmwareType": 3,     # 3=ArduPilot
            "version": 2
        },
        "rallyPoints": {"points": [], "version": 2},
        "version": 1
    }
    with open(filename, "w", encoding="utf-8") as f:
        json.dump(plan, f, indent=2)
    print("Saved", filename)


def write_mp_waypoints_with_loiters(gps_pts: List[Tuple[float, float, float]],
                                    loiter_seconds: int,
                                    filename: str,
                                    add_takeoff: bool = True):
    """輸出 Mission Planner .waypoints（含 Loiter 時間）"""
    lines = ["QGC WPL 110"]
    seq = 0
    if add_takeoff:
        # TAKEOFF：cmd=22, frame=3
        lat0, lon0, _ = gps_pts[0]
        lines.append(f"{seq}\t0\t3\t22\t0\t0\t0\t0\t{lat0:.8f}\t{lon0:.8f}\t{TARGET_ALT_M:.2f}\t1"); seq += 1

    for (lat, lon, alt) in gps_pts:
        # 航點
        lines.append(f"{seq}\t0\t3\t16\t0\t0\t0\t0\t{lat:.8f}\t{lon:.8f}\t{alt:.2f}\t1"); seq += 1
        # Loiter 時間（cmd=19, param1=秒數）
        lines.append(f"{seq}\t0\t3\t19\t{int(loiter_seconds)}\t0\t0\t0\t{lat:.8f}\t{lon:.8f}\t{alt:.2f}\t1"); seq += 1

    with open(filename, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    print("Saved", filename)


def plot_route(gps_pts: List[Tuple[float, float, float]], save_png: bool = True, show: bool = True, stem: str = "route"):
    """用 matplotlib 畫 2D 路徑（經緯度投影）"""
    try:
        import matplotlib.pyplot as plt
    except Exception as e:
        print("[Plot] 無法載入 matplotlib（可用 `pip install matplotlib` 安裝）。略過畫圖。")
        return
    lats = [lat for (lat, lon, alt) in gps_pts]
    lons = [lon for (lat, lon, alt) in gps_pts]
    plt.figure()
    plt.plot(lons, lats, 'o-', label="Route (WP + Loiter)")
    for i, (lat, lon, alt) in enumerate(gps_pts, start=1):
        plt.text(lon, lat, f"P{i}", fontsize=9, ha='left', va='bottom')
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.title("Flight Path (2D)")
    plt.legend()
    plt.axis("equal")
    if save_png:
        fname = f"{stem}.png"
        plt.savefig(fname, dpi=150, bbox_inches="tight")
        print("Saved", fname)
    if show:
        plt.show()
    else:
        plt.close()


# ------------------------- 主程式 -------------------------
def main():
    # 1) 建立 GPS 航點（lat,lon,alt）清單
    if USE_LATLON:
        if not BUILDINGS_LATLON:
            raise ValueError("USE_LATLON=True 時，請在 BUILDINGS_LATLON 填入至少一個 (lat, lon)")
        gps_pts = [(float(lat), float(lon), float(TARGET_ALT_M)) for (lat, lon) in BUILDINGS_LATLON]
    else:
        if not BUILDINGS_ENU_M:
            raise ValueError("USE_LATLON=False 時，請填 BUILDINGS_ENU_M 與 LAT0/LON0/ALT0")
        gps_pts = enu_to_geodetic(BUILDINGS_ENU_M, LAT0, LON0, ALT0, TARGET_ALT_M)

    # 2) 輸出任務檔（含 Loiter）
    plan_file = f"{OUT_STEM}.plan"
    wpts_file = f"{OUT_STEM}.waypoints"
    write_qgc_plan_with_loiters(gps_pts, LOITER_SECONDS, plan_file, add_takeoff=ADD_TAKEOFF)
    write_mp_waypoints_with_loiters(gps_pts, LOITER_SECONDS, wpts_file, add_takeoff=ADD_TAKEOFF)

    # 3) 畫 2D 路線圖
    plot_route(gps_pts, save_png=SAVE_PLOT, show=SHOW_PLOT, stem=OUT_STEM)

    print("\n✅ 完成！用 QGroundControl 匯入 .plan 或 Mission Planner 匯入 .waypoints 上傳即可。")
    print("   ※ 高度為 GLOBAL_RELATIVE_ALT，相對起飛點高度。")

if __name__ == "__main__":
    main()
