# build_voxel_map.py
# 目的：把 ntouCampus_fixed.stl 轉成兩個檔：
#  1) voxel_grid_map.npy  (3D 佔據地圖：True=障礙、False=空)
#  2) voxel_min_bound.npy (世界座標最小邊界，用來把索引轉回公尺座標)

import numpy as np
import open3d as o3d
from pathlib import Path

# === 參數（最重要的是 voxel_size）===
STL_PATH = "ntouCampus_fixed.stl"  # 你的 STL 檔名
VOXEL_SIZE = 1.0                   # 每一格幾公尺：1.0=一格一公尺（可改 0.5 會更細但更慢/更吃記憶體）

def main():
    stl = Path(STL_PATH)
    if not stl.exists():
        raise FileNotFoundError(f"找不到 STL 檔：{stl.resolve()}")

    print(f"[1/4] 讀取 STL：{stl.name}")
    mesh = o3d.io.read_triangle_mesh(str(stl))
    if mesh.is_empty():
        raise RuntimeError("STL 讀取失敗或是空模型。")
    mesh.compute_vertex_normals()

    # 若你的 STL 單位不是公尺，可在這裡做等比縮放（例如 mm→m 就 mesh.scale(0.001, center=mesh.get_center())）
    # mesh.scale(0.001, center=mesh.get_center())

    print(f"[2/4] 以 voxel_size={VOXEL_SIZE} m 體素化模型…")
    vg = o3d.geometry.VoxelGrid.create_from_triangle_mesh(mesh, voxel_size=VOXEL_SIZE)

    min_bound = vg.get_min_bound()
    max_bound = vg.get_max_bound()
    print(f"    min_bound = {min_bound}")
    print(f"    max_bound = {max_bound}")

    # 計算體素格數 (nx, ny, nz)
    nx = int(np.ceil((max_bound[0] - min_bound[0]) / VOXEL_SIZE))
    ny = int(np.ceil((max_bound[1] - min_bound[1]) / VOXEL_SIZE))
    nz = int(np.ceil((max_bound[2] - min_bound[2]) / VOXEL_SIZE))
    if nx == 0 or ny == 0 or nz == 0:
        raise RuntimeError("體素尺寸或模型邊界有問題，導致格數為 0。請檢查 STL 尺度或調整 VOXEL_SIZE。")
    print(f"[3/4] 佔據矩陣大小 = ({nx}, {ny}, {nz})")

    occ = np.zeros((nx, ny, nz), dtype=bool)
    for v in vg.get_voxels():
        ix, iy, iz = v.grid_index
        # 容錯：確保 index 在範圍內
        if 0 <= ix < nx and 0 <= iy < ny and 0 <= iz < nz:
            occ[ix, iy, iz] = True

    print("[4/4] 存檔：voxel_grid_map.npy, voxel_min_bound.npy")
    np.save("voxel_grid_map.npy", occ)
    np.save("voxel_min_bound.npy", np.array(min_bound, dtype=float))

    print("✅ 完成！")
    print(f"   - voxel_grid_map.npy  形狀: {occ.shape}  (True=障礙)")
    print(f"   - voxel_min_bound.npy 內容: {min_bound}")
    print("   之後請在同一資料夾執行你的 A* 規劃程式。")

if __name__ == "__main__":
    main()
