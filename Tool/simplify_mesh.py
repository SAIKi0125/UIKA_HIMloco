import trimesh
import os

# 1. 设置文件路径
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
mesh_dir = os.path.join(PROJECT_ROOT, "resources", "robots", "htdw_4438", "meshes")
input_file = os.path.join(mesh_dir, "base_link.STL")
output_file = os.path.join(mesh_dir, "base_link_simple.STL")

# 2. 加载模型
mesh = trimesh.load_mesh(input_file)
print(f"原始面数: {len(mesh.faces)}")

# 3. 简化模型 (减面)
# MuJoCo 限制是 200,000，建议减到 100,000 以下以保证仿真流畅度
target_faces = 100000 
if len(mesh.faces) > target_faces:
    simplified = mesh.simplify_quadric_decimation(face_count=target_faces)
    print(f"简化后面数: {len(simplified.faces)}")
    
    # 4. 导出为二进制 STL (MuJoCo 友好格式)
    simplified.export(output_file, file_type='stl')
    print(f"文件已保存至: {output_file}")
else:
    print("当前面数未超限，无需处理。")
