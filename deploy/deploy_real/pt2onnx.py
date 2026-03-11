import torch
import torch.nn as nn
import onnx
import os

# ===================== 选择模型文件（二选一，推荐用policy_400_OK.pt） =====================

model_filename = "model_800.pt"

# 拼接完整模型路径
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
model_dir = os.path.join(PROJECT_ROOT, "logs", "flat_htdw_4438", "exported", "policies")
model_path = os.path.join(model_dir, model_filename)

# ===================== 路径检查（无需修改） =====================
if not os.path.exists(model_path):
    raise FileNotFoundError(f"模型文件不存在！路径：{model_path}\n请检查文件名是否正确")
if os.path.isdir(model_path):
    raise ValueError(f"路径是目录，不是文件！当前路径：{model_path}")

# ===================== 加载模型（核心修正） =====================
model = torch.jit.load(model_path)  # 修正了原代码重复的torch.前缀
model.eval()  # 切换到推理模式，禁用训练相关层

# ===================== 导出ONNX配置（无需修改） =====================
input_names = ['input']
output_names = ['output']
x = torch.randn(1, 270, requires_grad=True)  # 保持你原有的输入维度

# 确保ONNX输出目录存在
onnx_output_dir = os.path.join(PROJECT_ROOT, "onnx")
if not os.path.exists(onnx_output_dir):
    os.makedirs(onnx_output_dir)
onnx_output_path = os.path.join(onnx_output_dir, 'policy02062206.onnx')

# ===================== 导出ONNX模型（修正verbose参数） =====================
torch.onnx.export(
    model,
    x,
    onnx_output_path,
    input_names=input_names,
    output_names=output_names,
    verbose=True,  # 修正为布尔值（原代码是字符串'True'）
    opset_version=12  # 指定opset版本，提升兼容性
)

# ===================== 验证ONNX模型（可选，确保导出成功） =====================
print(f"开始验证导出的ONNX模型：{onnx_output_path}")
onnx_model = onnx.load(onnx_output_path)
onnx.checker.check_model(onnx_model)
print("✅ ONNX模型导出并验证成功！文件路径：", onnx_output_path)
