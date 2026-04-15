#!/bin/bash
set -e

# ============================================
# UIKA RL SAR 一键环境配置脚本
# 用于在新电脑上快速配置开发/运行环境
# ============================================

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
print_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
print_warning() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
print_error() { echo -e "${RED}[ERROR]${NC} $1"; }

show_usage() {
    echo "=========================================="
    echo "   UIKA RL SAR 环境配置脚本"
    echo "=========================================="
    echo ""
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  --cmake-only    仅配置CMake构建环境(无ROS,用于硬件部署)"
    echo "  --ros2          配置完整ROS2 Humble环境"
    echo "  --all           配置所有依赖(默认)"
    echo "  --help          显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  $0 --ros2           # 配置ROS2 Humble环境"
    echo "  $0 --cmake-only     # 仅配置CMake(硬件部署)"
}

# 默认配置
MODE="all"

# 解析参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --cmake-only)
            MODE="cmake"
            shift
            ;;
        --ros2)
            MODE="ros2"
            shift
            ;;
        --all)
            MODE="all"
            shift
            ;;
        --help|-h)
            show_usage
            exit 0
            ;;
        *)
            print_error "未知选项: $1"
            show_usage
            exit 1
            ;;
    esac
done

# ============================================
# Step 1: 检查系统依赖
# ============================================
check_system_deps() {
    print_info "=========================================="
    print_info "Step 1: 检查系统依赖"
    print_info "=========================================="

    local missing_deps=()

    # 检查必需的命令
    local required_cmds=("cmake" "gcc" "g++" "unzip" "curl" "wget" "tar" "git")
    for cmd in "${required_cmds[@]}"; do
        if ! command -v "$cmd" &> /dev/null; then
            missing_deps+=("$cmd")
        fi
    done

    if [ ${#missing_deps[@]} -ne 0 ]; then
        print_warning "缺少系统依赖: ${missing_deps[*]}"
        print_info "请运行以下命令安装:"
        print_info "  sudo apt-get update && sudo apt-get install -y ${missing_deps[*]}"
    fi

    # 检查 rl_sar 必需的系统库
    print_info "检查 rl_sar 必需的系统库..."
    local required_libs=("libyaml-cpp-dev" "libeigen3-dev" "libboost-all-dev" "libspdlog-dev" "libfmt-dev" "libtbb-dev" "liblcm-dev")
    local missing_libs=()
    for lib in "${required_libs[@]}"; do
        if ! dpkg -l | grep -q "^ii  $lib "; then
            missing_libs+=("$lib")
        fi
    done

    if [ ${#missing_libs[@]} -ne 0 ]; then
        print_warning "缺少库: ${missing_libs[*]}"
        print_info "请运行以下命令安装:"
        print_info "  sudo apt-get update && sudo apt-get install -y ${missing_libs[*]}"
    fi

    print_success "系统依赖检查通过"
    return 0
}

# ============================================
# Step 2: 安装ROS 2 Humble (可选)
# ============================================
install_ros2() {
    print_info "=========================================="
    print_info "Step 2: 安装ROS 2 Humble"
    print_info "=========================================="

    # 检查是否已安装
    if [ -n "$ROS_DISTRO" ]; then
        if [ "$ROS_DISTRO" = "humble" ]; then
            print_success "ROS 2 Humble 已安装 (source /opt/ros/humble/setup.bash)"
            return 0
        else
            print_warning "检测到 ROS $ROS_DISTRO，不是 Humble"
        fi
    fi

    # 检查是否在 /opt/ros
    if [ -d "/opt/ros/humble" ]; then
        print_success "ROS 2 Humble 已安装在 /opt/ros/humble"
        return 0
    fi

    print_info "ROS 2 Humble 未安装"
    print_info "请手动安装: https://docs.ros.org/en/humble/Installation.html"
    print_info ""
    print_info "快捷安装命令:"
    print_info "  sudo apt update && sudo apt install -y ros-humble-desktop"
    print_info ""
    print_info "或使用鱼香ROS一键安装: wget http://fishros.com/install -O fishros_install.sh && bash fishros_install.sh"

    return 1
}

# ============================================
# Step 3: 配置Conda环境
# ============================================
setup_conda_env() {
    print_info "=========================================="
    print_info "Step 3: 配置Conda环境"
    print_info "=========================================="

    # 检查conda是否安装
    if ! command -v conda &> /dev/null; then
        print_info "Conda未安装，安装Miniconda..."
        cd /tmp

        ARCH_TYPE="$(uname -m)"
        if [ "${ARCH_TYPE}" = "aarch64" ]; then
            # ARM64: 使用aarch64版本的Miniconda (-k for SSL issues on embedded)
            curl -Lk https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh -o miniconda.sh
        else
            # x86_64: 使用标准版本
            curl -Lk https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -o miniconda.sh
        fi

        bash miniconda.sh -b -p ~/.miniconda
        rm miniconda.sh
        export PATH="$HOME/.miniconda/bin:$PATH"
        cd - > /dev/null
    fi

    # 激活base环境以使用conda命令
    if [ -f "$HOME/.miniconda3/etc/profile.d/conda.sh" ]; then
        source "$HOME/.miniconda3/etc/profile.d/conda.sh"
    elif [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
        source "$HOME/miniconda3/etc/profile.d/conda.sh"
    fi

    # 检查ros2_humble环境是否存在
    if conda env list | grep -q "^ros2_humble "; then
        print_success "ros2_humble 环境已存在"
    else
        print_info "创建 ros2_humble 环境..."
        conda create -n ros2_humble python=3.10 -y
        print_success "ros2_humble 环境创建成功"
    fi

    # 激活环境并安装基础包
    print_info "安装Python依赖..."

    conda run -n ros2_humble pip install --upgrade pip
    conda run -n ros2_humble pip install numpy

    # 检查是否在 Jetson (ARM64) 上
    ARCH_TYPE="$(uname -m)"
    if [ "${ARCH_TYPE}" = "aarch64" ]; then
        print_info "检测到 ARM64 架构 (Jetson)"
        print_info "跳过 LibTorch (无通用 ARM64 版本，UIKA 使用 ONNX 模型不需要)"
        print_info "PyTorch 需手动安装 (参考 NVIDIA JetPack 文档)"
    else
        # x86_64: 使用官方 PyTorch CPU 版本
        conda run -n ros2_humble pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
    fi

    print_success "Conda环境配置完成"
}

# ============================================
# Step 4: 下载推理运行时
# ============================================
setup_inference_runtime() {
    print_info "=========================================="
    print_info "Step 4: 下载推理运行时"
    print_info "=========================================="

    cd "$PROJECT_ROOT"

    ARCH_TYPE="$(uname -m)"

    if [ "${ARCH_TYPE}" = "aarch64" ]; then
        print_info "ARM64 架构 (Jetson) - 仅下载 ONNX Runtime"
        print_info "LibTorch 在 ARM64 上不可用，跳过"
        bash "$SCRIPT_DIR/download_inference_runtime.sh" onnx
    else
        # x86_64: 下载 LibTorch 和 ONNX Runtime
        print_info "x86_64 架构 - 下载 LibTorch + ONNX Runtime"
        bash "$SCRIPT_DIR/download_inference_runtime.sh" all
    fi

    print_success "推理运行时配置完成"
}

# ============================================
# Step 5: 下载机器人描述文件
# ============================================
setup_robot_descriptions() {
    print_info "=========================================="
    print_info "Step 5: 下载机器人描述文件"
    print_info "=========================================="

    cd "$PROJECT_ROOT"
    bash "$SCRIPT_DIR/download_robot_descriptions.sh"

    print_success "机器人描述文件配置完成"
}

# ============================================
# Step 6: 构建项目
# ============================================
build_project() {
    print_info "=========================================="
    print_info "Step 6: 构建项目"
    print_info "=========================================="

    cd "$PROJECT_ROOT"

    if [ "$MODE" = "cmake" ]; then
        print_info "使用CMake模式构建(无ROS)..."
        ./build.sh -m
    else
        # 检查ROS是否可用
        if [ -z "$ROS_DISTRO" ]; then
            print_warning "ROS环境未激活，使用CMake模式构建..."
            ./build.sh -m
        else
            print_info "使用ROS模式构建..."
            ./build.sh
        fi
    fi

    print_success "项目构建完成"
}

# ============================================
# Step 7: 环境变量配置
# ============================================
setup_env_vars() {
    print_info "=========================================="
    print_info "Step 7: 生成环境变量配置"
    print_info "=========================================="

    local env_file="$PROJECT_ROOT/env_uika.sh"

    cat > "$env_file" << 'EOF'
# ============================================
# UIKA RL SAR 环境变量配置
# ============================================
# 使用方法: source env_uika.sh

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Conda ros2_humble 环境
if [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
    source "$HOME/miniconda3/etc/profile.d/conda.sh"
    conda activate ros2_humble
fi

# ROS 2 Humble (如果已安装)
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# 项目workspace
if [ -d "$PROJECT_ROOT/install" ]; then
    source "$PROJECT_ROOT/install/setup.bash"
fi

# LibTorch和ONNX Runtime库路径
export LD_LIBRARY_PATH="$PROJECT_ROOT/library/inference_runtime/libtorch/lib:$LD_LIBRARY_PATH"
export LD_LIBRARY_PATH="$PROJECT_ROOT/library/inference_runtime/onnxruntime/lib:$LD_LIBRARY_PATH"

# 项目根目录
export RL_SAR_ROOT="$PROJECT_ROOT"

echo "UIKA RL SAR 环境已激活"
echo "  项目路径: $PROJECT_ROOT"
echo "  Python: $(which python)"
echo "  ROS: ${ROS_DISTRO:-未安装}"
EOF

    print_success "环境变量配置文件已创建: $env_file"
    print_info "使用方法: source $env_file"
}

# ============================================
# 主流程
# ============================================
main() {
    print_info "=========================================="
    print_info "   UIKA RL SAR 环境配置脚本"
    print_info "   模式: $MODE"
    print_info "=========================================="

    # Step 1: 系统依赖
    check_system_deps || exit 1

    # Step 2: ROS 2 (如果不是cmake模式)
    if [ "$MODE" != "cmake" ]; then
        install_ros2 || print_warning "ROS 2未安装，某些功能可能不可用"
    fi

    # Step 3: Conda环境
    setup_conda_env

    # Step 4: 推理运行时
    setup_inference_runtime

    # Step 5: 机器人描述
    setup_robot_descriptions

    # Step 6: 构建项目
    build_project

    # Step 7: 环境变量
    setup_env_vars

    print_info "=========================================="
    print_success "环境配置完成!"
    print_info "=========================================="
    print_info ""
    print_info "下一步:"
    print_info "  1. 运行: source $PROJECT_ROOT/env_uika.sh"
    print_info "  2. 启动节点: ros2 run rl_sar rl_real_uika"
    print_info ""
    print_info "或直接构建ROS包:"
    print_info "  ./build.sh"
    print_info ""
}

main
