from pathlib import Path

from setuptools import find_packages, setup

ROOT = Path(__file__).parent
README = ROOT / "README.md"

legged_gym_packages = find_packages(include=["legged_gym", "legged_gym.*"])
rsl_rl_packages = find_packages(where="rsl_rl", include=["rsl_rl", "rsl_rl.*"])

setup(
    name="htdw4438-himloco",
    version="1.0.0",
    author="HTDW4438_HIMloco Contributors",
    author_email="",
    license="BSD-3-Clause",
    description="RL training and deployment framework for the HTDW4438 quadruped",
    long_description=README.read_text(encoding="utf-8") if README.exists() else "",
    long_description_content_type="text/markdown",
    python_requires=">=3.8,<3.11",
    packages=legged_gym_packages + rsl_rl_packages,
    package_dir={"rsl_rl": "rsl_rl/rsl_rl"},
    install_requires=[
        # Keep torch/isaacgym out of pip dependencies due CUDA-specific install channels.
        "matplotlib>=3.5",
        "mujoco==3.2.3",
        "numpy>=1.20,<2.0",
        "onnxruntime>=1.14",
        "pygame>=2.0",
        "pynput>=1.7",
        "pyyaml>=6.0",
        "scipy>=1.9",
        "tensorboard>=2.10",
        "tqdm>=4.60",
        "trimesh>=3.9",
    ],
)
