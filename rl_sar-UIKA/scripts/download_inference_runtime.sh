#!/bin/bash
set -e

# Simplified inference runtime setup for UIKA
# UIKA uses ONNX Runtime only, no LibTorch needed

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
INFER_DIR="${PROJECT_ROOT}/library/inference_runtime"
ONNX_DIR="${INFER_DIR}/onnxruntime"

echo "[INFO] Checking ONNX Runtime..."

# Check if ONNX already exists
if [ -f "${ONNX_DIR}/lib/libonnxruntime.so" ]; then
    echo "[INFO] ONNX Runtime already installed at ${ONNX_DIR}"
    exit 0
fi

echo "[ERROR] ONNX Runtime not found at ${ONNX_DIR}"
echo "[INFO] Please manually extract onnxruntime-linux-aarch64-*.tgz to ${ONNX_DIR}"
exit 1
