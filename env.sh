#!/bin/bash

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

conda activate HIMloco
export PYTHONPATH="${REPO_ROOT}:${REPO_ROOT}/rsl_rl:${REPO_ROOT}/../isaacgym/python:${PYTHONPATH}"
cd "${REPO_ROOT}"
