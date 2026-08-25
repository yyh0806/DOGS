#!/bin/bash
# ============================================================
# deploy_nx.sh — retired 兼容入口 (转发到原子发布链)
# 生产部署唯一入口: build_release.sh + deploy_release.sh (内容寻址 + 原子切换)
# 详见 docs/NX_REDEPLOY.md
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "deploy_nx.sh is retired; forwarding to the atomic motion release flow" >&2
artifact="$("$SCRIPT_DIR/build_release.sh" motion)"
exec "$SCRIPT_DIR/deploy_release.sh" "$artifact" "$@"

# LEGACY IMPLEMENTATION BELOW IS UNREACHABLE
# (历史直接 scp 部署实现已删除; 保留此标记供发布门禁识别 wrapper 边界)
