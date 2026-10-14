#!/usr/bin/env bash
# =============================================================================
# genDeb.sh - Build a Debian (.deb) package from a staged install tree.
# 从已安装(staged)的目录生成 Debian(.deb) 安装包。
#
# Usage / 用法:
#   ./genDeb.sh [install_dir] [dst_folder]
#
#   [install_dir]  A staged install tree produced by build.sh, e.g. build/install
#                  containing include/ and lib/ (and optionally bin/).
#                  Default: <script_dir>/build/install
#                  由 build.sh 产生的安装目录，如 build/install，内含 include/、lib/。
#                  默认: 脚本所在目录下的 build/install
#   [dst_folder]   Output directory where the .deb is written (created if absent).
#                  Default: current working directory
#                  .deb 的输出目录（不存在会自动创建）。默认: 当前目录
#
# Env overrides / 环境变量覆盖:
#   PKG_VERSION    Package version (default: parsed from sdk/CMakeLists.txt).
#   PKG_NAME       Package name    (default: odin-sdk).
#   KEEP_BIN=1     Also ship <install_dir>/bin into /usr/bin (default: skip).
#   DEB_DEPENDS    Override the auto-detected runtime Depends line.
#
# Example / 示例:
#   ./build.sh                       # produces build/install
#   ./genDeb.sh build/install ./out  # produces ./out/odin-sdk_3.0.0_amd64.deb
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ----------------------------------------------------------------------------
# Usage help / 用法说明
# ----------------------------------------------------------------------------
print_usage() {
    cat <<EOF
Usage / 用法: $(basename "$0") [install_dir] [dst_folder]
  install_dir   staged install tree (default: ${SCRIPT_DIR}/build/install)
                安装目录 (默认: build/install)
  dst_folder    output dir for the .deb (default: current directory)
                .deb 输出目录 (默认: 当前目录)

Env overrides / 环境变量:
  PKG_VERSION   package version   (default: parsed from sdk/CMakeLists.txt)
  PKG_NAME      package name      (default: odin-sdk)
  KEEP_BIN=1    also ship bin/    (default: skip sample executables)
  DEB_DEPENDS   override runtime Depends line

Examples / 示例:
  ./genDeb.sh                       # build/install -> current dir
  ./genDeb.sh build/install ./out   # explicit paths
  KEEP_BIN=1 ./genDeb.sh            # include sample executables
EOF
}

# ----------------------------------------------------------------------------
# Arguments (both optional)
#   $1 install_dir : defaults to <script_dir>/build/install (produced by build.sh)
#   $2 dst_folder  : defaults to the current working directory
# 两个参数都可省略：install_dir 默认 build/install，dst_folder 默认当前目录。
# ----------------------------------------------------------------------------
if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    print_usage
    exit 0
fi

# When running with defaults (some args omitted), show the usage hint so the
# user is aware of the available options before the build proceeds.
# 使用默认值(省略了参数)运行时，先打印用法提示，让用户知道有哪些可选项。
if [[ $# -lt 2 ]]; then
    print_usage
    echo "----------------------------------------"
    echo "No/partial args given -> using defaults. 未指定完整参数，使用默认值继续。"
    echo ""
fi

IN_SRC="${1:-$SCRIPT_DIR/build/install}"
SRC_DIR="$(cd "$IN_SRC" 2>/dev/null && pwd || true)"
if [[ -z "$SRC_DIR" || ! -d "$SRC_DIR" ]]; then
    echo "Error: install_dir '$IN_SRC' does not exist." >&2
    echo "Hint: run ./build.sh first to produce build/install." >&2
    exit 1
fi

DST_DIR="${2:-$PWD}"
mkdir -p "$DST_DIR"
DST_DIR="$(cd "$DST_DIR" && pwd)"

if [[ ! -d "$SRC_DIR/include" && ! -d "$SRC_DIR/lib" ]]; then
    echo "Error: '$SRC_DIR' has neither include/ nor lib/; is it a valid install tree?" >&2
    exit 1
fi

# ----------------------------------------------------------------------------
# Package metadata
# ----------------------------------------------------------------------------
PKG_NAME="${PKG_NAME:-odin-sdk}"

# Version: env override, else parse from sdk/CMakeLists.txt, else fallback.
if [[ -z "${PKG_VERSION:-}" ]]; then
    PKG_VERSION="$(grep -oP 'set\(ODIN_SDK_VERSION\s+"\K[0-9]+\.[0-9]+\.[0-9]+' \
        "$SCRIPT_DIR/sdk/CMakeLists.txt" 2>/dev/null | head -n1 || true)"
fi
PKG_VERSION="${PKG_VERSION:-0.0.0}"

# Debian architecture (amd64 / arm64 / ...).
PKG_ARCH="$(dpkg --print-architecture 2>/dev/null || echo unknown)"

MAINTAINER="${DEB_MAINTAINER:-support@example.com}"
DESCRIPTION="Odin LiDAR SDK (libodin_sdk + headers)"

echo "========================================"
echo "genDeb"
echo "  Package : ${PKG_NAME}"
echo "  Version : ${PKG_VERSION}"
echo "  Arch    : ${PKG_ARCH}"
echo "  Source  : ${SRC_DIR}"
echo "  Output  : ${DST_DIR}"
echo "========================================"

# ----------------------------------------------------------------------------
# Stage the package root: <install_dir> -> /usr
# ----------------------------------------------------------------------------
PKG_ROOT="$(mktemp -d)"
trap 'rm -rf "$PKG_ROOT"' EXIT

mkdir -p "$PKG_ROOT/usr"
# Copy public headers and libraries (cmake config lives under lib/cmake).
[[ -d "$SRC_DIR/include" ]] && cp -a "$SRC_DIR/include" "$PKG_ROOT/usr/"
[[ -d "$SRC_DIR/lib" ]]     && cp -a "$SRC_DIR/lib"     "$PKG_ROOT/usr/"

# Sample executables under bin/ are skipped by default (system pollution).
if [[ -d "$SRC_DIR/bin" ]]; then
    if [[ "${KEEP_BIN:-0}" == "1" ]]; then
        cp -a "$SRC_DIR/bin" "$PKG_ROOT/usr/"
        echo "  (including bin/ per KEEP_BIN=1)"
    else
        echo "  (skipping bin/ - set KEEP_BIN=1 to include sample executables)"
    fi
fi

# ----------------------------------------------------------------------------
# ld.so.conf.d snippet: the SDK ships libodin_sdk.so under a non-standard
# per-arch path (/usr/lib/<arch>). Register it so ldconfig can locate it.
# SDK 的库位于非标准的 per-arch 路径(/usr/lib/<arch>)，写入 ld.so.conf.d 片段，
# 安装后由 postinst 执行 ldconfig 使动态库可被全局定位。
# ----------------------------------------------------------------------------
LIB_SUBDIR=""
if [[ -d "$PKG_ROOT/usr/lib" ]]; then
    while IFS= read -r so; do
        LIB_SUBDIR="$(basename "$(dirname "$so")")"
        break
    done < <(find "$PKG_ROOT/usr/lib" -name 'libodin_sdk.so*' -type f 2>/dev/null)
fi

if [[ -n "$LIB_SUBDIR" && "$LIB_SUBDIR" != "lib" ]]; then
    mkdir -p "$PKG_ROOT/etc/ld.so.conf.d"
    echo "/usr/lib/${LIB_SUBDIR}" > "$PKG_ROOT/etc/ld.so.conf.d/${PKG_NAME}.conf"
    echo "  ld.so.conf.d -> /usr/lib/${LIB_SUBDIR}"
fi

# ----------------------------------------------------------------------------
# Runtime dependencies: resolve via dpkg-shlibdeps (the canonical tool, which
# also ignores injected/preloaded libraries), else fall back to a static list.
# 运行时依赖：优先用官方 dpkg-shlibdeps 解析(会自动忽略被注入/预加载的库)，
# 失败则退回到静态默认列表。
# ----------------------------------------------------------------------------
if [[ -z "${DEB_DEPENDS:-}" ]]; then
    # dpkg-shlibdeps needs an absolute path to the .so and a debian/control skeleton.
    MAIN_SO="$(find "$PKG_ROOT/usr/lib" -name 'libodin_sdk.so.*.*' -type f 2>/dev/null | head -n1 || true)"
    if [[ -n "$MAIN_SO" ]] && command -v dpkg-shlibdeps >/dev/null 2>&1; then
        MAIN_SO="$(readlink -f "$MAIN_SO")"
        SHLIB_TMP="$(mktemp -d)"
        mkdir -p "$SHLIB_TMP/debian"
        printf 'Source: %s\n\nPackage: %s\nArchitecture: any\nDepends: ${shlibs:Depends}\n' \
            "$PKG_NAME" "$PKG_NAME" > "$SHLIB_TMP/debian/control"
        DEB_DEPENDS="$( (cd "$SHLIB_TMP" && dpkg-shlibdeps -O "$MAIN_SO" 2>/dev/null) \
            | sed -n 's/^shlibs:Depends=//p' )"
        rm -rf "$SHLIB_TMP"
    fi
fi
# Fallback if detection produced nothing.
DEB_DEPENDS="${DEB_DEPENDS:-libc6, libstdc++6, libgcc-s1, libssl3}"
echo "  Depends : ${DEB_DEPENDS}"

# ----------------------------------------------------------------------------
# DEBIAN control files
# ----------------------------------------------------------------------------
mkdir -p "$PKG_ROOT/DEBIAN"

INSTALLED_SIZE="$(du -k -s --exclude=DEBIAN "$PKG_ROOT" | cut -f1)"

cat > "$PKG_ROOT/DEBIAN/control" <<EOF
Package: ${PKG_NAME}
Version: ${PKG_VERSION}
Section: libs
Priority: optional
Architecture: ${PKG_ARCH}
Depends: ${DEB_DEPENDS}
Maintainer: ${MAINTAINER}
Installed-Size: ${INSTALLED_SIZE}
Description: ${DESCRIPTION}
EOF

cat > "$PKG_ROOT/DEBIAN/postinst" <<'EOF'
#!/bin/sh
set -e
if [ "$1" = "configure" ]; then
    ldconfig
fi
EOF

cat > "$PKG_ROOT/DEBIAN/postrm" <<'EOF'
#!/bin/sh
set -e
if [ "$1" = "remove" ]; then
    ldconfig
fi
EOF

chmod 0755 "$PKG_ROOT/DEBIAN/postinst" "$PKG_ROOT/DEBIAN/postrm"

# ----------------------------------------------------------------------------
# Build the .deb
# ----------------------------------------------------------------------------
DEB_FILE="${DST_DIR}/${PKG_NAME}_${PKG_VERSION}_${PKG_ARCH}.deb"
dpkg-deb --root-owner-group --build "$PKG_ROOT" "$DEB_FILE"

echo ""
echo "========================================"
echo "Package created:"
echo "  ${DEB_FILE}"
echo "========================================"
echo "Install with:  sudo dpkg -i ${DEB_FILE}"
echo "Remove  with:  sudo dpkg -r ${PKG_NAME}"
