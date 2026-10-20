#!/usr/bin/env bash
#
# build-cannon.sh - build android_kernel_xiaomi_cannon without the full
# LineageOS build, using the LineageOS 20 prebuilt kernel toolchain.
#
# Uses the toolchain vendored under prebuilts/ (gitignored); falls back to
# LINEAGE_ROOT if that directory is missing.
#
# Mirrors vendor/lineage/build/tasks/kernel.mk + BoardConfigKernel.mk:
#   clang-r450784d, gcc 4.9 (aarch64 + arm), CLANG_TRIPLE, CROSS_COMPILE_ARM32.
#
# Usage:
#   ./build-cannon.sh [options] [make-targets...]
#
# Options:
#   -k, --kernel DIR    kernel tree (default: this directory)
#   -o, --out DIR       build output dir (default: ./out)
#   -j, --jobs N        parallel jobs (default: nproc)
#   -c, --config        only run the defconfig, then stop
#   -r, --reconfig      force re-running the defconfig
#   -C, --clean         remove the output dir first
#       --check         only verify the toolchain, then stop
#   -h, --help          show this help
#
# Targets default to "Image.gz dtbs" (dtbs produces mt6853.dtb + cannon.dtbo).
# Examples:
#   ./build-cannon.sh --check
#   ./build-cannon.sh -c
#   ./build-cannon.sh dtbs
#   ./build-cannon.sh Image.gz dtbs
#
# Environment overrides: LINEAGE_ROOT, KERNEL_DIR, OUT_DIR, JOBS, DEFCONFIG,
# CLANG_VER, EXTRA_MAKE_ARGS.
#
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"

KERNEL_DIR="${KERNEL_DIR:-$SCRIPT_DIR}"
OUT_DIR="${OUT_DIR:-$SCRIPT_DIR/out}"
JOBS="${JOBS:-$(nproc)}"
DEFCONFIG="${DEFCONFIG:-cannon_defconfig}"
CLANG_VER="${CLANG_VER:-clang-r450784d}"

# Prefer the toolchain vendored in this tree (prebuilts/, gitignored); fall
# back to a LineageOS checkout.
if [ -d "$SCRIPT_DIR/prebuilts/clang/host/linux-x86/$CLANG_VER" ]; then
	PREBUILTS="$SCRIPT_DIR/prebuilts"
else
	if [ -z "${LINEAGE_ROOT:-}" ]; then
		for d in "$HOME/LineageOS" /home/link/LineageOS; do
			if [ -d "$d/prebuilts/clang/host/linux-x86" ]; then
				LINEAGE_ROOT="$d"
				break
			fi
		done
	fi
	if [ -z "${LINEAGE_ROOT:-}" ]; then
		echo "no in-tree prebuilts/ and LINEAGE_ROOT is not set" >&2
		exit 1
	fi
	PREBUILTS="$LINEAGE_ROOT/prebuilts"
fi

CLANG_DIR="$PREBUILTS/clang/host/linux-x86/$CLANG_VER"
GCC64="$PREBUILTS/gcc/linux-x86/aarch64/aarch64-linux-android-4.9/bin"
GCC32="$PREBUILTS/gcc/linux-x86/arm/arm-linux-androideabi-4.9/bin"
TOOLS_BIN="$PREBUILTS/tools-lineage/linux-x86/bin"
PERL_BASE="$PREBUILTS/tools-lineage/common/perl-base"

usage() {
	awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "${BASH_SOURCE[0]}"
}

CHECK_ONLY=0
CONFIG_ONLY=0
CLEAN=0
RECONFIG=0

while [ $# -gt 0 ]; do
	case "$1" in
	-k|--kernel)	KERNEL_DIR="$2"; shift 2 ;;
	-o|--out)	OUT_DIR="$2"; shift 2 ;;
	-j|--jobs)	JOBS="$2"; shift 2 ;;
	-c|--config)	CONFIG_ONLY=1; shift ;;
	-r|--reconfig)	RECONFIG=1; shift ;;
	-C|--clean)	CLEAN=1; shift ;;
	--check)	CHECK_ONLY=1; shift ;;
	-h|--help)	usage; exit 0 ;;
	--)		shift; break ;;
	-*)		echo "unknown option: $1" >&2; usage; exit 1 ;;
	*)		break ;;
	esac
done

fail=0
for f in \
	"$CLANG_DIR/bin/clang" \
	"$GCC64/aarch64-linux-android-gcc" \
	"$GCC32/arm-linux-androidkernel-gcc"
do
	if [ ! -x "$f" ]; then
		echo "missing toolchain: $f" >&2
		fail=1
	fi
done
[ "$fail" -eq 0 ] || exit 1

if [ ! -f "$KERNEL_DIR/Makefile" ]; then
	echo "not a kernel tree: $KERNEL_DIR" >&2
	exit 1
fi

export PATH="$CLANG_DIR/bin:$GCC64:$GCC32:$TOOLS_BIN:$PATH"
export LD_LIBRARY_PATH="$CLANG_DIR/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PERL5LIB="$PERL_BASE${PERL5LIB:+:$PERL5LIB}"

MAKE_ARGS=(
	-C "$KERNEL_DIR"
	O="$OUT_DIR"
	ARCH=arm64
	CROSS_COMPILE="$GCC64/aarch64-linux-android-"
	CROSS_COMPILE_ARM32="$GCC32/arm-linux-androidkernel-"
	CROSS_COMPILE_COMPAT="$GCC32/arm-linux-androidkernel-"
	CLANG_TRIPLE=aarch64-linux-gnu-
	CC=clang
	CPATH="/usr/include:/usr/include/x86_64-linux-gnu"
	HOSTLDFLAGS="-L/usr/lib/x86_64-linux-gnu -L/usr/lib64 -fuse-ld=lld"
)
# shellcheck disable=SC2206
[ -n "${EXTRA_MAKE_ARGS:-}" ] && MAKE_ARGS+=($EXTRA_MAKE_ARGS)

if [ "$CHECK_ONLY" -eq 1 ]; then
	echo "PREBUILTS    : $PREBUILTS"
	echo "KERNEL_DIR   : $KERNEL_DIR"
	echo "OUT_DIR      : $OUT_DIR"
	echo "clang        : $("$CLANG_DIR/bin/clang" --version | head -1)"
	echo "aarch64 gcc  : $("$GCC64/aarch64-linux-android-gcc" --version | head -1)"
	echo "arm gcc      : $("$GCC32/arm-linux-androidkernel-gcc" --version | head -1)"
	echo "jobs         : $JOBS"
	exit 0
fi

if [ "$CLEAN" -eq 1 ]; then
	echo "==> removing $OUT_DIR"
	rm -rf "$OUT_DIR"
fi

mkdir -p "$OUT_DIR"

if [ "$CONFIG_ONLY" -eq 1 ] || [ "$RECONFIG" -eq 1 ] || [ ! -f "$OUT_DIR/.config" ]; then
	echo "==> $DEFCONFIG"
	make "${MAKE_ARGS[@]}" -j"$JOBS" "$DEFCONFIG"
fi

if [ "$CONFIG_ONLY" -eq 1 ]; then
	exit 0
fi

if [ $# -eq 0 ]; then
	set -- Image.gz dtbs
fi

echo "==> make -j$JOBS $*"
make "${MAKE_ARGS[@]}" -j"$JOBS" "$@"

echo
echo "==> outputs"
for f in \
	"$OUT_DIR/arch/arm64/boot/Image.gz" \
	"$OUT_DIR/arch/arm64/boot/Image.gz-dtb" \
	"$OUT_DIR/arch/arm64/boot/dts/mediatek/mt6853.dtb" \
	"$OUT_DIR/arch/arm64/boot/dts/mediatek/cannon.dtbo"
do
	if [ -e "$f" ]; then
		ls -l "$f"
	fi
done
