#!/usr/bin/env bash
#
# build-cannon.sh - build android_kernel_xiaomi_cannon without the full
# LineageOS build, using the LineageOS 20 prebuilt kernel toolchain, and
# package the DT images into an AnyKernel3 flashable zip.
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
#   -P, --package       build, then create dtb.img/dtbo.img + AK3 zip
#       --package-only  only package the existing out/ artifacts (no build)
#       --ak3-dir DIR   AnyKernel3 template (default: prebuilts/AnyKernel3)
#       --zip-name NAME AK3 zip name in OUT_DIR (default: cannon-ak3.zip)
#       --dtbo-dir DIR  dtbo certs + append_certs.py (default: ./dtbo)
#       --no-dtbo-sign  leave dtbo.img raw (no SBC certs / AVB footer)
#       --check         only verify the toolchain, then stop
#   -h, --help          show this help
#
# Targets default to "Image.gz dtbs" (dtbs produces mt6853.dtb + cannon.dtbo).
# Examples:
#   ./build-cannon.sh --check
#   ./build-cannon.sh -c
#   ./build-cannon.sh dtbs
#   ./build-cannon.sh -P                 # build + dtb.img/dtbo.img + zip
#   ./build-cannon.sh --package-only     # repackage existing output
#
# Environment overrides: LINEAGE_ROOT, KERNEL_DIR, OUT_DIR, JOBS, DEFCONFIG,
# CLANG_VER, AK3_DIR, AK3_ZIP, MKDTIMG, EXTRA_MAKE_ARGS.
#
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"

KERNEL_DIR="${KERNEL_DIR:-$SCRIPT_DIR}"
OUT_DIR="${OUT_DIR:-$SCRIPT_DIR/out}"
JOBS="${JOBS:-$(nproc)}"
DEFCONFIG="${DEFCONFIG:-cannon_defconfig}"
CLANG_VER="${CLANG_VER:-clang-r450784d}"
AK3_DIR="${AK3_DIR:-$SCRIPT_DIR/prebuilts/AnyKernel3}"
AK3_ZIP="${AK3_ZIP:-$OUT_DIR/cannon-ak3.zip}"
DTBO_DIR="${DTBO_DIR:-$SCRIPT_DIR/dtbo}"
DTBO_CERT_DIR="${DTBO_CERT_DIR:-$DTBO_DIR/security}"
DTBO_APPEND="${DTBO_APPEND:-$DTBO_DIR/append_certs.py}"
DTBO_PARTITION_SIZE="${DTBO_PARTITION_SIZE:-33554432}"
DTBO_SIGN="${DTBO_SIGN:-1}"

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

# mkdtimg (native host tool) for the DTBO containers.
if [ -z "${MKDTIMG:-}" ]; then
	for f in \
		"$SCRIPT_DIR/prebuilts/kernel-build-tools/bin/mkdtimg" \
		"$PREBUILTS/kernel-build-tools/linux-x86/bin/mkdtimg"
	do
		if [ -x "$f" ]; then
			MKDTIMG="$f"
			break
		fi
	done
fi

# avbtool for the dtbo hash footer (matches the Lineage-built dtbo.img,
# which is the prebuilt dtbo + a 32 MiB AVB hash footer).
if [ -z "${AVBTOOL:-}" ]; then
	for f in \
		"$SCRIPT_DIR/prebuilts/kernel-build-tools/linux-x86/bin/avbtool" \
		"$PREBUILTS/kernel-build-tools/linux-x86/bin/avbtool" \
		"${LINEAGE_ROOT:-/home/link/LineageOS}/out/host/linux-x86/bin/avbtool" \
		"${LINEAGE_ROOT:-/home/link/LineageOS}/external/avb/avbtool"
	do
		if [ -x "$f" ]; then
			AVBTOOL="$f"
			break
		fi
	done
fi
if [ -z "${AVBTOOL:-}" ] && command -v avbtool >/dev/null 2>&1; then
	AVBTOOL="$(command -v avbtool)"
fi

usage() {
	awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "${BASH_SOURCE[0]}"
}

CHECK_ONLY=0
CONFIG_ONLY=0
CLEAN=0
RECONFIG=0
PACKAGE=0
PACKAGE_ONLY=0

while [ $# -gt 0 ]; do
	case "$1" in
	-k|--kernel)	KERNEL_DIR="$2"; shift 2 ;;
	-o|--out)	OUT_DIR="$2"; shift 2 ;;
	-j|--jobs)	JOBS="$2"; shift 2 ;;
	-c|--config)	CONFIG_ONLY=1; shift ;;
	-r|--reconfig)	RECONFIG=1; shift ;;
	-C|--clean)	CLEAN=1; shift ;;
	-P|--package)	PACKAGE=1; shift ;;
	--package-only)	PACKAGE_ONLY=1; shift ;;
	--ak3-dir)	AK3_DIR="$2"; shift 2 ;;
	--zip-name)	AK3_ZIP="$OUT_DIR/$2"; shift 2 ;;
	--dtbo-dir)	DTBO_DIR="$2"; DTBO_CERT_DIR="$DTBO_DIR/security"; DTBO_APPEND="$DTBO_DIR/append_certs.py"; shift 2 ;;
	--no-dtbo-sign)	DTBO_SIGN=0; shift ;;
	--check)	CHECK_ONLY=1; shift ;;
	-h|--help)	usage; exit 0 ;;
	--)		shift; break ;;
	-*)		echo "unknown option: $1" >&2; usage; exit 1 ;;
	*)		break ;;
	esac
done

if [ "$PACKAGE_ONLY" -eq 0 ]; then
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
fi

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

package_ak3() {
	local image="$OUT_DIR/arch/arm64/boot/Image.gz"
	local dtb="$OUT_DIR/arch/arm64/boot/dts/mediatek/mt6853.dtb"
	local dtbo="$OUT_DIR/arch/arm64/boot/dts/mediatek/cannon.dtbo"
	local work="$OUT_DIR/ak3"

	for f in "$image" "$dtb" "$dtbo"; do
		if [ ! -f "$f" ]; then
			echo "missing $f (build 'Image.gz dtbs' first)" >&2
			exit 1
		fi
	done
	if [ -z "${MKDTIMG:-}" ] || [ ! -x "$MKDTIMG" ]; then
		echo "mkdtimg not found (expected in prebuilts/kernel-build-tools/bin)" >&2
		exit 1
	fi
	if [ ! -d "$AK3_DIR" ]; then
		echo "AnyKernel3 template not found: $AK3_DIR" >&2
		echo "  git clone --depth 1 https://github.com/osm0sis/AnyKernel3.git $AK3_DIR" >&2
		exit 1
	fi
	if ! command -v zip >/dev/null 2>&1; then
		echo "zip not found" >&2
		exit 1
	fi

	echo "==> dtb.img / dtbo.img"
	"$MKDTIMG" create "$OUT_DIR/dtb.img" --page_size=2048 "$dtb"
	"$MKDTIMG" create "$OUT_DIR/dtbo.img" --page_size=2048 "$dtbo"

	if [ "$DTBO_SIGN" -eq 1 ]; then
		local cert1="$DTBO_CERT_DIR/cert1.der"
		local cert2="$DTBO_CERT_DIR/cert2.der"
		if [ -f "$DTBO_APPEND" ] && [ -f "$cert1" ] && [ -f "$cert2" ]; then
			echo "==> dtbo SBC certs (cert1.der / cert2.der)"
			python3 "$DTBO_APPEND" --alignment 16 \
				--cert1 "$cert1" --cert2 "$cert2" \
				--dtbo "$OUT_DIR/dtbo.img"
		else
			echo "warning: dtbo certs or append script missing; dtbo stays unsigned" >&2
			echo "  script: $DTBO_APPEND" >&2
			echo "  certs : $DTBO_CERT_DIR" >&2
		fi
		if [ -n "${AVBTOOL:-}" ] && [ -x "$AVBTOOL" ]; then
			echo "==> dtbo AVB hash footer (partition size $DTBO_PARTITION_SIZE)"
			"$AVBTOOL" add_hash_footer \
				--image "$OUT_DIR/dtbo.img" \
				--partition_name dtbo \
				--partition_size "$DTBO_PARTITION_SIZE" \
				--hash_algorithm sha256 \
				--algorithm NONE \
				--prop "com.android.build.dtbo.fingerprint:Redmi/cannong_global/cannong:12/SP1A.210812.016/V13.0.3.0.SJEMIXM:user/release-keys"
		else
			echo "warning: avbtool not found; dtbo has SBC certs but no AVB footer" >&2
		fi
	fi

	echo "==> AnyKernel3 zip"
	rm -rf "$work"
	mkdir -p "$work"
	cp -a "$AK3_DIR/." "$work/"
	rm -rf "$work/.git"
	cp -f "$image" "$work/Image.gz"
	cp -f "$OUT_DIR/dtb.img" "$work/dtb.img"
	cp -f "$OUT_DIR/dtbo.img" "$work/dtbo.img"

	cat > "$work/anykernel.sh" <<'AK3'
### AnyKernel3 Ramdisk Mod Script
# Xiaomi Redmi Note 9 5G / 9T (cannon) — kernel + base DTB + dtbo

properties() { '
kernel.string=Cannon kernel (android_kernel_xiaomi_cannon)
do.devicecheck=1
do.modules=0
do.systemless=0
do.cleanup=1
do.cleanuponabort=0
device.name1=cannon
device.name2=cannong
supported.versions=
supported.patchlevels=
supported.vendorpatchlevels=
'; } # end properties

## boot files attributes
boot_attributes() {
set_perm_recursive 0 0 755 644 $RAMDISK/*;
set_perm_recursive 0 0 750 750 $RAMDISK/init* $RAMDISK/sbin;
} # end attributes

# boot shell variables
BLOCK=/dev/block/by-name/boot;
IS_SLOT_DEVICE=0;
RAMDISK_COMPRESSION=auto;
PATCH_VBMETA_FLAG=auto;

. tools/ak3-core.sh;

dump_boot;
write_boot;
AK3

	rm -f "$AK3_ZIP"
	( cd "$work" && zip -qr -9 "$AK3_ZIP" . )

	echo
	echo "==> packaged"
	ls -l "$OUT_DIR/dtb.img" "$OUT_DIR/dtbo.img" "$AK3_ZIP"
}

if [ "$CHECK_ONLY" -eq 1 ]; then
	echo "PREBUILTS    : $PREBUILTS"
	echo "KERNEL_DIR   : $KERNEL_DIR"
	echo "OUT_DIR      : $OUT_DIR"
	echo "clang        : $("$CLANG_DIR/bin/clang" --version | head -1)"
	echo "aarch64 gcc  : $("$GCC64/aarch64-linux-android-gcc" --version | head -1)"
	echo "arm gcc      : $("$GCC32/arm-linux-androidkernel-gcc" --version | head -1)"
	echo "mkdtimg      : ${MKDTIMG:-MISSING}"
	echo "avbtool      : ${AVBTOOL:-MISSING}"
	echo "DTBO_DIR     : $DTBO_DIR"
	echo "dtbo certs   : $DTBO_CERT_DIR"
	echo "dtbo append  : $DTBO_APPEND"
	echo "dtbo sign    : $DTBO_SIGN (partition size $DTBO_PARTITION_SIZE)"
	echo "AnyKernel3   : $AK3_DIR"
	echo "jobs         : $JOBS"
	exit 0
fi

if [ "$PACKAGE_ONLY" -eq 1 ]; then
	package_ak3
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

if [ "$PACKAGE" -eq 1 ]; then
	package_ak3
fi

echo
echo "==> outputs"
for f in \
	"$OUT_DIR/arch/arm64/boot/Image.gz" \
	"$OUT_DIR/arch/arm64/boot/Image.gz-dtb" \
	"$OUT_DIR/arch/arm64/boot/dts/mediatek/mt6853.dtb" \
	"$OUT_DIR/arch/arm64/boot/dts/mediatek/cannon.dtbo" \
	"$OUT_DIR/dtb.img" \
	"$OUT_DIR/dtbo.img" \
	"$AK3_ZIP"
do
	if [ -e "$f" ]; then
		ls -l "$f"
	fi
done
