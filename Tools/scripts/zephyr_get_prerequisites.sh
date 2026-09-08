#!/usr/bin/env bash

set -euo pipefail

if [[ ! -d modules || ! -d Tools/scripts ]]; then
	echo "This script must be run from the root of the ArduPilot repository."
	exit 1
fi

repo_root="$PWD"
zephyr_dir="$repo_root/modules/zephyr"
zephyr_url_default="https://github.com/zephyrproject-rtos/zephyr.git"
zephyr_url="${ZEPHYR_SUBMODULE_URL:-$zephyr_url_default}"
zephyr_ref="${ZEPHYR_REF:-main}"
zephyr_manifest_map_default="$repo_root/Tools/scripts/zephyr_manifest_v4_4_0_map.tsv"
zephyr_manifest_map="${ZEPHYR_MANIFEST_MAP:-$zephyr_manifest_map_default}"

echo "Zephyr prerequisite installer"
echo

if command -v sudo >/dev/null 2>&1; then
	sudo_cmd=(sudo)
else
	sudo_cmd=()
fi

is_zephyr_submodule_registered() {
	git config -f .gitmodules --get-regexp '^submodule\..*\.path$' 2>/dev/null | awk '{print $2}' | grep -qx 'modules/zephyr'
}

is_zephyr_checkout_usable() {
	[[ -f "$zephyr_dir/scripts/requirements-base.txt" || -f "$zephyr_dir/scripts/requirements.txt" ]]
}

is_directory_empty() {
	local dir="$1"
	[[ -d "$dir" ]] || return 1
	[[ -z "$(ls -A "$dir")" ]]
}

clone_zephyr_checkout() {
	echo "Cloning Zephyr into modules/zephyr from $zephyr_url (ref: $zephyr_ref)"
	rm -rf "$zephyr_dir"
	git clone --branch "$zephyr_ref" --single-branch "$zephyr_url" "$zephyr_dir"
}

zephyr_checkout_needs_repair() {
	local required=(
		"include/zephyr/dt-bindings/gpio/gpio.h"
		"include/zephyr/dt-bindings/i2c/i2c.h"
		"include/zephyr/dt-bindings/dma/stm32_dma.h"
	)

	for rel in "${required[@]}"; do
		if [[ ! -f "$zephyr_dir/$rel" ]]; then
			return 0
		fi
	done

	if git -C "$zephyr_dir" status --porcelain -- include/zephyr/dt-bindings 2>/dev/null | grep -q '^ D '; then
		return 0
	fi

	return 1
}

repair_zephyr_checkout_if_needed() {
	if [[ ! -e "$zephyr_dir/.git" ]]; then
		return
	fi

	if ! zephyr_checkout_needs_repair; then
		return
	fi

	echo "Detected inconsistent modules/zephyr checkout (missing/deleted dt-bindings)."
	echo "Repairing Zephyr checkout with hard reset and clean..."
	git -C "$zephyr_dir" reset --hard HEAD
	git -C "$zephyr_dir" clean -ffd
	if is_zephyr_submodule_registered; then
		git submodule update --init --recursive modules/zephyr
	fi
}

# NOTE: this script deliberately does NOT touch .gitmodules. The single
# source of truth for the Zephyr dependency set (name/url/SHA/path) is
# the manifest map TSV; sync_generated_zephyr_submodules() below
# materializes the checkouts directly from it.

sync_generated_zephyr_submodules() {
	if [[ ! -f "$zephyr_manifest_map" ]]; then
		echo "Zephyr manifest map not found at $zephyr_manifest_map"
		echo "Skipping generated Zephyr submodule initialization/sync."
		return
	fi

	echo "Ensuring generated Zephyr submodules are initialized and synced"

	local name
	local url
	local sha
	local rel_path
	local checkout_path
	local initialized_count=0
	local synced_count=0

	while IFS=$'\t' read -r name url sha rel_path; do
		[[ -z "$name" ]] && continue
		if [[ "$rel_path" == "zephyr" ]]; then
			echo "Refusing to sync $name into reserved Zephyr source path: $rel_path"
			echo "Fix $zephyr_manifest_map before rerunning this script."
			return 1
		fi
		checkout_path="$zephyr_dir/$rel_path"
		mkdir -p "$(dirname "$checkout_path")"

		if [[ -d "$checkout_path/.git" ]]; then
			echo "sync: $checkout_path"
			if git -C "$checkout_path" remote get-url origin >/dev/null 2>&1; then
				git -C "$checkout_path" remote set-url origin "$url"
			else
				git -C "$checkout_path" remote add origin "$url"
			fi
			git -C "$checkout_path" fetch --tags --force --prune origin
			synced_count=$((synced_count + 1))
		else
			echo "init: $checkout_path"
			rm -rf "$checkout_path"
			git clone --no-checkout "$url" "$checkout_path"
			git -C "$checkout_path" fetch --tags --force --prune origin
			initialized_count=$((initialized_count + 1))
		fi

		if ! git -C "$checkout_path" cat-file -e "${sha}^{commit}" 2>/dev/null; then
			git -C "$checkout_path" fetch origin "$sha"
		fi

		git -C "$checkout_path" checkout --detach "$sha"
	done <"$zephyr_manifest_map"

	echo "Generated Zephyr submodule status: initialized=$initialized_count synced=$synced_count"

	# Keep the generated checkouts out of the zephyr repo's git status:
	# they are untracked embedded repos by design (see the manifest TSV).
	local exclude_file
	exclude_file="$(git -C "$zephyr_dir" rev-parse --absolute-git-dir)/info/exclude"
	if ! grep -q "TSV-generated dependency checkouts" "$exclude_file" 2>/dev/null; then
		{
			echo "# ArduPilot: TSV-generated dependency checkouts (see $(basename "$zephyr_manifest_map"))"
			awk -F '\t' '{split($4,a,"/"); print "/"a[1]"/"}' "$zephyr_manifest_map" | sort -u
		} >>"$exclude_file"
		echo "Added generated-checkout patterns to $exclude_file"
	fi
}

# hal_espressif gitignores zephyr/blobs/lib: the ESP32 WiFi/BT libraries are
# meant to be fetched, not vendored. Upstream uses `west blobs fetch`; we do
# not need west for it, because module.yml already pins a URL and a sha256 for
# every blob. Only the SoCs we build for are fetched.
espressif_blob_socs="${ZEPHYR_ESPRESSIF_BLOB_SOCS:-esp32s3}"

fetch_espressif_blobs() {
	local mod="$zephyr_dir/modules/hal/espressif/zephyr" path sha url dest have=0 got=0 bad=0
	[[ -f "$mod/module.yml" ]] || { echo "hal_espressif absent; skipping RF blobs."; return 0; }

	while read -r path sha url; do
		dest="$mod/blobs/$path"
		if [[ -f "$dest" ]] && echo "$sha  $dest" | sha256sum -c --status 2>/dev/null; then
			have=$((have + 1)); continue
		fi
		echo "  fetching $path"
		mkdir -p "$(dirname "$dest")"
		if wget -qO "$dest.part" "$url" && echo "$sha  $dest.part" | sha256sum -c --status 2>/dev/null; then
			mv "$dest.part" "$dest"; got=$((got + 1))
		else
			rm -f "$dest.part"; bad=$((bad + 1)); echo "  FAILED (download or sha256): $path" >&2
		fi
	done < <(awk '/^blobs:/ {b=1}
			b && /^[[:space:]]*-[[:space:]]*path:/ {p=$NF}
			b && /^[[:space:]]*sha256:/ {s=$NF}
			b && /^[[:space:]]*url:/ {print p, s, $NF}' \
		"$mod/module.yml" | grep -E "lib/($espressif_blob_socs)/")

	echo "Espressif RF blobs: $have present, $got fetched, $bad failed"
	[[ $bad -eq 0 ]] || echo "  ESP32S3Zephyr will not link; other boards are unaffected." >&2
}

update_ardupilot_submodules() {
	echo "Updating ArduPilot submodules..."

	# Use only real gitlinks tracked by the superproject index. This avoids
	# trying to update generated Zephyr nested entries in .gitmodules that are
	# intentionally not superproject submodules.
	local zephyr_registered=0
	if is_zephyr_submodule_registered; then
		zephyr_registered=1
	fi

	local -a paths=()
	while IFS= read -r path; do
		[[ -z "$path" ]] && continue
		if [[ "$path" == "modules/zephyr" && $zephyr_registered -ne 1 ]]; then
			continue
		fi
		paths+=("$path")
	done < <(git ls-files --stage | awk '$1 == 160000 {print $4}')

	if [[ ${#paths[@]} -eq 0 ]]; then
		echo "No tracked submodules found in git index."
		return
	fi

	for path in "${paths[@]}"; do
		git submodule sync -- "$path" >/dev/null || true
	done

	git submodule update --init --recursive "${paths[@]}"
}

ensure_zephyr_source_tree() {
	if is_directory_empty "$zephyr_dir"; then
		echo "Found empty modules/zephyr directory. Bootstrapping Zephyr source tree..."
	fi

	if is_zephyr_submodule_registered; then
		echo "Zephyr is configured as a git submodule. Initializing modules/zephyr..."
		# If modules/zephyr exists but is empty/non-git, remove it so submodule
		# update can recreate a proper git checkout. Note: in submodules .git can
		# be either a directory or a gitdir redirection file.
		if [[ -d "$zephyr_dir" && ! -e "$zephyr_dir/.git" ]]; then
			rm -rf "$zephyr_dir"
		fi
		git submodule sync -- modules/zephyr >/dev/null
		git submodule update --init --recursive modules/zephyr
		return
	fi

	echo "modules/zephyr is not configured as a git submodule in .gitmodules."

	if [[ -d "$zephyr_dir" ]]; then
		if is_zephyr_checkout_usable; then
			echo "Existing modules/zephyr checkout found and appears usable."
			return
		fi
		echo "Existing modules/zephyr directory is incomplete. Replacing with a fresh clone."
	fi

	clone_zephyr_checkout
}

install_with_apt() {
	local packages=(
		cmake
		ninja-build
		ccache
		gperf
		device-tree-compiler
		wget
		xz-utils
		file
		make
		gcc
		g++
		gcc-arm-none-eabi
		binutils-arm-none-eabi
		libnewlib-arm-none-eabi
		python3-dev
		python3-pip
		python3-setuptools
		python3-wheel
		python3-venv
	)

	echo "Installing host packages with apt..."
	"${sudo_cmd[@]}" apt-get update
	"${sudo_cmd[@]}" apt-get install -y "${packages[@]}"
}

ensure_west_available() {
	if command -v west >/dev/null 2>&1; then
		return 0
	fi

	local user_bin
	user_bin="$(python3 -m site --user-base)/bin"
	if [[ -x "$user_bin/west" ]]; then
		export PATH="$user_bin:$PATH"
	fi

	command -v west >/dev/null 2>&1
}

ensure_zephyr_sdk_installed() {
	local sdk_dir="$HOME/zephyr-sdk-1.0.1"

	if [[ -d "$sdk_dir" ]]; then
		echo "Zephyr SDK already present at $sdk_dir"
		return 0
	fi

	echo "Zephyr SDK not found at $sdk_dir"
	echo "Installing Zephyr SDK with west..."
	(
		cd "$zephyr_dir"
		west sdk install -d "$sdk_dir"
	)
}

if command -v apt-get >/dev/null 2>&1; then
	install_with_apt
else
	echo "No supported package manager automation is available in this script."
	echo "Please install at least: cmake ninja-build ccache gperf device-tree-compiler python3-pip"
fi

echo
echo "Verifying generated Zephyr entries in .gitmodules..."

echo
update_ardupilot_submodules

echo
echo "Ensuring Zephyr source tree is present..."
ensure_zephyr_source_tree

# Heal partial/corrupted Zephyr checkouts (for example, missing dt-binding
# headers after interrupted updates or accidental deletions).
repair_zephyr_checkout_if_needed

echo
sync_generated_zephyr_submodules

echo
fetch_espressif_blobs

if [[ ! -d "$zephyr_dir" ]]; then
	echo
	echo "modules/zephyr was not found."
	echo "Failed to populate modules/zephyr."
	exit 0
fi

requirements_file=""
for candidate in \
	"$zephyr_dir/scripts/requirements-base.txt" \
	"$zephyr_dir/scripts/requirements.txt"; do
	if [[ -f "$candidate" ]]; then
		requirements_file="$candidate"
		break
	fi
done

if [[ -n "$requirements_file" ]]; then
	echo
	echo "Installing Python dependencies from $requirements_file"
	python3 -m pip install --user -r "$requirements_file"
else
	echo
	echo "No Zephyr Python requirements file found under modules/zephyr/scripts/."
fi

echo
if ensure_west_available; then
	echo "west is available at $(command -v west)"
	ensure_zephyr_sdk_installed
else
	echo "west is not available on PATH after Python dependency installation."
	echo "Skipping automatic Zephyr SDK installation."
fi

# ── udev rules: named serial symlinks for the RT1176 composite USB device ──
# The firmware labels its CDC-ACM interfaces ("MAVLink"/"SMP" USB interface
# string descriptors); this rules file matches those to create
# /dev/serial/by-ap/mavlink and /dev/serial/by-ap/smp, sparing tooling from
# decoding the -if00/-if02 interface numbers. Idempotent: only copies when
# missing or changed.
udev_rules_src="$repo_root/Tools/scripts/61-ardupilot-rt1176.rules"
udev_rules_dst="/etc/udev/rules.d/61-ardupilot-rt1176.rules"
if [[ -f "$udev_rules_src" && -d /etc/udev/rules.d ]]; then
	if ! cmp -s "$udev_rules_src" "$udev_rules_dst" 2>/dev/null; then
		echo "Installing udev rules to $udev_rules_dst ..."
		"${sudo_cmd[@]}" cp "$udev_rules_src" "$udev_rules_dst"
		"${sudo_cmd[@]}" udevadm control --reload || true
		"${sudo_cmd[@]}" udevadm trigger --subsystem-match=tty || true
	else
		echo "udev rules already installed ($udev_rules_dst)."
	fi
fi

echo
echo "West module sync is intentionally disabled in this repository workflow."
echo "Zephyr dependencies must be pinned and managed via git/submodules."
echo "If Zephyr configure reports missing modules, add the required repos as submodules."

echo
echo "Zephyr prerequisites complete."
echo "If this is a fresh shell, ensure ~/.local/bin is on your PATH for user-installed Python tools."

echo "possible next commands might be:"
echo "ls libraries/AP_HAL_Zephyr/README*"
echo "./waf configure --board=mr_vmu_rt1176 --debug"
echo "./waf copter -j12"
