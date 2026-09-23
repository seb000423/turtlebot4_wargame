#!/bin/bash
# One-time system setup for using Iriun Webcam (phone camera) as a Linux video source
# for this project. Needs sudo — run this yourself in a real terminal:
#
#   bash setup_iriun.sh
#
set -euo pipefail

DEB_URL="https://iriun.gitlab.io/iriunwebcam-2.9.1.deb"
DEB_PATH="/tmp/iriunwebcam.deb"
V4L2LOOPBACK_SRC="/usr/src/v4l2loopback-0.12.7/v4l2loopback.c"

echo "== 1/4: installing v4l-utils + v4l2loopback-dkms =="
sudo apt update
# Don't fail the script if the dkms build errors here — the Ubuntu jammy package
# (0.12.7) fails to compile against kernel 6.8+ because it uses the removed
# strlcpy() kernel function. We patch it below and retry.
sudo apt install -y v4l-utils v4l2loopback-dkms "linux-headers-$(uname -r)" || true

if [ -f "$V4L2LOOPBACK_SRC" ] && grep -q "strlcpy" "$V4L2LOOPBACK_SRC"; then
    echo "== 2/4: patching v4l2loopback source for kernel 6.8+ (strlcpy -> strscpy) =="
    sudo sed -i 's/strlcpy(/strscpy(/g' "$V4L2LOOPBACK_SRC"
    sudo dkms remove v4l2loopback/0.12.7 --all 2>/dev/null || true
    sudo dkms install v4l2loopback/0.12.7
    sudo depmod -a
    # package was left half-configured by the earlier failed build; finish that now
    sudo apt install -f -y
else
    echo "== 2/4: v4l2loopback source already OK, skipping patch =="
fi

echo "== 3/4: downloading + installing Iriun Webcam client =="
curl -sL -o "$DEB_PATH" "$DEB_URL"
sudo apt install -y "$DEB_PATH"

echo "== 4/4: verifying =="
sudo modprobe v4l2loopback exclusive_caps=1 card_label="Iriun Webcam" 2>&1 || true
lsmod | grep -i v4l2loopback && echo "v4l2loopback loaded OK" || echo "WARNING: v4l2loopback still not loaded"
v4l2-ctl --list-devices || true

cat <<'EOF'

Done. Next steps:
1. Install the "Iriun Webcam" app on your iPhone (App Store) and connect it to the
   same Wi-Fi network as this PC.
2. Launch the desktop client:  iriunwebcam
3. Open the Iriun Webcam app on the phone — it should connect automatically and
   create a new /dev/videoN device (check with: v4l2-ctl --list-devices).
4. Use that device index with the project scripts, e.g.:
     cd <이 레포 경로>/judge-pc-essentials/cod_portable/cod
     source .venv/bin/activate
     python zero_shot_track.py --camera <N> --prompt "a cup"
EOF
