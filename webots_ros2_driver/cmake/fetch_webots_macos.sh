#!/usr/bin/env bash
# Fetch the official Webots macOS release (a universal, prebuilt Webots.app) and cache it, so
# webots_ros2_driver can link the prebuilt controller/vehicle/window libraries on macOS instead of
# building the stripped in-tree copy from source (which has no proper .app layout on macOS).
#   usage: fetch_webots_macos.sh <VERSION e.g. R2025a> <CACHE_DIR>
# Idempotent: no-op if <CACHE_DIR>/<VERSION>/Webots.app is already present.
set -euo pipefail
VERSION="${1:?webots version required}"
CACHE="${2:?cache dir required}"
DEST="${CACHE}/${VERSION}"
APP="${DEST}/Webots.app"

if [ -d "${APP}/Contents/lib/controller" ]; then
  echo "fetch_webots_macos: ${APP} already present"; exit 0
fi

mkdir -p "${DEST}"
DMG="${DEST}/webots-${VERSION}.dmg"
URL="https://github.com/cyberbotics/webots/releases/download/${VERSION}/webots-${VERSION}.dmg"
if [ ! -f "${DMG}" ]; then
  echo "fetch_webots_macos: downloading ${URL}"
  curl -fSL --retry 3 --retry-delay 5 -o "${DMG}.part" "${URL}"
  mv "${DMG}.part" "${DMG}"
fi

MNT="$(mktemp -d)"
cleanup() { hdiutil detach "${MNT}" -quiet 2>/dev/null || true; rmdir "${MNT}" 2>/dev/null || true; }
trap cleanup EXIT
echo "fetch_webots_macos: mounting ${DMG}"
hdiutil attach -nobrowse -noverify -quiet -mountpoint "${MNT}" "${DMG}"
rm -rf "${APP}"
echo "fetch_webots_macos: copying Webots.app -> ${APP}"
cp -R "${MNT}/Webots.app" "${APP}"

[ -d "${APP}/Contents/lib/controller" ] || { echo "fetch_webots_macos: extraction failed (no lib/controller)"; exit 1; }
echo "fetch_webots_macos: ready -> ${APP}"
