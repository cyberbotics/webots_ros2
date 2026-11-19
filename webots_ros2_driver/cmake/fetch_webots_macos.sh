#!/usr/bin/env bash
# Download the official Webots macOS release into <CACHE_DIR>/<VERSION>/Webots.app (no-op if present).
#   usage: fetch_webots_macos.sh <VERSION> <CACHE_DIR>
set -euo pipefail
VERSION="${1:?webots version required}"
CACHE="${2:?cache dir required}"
DEST="${CACHE}/${VERSION}"
APP="${DEST}/Webots.app"
LOCK="${DEST}.lock"

ready() { [ -d "${APP}/Contents/lib/controller" ]; }

if ready; then
  echo "fetch_webots_macos: ${APP} already present"; exit 0
fi

MNT=""; STAGE=""; HELD=0
cleanup() {
  if [ -n "${MNT}" ]; then
    hdiutil detach "${MNT}" -quiet 2>/dev/null || true
    rmdir "${MNT}" 2>/dev/null || true
  fi
  [ -n "${STAGE}" ] && rm -rf "${STAGE}"
  [ "${HELD}" = 1 ] && rmdir "${LOCK}" 2>/dev/null
  return 0
}
trap cleanup EXIT

# Serialize concurrent configures (mkdir is atomic; macOS has no flock)
mkdir -p "${CACHE}"
waited=0
until mkdir "${LOCK}" 2>/dev/null; do
  if ready; then
    echo "fetch_webots_macos: ${APP} populated by a concurrent configure"; exit 0
  fi
  waited=$((waited + 1))
  if [ "${waited}" -ge 1800 ]; then
    echo "fetch_webots_macos: timed out waiting for ${LOCK} (remove it if stale)" >&2
    exit 1
  fi
  sleep 1
done
HELD=1
# Re-check under the lock: the previous holder may have just published.
if ready; then
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
echo "fetch_webots_macos: mounting ${DMG}"
hdiutil attach -nobrowse -noverify -quiet -mountpoint "${MNT}" "${DMG}"
# Publish only a complete copy
STAGE="${APP}.staging.$$"
rm -rf "${STAGE}"
echo "fetch_webots_macos: copying Webots.app -> ${STAGE}"
cp -R "${MNT}/Webots.app" "${STAGE}"

[ -d "${STAGE}/Contents/lib/controller" ] || { echo "fetch_webots_macos: extraction failed (no lib/controller)" >&2; exit 1; }
rm -rf "${APP}"
mv "${STAGE}" "${APP}"
STAGE=""
echo "fetch_webots_macos: ready -> ${APP}"
