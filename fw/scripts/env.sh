# Puts the workspace from setup_zephyr_ws.sh on the path. Source it, don't run it:
#
#   . fw/scripts/env.sh
#   west build -b native_sim/native/64 fw/app -d build/fw/app

WS=${ZEPHYR_WS:-$HOME/zephyr-ws}

# shellcheck disable=SC1091
. "$WS/.venv/bin/activate"
export ZEPHYR_BASE="$WS/zephyr"

# The NCS install on the same machine registers its own Zephyr CMake package.
# ZEPHYR_BASE wins over that registry, so this build always uses $WS.
echo "zephyr: $ZEPHYR_BASE ($(sed -n 's/^VERSION_\(MAJOR\|MINOR\) = //p; s/^PATCHLEVEL = //p' "$ZEPHYR_BASE/VERSION" | paste -sd.))"
