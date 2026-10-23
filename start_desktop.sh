#!/usr/bin/env bash
# Starts a virtual desktop (display :1) viewable in a browser at
#   http://localhost:6080/vnc.html?autoconnect=true&resize=remote
# resize=remote makes the desktop match the browser window 1:1, so nothing is blurred by scaling.
export DISPLAY=:1

# Open every window maximized (Gazebo, RViz); switch between them via the taskbar.
mkdir -p ~/.fluxbox
cat > ~/.fluxbox/apps <<'EOF'
[app] (name=.*)
  [Maximized] {yes}
[end]
EOF

if ! pgrep -x Xvnc >/dev/null; then
    rm -f /tmp/.X1-lock /tmp/.X11-unix/X1
    setsid Xvnc :1 -geometry 1920x1080 -depth 24 -SecurityTypes None -localhost \
        -rfbport 5900 -AlwaysShared -AcceptSetDesktopSize >/tmp/xvnc.log 2>&1 < /dev/null &
    sleep 1
fi
pgrep -x fluxbox >/dev/null || setsid fluxbox >/tmp/fluxbox.log 2>&1 < /dev/null &
(echo > /dev/tcp/127.0.0.1/6080) 2>/dev/null || \
    setsid websockify --web /usr/share/novnc 6080 localhost:5900 >/tmp/novnc.log 2>&1 < /dev/null &

echo "Desktop running. Open http://localhost:6080/vnc.html?autoconnect=true&resize=remote"
