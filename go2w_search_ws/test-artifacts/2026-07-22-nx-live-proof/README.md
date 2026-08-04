# NX live browser proof — 2026-07-22

This acceptance run used a real headed Chromium against the production NX at
`192.168.1.105`. No motion/search command was sent, and no NX service was
stopped or restarted. A `search_room` task was already active before the run;
the browser only observed it.

## Outcome

- Real feeds and connectivity: PASS. Visible camera, infrared, MID360, map,
  localization, task state, and all displayed services were present.
- At least 60 seconds without refresh: FAIL. The 86.19-second window reproduced
  the stale UI: WebSocket stayed OPEN, but the canvas pose and image surfaces
  stopped following the HTTP-polled authoritative pose.
- Browser-only disconnect/reconnect: PASS with a degraded transition. The page
  became Chrome's offline error document; after network restoration it returned
  to the panel automatically within 10 seconds and rehydrated camera/map/task
  state. No explicit reload/goto was issued after going online.
- Current calibrated camera frustum: FAIL because it is not deployed. The NX
  `map.js` lacks `_cameraFrustumGeometry` and the dirty-frame scheduler that are
  present in this checkout. The screenshot records the current production map
  surface, but it does not contain the new calibrated frustum contract.

Machine-readable details are in `summary.json`.

## Exact commands

Prerequisite and link checks:

```powershell
Get-NetIPAddress -AddressFamily IPv4 | Where-Object {$_.IPAddress -eq '192.168.1.102'}
Test-NetConnection 192.168.1.105 -Port 8000 -InformationLevel Detailed
Test-NetConnection 192.168.1.105 -Port 8001 -InformationLevel Detailed
Invoke-WebRequest -UseBasicParsing -Uri 'http://192.168.1.105:8000/api/status' -TimeoutSec 10
```

Real headed browser (the production panel is served at `/`, not
`/panel.html`):

```powershell
& 'C:\Program Files\nodejs\npx.cmd' --yes --package @playwright/cli playwright-cli -s=nxlive open http://192.168.1.105:8000/ --headed
& 'C:\Program Files\nodejs\npx.cmd' --yes --package @playwright/cli playwright-cli -s=nxlive snapshot
& 'C:\Program Files\nodejs\npx.cmd' --yes --package @playwright/cli playwright-cli -s=nxlive resize 1600 1000
```

The browser page was instrumented at runtime only. `map.update`, `map._draw`,
and image `load` events were counted, while one-second samples recorded the
map pose, header pose, WebSocket state/buffered amount, FPS badge, and event-loop
lag. The production source and NX filesystem were not changed.

Browser-only network interruption:

```powershell
& 'C:\Program Files\nodejs\npx.cmd' --yes --package @playwright/cli playwright-cli -s=nxlive network-state-set offline
# observe five seconds; Chrome showed ERR_INTERNET_DISCONNECTED
& 'C:\Program Files\nodejs\npx.cmd' --yes --package @playwright/cli playwright-cli -s=nxlive network-state-set online
# wait ten seconds; the panel returned automatically, with no explicit goto/reload
```

Production/local asset comparison was read-only:

```powershell
$remoteMap=(Invoke-WebRequest -UseBasicParsing 'http://192.168.1.105:8000/map.js').Content
$remotePanel=(Invoke-WebRequest -UseBasicParsing 'http://192.168.1.105:8000/').Content
$remoteMap.Contains('_cameraFrustumGeometry')
$remoteMap.Contains('_scheduleFrame')
$remotePanel.Contains('createSocketLifecycle')
$remotePanel.Contains('createStatusPoller')
Get-FileHash 'go2w_search_ws\web\static\map.js' -Algorithm SHA256
```

## Limitations

- The deployed `/api/status` does not expose the new reliable/latest-value WebSocket
  queue telemetry. Browser `ws.bufferedAmount` was zero, but that measures the
  browser's outgoing buffer, not the server-to-browser queue. The stale surfaces,
  low update count, pose divergence, and redraw storm are direct browser symptoms;
  they do not by themselves identify which server queue was responsible.
- `timer_lag_max_ms_upper_bound` includes a conservative instrumentation-start
  offset, so it is retained as an upper bound rather than a clean percentile.
- The active mission was not started by this run. Its changing state provided
  real live data but also means the scene/workload was not controlled.
- A second browser document rehydration was observed after recovery; transient
  runtime probes do not survive such document replacement.
