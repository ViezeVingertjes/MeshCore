Param(
  [string[]] $Envs = @('LilyGo_T-Echo_repeater', 'LilyGo_T-Echo_room_server', 'LilyGo_T-Echo_companion_radio_ble'),
  [string] $OutputDir = 'out',
  [string] $Image = 'python:3.11-slim'
)

$ErrorActionPreference = 'Stop'

Write-Host "Using Docker image: $Image"

$root = (Get-Location).Path
$proj = $root
$outPath = Join-Path $proj $OutputDir
if (Test-Path $outPath) { Remove-Item -Recurse -Force $outPath }
New-Item -ItemType Directory -Force -Path $outPath | Out-Null

# Create a long-lived container so all work happens on container-native FS
$containerId = (& docker create --workdir /work $Image /bin/sh -lc 'sleep infinity').Trim()
try {
  if (-not $containerId) { throw 'Failed to create docker container' }
  Write-Host "Container: $containerId"

  # Start container
  docker start $containerId | Out-Null

  # Copy project into container native FS
  docker cp "${proj}/." "${containerId}:/work" | Out-Null

  # Install PlatformIO inside container
  docker exec $containerId /bin/sh -lc 'python3 -m pip install -q -U pip platformio' | Out-Host
  if ($LASTEXITCODE -ne 0) { throw 'Failed to install PlatformIO inside container' }

  # Build all environments
  foreach ($env in $Envs) {
    Write-Host "Building env: $env"
    docker exec $containerId /bin/sh -lc "cd /work && pio run -e $env" | Out-Host
    if ($LASTEXITCODE -ne 0) { throw "Build failed for env: $env" }
  }

  # After successful builds, copy artifacts back in one go per env
  foreach ($env in $Envs) {
    $envOut = Join-Path $outPath $env
    New-Item -ItemType Directory -Force -Path $envOut | Out-Null
    # Prepare artifacts directory and copy common outputs
    docker exec $containerId /bin/sh -lc "cd /work/.pio/build/$env && mkdir -p /tmp/artifacts && cp -f *.bin *.elf *.map *.hex /tmp/artifacts 2>/dev/null || true" | Out-Null
    # Generate UF2 from HEX for nRF52840 (T-Echo) if available
    docker exec $containerId /bin/sh -lc "if [ -f /work/.pio/build/$env/firmware.hex ]; then python3 /work/bin/uf2conv/uf2conv.py -f 0xADA52840 -c /work/.pio/build/$env/firmware.hex -o /tmp/artifacts/firmware.uf2; fi" | Out-Null
    docker cp "${containerId}:/tmp/artifacts/." "$envOut" | Out-Null
    # Clean temp artifacts in container
    docker exec $containerId /bin/sh -lc 'rm -rf /tmp/artifacts' | Out-Null
  }

  Write-Host "Artifacts copied to: $outPath"
} finally {
  if ($containerId) {
    docker rm -f $containerId | Out-Null
  }
}




