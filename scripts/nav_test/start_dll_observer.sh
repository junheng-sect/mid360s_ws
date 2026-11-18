#!/usr/bin/env bash
# Optional child of nav_test/common.sh. All preflight work stays in this process.
set -euo pipefail
map=$1
reloc_config=$2
out=$3
fail() {
  printf '%s\n' "$*" > "$out/dll_unavailable.txt"
  echo "$*" >&2
  exit 1
}
trap 'fail "DLL 旁路启动失败，请查看 dll.log"' ERR
nodes=$(timeout 5 rosnode list)
if grep -qx /fastlio_dll <<<"$nodes"; then
  fail '已有其他 DLL 节点，本任务未接管'
fi
package=$(rospack find fastlio_dll)
target=$map
config="$package/config/indoor.yaml"
if [[ "$map" == "$HOME/test_data/mapping/20261105_114605_outdoor" ]]; then
  target="${map}_screenshot_roi"
  config="$package/config/outdoor_large_field.yaml"
fi
cp -- "$reloc_config" "$out/dll_reloc.yaml"
cp -- "$config" "$out/dll_config.yaml"
python3 - "$map" "$target" "$out" <<'PY'
import hashlib
import json
from pathlib import Path
import sys

source, target, out = map(Path, sys.argv[1:])
def digest(file):
    h = hashlib.sha256()
    with file.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1048576), b''):
            h.update(chunk)
    return h.hexdigest()

def map_hashes(folder):
    return {n: digest(folder / n) for n in ('map_world.pcd', 'meta.json')}

origin = map_hashes(source)
tracked = origin if target == source else map_hashes(target)
if source != target:
    manifest = json.loads((target / 'manifest.json').read_text())
    parent = manifest['source']
    if (manifest['map_frame'] != 'drone_map' or Path(parent['map_dir']) != source
            or parent['map_sha256'] != origin['map_world.pcd'] or parent['meta_sha256'] != origin['meta.json']
            or any(manifest['sha256'][n] != v for n, v in tracked.items())):
        raise ValueError('DLL crop/source hash mismatch')
record = dict(map_dir=str(target), source_map_dir=str(source), map_sha256=tracked,
              source_sha256=origin, reloc_config_sha256=digest(out/'dll_reloc.yaml'))
temp = out/'dll_observer.json.tmp'
temp.write_text(json.dumps(record, indent=2) + '\n')
temp.replace(out/'dll_observer.json')
PY
exec roslaunch fastlio_dll observe.launch "map_dir:=$target" "legacy_map_dir:=$map" "config:=$config"
