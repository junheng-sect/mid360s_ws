#!/usr/bin/env python3
"""Measure cold/warm cache in separate processes using a temporary copy of the indoor map."""
import json,os,resource,shutil,subprocess,sys,tempfile
from pathlib import Path


def main():
    if len(sys.argv)==3 and sys.argv[1]=='--child':
        from drone_relocalization.matching import MapModel
        m=MapModel(sys.argv[2])
        print(json.dumps(dict(cache_hit=m.cache_hit,load_s=m.load_s,points=len(m.P),max_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)))
        return
    source=Path.home()/'test_data/mapping/20261018_194304_voxelslam'
    with tempfile.TemporaryDirectory(prefix='indoor-cache-bench-') as d:
        for name in ('map_world.pcd','meta.json'):shutil.copy2(str(source/name),d+'/'+name)
        results=[]
        for _ in range(2):
            out=subprocess.check_output([sys.executable,'-B',__file__,'--child',d],text=True)
            results.append(json.loads(out))
        assert not results[0]['cache_hit'] and results[1]['cache_hit']
    out=Path.home()/'test_data/reloc_acceptance/20261020/cache_bench.json'
    out.write_text(json.dumps(dict(cold=results[0],warm=results[1]),indent=2))
    print(out.read_text())


if __name__=='__main__':main()
