#!/usr/bin/env python3
import json,os,resource,time
import numpy as np
from drone_relocalization.matching import MapModel
from drone_relocalization.session import BagSession,run_once

base=os.path.expanduser('~/test_data/mapping');out=os.path.expanduser('~/test_data/reloc_acceptance/20261020')
os.makedirs(out,exist_ok=True);M=MapModel(base+'/20261018_194304_voxelslam')
summary={}
for name,bag,step in [('flight','flight_raw_000001.bag',3),('hand194304','20261018_194304/raw.bag',5),('hand192355','20261018_192355/raw.bag',5)]:
    S=BagSession(base+'/'+bag);results=[];started=time.monotonic()
    for t in np.arange(1,S.dur-2,step):
        r=run_once(M,S,float(t),2);results.append(r)
        print(name,t,r['ok'],round(r.get('fitness',0),3),round(r.get('degen',0),3),round(r.get('tilt_deg',0),2),r['why'],flush=True)
    S.close();dur=time.monotonic()-started
    data=dict(schema_version=2,map=base+'/20261018_194304_voxelslam',bag=base+'/'+bag,window=2.,results=results,
              summary=dict(trusted=sum(r['ok'] for r in results),total=len(results),median_s=float(np.median([r['secs'] for r in results])),
                           total_s=dur,max_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss))
    json.dump(data,open(out+'/'+name+'.json','w'),ensure_ascii=False,indent=2);summary[name]=data['summary'];print('SUMMARY',name,summary[name],flush=True)
json.dump(summary,open(out+'/regression_summary.json','w'),indent=2)
