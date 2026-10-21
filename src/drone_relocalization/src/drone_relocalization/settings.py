"""Defaults are the legacy quality gates; no relaxation for the new solver."""
import math
import os
import yaml

DEFAULTS=dict(res=.05,sigma=.05,max_cands=6000,yaw_step_deg=2.,nn_reach=.15,
              min_range=1.,score_ok=.45,inlier_ok=.25,fit_ok=.8,rmse_ok=.02,
              degen_ok=.12,amb_max=.9,candidate_ratio=.6,n_candidates=8,max_tilt_deg=3.,
              window=2.,confirmations=2,consensus_m=.05,consensus_deg=1.,
              still_m=.03,still_deg=2.,freshness=.5,timeout=60.,tries=3,
              max_grid_cells=50000000,max_expand_bytes=1073741824)


def load(path=None,overrides=None):
    c=dict(DEFAULTS)
    if path:
        with open(os.path.expanduser(path)) as f:
            d=yaml.safe_load(f) or {}
        if not isinstance(d,dict) or set(d)-set(c):raise ValueError('unknown configuration keys')
        c.update(d)
    c.update({k:v for k,v in (overrides or {}).items() if v is not None})
    for k,v in c.items():
        if not isinstance(v,(float,int)) or isinstance(v,bool) or not math.isfinite(v) or v<=0:
            raise ValueError('invalid positive parameter: '+k)
    for k in ['max_cands','n_candidates','confirmations','tries','max_grid_cells','max_expand_bytes']:
        if int(c[k])!=c[k]:raise ValueError('integer required: '+k)
        c[k]=int(c[k])
    if c['confirmations']<2 or c['confirmations']>c['tries']:raise ValueError('require 2 <= confirmations <= tries')
    if c['n_candidates']>8 or c['max_tilt_deg']>10:raise ValueError('candidate/tilt budget exceeded')
    return c
