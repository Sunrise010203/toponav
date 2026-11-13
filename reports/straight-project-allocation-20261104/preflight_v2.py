"""Fixed real six-case symmetric dispatch validation, not performance timing."""
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import sys
import hashlib
import cv2
ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(HERE))
from baseline_v2 import SymmetricBaseline, CURRENT_PAIR
from roadvision import corridor_rgb_probe as core
from roadvision.prepared_descriptor_checks import filter_h,filter_i
from tools.straight_paired_population_benchmark import local_cases,encode,write

def main():
    b=SymmetricBaseline(HERE/'before/roadvision/corridor_rgb_probe.py')
    original=cv2.matchTemplate
    cases=[]
    for name,images,region in local_cases():
        rgb=[hashlib.sha256(im.tobytes()).hexdigest() for im in images]
        expected=encode(b.before(images,region));records=[]
        for variant in (b.before,b.after):
            for explicit in (False,True):
                for controller_context in (False,True):
                    n=[0]
                    def matched(*a,**kw): n[0]+=1;return original(*a,**kw)
                    kw=dict(filters=(filter_h,filter_i)) if explicit else {}
                    with patch.object(cv2,'matchTemplate',matched):
                        if controller_context:
                            ns=SimpleNamespace(pair=CURRENT_PAIR)
                            with b.replace_aliases([(ns,'pair','pair')],variant):
                                assert core._project is variant.project
                                assert encode(ns.pair(images,region,**kw))==expected
                        else: assert encode(variant(images,region,**kw))==expected
                    assert all(vars(owner)[key] is b.new for owner,key,_ in b.bindings)
                    records.append(dict(variant='before' if variant is b.before else 'after',
                        explicit=explicit,controller_context=controller_context,matches=n[0],exact=True))
        assert len({r['matches'] for r in records})==1
        if name=='random':assert records[0]['matches']==100
        if name=='flat':assert records[0]['matches']==0
        assert rgb==[hashlib.sha256(im.tobytes()).hexdigest() for im in images]
        cases.append(dict(case=name,dispatches=records,arrays_unchanged=True))
    write(HERE/'PREFLIGHT-v2.json',dict(cases=cases,symmetric_binding=b.record(),
        all_outputs_exact=True,actual_matches_unchanged=True,aliases_restored=True))
    print('symmetric preflight exact: 6 cases x 2 variants x 2 filter dispatches x 2 scopes')
if __name__=='__main__':main()
