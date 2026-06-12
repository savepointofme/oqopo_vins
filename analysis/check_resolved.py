import json
with open('C:/Users/baloney/Desktop/实验目录/gpsz_oc2_1e4_fly3/20260606_fly3_oc2_gpsz_fi1e4_成功/metadata/run_spec_resolved.json', encoding='utf-8') as f:
    r = json.load(f)
print('run_spec_resolved inputs:')
for k, v in r['inputs'].items():
    print(f"  {k}: state={v['state']} raw={v['raw'][:80]}")
print('velocity_source:', r['velocity_source'])
print('alignment:', r['alignment'])
