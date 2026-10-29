"""Export the fixed recommended actors to a trainer-free TorchScript bundle."""
import hashlib
import argparse
import json
from pathlib import Path
import shutil
import warnings
import torch
from afor_local.formations import make_formation
from afor_local.policy import load_actor


class TensorActor(torch.nn.Module):
    def __init__(self, actor):
        super().__init__()
        self.actor=actor

    def forward(self, ego, scan, neighbours, mask, memory):
        return self.actor({'ego':ego,'scan':scan,'neighbours':neighbours,'mask':mask},memory)


def example(batch, generator):
    return (torch.randn(batch,5,generator=generator),torch.rand(batch,32,generator=generator),
            torch.randn(batch,4,8,generator=generator),torch.rand(batch,4,generator=generator)>.4,
            torch.randn(batch,64,generator=generator))


def main():
    root=Path(__file__).resolve().parents[1]
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--models',type=Path,default=root/'models')
    parser.add_argument('--output',type=Path,default=root/'runs/deployment_bundle')
    args=parser.parse_args()
    torch.set_num_threads(1)
    output=args.output.resolve()
    output.mkdir(parents=True,exist_ok=True)
    if any(output.iterdir()):
        raise FileExistsError('Choose an empty output directory; the delivered bundle is immutable')
    original=json.loads((args.models/'manifest.json').read_text())
    contract={'version':1,'task':'Original gradual narrowing; three independently trained formation actors',
              'dt_s':.05,'radius_m':.13,'lidar_rays':32,'lidar_range_m':2.,
              'neighbour_range_m':2.,'memory_size':64,'agents':5,
              'frame':'Fixed mission axes; original forward direction +y; +x points right; world uses ENU metres and radians.',
              'observation':'Final assigned goal delta, ego velocity/radius, 32 metric rays, local peers with fixed role/template relations. No map, opening width, phase or waypoint.',
              'action':'Acceleration in mission axes, each component in [-1,1] m/s^2. Target velocity=clip(measured_velocity+0.05*acceleration,-1,1) per mission axis.',
              'formulations':'Planar holonomic dynamics. UAV altitude/yaw use an external low-level controller. Differential drive and flight dynamics have not been validated.',
              'formations':{},'exports':[]}
    rng=torch.Generator().manual_seed(59001)
    for entry in original['models']:
        formation=entry['formation']
        path=args.models/entry['checkpoint']
        if hashlib.sha256(path.read_bytes()).hexdigest()!=entry['actor_sha256']:
            raise RuntimeError('Recommended actor identity mismatch')
        actor,dt=load_actor(path)
        assert dt==contract['dt_s']
        wrapper=TensorActor(actor).eval()
        with warnings.catch_warnings(record=True) as captured:
            traced=torch.jit.trace(wrapper,example(1,rng),check_inputs=[example(5,rng),example(16,rng)])
        traced=torch.jit.freeze(traced)
        errors=[]
        for batch in (1,5,16):
            for _ in range(10):
                sample=example(batch,rng)
                with torch.inference_mode():
                    eager=wrapper(*sample);actual=traced(*sample)
                for a,b in zip(eager,actual):
                    torch.testing.assert_close(a,b,rtol=1e-5,atol=1e-6)
                    errors.append(float((a-b).abs().max()))
        folder=output/formation;folder.mkdir(exist_ok=True)
        traced.save(str(folder/'actor.ts'))
        f=make_formation(formation,'cpu')
        contract['formations'][formation]={'template_m':f.template.tolist(),'edges':f.edges}
        contract['exports'].append({'formation':formation,'actor_pt_sha256':entry['actor_sha256'],
            'actor_ts_sha256':hashlib.sha256((folder/'actor.ts').read_bytes()).hexdigest(),
            'max_export_error':max(errors),'trace_warnings':list(dict.fromkeys(str(w.message) for w in captured)),
            'source':'Fixed recommended seed2/update500; no retraining or checkpoint selection'})
    (output/'contract.json').write_text(json.dumps(contract,indent=2)+'\n')
    shutil.copyfile(root/'deployment/local_controller.py',output/'local_controller.py')
    print(json.dumps(contract['exports'],indent=2))


if __name__ == '__main__':
    main()
