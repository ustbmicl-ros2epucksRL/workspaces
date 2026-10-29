"""Fixed-seed, first-episode evaluation with an independent swept oracle."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import numpy as np
import torch
from .collision_geometry import segment_polygon_distance
from .environment import LocalFormationEnv, TaskConfig
from .policy import load_actor


def independent_clearances(env, start, end):
    """NumPy float64 reference, separate from the training geometry kernel."""
    start, end = (x.detach().cpu().numpy().astype(np.float64) for x in (start,end))
    vertices = env.vertices.cpu().numpy().astype(np.float64)[:,None]
    masks = env.obstacle_mask.cpu().numpy()[:,None]
    i,j = np.triu_indices(env.n_agents,1)
    relative = start[:,i]-start[:,j]
    change = end[:,i]-end[:,j]-relative
    norm = (change*change).sum(-1)
    time = np.zeros_like(norm)
    np.divide(-(relative*change).sum(-1),norm,out=time,where=norm>1e-20)
    pair = np.linalg.norm(relative+time.clip(0,1)[...,None]*change,axis=-1).min(-1)-2*env.radius
    wall = np.where(masks,segment_polygon_distance(start,end,vertices),np.inf).min((1,2))-env.radius
    boundary = np.minimum(env.bounds.cpu().numpy()-np.abs(start),env.bounds.cpu().numpy()-np.abs(end)).min((1,2))-env.radius
    return np.stack((pair,wall,boundary),-1)


@torch.no_grad()
def evaluate(actor, dt, scenario, formation, episodes, seed, horizon=750, trajectory=False, passage_width=1.2):
    device = next(actor.parameters()).device
    config = TaskConfig(num_envs=episodes,horizon=horizon,scenario=scenario,formation=formation,dt=dt,
                        passage_width=passage_width)
    env = LocalFormationEnv(config,actor.sensors,device,seed)
    state = actor.initial_state((episodes,env.n_agents))
    finished = torch.zeros(episodes,device=device,dtype=torch.bool)
    crossed = torch.zeros(episodes,env.n_agents,device=device,dtype=torch.bool)
    results = [None]*episodes
    minima = np.full((episodes,3),np.inf)
    trace = None
    if trajectory:
        trace = {'task':asdict(config),'sensors':asdict(actor.sensors),'seed':seed,'bounds':env.bounds.cpu().clone(),
                 'goal':env.goal[:4].cpu().clone(),'vertices':env.vertices[:4].cpu().clone(),
                 'obstacle_mask':env.obstacle_mask[:4].cpu().clone(),'template':env.template[:4].cpu().clone(),
                 'position':[env.pos[:4].cpu().clone()],'velocity':[env.vel[:4].cpu().clone()],
                 'proposal':[],'active':[]}
    for _ in range(horizon):
        action,state = actor(env.observe(),state)
        candidate_velocity = (env.vel+dt*action.clamp(-1,1)).clamp(-1,1)
        candidate = env.pos+dt*candidate_velocity
        # Evaluation-only evidence of traversing the actual opening. Reaching
        # the goals by circling outside the two wall strips is reported apart.
        if scenario == 'narrowing':
            crossing = (env.pos[...,1]<=1.3)&(candidate[...,1]>1.3)
            fraction = ((1.3-env.pos[...,1])/(candidate[...,1]-env.pos[...,1]).clamp_min(1e-12)).clamp(0,1)
            crossing_x = env.pos[...,0]+fraction*(candidate[...,0]-env.pos[...,0])
            at_opening = crossing_x.abs() < passage_width/2-env.radius
        oracle = independent_clearances(env,env.pos,candidate)
        previous_velocity = env.vel.clone()
        active = ~finished
        reward,term,trunc,info = env.step(action)
        if scenario == 'narrowing':
            crossed |= crossing&at_opening&active[:,None]&~info['collision'][:,None]
        np.testing.assert_allclose(info['clearance'].cpu().numpy(),oracle,atol=1e-8,rtol=1e-7)
        if ((candidate_velocity-previous_velocity).abs()>dt+1e-6).any() or (candidate_velocity.abs()>1+1e-6).any():
            raise AssertionError('Actuator limits violated')
        minima[active.cpu().numpy()] = np.minimum(minima[active.cpu().numpy()],oracle[active.cpu().numpy()])
        if trace is not None:
            trace['position'].append(env.pos[:4].cpu().clone())
            trace['velocity'].append(env.vel[:4].cpu().clone())
            trace['proposal'].append(candidate[:4].cpu().clone())
            trace['active'].append(active[:4].cpu().clone())
        done = term|trunc
        for i in (done&active).nonzero().flatten().tolist():
            passage = bool(crossed[i].all()) if scenario == 'narrowing' else True
            results[i] = {'episode':i,'success':bool(info['success'][i]) and passage,
                          'goal_success':bool(info['success'][i]),'corridor_passed':passage,
                          'collision':bool(info['collision'][i]),
                          'timeout':bool(trunc[i]),'length':int(info['steps'][i]),
                          'goal_distance':float(info['goal_distance'][i]),'formation_error':float(info['formation_error'][i]),
                          'max_goal_distance':float((env.goal[i]-env.pos[i]).norm(dim=-1).max()),
                          'max_speed':float(env.vel[i].norm(dim=-1).max()),
                          'final_formation_ok':bool(env.final_formation_ok()[i]),'hold':int(env.hold[i]),
                          'min_clearance':[float(x) if np.isfinite(x) else None for x in minima[i]]}
        finished |= done
        if finished.all():
            break
        env.reset(done.nonzero().flatten())
        state = torch.where(done[:,None,None],0.,state)
    if not all(r is not None for r in results):
        raise AssertionError('Evaluation failed to account for all episodes')
    if trace is not None:
        for key in ('position','velocity','proposal','active'):
            trace[key] = torch.stack(trace[key])
    summary = {'scenario':scenario,'formation':formation,'seed':seed,'episodes':episodes,
               'successes':sum(r['success'] for r in results),'collisions':sum(r['collision'] for r in results),
               'goal_successes':sum(r['goal_success'] for r in results),
               'corridor_passes':sum(r['corridor_passed'] for r in results) if scenario == 'narrowing' else None,
               'bypass_completions':sum(r['goal_success'] and not r['corridor_passed'] for r in results),
               'timeouts':sum(r['timeout'] for r in results),
               'mean_goal_distance':sum(r['goal_distance'] for r in results)/episodes,
               'mean_formation_error':sum(r['formation_error'] for r in results)/episodes,'results':results}
    return summary,trace


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('actor')
    parser.add_argument('--output',required=True)
    parser.add_argument('--device',default='cpu')
    parser.add_argument('--episodes',type=int,default=32,help='Per map/shape/seed combination')
    parser.add_argument('--seeds',type=int,nargs='+',default=[30001,30002,30003])
    parser.add_argument('--horizon',type=int,default=750)
    parser.add_argument('--passage-width',type=float,default=1.2)
    parser.add_argument('--scenarios',nargs='+',default=['narrowing'],choices=LocalFormationEnv.names)
    parser.add_argument('--formation',required=True,choices=['line','circle','wedge'],help='Use the formation matching this actor')
    parser.add_argument('--trajectory',action='store_true')
    args = parser.parse_args()
    torch.set_num_threads(1)
    actor,dt = load_actor(args.actor,args.device)
    destination = Path(args.output)
    destination.parent.mkdir(parents=True,exist_ok=True)
    report = {'actor':str(Path(args.actor).resolve()),'deterministic':True,'filter':'none',
              'independent_swept_oracle':True,'horizon':args.horizon,'policy':asdict(actor.config),
              'sensors':asdict(actor.sensors),'cases':[]}
    for scenario in args.scenarios:
        for formation in [args.formation]:
            for seed in args.seeds:
                case,trace = evaluate(actor,dt,scenario,formation,args.episodes,seed,args.horizon,args.trajectory,args.passage_width)
                report['cases'].append(case)
                print(json.dumps({k:v for k,v in case.items() if k!='results'}),flush=True)
                if trace is not None:
                    trace_path = destination.with_name(f'{destination.stem}_{scenario}_{formation}_{seed}.pt')
                    torch.save(trace,trace_path)
                destination.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')


if __name__ == '__main__':
    main()
