"""Single stochastic rollout stream and sequence-minibatched recurrent PPO."""
import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
from pathlib import Path
import time
import torch
from torch.nn import functional as F
from .environment import LocalFormationEnv, TaskConfig
from .observation import SensorConfig
from .policy import ActorCritic, PolicyConfig, SCHEMA, latent_log_prob, export_actor
from .barrier import local_braking_loss


@dataclass(frozen=True)
class PPOConfig:
    steps: int = 128
    sequence: int = 16
    sequences_per_batch: int = 64
    epochs: int = 4
    lr: float = 3e-4
    gamma: float = .99
    gae_lambda: float = .95
    clip: float = .2
    entropy: float = .001
    independent_value: bool = True
    barrier_weight: float = 1.

    def __post_init__(self):
        if min(self.steps,self.sequence,self.sequences_per_batch,self.epochs) < 1 or self.steps%self.sequence:
            raise ValueError('Positive PPO dimensions required; steps must be divisible by sequence')
        if not (0 < self.gamma <= 1 and 0 <= self.gae_lambda <= 1 and self.lr > 0):
            raise ValueError('Invalid PPO hyperparameters')
        if self.barrier_weight < 0:
            raise ValueError('Barrier weight must be nonnegative')
        if not self.independent_value:
            raise ValueError('The current recipe uses an independent task critic')


def advantages(reward, value, next_value, terminated, truncated, gamma, lam):
    """Time limits bootstrap the pre-reset state, but never cross an episode."""
    delta = reward+gamma*next_value*(~terminated)[...,None]-value
    output = torch.zeros_like(reward)
    carry = torch.zeros_like(reward[0])
    for t in reversed(range(len(reward))):
        carry = delta[t]+gamma*lam*(~(terminated[t]|truncated[t]))[...,None]*carry
        output[t] = carry
    return output, output+value


@torch.no_grad()
def collect(model, env, state, starts, config):
    observations, states, resets, latents, logprobs = [], [], [], [], []
    rewards, values, next_values, terms, truncs = [], [], [], [], []
    episodes = []
    for _ in range(config.steps):
        obs = env.observe()
        observations.append(obs)
        states.append(state.clone())
        resets.append(starts.clone())
        action, latent, lp, value, next_state = model(obs,state,starts)
        reward, term, trunc, info = env.step(action)
        # This observation is BEFORE reset, essential for time-limit bootstrap.
        _, _, _, next_value, _ = model(env.observe(),next_state,deterministic=True)
        latents.append(latent); logprobs.append(lp); rewards.append(reward)
        values.append(value); next_values.append(next_value); terms.append(term); truncs.append(trunc)
        done = term|trunc
        for i in done.nonzero().flatten().tolist():
            episodes.append({'success':bool(info['success'][i]),'collision':bool(info['collision'][i]),
                             'return':float(info['episode_return'][i]),'length':int(info['steps'][i]),
                             'goal_distance':float(info['goal_distance'][i]),'formation_error':float(info['formation_error'][i])})
        env.reset(done.nonzero().flatten())
        starts = done[:,None].expand(-1,env.n_agents)
        state = torch.where(starts[...,None],0.,next_state)
    reward, value, next_value, term, trunc = map(torch.stack,(rewards,values,next_values,terms,truncs))
    advantage, returns = advantages(reward,value,next_value,term,trunc,config.gamma,config.gae_lambda)
    data = {'obs':{k:torch.stack([o[k] for o in observations]) for k in observations[0]},
            'state':torch.stack(states),'reset':torch.stack(resets),'latent':torch.stack(latents),
            'logprob':torch.stack(logprobs),'advantage':advantage,'returns':returns}
    return data, state, starts, episodes


def sequence_view(tensor, length):
    # [T,E,A,...] -> [chunks*E*A, L,...], preserving chronology per robot.
    t,e,a = tensor.shape[:3]
    return tensor.reshape(t//length,length,e*a,*tensor.shape[3:]).transpose(1,2).reshape(-1,length,*tensor.shape[3:])


def update(model, optimizer, data, config, dt=.05):
    obs = {k:sequence_view(v,config.sequence) for k,v in data['obs'].items()}
    sequences = {k:sequence_view(v,config.sequence) for k,v in data.items() if k!='obs'}
    adv = sequences['advantage']
    sequences['advantage'] = (adv-adv.mean())/adv.std(unbiased=False).clamp_min(1e-8)
    count = len(adv)
    totals = {'policy_loss':0.,'value_loss':0.,'entropy':0.,'approx_kl':0.,'grad_norm':0.,'value_grad_norm':0.,'barrier_loss':0.,'optimizer_steps':0}
    for _ in range(config.epochs):
        order = torch.randperm(count,device=adv.device)
        for ids in order.split(config.sequences_per_batch):
            state = sequences['state'][ids,0].detach()
            logprobs, values, entropy, barriers = [], [], [], []
            # Backpropagate THROUGH a chronological chunk; detach only at its
            # beginning. Reset masks prevent memory crossing team episodes.
            for t in range(config.sequence):
                current={k:v[ids,t] for k,v in obs.items()}
                features, critic_features, state = model.features(current,state,sequences['reset'][ids,t])
                dist = model.actor.distribution(features)
                logprobs.append(latent_log_prob(dist,sequences['latent'][ids,t]))
                values.append(model.value(critic_features).squeeze(-1))
                sample = dist.rsample()
                entropy.append(-latent_log_prob(dist,sample))
                if config.barrier_weight:
                    barriers.append(local_braking_loss(current,dist.mean.tanh(),model.actor.sensors,dt))
            logprob, value, ent = map(lambda x:torch.stack(x,1),(logprobs,values,entropy))
            logratio = logprob-sequences['logprob'][ids]
            ratio = logratio.exp()
            advantage = sequences['advantage'][ids]
            policy_loss = -torch.minimum(ratio*advantage,ratio.clamp(1-config.clip,1+config.clip)*advantage).mean()
            value_loss = F.mse_loss(value,sequences['returns'][ids])
            barrier=torch.stack(barriers).mean() if barriers else policy_loss.new_zeros(())
            loss = policy_loss+.5*value_loss-config.entropy*ent.mean()+config.barrier_weight*barrier
            if not torch.isfinite(loss):
                raise FloatingPointError('Nonfinite PPO loss')
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            norm=torch.nn.utils.clip_grad_norm_(model.actor.parameters(),.5,error_if_nonfinite=True)
            value_norm=torch.nn.utils.clip_grad_norm_(list(model.critic.parameters())+list(model.value.parameters()),.5,error_if_nonfinite=True)
            optimizer.step()
            for key,val in (('policy_loss',policy_loss),('value_loss',value_loss),('entropy',ent.mean()),
                            ('approx_kl',((ratio-1)-logratio).mean()),('grad_norm',norm),('value_grad_norm',value_norm),('barrier_loss',barrier)):
                totals[key] += float(val.detach())
            totals['optimizer_steps'] += 1
    for key in totals:
        if key != 'optimizer_steps':
            totals[key] /= totals['optimizer_steps']
    return totals


def source_hash():
    root = Path(__file__).resolve().parent
    files = sorted(root.glob('*.py'))
    digest = hashlib.sha256()
    for file in files:
        digest.update(str(file.relative_to(root)).encode()); digest.update(file.read_bytes())
    return digest.hexdigest()


def train(output, updates, seed=2, device='cpu', task=TaskConfig(), policy=PolicyConfig(),
          sensors=SensorConfig(), ppo=PPOConfig(), resume=None):
    output = Path(output)
    output.mkdir(parents=True,exist_ok=True)
    if not resume and (output/'latest.pt').exists():
        raise FileExistsError('Choose a new output directory or explicitly resume')
    torch.manual_seed(seed)
    env = LocalFormationEnv(task,sensors,device,seed+1000)
    model = ActorCritic(policy,sensors,independent_value=ppo.independent_value).to(device)
    optimizer = torch.optim.Adam(model.parameters(),lr=ppo.lr,eps=1e-5)
    state = model.initial_state((task.num_envs,env.n_agents))
    starts = torch.ones(task.num_envs,env.n_agents,device=device,dtype=torch.bool)
    config = {'task':asdict(task),'policy':asdict(policy),'sensors':asdict(sensors),'ppo':asdict(ppo),'seed':seed}
    start = 0
    if resume:
        saved = torch.load(resume,map_location=device,weights_only=False)
        if saved.get('schema') != SCHEMA or saved['config'] != config or saved['source_hash'] != source_hash():
            raise ValueError('Resume requires same local schema, configuration and source; start a new experiment for changes')
        if saved['device'] != str(torch.device(device)):
            raise ValueError('Exact resume requires the same device')
        model.load_state_dict(saved['model']); optimizer.load_state_dict(saved['optimizer'])
        env.load_state_dict(saved['env']); state, starts = saved['state'],saved['starts']
        torch.set_rng_state(saved['torch_rng'].cpu())
        if str(device).startswith('cuda'):
            torch.cuda.set_rng_state_all([s.cpu() for s in saved['cuda_rng']])
        start = saved['update']
    if updates <= start:
        raise ValueError('Requested total updates must exceed the resumed update')
    (output/'config.json').write_text(json.dumps({**config,'schema':SCHEMA,'source_hash':source_hash(),
                                                'torch':torch.__version__,'device':str(device)},indent=2)+'\n')
    for iteration in range(start+1,updates+1):
        begin = time.monotonic()
        data,state,starts,episodes = collect(model,env,state,starts,ppo)
        metrics = update(model,optimizer,data,ppo,task.dt)
        metrics.update(update=iteration,team_transitions=iteration*task.num_envs*ppo.steps,
                       agent_transitions=iteration*task.num_envs*ppo.steps*env.n_agents,
                       seconds=time.monotonic()-begin,completed_episodes=len(episodes))
        for key in ('success','collision','return','length','goal_distance','formation_error'):
            metrics[key] = sum(e[key] for e in episodes)/len(episodes) if episodes else None
        with (output/'metrics.jsonl').open('a') as stream:
            stream.write(json.dumps(metrics)+'\n')
        checkpoint = {'schema':SCHEMA,'config':config,'source_hash':source_hash(),'device':str(torch.device(device)),
                      'update':iteration,'model':model.state_dict(),'optimizer':optimizer.state_dict(),
                      'env':env.state_dict(),'state':state,'starts':starts,'torch_rng':torch.get_rng_state(),
                      'cuda_rng':torch.cuda.get_rng_state_all() if str(device).startswith('cuda') else []}
        temporary = output/'latest.tmp'
        torch.save(checkpoint,temporary); temporary.replace(output/'latest.pt')
        export_actor(model,output/'actor.pt',task.dt)
        print(json.dumps(metrics),flush=True)
    return model


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True)
    parser.add_argument('--resume')
    parser.add_argument('--updates',type=int,default=500)
    parser.add_argument('--device',default='cpu')
    parser.add_argument('--threads',type=int,default=1)
    parser.add_argument('--seed',type=int,default=2)
    parser.add_argument('--num-envs',type=int,default=16)
    parser.add_argument('--horizon',type=int,default=750)
    parser.add_argument('--formation',default='line',choices=('line','circle','wedge'))
    parser.add_argument('--scenario',default='narrowing',choices=LocalFormationEnv.names)
    parser.add_argument('--passage-width',type=float,default=1.2)
    parser.add_argument('--formation-weight',type=float,default=.15)
    parser.add_argument('--steps',type=int,default=128)
    parser.add_argument('--sequence',type=int,default=16)
    parser.add_argument('--epochs',type=int,default=4)
    parser.add_argument('--sequences-per-batch',type=int,default=64)
    parser.add_argument('--lr',type=float,default=3e-4)
    parser.add_argument('--goal-distance-weight',type=float,default=1.)
    args = parser.parse_args()
    torch.set_num_threads(args.threads)
    train(args.output,args.updates,args.seed,args.device,
          TaskConfig(num_envs=args.num_envs,horizon=args.horizon,formation=args.formation,scenario=args.scenario,
                     formation_weight=args.formation_weight,
                     goal_distance_weight=args.goal_distance_weight,passage_width=args.passage_width),
          PolicyConfig(),
          ppo=PPOConfig(steps=args.steps,sequence=args.sequence,epochs=args.epochs,
                        sequences_per_batch=args.sequences_per_batch,lr=args.lr),resume=args.resume)


if __name__ == '__main__':
    main()
