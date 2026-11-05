"""GAE and chronological recurrent minibatch views for PPO."""
import torch


def advantages(reward, value, next_value, terminated, truncated, gamma, lam):
    """Time limits bootstrap the pre-reset state, but never cross an episode."""
    delta = reward+gamma*next_value*(~terminated)[...,None]-value
    output = torch.zeros_like(reward)
    carry = torch.zeros_like(reward[0])
    for t in reversed(range(len(reward))):
        carry = delta[t]+gamma*lam*(~(terminated[t]|truncated[t]))[...,None]*carry
        output[t] = carry
    return output, output+value


def sequence_view(tensor, length):
    # [T,E,A,...] -> [chunks*E*A, L,...], preserving chronology per robot.
    t,e,a = tensor.shape[:3]
    return tensor.reshape(t//length,length,e*a,*tensor.shape[3:]).transpose(1,2).reshape(-1,length,*tensor.shape[3:])
