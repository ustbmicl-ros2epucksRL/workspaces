"""Independent local recurrent task critic."""
import torch
from torch import nn
from .config import PolicyConfig
from .observation import SensorConfig


class LocalCritic(nn.Module):
    def __init__(self, config=PolicyConfig(), sensors=SensorConfig()):
        super().__init__()
        self.config, self.sensors = config, sensors
        h = config.hidden
        if h < 8:
            raise ValueError('Hidden dimension must be at least eight')
        self.peer = nn.Sequential(nn.Linear(8,h//2), nn.Tanh(), nn.Linear(h//2,h//2), nn.Tanh())
        self.encoder = nn.Sequential(nn.Linear(5+sensors.lidar_rays+h//2+1,h),nn.Tanh(),nn.Linear(h,h),nn.Tanh())
        self.cell = nn.GRUCell(h,h)
    def initial_state(self, batch_shape=()):
        return self.encoder[0].weight.new_zeros(tuple(batch_shape)+(self.config.hidden,))

    def features(self, obs, state, reset=None):
        shape = obs['ego'].shape[:-1]
        mask = obs['mask']
        # Mask before encoding, so even NaN padding cannot affect a robot.
        peer = self.peer(torch.where(mask[...,None],obs['neighbours'],0.))
        count = mask.sum(-1,keepdim=True)
        aggregate = (peer*mask[...,None]).sum(-2)/count.clamp_min(1)
        encoded = self.encoder(torch.cat((obs['ego'],obs['scan'],aggregate,count.clamp_max(10)/10),-1))
        if reset is not None:
            state = torch.where(reset[...,None],0.,state)
        output = self.cell(encoded.reshape(-1,self.config.hidden),state.reshape(-1,self.config.hidden)).reshape(*shape,self.config.hidden)
        return output, output
