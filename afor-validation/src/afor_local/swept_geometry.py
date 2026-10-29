"""Batched exact swept geometry for training costs, independently checked in NumPy."""
import torch


def point_polygon_distance(points,vertices):
    point=points[...,None,None,:]
    left=vertices
    right=torch.roll(left,-1,-2)
    edge=right-left
    fraction=((point-left)*edge).sum(-1)/edge.square().sum(-1).clamp_min(1e-20)
    nearest=left+fraction.clamp(0,1)[...,None]*edge
    distance=torch.linalg.vector_norm(nearest-point,dim=-1)
    offset=point-left
    cross=edge[...,0]*offset[...,1]-edge[...,1]*offset[...,0]
    inside=(cross>=-1e-12).all(-1)|(cross<=1e-12).all(-1)
    return torch.where(inside,torch.zeros_like(distance[...,0]),distance.amin(-1))


def segment_polygon_distance(start,end,vertices):
    point=start[...,None,None,:]
    finish=end[...,None,None,:]
    left=vertices
    right=torch.roll(left,-1,-2)
    edge=right-left
    trajectory=finish-point

    def point_segment_distance(query,a,b):
        direction=b-a
        fraction=((query-a)*direction).sum(-1)/direction.square().sum(-1).clamp_min(1e-20)
        nearest=a+fraction.clamp(0,1)[...,None]*direction
        return torch.linalg.vector_norm(query-nearest,dim=-1)

    endpoint=torch.minimum(point_segment_distance(point,left,right),
                           point_segment_distance(finish,left,right))
    vertex=point_segment_distance(left,point,finish)
    distance=torch.minimum(endpoint,vertex).amin(-1)
    cross=lambda a,b:a[...,0]*b[...,1]-a[...,1]*b[...,0]
    denominator=cross(trajectory,edge)
    relative=left-point
    safe=torch.where(denominator.abs()>1e-14,denominator,torch.ones_like(denominator))
    t=cross(relative,edge)/safe
    u=cross(relative,trajectory)/safe
    intersects=((denominator.abs()>1e-14)&(t>=0)&(t<=1)&(u>=0)&(u<=1)).any(-1)
    inside=(point_polygon_distance(start,vertices)==0)|(point_polygon_distance(end,vertices)==0)
    return torch.where(intersects|inside,torch.zeros_like(distance),distance)
