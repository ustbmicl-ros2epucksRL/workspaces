"""Independent NumPy continuous collision geometry for strict evaluation."""
import numpy as np


def point_polygon_distance(points,vertices):
    point=points[...,None,None,:]
    left=vertices
    right=np.roll(left,-1,axis=-2)
    edge=right-left
    fraction=((point-left)*edge).sum(-1)/np.maximum((edge*edge).sum(-1),1e-20)
    nearest=left+np.clip(fraction,0,1)[...,None]*edge
    distance=np.linalg.norm(nearest-point,axis=-1)
    offset=point-left
    cross=edge[...,0]*offset[...,1]-edge[...,1]*offset[...,0]
    inside=np.all(cross>=-1e-12,axis=-1)|np.all(cross<=1e-12,axis=-1)
    return np.where(inside,0.,distance.min(-1))


def segment_polygon_distance(start,end,vertices):
    """Exact 2-D segment distance to each convex polygon."""
    point=start[...,None,None,:]
    finish=end[...,None,None,:]
    left=vertices
    right=np.roll(left,-1,axis=-2)
    edge=right-left
    trajectory=finish-point

    def point_segment_distance(query,a,b):
        direction=b-a
        fraction=((query-a)*direction).sum(-1)/np.maximum((direction*direction).sum(-1),1e-20)
        return np.linalg.norm(query-(a+np.clip(fraction,0,1)[...,None]*direction),axis=-1)

    endpoint=np.minimum(point_segment_distance(point,left,right),
                        point_segment_distance(finish,left,right))
    vertex=point_segment_distance(left,point,finish)
    distance=np.minimum(endpoint,vertex).min(-1)
    cross=lambda a,b:a[...,0]*b[...,1]-a[...,1]*b[...,0]
    denominator=cross(trajectory,edge)
    relative=left-point
    safe=np.where(np.abs(denominator)>1e-14,denominator,1.)
    t=cross(relative,edge)/safe
    u=cross(relative,trajectory)/safe
    intersects=((np.abs(denominator)>1e-14)&(t>=0)&(t<=1)&(u>=0)&(u<=1)).any(-1)
    inside=(point_polygon_distance(start,vertices)==0)|(point_polygon_distance(end,vertices)==0)
    return np.where(intersects|inside,0.,distance)
