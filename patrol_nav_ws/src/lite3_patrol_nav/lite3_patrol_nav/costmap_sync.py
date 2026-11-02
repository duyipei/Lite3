"""Verify near projected obstacles are represented in the local costmap.

This is spatial correspondence, not plugin provenance. Existing static lethal
cells are sufficient: the planner already sees that region as occupied. Clouds
use the sensor monotonic clock; Nav2 costmap stamps use the ROS clock. Never
subtract one clock domain from the other.
"""
import math
from collections import namedtuple

import numpy as np

Cloud = namedtuple('Cloud','stamp xy')
Grid = namedtuple('Grid','stamp arrival resolution width height origin yaw cells lethal')


class CostmapSync:
    def __init__(self, timeout=.75, radius=1., quantization=.04):
        if not (.5 <= timeout <= 1. and .9 <= radius <= 2. and .02 <= quantization <= .05):
            raise ValueError('invalid costmap synchronization settings')
        self.timeout,self.radius,self.quantization=timeout,radius,quantization
        self.cloud=None;self.grid=None;self.error=''

    @staticmethod
    def stamp(header):
        s=header.stamp
        if s.sec<0 or not 0<=s.nanosec<1000000000:
            raise ValueError('invalid stamp')
        return s.sec+s.nanosec/1e9

    def receive_cloud(self,msg,now):
        try:
            stamp=self.stamp(msg.header)
            if msg.header.frame_id!='map' or not -.05<=now-stamp<=.5:
                return False
            if self.cloud is not None and stamp<=self.cloud.stamp:return False
            fields={f.name:f for f in msg.fields};types={7:'f4',8:'f8'}
            endian='>' if msg.is_bigendian else '<'
            dtype=np.dtype(dict(names=['x','y','z'],
                formats=[endian+types[fields[k].datatype] for k in ('x','y','z')],
                offsets=[fields[k].offset for k in ('x','y','z')],itemsize=msg.point_step))
            a=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,
                         strides=(msg.row_step,msg.point_step))
            xyz=np.column_stack([a[k].ravel() for k in ('x','y','z')])
            if not np.isfinite(xyz).all():raise ValueError('nonfinite obstacle cloud')
            xy=xyz[:,:2].copy();xy.setflags(write=False)
            self.cloud=Cloud(stamp,xy);self.error='';return True
        except (ValueError,TypeError,KeyError,AttributeError,BufferError):
            self.cloud=None;self.error='invalid_sync_cloud';return False

    def receive_grid(self,msg,now_mono,now_ros):
        return self._receive_grid(msg.header,msg.info.resolution,msg.info.width,msg.info.height,
            msg.info.origin,msg.data,now_mono,now_ros,100)

    def receive_raw(self,msg,now_mono,now_ros):
        # Foxy OccupancyGrid visualization stamps are zero and may publish only
        # deltas. Costmap_raw always has a complete grid and node_->now stamp.
        m=msg.metadata
        return self._receive_grid(msg.header,m.resolution,m.size_x,m.size_y,m.origin,
                                  msg.data,now_mono,now_ros,254)

    def _receive_grid(self,header,resolution,width,height,origin,data,now_mono,now_ros,lethal):
        try:
            stamp=self.stamp(header);p=origin.position;q=origin.orientation
            if (header.frame_id!='map' or not -.05<=now_ros-stamp<=self.timeout
                    or not .01<=resolution<=.2 or width<=0 or height<=0
                    or len(data)!=width*height
                    or not np.isfinite([p.x,p.y,q.x,q.y,q.z,q.w]).all()
                    or abs(q.x)+abs(q.y)>1e-6 or abs(q.z*q.z+q.w*q.w-1)>.01):
                return False
            if self.grid is not None and stamp<=self.grid.stamp:return False
            cells=np.asarray(data,dtype=np.int16).reshape(height,width).copy()
            if ((cells < (-1 if lethal==100 else 0))|(cells > (100 if lethal==100 else 255))).any():
                raise ValueError('invalid occupancy')
            cells.setflags(write=False)
            self.grid=Grid(stamp,now_mono,resolution,width,height,
                           (p.x,p.y),2*math.atan2(q.z,q.w),cells,lethal)
            self.error='';return True
        except (ValueError,TypeError,AttributeError):
            self.grid=None;self.error='invalid_sync_costmap';return False

    def healthy(self,now_mono,now_ros):
        if self.cloud is None or not -.05<=now_mono-self.cloud.stamp<=.5:
            return False,'costmap_sync_cloud_missing_or_stale'
        if (self.grid is None or not 0<=now_mono-self.grid.arrival<=self.timeout
                or not -.05<=now_ros-self.grid.stamp<=self.timeout):
            return False,'local_costmap_missing_or_stale'
        return True,'healthy'

    def check(self,pose,now_mono,now_ros):
        ok,reason=self.healthy(now_mono,now_ros)
        if not ok:return False,reason,{}
        if pose is None or not np.isfinite(pose[:2]).all():
            return False,'costmap_sync_pose_missing',{}
        cloud,grid=self.cloud,self.grid
        near=np.sum((cloud.xy-np.asarray(pose[:2]))**2,axis=1)<=self.radius**2
        points=cloud.xy[near]
        if not len(points):return True,'no_near_projected_obstacles',dict(near_points=0,unrepresented_points=0)
        d=points-grid.origin;c,s=math.cos(grid.yaw),math.sin(grid.yaw)
        local=np.column_stack((d[:,0]*c+d[:,1]*s,-d[:,0]*s+d[:,1]*c))
        ij=np.floor(local/grid.resolution).astype(int)
        represented=np.zeros(len(points),dtype=bool)
        # OpenVDB quantizes 5 cm points; permit only the resulting small
        # distance to a lethal cell's rectangle, not an arbitrary nearby cost.
        for dy in (-1,0,1):
            for dx in (-1,0,1):
                col,row=ij[:,0]+dx,ij[:,1]+dy
                inside=(col>=0)&(col<grid.width)&(row>=0)&(row<grid.height)
                index=np.flatnonzero(inside)
                occupied=grid.cells[row[index],col[index]]==grid.lethal
                index=index[occupied]
                delta=abs(local[index]-np.column_stack(((col[index]+.5)*grid.resolution,
                                                        (row[index]+.5)*grid.resolution)))-grid.resolution/2
                distance=np.hypot(np.maximum(delta[:,0],0),np.maximum(delta[:,1],0))
                represented[index[distance<=self.quantization]]=True
        metrics=dict(near_points=len(points),unrepresented_points=int((~represented).sum()),
                     local_costmap_arrival_age_s=now_mono-grid.arrival,
                     local_costmap_ros_age_s=now_ros-grid.stamp,
                     projection_sensor_age_s=now_mono-cloud.stamp)
        if not represented.all():return False,'waiting_obstacles_in_local_costmap',metrics
        return True,'near_obstacles_represented',metrics
