from __future__ import print_function
from math import sqrt
from collections import namedtuple
from .util import sign
from .sim import SimSegment
        
DEFAULT_ACCELERATION = 50000


class SegmentError(Exception):
    pass


class CantNormalize(Exception):
    pass
    

class VrOutOfRangeError(Exception):
    pass


class TimeTooShortError(SegmentError):
    
    def __init__(self, message, calc_x=None, t_min= None, x_max=None):
        super(SegmentError, self).__init__(message)
        self.calc_x = calc_x
        self.x_max = x_max
        self.t_min = t_min



class SegmentList(object):
    
    def __init__(self, n_joints, v_max, a_max, d_max = None):
        
        self.n_joints = n_joints
        
        self.segments = []

        self.velocities = [0] * self.n_joints
        self.positions = [0] * self.n_joints
        
        self.v_max = v_max
        self.a_max = a_max
        
        self.d_max = d_max if d_max is not None else self.a_max
        
        
    def set_velocities(self, velocities):
        assert len(velocities) == self.n_joints
        
        self.velocities = velocities
        
        
    def set_positions(self, positions):
        assert len(positions) == self.n_joints
        
        self.positions = positions
        
    def add_velocity_segment(self, joints, t=None, x = None):
        """Add a new segment, with joints expressing velocities"""
        
        return self.add_distance_segment([_v*t for _v in joints], t = t)
        
        
    def add_distance_segment(self, joints, t=None, v=None):
        """Add a new segment, with joints expressing joint distance """
        assert len(joints) == self.n_joints
        
        max_t = max(float(x)/float(self.v_max) for x in joints )
        
        if t is None:
            if v is None:
                v = self.v_max
                t = max_t
            else:
                vector_x = sqrt(sum( x**2 for x in joints)) # Total length of the vector
                t = vector_x/v

        elif max_t > t:
            t  = max_t

        segmentjoints = []
        
        for i,x in enumerate(joints):
            
            if len(self.segments) == 0:       
                v0 = self.velocities[i]
            else:
                # Since we're getting a new segment, we can allow the previous one to 
                # have a non-zero final velocity. However, if the next segment has an opposite
                # direction, then we have to keep the 0 final velocity. 
                last_seg_joint = self.segments[-1].joints[i]

                # Setup the re-calc to let the final velocity float. Then the last segment
                # will continue from whatever velocity we get in the second-to-last
                if last_seg_joint.direction == sign(x):
                    last_seg_joint.v1 = None
                    last_seg_joint.td = 0
                    last_seg_joint.calc_vr()
              
                v0 = last_seg_joint.v1

            # Always give the new segment a 0 final velocity, so if no more segments
            # are added the system will stop. 
            try:
                js = JointSegment(x=x, t=t, v0=v0, v1=0, a=self.a_max, d=self.d_max, v_max=self.v_max)
            except TimeTooShortError as e:
                # Error b/c the time was too short for the commanded distnace. Insert the errors in the list, 
                # then check for them later, collecting the minimum time to complete all of the
                # joint distances
                js = e
            
            segmentjoints.append(js)
        
        # Now look for errors, and determine the min time to complete the segment, 
        # the we can complete re-run the segment. 
        if sum( 1 for e in segmentjoints if isinstance(e, TimeTooShortError )) > 0:
            
            t_min = max( j.t_min for j in segmentjoints if isinstance(j, TimeTooShortError ))
            return self.add_distance_segment(joints, t_min)
           
        if len(self.segments):
            self.segments[-1].normalize_times()
        
        segment = Segment(segmentjoints, t=t, a=self.a_max, d=self.d_max, v_max = self.v_max )
        segment.normalize_times()
        
        self.segments.append(segment)
        
        return None
        
    def normalize_times(self):
        
        for s in self.segments:
            s.normalize_times()
        
    def iter_subsegments(self):
        
        for segment in self.segments:
            for ss in segment:
                yield ss
      
    def pop(self):
        return self.segments.pop(0)
      
    def __len__(self):
        return len(self.segments)
    
    def __str__(self):
        out = ''
        for s in self.segments:
            out += str(s)
        return out


class SegmentBuffer(SegmentList):
        
        
    def add_segment(self, joints, t=None, v=None):
        
        super(SegmentBuffer, self).add_segment(joints, t, v)

        if len(self) > 1:
            return self.pop();
        else:
            return None
            
SubSegment = namedtuple('SubSegment','t x t_seg joints'.split())
SubSegmentJoint = namedtuple('SubSegmentJoint','x v0 v1'.split())


class Segment(object):
    
    def __init__(self, joints, t, a, d, v_max):
        
        self.joints = joints
        self.x  = sqrt(sum( j.x**2 for j in joints)) # Vector length
        self.t = t
       
        self.a = a
        self.d = d
        self.v_max = v_max
        
        self.ta = None
        self.td = None
        
        self._iter = None
    
    def normalize_times(self):
        
        max_ta = max(j.ta for j in self.joints)
        max_td = max(j.td for j in self.joints)

        if max_ta + max_td > self.t:
            print("VVVVVV")
            print(self)
            print("^^^^^^")
            raise CantNormalize("Can't normalize: accl {} + decl {} is greater than total time {}"\
            .format(max_ta, max_td, self.t ))
        
        for j in self.joints:
            if j.ta != max_ta and j.td != max_td and j.x > 0 and ( j.ta > 0 or j.td > 0):
                j.ta = max_ta
                j.td = max_td
                j.calc_vr()
        
        self.ta = max_ta
        self.td = max_td
            
    def __iter__(self):

         #self.normalize_times()
        
         def calc_x(t, v0, v1, dir):
             x = t * (v0+v1)/2.
        
             return SubSegmentJoint( x=(round(x,0)*dir), v0=(round(v0,3)*dir), v1=(round(v1,3)*dir))
        
            
         if self.ta > 0:
             yield SubSegment(t_seg=(round(self.ta,3)), x=None, t=None,
                              joints=[ calc_x(self.ta, j.v0, j.vr, j.direction ) for j in self.joints ])
            
         tr = self.t-self.ta-self.td
            
         if tr > 0:
             yield SubSegment(t_seg=tr,  x=None, t=None,
                              joints=[ calc_x(tr, j.vr, j.vr, j.direction ) for j in self.joints ])
        
         if self.td > 0:
             yield SubSegment(t_seg=(round(self.td,3)),  x=None, t=None,
                              joints=[ calc_x(self.td, j.vr, j.v1, j.direction )  for j in self.joints ])
    
    def set_joint(self, i, js):
        
        self.joints[i] = js
        
    def new_seg(self, x=None, v=None, t=None, v0=0, v1=None):
    
        js = JointSegment(x=x,v=v,t=t,v0=v0, v1=v1, a=self.a, d=self.d, v_max=self.v_max )
    
        
    def __str__(self):
        
        out = "== t={} x={}\n".format(self.t, self.x)
        for js in self.joints:
            out += '   '+str(js)+'\n'
            
        return out


class JointSegment(object):
    """ Description of a trapezoidal velocity profile for a segment of motion on 
    a single joint 
    
    A JointSegment has a constant x. A SubSegment has a constant a.
    
    """
    
    def __init__(self, x=None, v=None, t=None, v0=0, v1=None, v_max = None, a = DEFAULT_ACCELERATION, d = None):
        
        # Acceleration and deceleration
        self.a = float(a)
        self.d = float(d) if d is not None else float(a)
        
        self.v_max = float(v_max) # Maximum allowed velocity
        
        #Each segment must have a consistent, single-valued accceleration
        if sign(v0)*sign(v1) < 0: # both signs are nonzero and opposite
            raise Exception("The velocity trajectory cannot cross zero. Got {}->{}".format(v0, v1))
        
        self.v0 = float(v0) # Velocity at start of segment
        self.v1 = float(v1) if v1 is not None else None # Velocity at end of segment
        
        # These three parameters must be related by x = vt
        # at least two of these must be specified
        self.direction = sign(x)
        self.x = float(abs(x)) if x is not None else None # segment distance
        self.v = float(v) if v is not None else None # average velocity
        self.t = float(t) if t is not None else None # Total segment time
        
        specd_vars = sum( 1 for var in (self.x,self.v,self.t) if var is not None)
        
        if specd_vars == 3:
            raise ValueError("Must specify 2 or fewer of x, v or t")
        elif specd_vars == 1 and self.x is None:
            raise ValueError("If only one of x,v,t is specified, it must be x")
        
        
        if self.x is None:
            self.x = self.v * self.t
        elif self.t is None:
            if self.v is None:
                self.t = self.x / self.v
            else:
                # Case when only x is specifies
                # Compute the minimum t for x. 
                
                v1 = v1 if v1 else 0
                self.t = (v1-self.v0)/(self.a)
                
        elif self.v is None:
            self.v = self.x / self.t
        else:
            assert False, (x,v,t)
        
        
        
        self.vrmax = None # Max v achievable for acel from v0 and decel to v1
        self.xmax = None # Max
        self.xmin = None
        self.ts = None # Time at vrmax
        
        self.vr = None # Run velocity
        self.ta = None # Acceleration time, from start of seg to end of accelerations
        self.td = None # Decleration time, from start of deceleration to end of segment

        self.calc_vr()



    def calc_vr(self):
        
        self.ta, self.td, self.vr, new_v1, self.ts, self.vrmax, self.xmin, calc_x, self.xmax = \
            self._calc_vr(self.x, self.t, self.v0, self.v1, self.a, self.d)
        
        if self.x != 0 and abs(calc_x - self.x) / abs(self.x) > .01:
            diff = (calc_x - self.x)
            r_err =  (calc_x - self.x) / self.x
            raise VrOutOfRangeError("Calc x is {}, expected {}, diff {}, rel err {}"\
                                   .format(calc_x, self.x, diff, r_err))
            
        assert int(self.xmin) <= int(calc_x) <= int(self.xmax), (self.xmin,calc_x,self.xmax)

        self.v1 = new_v1
        
        
    @staticmethod
    def calc_trap_area(t,vr, v1, v0, ta, td, a, d):
        """Find x for Vr for use with a binary search. There is probably an analytical solution, 
           but I'm tired of trying to figure it out. The function isn't continuous, so
           even an analytical solution would probably have some if statements in it."""

        if ta is None:
            ta = abs((vr-v0)/a) # Linear extension from vo at slope a to vr
            
        if td is None: 
            td = abs((vr-v1)/d) # Backward linear extension from v1 at slope d to vr

        x = ( .5*ta*(v0+vr) + # Area under accel trapezoid
              .5*td*(v1+vr) + # Area under decel trapezoid
              vr*(t-ta-td)) # Area under constant v section
      

        return x, ta, td, v1
        

    def _calc_vr(self, x,t,v0,v1, a, d):
        """Computue the run velocity, acceleration break time, and deceleration break time
        for other trapezoidial profile parameters. """
        import math
        
        ta_in = self.ta
        td_in = self.td
        
        assert t != 0
        
        if self.v1 is None: # 

            vrmax = a*t+v0 # Accelerate all the way through the segment
            vrmax = min(vrmax, self.v_max)
            
            xmax = .5 * a * t * t + v0
            xmin = .5*v0**2/a
            ts = float('nan')
            td_in = 0
            v1 = 0
        else:
            v1 = self.v1
            
            ts = (d*t + v1 - v0) / (2*a) # Point where acel line from v0 meets decel line from v1
            
            assert ts <= t, (ts, t, v1, v0, a, d)
            
            # Calc vrmax from acceleration from v0 + decel from v1, and average
            vrmax = (v0+v1+a*ts+a*(t-ts)) / 2.
            
            
            if vrmax > self.v_max:
                # When vrmax > x_max, the maximum distnace shape won't be a triangle profile, 
                # so the triangle equation wont work
                xmax,_,_,_ = self.calc_trap_area(t=t, vr=self.v_max, v0=v0,v1=v1, a=a,d=d, ta=None, td=None)
                vrmax = self.v_max
                assert(xmax > 0), (xmax,t, v0, v1)
            else:
                xmax = .5*ts*(v0+vrmax) + .5 * (t-ts)*(v1+vrmax)
                assert(xmax > 0), (xmax,ts, vrmax, t, v0, v1)
                
            xmin = .5*v0**2/a + .5*v1**2/d


        if round(x,0) > round(xmax,0): # Error, time is not long enough to cover commanded distance
            
            #print "!!!! t={} x={} xmax={}, vrmax={}, self.v_max={}".format(t, int(x), int(xmax), vrmax, self.v_max)
            
            # calculate the minimum distance for accelerating to max velocity, then
            # back down to v1
            ta = (self.v_max-v0)/a
            td = (self.v_max)/d
            x1 = ( .5*ta*(v0+self.v_max) + # Area under accel trapezoid
                   .5*td*(v1+self.v_max) ) # Area under decel trapezoid
            
            # Any remaining distance to cover will be at constant v_max
            if x > x1:
                xr = x - x1
                tr = xr / self.v_max
            else:
                tr = 0 
                xr = 0
            
            xmin_vmax = x1+xr
            tmin_vmax = ta+td+tr
            
            #print 'XXX', t, xmin_vmax, tmin_vmax
            
            raise TimeTooShortError(
                "Commanded distance {} is larger that xmax {} for given time of {}s "\
                .format(self.x, xmax, t),
                t_min = tmin_vmax
            )

        if round(x,0) < round(xmin,0):
            raise SegmentError("Commanded distance {} is smaller that xmin {} for given time ".format(self.x, xmin))

        low = 0
        high = int(math.ceil(vrmax))

        while high - low > .00001:
    
            vr = (low + high) / 2.0

            x_l, _, _  , v1 = self.calc_trap_area(t, low,  v1, v0, ta_in, td_in, a, d)
            x_m, ta, td, v1 = self.calc_trap_area(t, vr,   v1, v0, ta_in, td_in, a, d)
            #x_h, _, _  , v1 = self.calc_trap_area(t, high, v1, v0, ta_in, td_in, a, d)
    
            if round(x_l,5) <= x <= round(x_m,5):
                high = vr
            else:
                low = vr
                
        if td == 0:
            v1 = vr


        return round(ta,4), round(td,4), round(vr,4), v1, ts, vrmax, xmin, x_m, xmax
 
    def _calc_min_t(self, x,v0,v1, a, d):
        """Compute the minimum time to travel the distance. This will be a triangular """

 
    @property
    def dir_x(self):
        """X with the direction, as a sign"""
        return self.direction * self.x
 
    @property
    def dir_v0(self):
        """V0 with the direction, as a sign"""
        return self.direction * self.v0
    
    @property    
    def dir_v1(self):
        """V1 with the direction, as a sign"""
        return self.direction * self.v1
        
    @property
    def dir_vr(self):
        """Vr with the direction, as a sign"""
        return self.direction * self.vr
 
    def __str__(self):

        return "{} v0={:6.2f} ta={:6.2f} vr={:6.2f} v1={:6.2f} td={:6.2f} xmin={:10.2f} x={:10.2f} xmax={:10.2f} vrmax={:6.2f} ts={:6.2f}"\
        .format(self.t, self.dir_v0, self.ta, self.dir_vr, self.dir_v1, self.td, self.xmin * self.direction, self.dir_x, 
        self.xmax * self.direction, self.vrmax *self.direction, self.ts)
        

class SegmentIterator(object):
    
    def __init__(self, segment_list, positions = None, time = 0):
        
        self.segment_list = segment_list
        
        self.current_segment = None
        
        self.sub_segments = None
        
        self.time = time
        
        self.positions = positions if positions is not None else [0]*segment_list.n_joints
        self.velocities = positions if positions is not None else [0]*segment_list.n_joints
        
    def __iter__(self):
        return self
        
    def next(self):
        return self.__next__()
        
    def __next__(self):
        
        if self.current_segment is None:
            
            if len(self.segment_list) == 0:
                raise StopIteration()
            
            self.current_segment = self.segment_list.pop()
            self.segment_list.set_velocities([j.v1 for j in self.current_segment.joints])
            self.sub_segments = list(self.current_segment)
            
        ss = self.sub_segments.pop(0)
        
        self.positions = [ sum(e) for e in zip(self.positions, [ssj.x for ssj in ss.joints])]
        self.velocities = [ssj.v1 for ssj in ss.joints]
        self.time += ss.t_seg

        
        if len(self.sub_segments) == 0:
            self.current_segment = None
        
        # Can't modify a named tuple
        return SubSegment(t_seg=ss.t_seg, t=self.time, x=self.positions, joints=ss.joints)
            
        

        
        
    
        