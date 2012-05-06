import roslib; roslib.load_manifest('velocity_profiler_alpha');

from msg_alpha.msg._PathSegment import PathSegment

class PathSegmentMsg(PathSegment):
    def __init__(self):
        PathSegment.__init__(self)

    def __eq__(self,obj):
        if isinstance(other, self.__class__):
            return self.__dict__ == other.__dict__
        else:
            return False
            
