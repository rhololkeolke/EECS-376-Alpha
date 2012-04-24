# this is the class that stores computed trajectory information that the velocity profiler
# will use to execute the desired path

class TrajSeg:
    '''
    This class stores computed trajectory information that the velocity profiler will use to execute the desired path
    '''
    # segment types
    ACCEL = 0
    CONST = 1
    DECEL = 2

    def __init__(self,segType,endS,v_i,v_f,segNumber):
        '''
        segType is one of TrajSeg's class variables
        endS is the s value this segment should end at for the associated path segment. This should be between 0 and 1
        v_i is the initial velocity of this segment
        v_f is the final velocity of this segment
        segNumber is the path segment number this trajectory segment is associated with
        '''

        if(segType < 0 or segType > 2):
            raise NameError('segment type must one of the values defined in TrajSeg class')
        self.segType = segType # segment type e.g. ACCEL
        self.v_i = v_i # initial velocity
        self.v_f = v_f # final velocity
        if(endS < 0.0 or endS > 1.0):
            raise NameError('ending s value must be between 0 and 1')
        self.endS = endS # ending s value (must be between 0 and 1)
        self.segNumber = segNumber# the path segment number. This will be used to access the rest of the path segment properties
        
