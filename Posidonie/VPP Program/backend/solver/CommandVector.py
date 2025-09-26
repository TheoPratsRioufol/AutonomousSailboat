

class CommandVector():
    # Command vector of the boat
    def __init__(self, sail0, rudder0, wang0, wspeed0):
        # Prefix 'u' for command vector
        self._u_sail   = sail0
        self._u_rudder = rudder0
        self._u_wang   = wang0
        self._u_wspeed = wspeed0
        
    def copy(self):
        """
        Return a copy of this object
        """
        return CommandVector(self._u_sail, self._u_rudder, self._u_wang, self._u_wspeed)
    
    def toJSON(self):
        """
        Serialize this object to a JSON format
        """
        return {'type':'CommandVector',
                'sail':self._u_sail,
                'rudder':self._u_rudder,
                'wang':self._u_wang,
                'wspeed':self._u_wspeed
                }
    
    def fromJSON(dic):
        """
        Return an object from a JSON dictionnary
        """
        if (dic['type'] != 'CommandVector'):
            return None
        
        return CommandVector(dic['sail'],
                             dic['rudder'],
                             dic['wang'],
                             dic['wspeed'])
    
    def __add__(self, b):
        """
        Add Two Command vector
        """
        if not isinstance(b, CommandVector):
            raise TypeError()
        
        return CommandVector(self._u_sail + b._u_sail,
                             self._u_rudder + b._u_rudder,
                             self._u_wang + b._u_wang,
                             self._u_wspeed + b._u_wspeed)

    def __mul__(self, b):
        """
        Multiply a CommandVector by a scalar
        """ 
        if not (isinstance(b, int) or isinstance(b, float)):
            raise TypeError()
        
        return CommandVector(self._u_sail*b,
                             self._u_rudder*b,
                             self._u_wang*b,
                             self._u_wspeed*b)
    
    def __sub__(self, b):
        """
        Substract Two CommandVector vector
        """
        return self.__add__(b.__mul__(-1))

    def __truediv__(self, b):
        """
        Divide a CommandVector by a scalar
        """
        return self.__mul__(1/b)
    
    def __str__(self):
        return str(self.toJSON())