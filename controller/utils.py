import json
class Quaternion():
    def __init__(self,a=1,b=0,c=0,d=0):
        self._a = a
        self._b = b
        self._c = c
        self._d = d

    def update(self,a,b,c,d):
        self._a = a
        self._b = b
        self._c = c
        self._d = d
    
    def plus(self,q):
        a = self._a + q._a
        b = self._b + q._b
        c = self._c + q._c
        d = self._d + q._d
        return Quaternion(a,b,c,d)
    
    def minus(self,q):
        a = self._a - q._a
        b = self._b - q._b
        c = self._c - q._c
        d = self._d - q._d
        return Quaternion(a,b,c,d)
    
    def hamilton(self,q):
        a1 = self._a
        b1 = self._b
        c1 = self._c
        d1 = self._d

        a2 = q._a
        b2 = q._b
        c2 = q._c
        d2 = q._d

        a = a1*a2 - b1*b2 - c1*c2 - d1*d2
        b = a1*b1 + b1*a2 + c1*d2 - d1*c2
        c = a1*c2 - b1*b2 + c1*a2 + d1*b2
        d = a1*d2 + b1*c2 - c1*b2 + d1*a2
        return Quaternion(a,b,c,d)
    def conjugate(self):
        a = self._a
        b = self._b
        c = self._c
        d = self._d

        return Quaternion(a,-b,-c,-d)

    def integrate(self,dt):
        a = self._a*dt
        b = self._b*dt
        c = self._c*dt
        d = self._d*dt

        return Vector(a,b,c,d)

    def unit(self):
        a = self._a
        b = self._b
        c = self._c
        d = self._d

        mag = (a**2+ b**2 + c**2 +d**2)**.5
        a/=mag
        b/=mag
        c/=mag
        d/=mag
        return Quaternion(a,b,c,d)
        
    def gravity(self,attitude,gravity):
        unit_attitude = attitude.unit()
        conjugate_unit_attitude = unit_attitude.conjugate()
        adjusted_gravity = unit_attitude.hamilton(gravity).hamilton(conjugate_unit_attitude)
        minus_gravity = self.minus(adjusted_gravity)
        return minus_gravity

    def json(self):
        a = self._a
        b = self._b
        c = self._c
        d = self._d
        return json.dumps({'a':a,'b':b,'c':c,'d':d})

    def print(self):
        a = self._a
        b = self._b
        c = self._c
        d = self._d

        string= "a: {:.2f} b: {:.2f} c: {:.2f} d: {:.2f}".format(a,b,c,d)
        print(string)

    def vector(self):
        a = self._a
        b = self._b
        c = self._c
        d = self._d

        return Vector(a,b,c,d)
        
class Vector():
    def __init__(self,a=1,b=0,c=0,d=0):
        self._a = a
        self._b = b
        self._c = c
        self._d = d
    
    def plus(self,q):
        a = self._a + q._a
        b = self._b + q._b
        c = self._c + q._c
        d = self._d + q._d
        return Vector(a,b,c,d)

    def integrate(self,dt):
        a = self._a*dt
        b = self._b*dt
        c = self._c*dt
        d = self._d*dt

        return Vector(a,b,c,d)

    def get(self):
        a = self._a
        b = self._b
        c = self._c
        d = self._d

        return(a,b,c,d)

    def unit(self):
        a = self._a
        b = self._b
        c = self._c
        d = self._d

        mag = (a**2+ b**2 + c**2 +d**2)**.5
        a/=mag
        b/=mag
        c/=mag
        d/=mag
        return Vector(a,b,c,d)

    def print(self):
        a = self._a
        b = self._b
        c = self._c
        d = self._d

        string= "a: {:.2f} b: {:.2f} c: {:.2f} d: {:.2f}".format(a,b,c,d)
        print(string)

    def json(self):
        a = self._a
        b = self._b
        c = self._c
        d = self._d
        return json.dumps({'a':a,'b':b,'c':c,'d':d})


    