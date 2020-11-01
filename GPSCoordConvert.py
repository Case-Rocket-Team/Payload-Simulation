import math
import numpy as np

a = 6378137
b = 6356752.3142
f = (a - b) / a
e_sq = f * (2-f)
class GPSCoordConvert:
    def geodetic_to_ecef(self, lat, lon, h):
        # (lat, lon) in WSG-84 degrees
        # h in meters
        lamb = math.radians(lat)
        phi = math.radians(lon)
        s = math.sin(lamb)
        N = a / math.sqrt(1 - e_sq * s * s)

        sin_lambda = math.sin(lamb)
        cos_lambda = math.cos(lamb)
        sin_phi = math.sin(phi)
        cos_phi = math.cos(phi)

        x = (h + N) * cos_lambda * cos_phi
        y = (h + N) * cos_lambda * sin_phi
        z = (h + (1 - e_sq) * N) * sin_lambda

        return x, y, z

    def ecef_to_geodetic(self, x, y, z):
        # constants: 
        a = 6378137
        f =  1/298.257223563
        e = math.sqrt(2*f-f**2) 
        b = 6356752.314 
        A = 42697.673
        B = 42841.312
        w = math.sqrt(x ** 2 + y ** 2)
        l = (e ** 2)/ 2
        m = (w/a) ** 2
        n = ((1-e**2)*z/b) ** 2
        i = -(2*(l**2) +m +n)/2
        k = l**2 * ((l**2)-m-n)
        q = ((m + n -4*(l**2)) ** 3)/216+m*n*(l**2)
        D = math.sqrt((2*q - m*n*(l**2))*m*n*(l**2))
        beta = i/3-math.pow(q+D, 1/3) - math.pow(q-D, 1/3)
        t = math.sqrt(math.sqrt((beta**2) - k) - (beta-i)/2)-np.sign(m-n)*math.sqrt((beta-i)/2)
        w1 = w/(t+l)
        z1 = (1-e**2) * z /(t-l)
        psi = math.atan(z1/((1-e**2)*w1))
        lamda = 2 * math.atan((w-x)/y)
        h = np.sign(t -1 +l) * math.sqrt((w-w1)**2+(z-z1)**2)
        return psi, lamda, h

    def ecef_to_enu(self, x, y, z, lat0, lon0, h0):
        lamb = math.radians(lat0)
        phi = math.radians(lon0)
        s = math.sin(lamb)
        N = a / math.sqrt(1 - e_sq * s * s)

        sin_lambda = math.sin(lamb)
        cos_lambda = math.cos(lamb)
        sin_phi = math.sin(phi)
        cos_phi = math.cos(phi)

        x0 = (h0 + N) * cos_lambda * cos_phi
        y0 = (h0 + N) * cos_lambda * sin_phi
        z0 = (h0 + (1 - e_sq) * N) * sin_lambda

        xd = x - x0
        yd = y - y0
        zd = z - z0

        xEast = -sin_phi * xd + cos_phi * yd
        yNorth = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd
        zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd

        return xEast, yNorth, zUp

    # Broken currently, need to check matrix math
    '''
    def enu_to_ecef(self, xEast, yNorth, zUp, lat0, lon0, h0):
        lamb = math.radians(lat0)
        phi = math.radians(lon0)
        s = math.sin(lamb)
        N = a / math.sqrt(1 - e_sq * s * s)

        sin_lambda = math.sin(lamb)
        cos_lambda = math.cos(lamb)
        sin_phi = math.sin(phi)
        cos_phi = math.cos(phi)

        x0 = (h0 + N) * cos_lambda * cos_phi
        y0 = (h0 + N) * cos_lambda * sin_phi
        z0 = (h0 + (1 - e_sq) * N) * sin_lambda

        xd = - sin_lambda * xEast - sin_phi * cos_lambda * yNorth + cos_phi * cos_lambda * zUp
        yd = cos_lambda * xEast - sin_phi * sin_lambda * yNorth + cos_phi * sin_lambda * zUp
        zd = cos_phi * yNorth + sin_phi * zUp

        x = xd + x0
        y = yd + y0
        z = zd + z0

        return x, y, z
    '''

    def geodetic_to_enu(self, lat, lon, h, lat_ref, lon_ref, h_ref):
        x, y, z = self.geodetic_to_ecef(lat, lon, h)
        
        return self.ecef_to_enu(x, y, z, lat_ref, lon_ref, h_ref)

    # Currently broken due to the enu to ecef conversion not working
    '''
    def enu_to_geodetic(self, xEast, yNorth, zUp, lat_ref, lon_ref, h_ref):
        x, y , z = self.enu_to_ecef(xEast, yNorth, zUp, lat_ref, lon_ref, h_ref)

        return self.ecef_to_geodetic(x, y, z)
    '''
