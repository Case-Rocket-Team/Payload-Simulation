# class stores common math methods
class util: 

    @staticmethod
    def clamp(value, max, min):
        if(value < min):
            return min
        if(value > max): 
            return max
        return value
    @staticmethod
    def magnitude(x1, y1, z1, x2, y2, z2):
        mag = ((x2-x1) ** 2 + (y2-y1) ** 2 + (z2-z1) ** 2) ** (1/2)
        return mag