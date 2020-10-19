# class stores common math methods
class util: 

    @staticmethod
    def clamp(value, max, min):
        if(value < min):
            return min
        if(value > max): 
            return max
        return value