from util import util 
import math
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import pandas as pd

def checkCurveRadius(a, b, targ_dist):
        y_deriv = a*b*math.pi/targ_dist*math.cos(math.pi/2)
        y_2_deriv = -a*b**2*math.pi**2/targ_dist**2*math.sin(math.pi/2)
        curve_radius = (1+y_deriv**2)**(3/2)/abs(y_2_deriv)
        return(curve_radius)

def waypointGenerator(r_min, req_arc_length, targ_dist):
    b = 1
    arc_length = 0
    a = 1
    curve_radius = 0
    flag = False
    # Generate sine wave that is the proper length
    # print(req_arc_length, arc_length)
    while (arc_length >= req_arc_length + 100 or arc_length <= req_arc_length - 100):
        a += 1
        arc_length = 2 * b * util.magnitude(0,0,0,targ_dist/2/b,a,0)
        curve_radius = checkCurveRadius(a,b,targ_dist)
        #print(arc_length)
        if(curve_radius <= r_min or a > 300):
            b += 1
            a = 1
            arc_length = 0
        if(b > 20):
            flag = True
            return flag
    return flag

def main():
    targ_limits = []
    check = 0
    prev_flag = False

    # targ_dist = 500
    # for needed_arc_length in range(500,0,-1):
    #     # print(targ_dist, needed_arc_length)
    #     flag = waypointGenerator(70, needed_arc_length, targ_dist)
    #     if(not flag):
    #         print(targ_dist, "Altitude: ", needed_arc_length * 7.35 / 17.07, "Good Path") 
    #     elif(flag):
    #         print(targ_dist, "Altitude: ", needed_arc_length * 7.35 / 17.07, "Bad Path")

    for targ_dist in range(500,0,-10):
        for needed_arc_length in range(1000,0,-1):
            # print(targ_dist, needed_arc_length)
            flag = waypointGenerator(70, needed_arc_length, targ_dist)
            if(not flag and check == 0):
                #targ_limits.append([targ_dist, needed_arc_length * 7.35 / 17.07, "Upper Bound"]) 
                upper_bound = needed_arc_length * 7.35 / 17.07
            elif(flag and not prev_flag and check != 0):
                #targ_limits.append([targ_dist, needed_arc_length * 7.35 / 17.07, "Lower Bound"])
                lower_bound = needed_arc_length * 7.35 / 17.07
                targ_limits.append([targ_dist,upper_bound,lower_bound])
                break
            elif(not flag and needed_arc_length == 1):
                #targ_limits.append([targ_dist, 0, "Lower Bound"])
                lower_bound = needed_arc_length * 7.35 / 17.07
                targ_limits.append([targ_dist,upper_bound,lower_bound])
            if(not flag):
                check += 1
            prev_flag = flag
        check = 0
    tl = pd.DataFrame(targ_limits, columns=["Target Distance", "Alt Upper Bound", "Alt Lower Bound"])
    tl.plot(x="Target Distance", y="Alt Upper Bound"-"Alt Lower Band")
    tl.plot(x="Target Distance", y="Alt Lower Bound")
    plt.show()
    tl.to_csv('GoodPathCoords.csv',index=None)
    print(tl)
    # for i in range(0,len(targ_limits),2):
    #     if(targ_limits[i][2] == "Upper Bound" and targ_limits[i+1][2] == "Lower Bound"):
    #         print(targ_limits[i][0], "Altitude Range", targ_limits[i][1]-targ_limits[i+1][1])

    # alt_limits = []
    # for needed_arc_length in range(5000,0,-100):
    #     for targ_dist in range(2000,0,-100):
    #         # print(targ_dist, needed_arc_length)
    #         flag = waypointGenerator(70, needed_arc_length, targ_dist)
    #         if(not flag and check == 0):
    #             alt_limits.append([targ_dist, needed_arc_length * 7.35 / 17.07, "Upper Bound"]) 
    #         elif(flag and not prev_flag and check != 0):
    #             alt_limits.append([targ_dist, needed_arc_length * 7.35 / 17.07, "Lower Bound"])
    #         if(not flag):
    #             check += 1
    #         prev_flag = flag
    #     check = 0
    # print(alt_limits)

if __name__ == "__main__":
    main() 