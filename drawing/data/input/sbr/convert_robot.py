import numpy as np
import random
import matplotlib.pyplot as plt
import math
from scipy.optimize import minimize, rosen, rosen_der
import random
import rospkg
import csv


class Point(object):
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def random(self, min= 0, max= 1):
        self.x = random.uniform(min,max)
        self.y = random.uniform(min,max)


class QuadBezier(object):
    def __init__(self, p0x= 0, p0y= 0, p1x= 0, p1y= 0, p2x= 0, p2y= 0):
        self.p0 = Point(p0x, p0y)
        self.p1 = Point(p1x, p1y)
        self.p2 = Point(p2x, p2y)
        self.obstacles = []

    def random(self,min= 0, max= 1):
        'Create a random quadratic Bezier curve within [min, max] limits. Default [0,1].'
        self.p0.random(min, max)
        self.p1.random(min, max)
        self.p2.random(min, max)

    def max_k(self, granuality=100):
        'Calculate maximal curvature of the quadratic Bezier curve.'
        k = 0
        for t in range(0, granuality):
            t = t / granuality
            x_d = 2 * (t - 1)*(self.p1.x - self.p0.x) + 2 * t * (self.p2.x - self.p1.x)
            y_d = 2 * (t - 1)*(self.p1.y - self.p0.y) + 2 * t * (self.p2.y - self.p1.y)
            x_dd = 2 * (self.p2.x - 2 * self.p1.x + self.p0.x)
            y_dd = 2 * (self.p2.y - 2 * self.p1.y + self.p0.y)
            k = max(k,abs(x_d*y_dd - y_d*x_dd)/math.pow(x_d**2 + y_d**2, 3/2))
        return k

    def calc_curve(self, granuality=100):
        'Calculate the quadratic Bezier curve with the given granuality.'
        B_x = []
        B_y = []
        for t in range(0, granuality):
            t = t / granuality
            x = self.p1.x + (1 - t)**2 * (self.p0.x-self.p1.x) + t**2 * (self.p2.x - self.p1.x)
            y = self.p1.y + (1 - t)**2 * (self.p0.y-self.p1.y) + t**2 * (self.p2.y - self.p1.y)
            B_x.append(x)
            B_y.append(y)
        return [B_x, B_y]

    def plot(self, granuality=10):
        'Plot the quadratic Bezier curve.'
        B = self.calc_curve(granuality)
        plt.plot(B[0], B[1], c = 'k')
        # plt.scatter([self.p0.x,self.p1.x,self.p2.x], [self.p0.y,self.p1.y,self.p2.y])
        for i in range(len(self.obstacles)):
            plt.gcf().gca().add_artist(plt.Circle((self.obstacles[i][0].x, self.obstacles[i][0].y), self.obstacles[i][1], color='r'))
        plt.axis('equal')

        return B
        # plt.ion()
        # plt.show()

    def arc_len(self, granuality=1000):
        'Calculate the arc-length of the quadratic Bezier curve.'
        B = self.calc_curve(granuality=granuality)
        a_l = 0
        for i in range(1,len(B[0])):
            a_l += math.sqrt((B[0][i]-B[0][i-1])**2 + (B[1][i]-B[1][i-1])**2)
        return a_l

    def optimize_k(self, granuality= 100, obs= True):
        'Optimize the quadratic Bezier curve to minimize the curvature. By setting obs=False, ignore the obstacles.'
        x0 = [0.0, 0.0]
        res = minimize(self.optimizer_k, x0, args= (granuality, obs), method='Nelder-Mead', tol=1e-7)
        self.p1.x = self.p1.x + res.x[0]
        self.p1.y = self.p1.y + res.x[1]

    def optimizer_k(self,x, *args):
        'Curvature optimizer function.'
        granuality = args[0]
        obs = args[1]
        o = QuadBezier()
        o.p0 = self.p0
        o.p1.x = self.p1.x+x[0]
        o.p1.y = self.p1.y+x[1]
        o.p2 = self.p2
        penalty = 0

        if obs:
            B = o.calc_curve(granuality)
            for i in range(len(B[0])):
                for j in range(len(self.obstacles)):
                    d = math.sqrt((B[0][i] - self.obstacles[j][0].x)**2 + (B[1][i] - self.obstacles[j][0].y)**2)
                    if d<self.obstacles[j][1]:
                        penalty += (self.obstacles[j][1]-d)*100
        return o.max_k(granuality) + penalty

    def optimize_l(self, granuality= 100, obs= True):
        'Optimize the quadratic Bezier curve to minimize the arc-length. By setting obs=False, ignore the obstacles.'
        x0 = [0.0, 0.0]
        res = minimize(self.optimizer_l, x0, args=(granuality, obs), method='Nelder-Mead', tol=1e-7)
        self.p1.x = self.p1.x + res.x[0]
        self.p1.y = self.p1.y + res.x[1]

    def optimizer_l(self,x, *args):
        'Arc-length optimizer function.'
        granuality = args[0]
        obs = args[1]
        o = QuadBezier()
        o.p0 = self.p0
        o.p1.x = self.p1.x+x[0]
        o.p1.y = self.p1.y+x[1]
        o.p2 = self.p2

        penalty = 0
        if obs:
            B = o.calc_curve(granuality)
            for i in range(len(B[0])):
                for j in range(len(self.obstacles)):
                    d = math.sqrt((B[0][i] - self.obstacles[j][0].x)**2 + (B[1][i] - self.obstacles[j][0].y)**2)
                    if d<self.obstacles[j][1]:
                        penalty += (self.obstacles[j][1]-d)*100

        return o.arc_len(granuality) + penalty

    def optimize(self, granuality=100, obs=True, l_multiplier=0.5, k_multiplier=0.5):
        """
        Optimize the quadratic Bezier curve to simultaniously minimize the arc-lenght and the curvature.
        Setting obs=False ignores the obstacles. l_multiplier and k_multiplier multiplies
        the outputs of their respective optimizer functions.
        """
        x0 = [0.0, 0.0]
        res = minimize(self.optimizer, x0, args=(granuality, obs, l_multiplier, k_multiplier), method='Nelder-Mead', tol=1e-7)
        self.p1.x = self.p1.x + res.x[0]
        self.p1.y = self.p1.y + res.x[1]

    def optimizer(self,x,*args):
        'Optimizer function of the arc-length and curvature simultanious optimization.'
        granuality = args[0]
        obs = args[1]
        l_multiplier = args[2]
        k_multiplier = args[3]

        return self.optimizer_l(x, granuality, obs) * l_multiplier + self.optimizer_k(x, granuality, obs) * k_multiplier

    def add_obstacle(self, x=0, y=0, radius=0):
        'Add an obstacle to the quadratic Bezier curve.'
        self.obstacles.append([Point(x,y), radius])

    def add_random_obstacle(self, min_x= 1, max_x= 0, min_y=1, max_y=0, min_radius=0.3, max_radius = 0.0):
        """Add a random obstacle to the quadratic Bezier curve. The obstacle will not cover the p0 and p2 points
        of the Bezier curve.
        """
        radius = random.uniform(min_radius,max_radius)

        d = 0
        x = 0
        y = 0
        while d<radius:
            x = random.uniform(min_x,max_x)
            y = random.uniform(min_y,max_y)
            d1 = math.sqrt((x - self.p0.x)**2 + (y - self.p0.y)**2)
            d2 = math.sqrt((x - self.p2.x) ** 2 + (y - self.p2.y) ** 2)
            d = min(d1,d2)

        self.obstacles.append([Point(x, y), radius])

    def clear(self):
        'Re-initialize the curve.'
        self.__init__()


def read_file (file_name):
    f = open(file_name)
    strokes = []
    stroke_num = 0
    rospack = rospkg.RosPack()
    w = open(rospack.get_path('drawing') + '/data/input/sbr/actions_path.csv', 'w')
    writer = csv.writer(w)

    while True:
        line = f.readline()
        if not line or line == "":
            w.close()
            break
        ctrl_points = []
        x0, y0, x1, y1, x2, y2, _, _, _, _, c, _, _= list(map(float,line.split(",")))
        s = QuadBezier(x0, y0, x1, y1, x2, y2)
        b = s.plot()

        # if i != c: # end of color
        #     i += 1
        #     w.close()
        #     w = open(rospack.get_path('drawing') + '/data/input/sbr/sbr_1_path_' + str(i)+'.txt', 'w')

        t = np.array(b).T.tolist()
        for el in t:
            data = [stroke_num, round(el[0],5), round(el[1],5), int(c)]
            writer.writerow(data)
            # w.write(str(stroke_num)+ " " +str(el[0])+ " " + str(el[1])+ " " + str(c) + "\n")
        # w.write("End\n")

        stroke_num += 1
    w.close()


    # return np.array(strokes)
    print("done")
    # plt.show()

if __name__ == '__main__':
    file_name = 'actions.csv'
    read_file(file_name)