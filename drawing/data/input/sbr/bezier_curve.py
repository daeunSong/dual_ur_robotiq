import numpy as np

class Curves:
    """
    Define a cubic Bezier curve interpolation
    """
    def __init__(self, waypoints):
        self.nd = waypoints.shape[1]
        self.num_points = waypoints.shape[0]
        self.num_curves = self.num_points - 1
        self.waypoints = waypoints

        self.A, self.B = self.get_bezier_coef(self.waypoints) # coefficients
        self.T = self.get_bezier_cubic() # functions

        # n - 1 curves
        self.curves = np.array([BezierCurve(self.A[i], self.B[i], self.T[i]) for i in range(self.num_curves)])
        self.total_length = sum(curve.length for curve in self.curves)

    def reset_bezeir (self):
        self.T = self.get_bezier_cubic()
        self.curves = np.array([BezierCurve(self.A[i], self.B[i], self.T[i]) for i in range(self.num_curves)])

    # find the a & b points
    def get_bezier_coef(self, waypoints):
        # since the formulas work given that we have n+1 points
        # then n must be this:
        n = self.num_curves

        # build coefficents matrix
        C = 4 * np.identity(n)
        np.fill_diagonal(C[1:], 1)
        np.fill_diagonal(C[:, 1:], 1)
        C[0, 0] = 2
        C[n - 1, n - 1] = 7
        C[n - 1, n - 2] = 2

        # build points vector
        P = [2 * (2 * waypoints[i] + waypoints[i + 1]) for i in range(n)]
        P[0] = waypoints[0] + 2 * waypoints[1]
        P[n - 1] = 8 * waypoints[n - 1] + waypoints[n]

        # solve system, find a & b
        A = np.linalg.solve(C, P)
        B = [0] * n
        for i in range(n - 1):
            B[i] = 2 * waypoints[i + 1] - A[i + 1]
        B[n - 1] = (A[n - 1] + waypoints[n]) / 2
        B = np.array(B)

        return A, B

    # returns the general Bezier cubic formula given 4 control points
    def get_cubic(self, a, b, c, d):
        return lambda t: np.power(1 - t, 3) * a + 3 * np.power(1 - t, 2) * t * b + 3 * (1 - t) * np.power(t, 2) * c + np.power(t, 3) * d

    # return one cubic curve for each consecutive points
    def get_bezier_cubic(self):
        return [self.get_cubic(self.waypoints[i], self.A[i], self.B[i], self.waypoints[i + 1])
            for i in range(self.num_curves)
        ]

class BezierCurve:
    """
    Define a cubic Bezier curve composed of four control points.
    """
    def __init__(self, A, B, T):
        self.nd = A.shape[0]
        self.Pi = T(0) # waypoint i
        self.A = A
        self.B = B
        self.Pi_1 = T(1) # waypoints i+1
        self.control_points = [self.Pi, self.A, self.B, self.Pi_1]
        self.t = T

        self.tp = self.get_first_deriv(self.Pi, self.A, self.B, self.Pi_1)
        self.tpp = self.get_second_deriv(self.Pi, self.A, self.B, self.Pi_1)
        self.length = self.get_curve_length()

    # returns the first derivative
    def get_first_deriv(self, a, b, c, d):
        return lambda t: (-3) * np.power(1 - t, 2) * a + 3 * (1 - 3*t) * (1 - t) * b + 3 * t * (2 - 3*t) * c + 3 * np.power(t, 2) * d

    # returns the second derivative
    def get_second_deriv(self, a, b, c, d):
        return lambda t: 6 * (1 - t) * a + 6 * (3*t - 2) * b + 6 * (1 - 3*t) * c + 6 * t * d

    # Add 100 points along the curve and measure each segmentation length
    # Approximation for measure the arc length
    def get_curve_length(self, n = 101):
        pt1 = self.t(0)
        len = 0
        for t in np.linspace(0, 1, n):
            pt2 = self.t(t)
            seg_len = np.linalg.norm(pt2 - pt1)
            pt1 = pt2
            len += seg_len
        return len

    def parameterized (self, u, n = 1001):
        if u == 1 : return self.t(1)
        target_len = u * self.length
        pt1 = self.t(0)
        len = 0
        for i in np.linspace(0, 1, n):
            pt2 = self.t(i)
            seg_len = np.linalg.norm(pt2 - pt1)
            len += seg_len
            if target_len == len : return self.t(i)
            elif target_len < len: # interpolate
                diff = len - target_len
                return pt2 - (diff * (pt2-pt1))/seg_len
            pt1 = pt2
        return self.t(u)

if __name__ == '__main__':
    # generate 5 (or any number that you want) random points that we want to fit (or set them youreself)
    # waypoints = np.random.rand(5, 2)
    from read_file import read_waypoints_from_file
    from plot_bezier import plot_bezier

    file_name = "../input/heart_path_c.txt"
    waypoints, width, height = read_waypoints_from_file(file_name)
    # c = Curves(waypoints)
    # plot_bezier(c, width/50, height/50)
    c = Curves(waypoints[:15])
    # plot_bezier(c)

    from plot_bezier import parameterize_bezier, write_file
    waypoints = parameterize_bezier(c)
    c = Curves(waypoints)
    plot_bezier(c)

    # write_file(waypoints)


    # ## control point vector가 너무 작으면 scale up하여 곡률을 조정한다.
    # scale = 1.5
    # A = []; B = []
    # A.append(c.curves[0].A)
    # for i in range(c.num_curves -1):
    #     curve_i = c.curves[i]
    #     curve_i1 = c.curves[i+1]
    #     vec = np.linalg.norm(curve_i.B - curve_i.Pi_1)
    #     if (vec < 0.002):
    #         curve_i.B = (curve_i.B - curve_i.Pi_1) * scale + curve_i.Pi_1
    #         B.append(curve_i.B)
    #         curve_i1.A = (curve_i1.A - curve_i1.Pi) * scale + curve_i1.Pi
    #         A.append(curve_i1.A)
    #     else :
    #         A.append(curve_i1.A)
    #         B.append(curve_i.B)
    # B.append(c.curves[-1].B)

    # ## 전체 곡률을 cale up 한다
    # scale = 1.5
    # A = []; B = []
    # for i in range(c.num_curves -1):
    #     curve = c.curves[i]
    #     curve.A = (curve.A - curve.Pi) * scale + curve.Pi
    #     curve.B = (curve.B - curve.Pi_1) * scale + curve.Pi_1
    #     A.append(curve.A)
    #     B.append(curve.B)

    # c.A = np.array(A)
    # c.B = np.array(B)
    # c.reset_bezeir()
    #
    # plot_bezier(c)
