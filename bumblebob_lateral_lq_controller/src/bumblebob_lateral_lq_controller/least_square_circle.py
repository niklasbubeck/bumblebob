
import numpy as np
from scipy import optimize


"""
@package LeastSquareCircle
Calculates the best fitting circle for the given points,
such that the distance of each point is minimalized.

"""


class LeastSquareCircle:
    def __init__(self):
        """
        The LeastSquareCircle Constructor
        @param self: The object pointer
        """
        self.xc = 0  # X-coordinate of circle center
        self.yc = 0  # Y-coordinate of circle center
        self.r = 0   # Radius of the circle
        self.xx = np.array([])  # Data points
        self.yy = np.array([])  # Data points

    def calc_distance(self, xc, yc):
        """!
        calculate the distance of each 2D points from the center (xc, yc)
        @param self: The object pointer

        @type xc: Float
        @param xc: The center coordinate x

        @type yc: Float
        @param yc: The center coordinate y

        @rtype: Float
        @return: The distance between two points
        """
        return np.sqrt((self.xx - xc)**2 + (self.yy - yc)**2)

    def f_least_square(self, c):
        """!
        calculate the algebraic distance between the data points and the
        mean circle centered at c=(xc, yc)

        @param self: The object pointer

        @type c: Tuple
        @param c: The center point

        @rtype: Float
        @return: The squared distance
        """
        ri = self.calc_distance(*c)
        return ri - ri.mean()

    def df_least_square(self, c):
        """!
        Jacobian of f_2b
        The axis corresponding to derivatives must be coherent with
        the col_deriv option of leastsq

        @param self: The object pointer

        @type c: Tuple
        @param c: The center point

        @rtype: np.ndarray
        @return: The jacobian matrix
        """
        xc, yc = c
        df_dc = np.empty((len(c), len(self.xx)))
        ri = self.calc_distance(xc, yc)
        df_dc[0] = (xc - self.xx) / ri                   # dR/dxc
        df_dc[1] = (yc - self.yy) / ri                   # dR/dyc
        df_dc = df_dc - df_dc.mean(axis=1)[:, np.newaxis]
        return df_dc

    def fit(self, xx, yy):
        """!
        Calculates the least square function and returns the curvature
        and the centerpoint
        @param self: The object pointer

        @type xx: np.ndarray
        @param xx: The x values

        @type yy: np.ndarray
        @param yy: The y values

        @rtype: Float, Float, Float
        @return: Curvature, Centerpoint of the circles in x and y
        """
        self.xx = xx
        self.yy = yy
        center_estimate = np.r_[np.mean(xx), np.mean(yy)]
        center = optimize.leastsq(
            self.f_least_square, center_estimate, Dfun=self.df_least_square, col_deriv=True)[0]

        self.xc, self.yc = center
        ri = self.calc_distance(*center)
        self.r = ri.mean()

        return 1 / self.r, self.xc, self.yc  # Return the curvature + the center


x = [64, 68.7, 70.6, 72.1, 71.7, 70.5, 70]
y = [28, 28.2, 27.5, 24.5, 20, 14, 11]
lsc = LeastSquareCircle()
curvature, center_x, center_y = lsc.fit(x, y)
print(curvature)
