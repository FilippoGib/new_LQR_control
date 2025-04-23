import scipy.interpolate
import scipy.integrate
import pandas as pd
import numpy as np
import math

df = pd.read_csv('varano_x_y_r_v.csv')
tck, u = scipy.interpolate.splprep(df[['x', 'y']].values.T, per=1, s=0.0)

def dlength(s, tck):
  # The derivative of the length is just the 2-norm of the derivative
  dx, dy = scipy.interpolate.splev(s, tck, 1)
  return np.sqrt(np.square(dx) + np.square(dy))

def curvature(dxs, ddxs):
  return (dxs[0,:] * ddxs[1,:] - dxs[1,:] * ddxs[0,:]) / (dxs[0,:]**2 + dxs[1,:]**2) ** 1.5

RESOLUTION = 0.01

len_meters = scipy.integrate.quad(dlength, 0, 1, args=(tck,))[0]
N = math.ceil(len_meters / RESOLUTION)

spline_ss = np.linspace(0, 1, N)
xs = np.vstack(scipy.interpolate.splev(spline_ss, tck, 0))
print(xs[:,0])
dxs = np.vstack(scipy.interpolate.splev(spline_ss, tck, 1))
ddxs = np.vstack(scipy.interpolate.splev(spline_ss, tck, 2))
ks = curvature(dxs, ddxs)
ss = spline_ss * len_meters
ts = dxs / np.linalg.norm(dxs, axis=0)

phis = np.arctan2(ts[1,:], ts[0,:])

pd.DataFrame({
  "x": xs[0,:],
  "y": xs[1,:],
  "r": 1/ks,
  "phi": phis,
  "vx": 0.0,
  "s": spline_ss
}).to_csv('track_s.csv', index=False)

import matplotlib.pyplot as plt
fig = plt.figure()
ax_xy, ax_k = fig.subplots(2,1)
ax_xy.scatter(xs[0,:], xs[1,:], s=0.01)
ax_xy.plot(df['x'], df['y'], color='orange')


ax_k.plot(ss, phis)
ax_k.plot(ss, ks)


plt.show()
