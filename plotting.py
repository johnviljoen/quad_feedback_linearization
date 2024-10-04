import matplotlib.pyplot as plt
from matplotlib import animation
import numpy as np

import geometry

class Animator:
    def __init__(
            self,
            p,
            x,
            t,
            r,
            max_frames = 500, # 500 works for my 16GB RAM machine
            dt = 0.1, # I think timestep of the frames of the gif
            save_path='data/gifs/test.gif',
            title='test'
        ):

        num_steps = len(t)
        max_frames = max_frames
        def compute_render_interval(num_steps, max_frames):
            render_interval = 1  # Start with rendering every frame.
            # While the number of frames using the current render interval exceeds max_frames, double the render interval.
            while num_steps / render_interval > max_frames:
                render_interval *= 2
            return render_interval
        render_interval = compute_render_interval(num_steps, max_frames)

        self.save_path = save_path
        self.dt = dt
        self.rp = None
        self.p = p
        self.x = x[::render_interval,:]
        self.t = t[::render_interval]
        self.r = r[::render_interval,:]

        # Instantiate the figure
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(projection='3d')

        # Draw the reference x y z (doesnt matter if static or dynamic)
        self.ax.plot(self.r[:,0], self.r[:,1], self.r[:,2], ':', lw=1.3, color='green')
        # these are the lines that draw the quadcopter
        self.line1, = self.ax.plot([], [], [], lw=2, color='red')
        self.line2, = self.ax.plot([], [], [], lw=2, color='blue')
        self.line3, = self.ax.plot([], [], [], '--', lw=1, color='blue')

        # Setting the limits correctly
        extra_each_side = 0.5

        x_min = min(np.min(self.x[:,0]), np.min(self.r[:,0]))
        y_min = min(np.min(self.x[:,1]), np.min(self.r[:,1]))
        z_min = min(np.min(self.x[:,2]), np.min(self.r[:,2]))
        x_max = max(np.max(self.x[:,0]), np.max(self.r[:,0]))
        y_max = max(np.max(self.x[:,1]), np.max(self.r[:,1]))
        z_max = max(np.max(self.x[:,2]), np.max(self.r[:,2]))

        max_range = 0.5*np.array([x_max-x_min, y_max-y_min, z_max-z_min]).max() + extra_each_side
        mid_x = 0.5*(x_max+x_min)
        mid_y = 0.5*(y_max+y_min)
        mid_z = 0.5*(z_max+z_min)
        
        self.ax.set_xlim3d([mid_x-max_range, mid_x+max_range])
        self.ax.set_xlabel('X')
        self.ax.set_ylim3d([mid_y-max_range, mid_y+max_range]) # NEU?

        self.ax.set_ylabel('Y')
        self.ax.set_zlim3d([mid_z-max_range, mid_z+max_range])
        self.ax.set_zlabel('Altitude')

        # add a dynamic time to the plot
        self.title_time = self.ax.text2D(0.05, 0.95, "", transform=self.ax.transAxes)

        # add the title itself
        title = self.ax.text2D(0.95, 0.95, title, transform=self.ax.transAxes, horizontalalignment='right')

    def update_lines(self, k):
        
        # current time
        tk = self.t[k]

        # history of x from 0 to current timestep k
        x_0k = self.x[0:k+1]
        xk = self.x[k]

        q_0k = np.array([
             self.x[0:k+1,3],
            -self.x[0:k+1,4],
            -self.x[0:k+1,5],
             self.x[0:k+1,6]
        ])
        qk = q_0k[:,-1]

        R = geometry.quaternion_to_dcm(qk)

        motor_points = R @ np.array([
            [self.p["dxm"], -self.p["dym"], self.p["dzm"]], 
            [0, 0, 0], 
            [self.p["dxm"], self.p["dym"], self.p["dzm"]], 
            [-self.p["dxm"], self.p["dym"], self.p["dzm"]], 
            [0, 0, 0], 
            [-self.p["dxm"], -self.p["dym"], self.p["dzm"]]
        ]).T
        motor_points[0:3,:] += xk[0:3][:,None]

        # plot the current point of the reference along the reference
        if self.rp is not None:
            self.rp.remove()
        self.rp = self.ax.scatter(self.r[k,0], self.r[k,1], self.r[k,2], color='magenta', alpha=1, marker = 'o', s = 25)

        self.line1.set_data(motor_points[0,0:3], motor_points[1,0:3])
        self.line1.set_3d_properties(motor_points[2,0:3])
        self.line2.set_data(motor_points[0,3:6], motor_points[1,3:6])
        self.line2.set_3d_properties(motor_points[2,3:6])
        self.line3.set_data(x_0k[:,0], x_0k[:,1])
        self.line3.set_3d_properties(x_0k[:,2])
        self.title_time.set_text(u"Time = {:.2f} s ".format(tk))

    def ini_plot(self):

        self.line1.set_data(np.empty([1]), np.empty([1]))
        self.line1.set_3d_properties(np.empty([1]))
        self.line2.set_data(np.empty([1]), np.empty([1]))
        self.line2.set_3d_properties(np.empty([1]))
        self.line3.set_data(np.empty([1]), np.empty([1]))
        self.line3.set_3d_properties(np.empty([1]))

        return self.line1, self.line2, self.line3   

    def animate(self):
        line_ani = animation.FuncAnimation(
            self.fig, 
            self.update_lines, 
            init_func=self.ini_plot, 
            frames=len(self.t)-1, 
            interval=(self.dt*10), 
            blit=False)

        line_ani.save(self.save_path, dpi=120, fps=25)

        return line_ani
    

if __name__ == "__main__":


    from params import quad_params as p
    from dynamics import f

    xk = np.array([0,0,0,1,0,0,0,0,0,0,0,0,0,*[522.9847140714692]*4])
    rk = np.copy(xk)

    Ti, Tf, Ts = 0.0, 1.0, 0.001
    t = np.arange(Ti, Tf, Ts)
    x = [np.copy(xk)]
    r = [np.copy(rk)]

    for tk in t:

        # find circular reference
        rk[0:2] = np.array([np.sin(tk), np.cos(tk)])
        
        # "calculate" uk from xk, rk
        uk = np.array([0.01, 0.01, 0.01, 0.00])

        # step system
        xk += f(p, xk, uk) * Ts

        # clip the rotor speeds within limits
        xk[13:17] = np.clip(xk[13:17], p["x_lb"][13:17], p["x_ub"][13:17])

        x.append(np.copy(xk))
        r.append(np.copy(rk))

    x = np.stack(x)
    r = np.stack(r)

    animator = Animator(p, x, t, r, save_path='data/gifs/test.gif')
    animator.animate()

    plt.close()
    plt.plot(x[:,0], x[:,1], label="xy")
    plt.plot(x[:,0], x[:,2], label="xz")
    plt.legend()
    plt.savefig('test.png', dpi=500)
