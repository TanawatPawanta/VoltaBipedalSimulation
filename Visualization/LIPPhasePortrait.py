import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, TextBox

def interactive_lip_stability():
    # --- 1. System Parameters ---
    h_c = 0.35  # CoM height [m]
    g = 9.81   # Gravity [m/s^2]
    eta = np.sqrt(g / h_c) # Natural frequency
    
    # Time vector for simulation
    t = np.linspace(0, 1.5, 100)

    # --- 2. Setup Figure and Axes ---
    fig, (ax_time, ax_phase) = plt.subplots(1, 2, figsize=(14, 6))
    plt.subplots_adjust(bottom=0.35, top=0.85, wspace=0.3)
    
    fig.suptitle(f'LIP Stability (eta={eta:.2f})', fontsize=14)

    # --- 3. Initial Plot Elements ---
    line_xc, = ax_time.plot(t, np.zeros_like(t), 'b-', lw=2, label=r'CoM Position ($x_c$)')
    line_xu, = ax_time.plot(t, np.zeros_like(t), 'r--', lw=2, label=r'Unstable Comp ($x_u$ / DCM)')
    line_xs, = ax_time.plot(t, np.zeros_like(t), 'g-.', lw=2, label=r'Stable Comp ($x_s$)')
    line_zmp, = ax_time.plot(t, np.zeros_like(t), 'k:', lw=2, label=r'Target ZMP ($x_z$)')
    
    ax_time.set_title('Time Evolution')
    ax_time.set_xlabel('Time [s]')
    ax_time.set_ylabel('Position [m]')
    ax_time.set_ylim(-2.0, 2.0)
    ax_time.grid(True, linestyle=':', alpha=0.6)
    ax_time.legend(loc='upper right', fontsize=9)

    line_phase, = ax_phase.plot([], [], 'b-', lw=2, label='Trajectory')
    line_stable_man, = ax_phase.plot([], [], 'g--', lw=2.5, label=r'Stable Manifold')
    line_unstable_man, = ax_phase.plot([], [], 'r:', lw=2.5, label=r'Unstable Manifold')
    point_start, = ax_phase.plot([], [], 'mo', markersize=8, label='Initial State')
    point_zmp, = ax_phase.plot([], [], 'ko', markersize=10, label='Target ZMP')

    ax_phase.set_title('Phase Portrait')
    ax_phase.set_xlabel(r'CoM Position $x_c$ [m]')
    ax_phase.set_ylabel(r'CoM Velocity $\dot{x}_c$ [m/s]') 
    ax_phase.set_xlim(-1.5, 1.5)
    ax_phase.set_ylim(-5.0, 5.0)
    ax_phase.grid(True, linestyle=':', alpha=0.6)
    ax_phase.legend(loc='upper left', fontsize=9)
    
    status_text = fig.text(0.5, 0.92, '', ha='center', va='top', fontsize=14, fontweight='bold')

    # --- 4. Define Sliders & TextBoxes ---
    axcolor = 'lightgoldenrodyellow'
    
    ax_pos = plt.axes([0.15, 0.20, 0.55, 0.03], facecolor=axcolor)
    ax_pos_text = plt.axes([0.72, 0.20, 0.08, 0.03])
    
    ax_vel = plt.axes([0.15, 0.15, 0.55, 0.03], facecolor=axcolor)
    ax_vel_text = plt.axes([0.72, 0.15, 0.08, 0.03])
    
    ax_zmp = plt.axes([0.15, 0.10, 0.55, 0.03], facecolor=axcolor)
    ax_zmp_text = plt.axes([0.72, 0.10, 0.08, 0.03])

    # FIXED: Removed valfmt='' and manually hid the text objects
    slider_pos = Slider(ax_pos, r'Initial Pos ($x_{c0}$)', -1.0, 1.0, valinit=0.0)
    slider_pos.valtext.set_visible(False)
    text_pos = TextBox(ax_pos_text, '', initial='0.00')

    slider_vel = Slider(ax_vel, r'Initial Vel ($v_{c0}$)', -4.0, 4.0, valinit=0.5)
    slider_vel.valtext.set_visible(False)
    text_vel = TextBox(ax_vel_text, '', initial='0.50')

    slider_zmp = Slider(ax_zmp, r'Target ZMP ($x_z$)', -1.0, 1.0, valinit=0.2)
    slider_zmp.valtext.set_visible(False)
    text_zmp = TextBox(ax_zmp_text, '', initial='0.20')

    # --- 5. Update Function ---
    is_updating = [False]

    def update(val=None):
        if is_updating[0]: return
        is_updating[0] = True

        x_c0 = slider_pos.val
        v_c0 = slider_vel.val
        x_z = slider_zmp.val

        text_pos.set_val(f"{x_c0:.2f}")
        text_vel.set_val(f"{v_c0:.2f}")
        text_zmp.set_val(f"{x_z:.2f}")

        x_u0 = x_c0 + v_c0 / eta
        x_s0 = x_c0 - v_c0 / eta

        error = x_u0 - x_z

        x_u_t = error * np.exp(eta * t) + x_z
        x_s_t = (x_s0 - x_z) * np.exp(-eta * t) + x_z
        
        x_c_t = (x_u_t + x_s_t) / 2
        v_c_t = eta * (x_u_t - x_s_t) / 2

        line_xc.set_ydata(x_c_t)
        line_xu.set_ydata(x_u_t)
        line_xs.set_ydata(x_s_t)
        line_zmp.set_ydata(np.full_like(t, x_z)) 

        line_phase.set_data(x_c_t, v_c_t)
        point_start.set_data([x_c0], [v_c0])
        point_zmp.set_data([x_z], [0])
        
        xr = np.linspace(-1.5, 1.5, 10)
        line_stable_man.set_data(xr, -eta * (xr - x_z))
        line_unstable_man.set_data(xr, eta * (xr - x_z))

        if abs(error) < 0.01:
            status_text.set_text(r'STABLE: Constraint Satisfied ($x_u = x_z$)')
            status_text.set_color('green')
        else:
            status_text.set_text(f'UNSTABLE: Diverging (Error = {error:.2f})')
            status_text.set_color('red')

        fig.canvas.draw_idle()
        is_updating[0] = False

    # --- 6. Handlers for Text Input ---
    def submit_pos(text):
        try: slider_pos.set_val(float(text))
        except ValueError: pass 

    def submit_vel(text):
        try: slider_vel.set_val(float(text))
        except ValueError: pass

    def submit_zmp(text):
        try: slider_zmp.set_val(float(text))
        except ValueError: pass

    text_pos.on_submit(submit_pos)
    text_vel.on_submit(submit_vel)
    text_zmp.on_submit(submit_zmp)

    slider_pos.on_changed(update)
    slider_vel.on_changed(update)
    slider_zmp.on_changed(update)

    # --- 7. Button: Satisfy Constraint ---
    def satisfy_constraint(event):
        x_c0 = slider_pos.val
        x_z = slider_zmp.val
        required_v = eta * (x_z - x_c0)
        slider_vel.set_val(required_v)

    ax_button = plt.axes([0.4, 0.025, 0.3, 0.05])
    btn_satisfy = Button(ax_button, 'Force Stability Constraint')
    btn_satisfy.on_clicked(satisfy_constraint)

    update(0)
    plt.show()

if __name__ == '__main__':
    interactive_lip_stability()