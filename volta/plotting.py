"""Matplotlib figures for tracking, CoM/ZMP, footsteps and GRF results."""
import os

import matplotlib.pyplot as plt
import matplotlib.transforms as transforms
import numpy as np
from matplotlib.patches import Rectangle


def _dims_by_time(data, n_samples):
    """Return data as (dims, N), using the time vector length to pick the axis."""
    data = np.atleast_2d(np.asarray(data))
    if data.shape[1] != n_samples and data.shape[0] == n_samples:
        data = data.T
    return data


def plot_comparison(
    time_log, 
    ref_data, 
    act_data, 
    dimension_labels=None, 
    title="State Comparison", 
    y_label="Position (m)",
    save_folder=None
):
    """
    Generates a comparison plot between Reference and Actual trajectories.
    
    Args:
        time_log (np.array): Time vector of shape (N,)
        ref_data (np.array): Reference data of shape (Dims, N) or (N, Dims)
        act_data (np.array): Actual/Measured data of shape (Dims, N) or (N, Dims)
        dimension_labels (list): List of labels for each subplot (e.g., ['X', 'Y', 'Z'])
        title (str): Overall title of the figure
        y_label (str): Label for the y-axis
        save_folder (str): If provided, saves the figure to this folder.
    """
    
    # 1. Standardization: Ensure data is (Dims, N)
    # Robotics logs are often (3, 1000). If (1000, 3), we transpose.
    ref_data = _dims_by_time(ref_data, len(time_log))
    act_data = _dims_by_time(act_data, len(time_log))
        
    num_dims = ref_data.shape[0]
    
    # Handle default labels if none provided
    if dimension_labels is None:
        dimension_labels = [f"Dim {i}" for i in range(num_dims)]

    # 2. Create Subplots (one for each dimension)
    fig, axes = plt.subplots(num_dims, 1, figsize=(8, 3 * num_dims), sharex=True)
    
    # Handle single dimension case (axes is not a list if num_dims=1)
    if num_dims == 1:
        axes = [axes]

    # 3. Plotting Loop
    for i, ax in enumerate(axes):
        # Plot Reference (Black Dashed)
        ax.plot(time_log, ref_data[i, :], 'k--', linewidth=2, label='Reference')
        
        # Plot Actual (Blue Solid)
        ax.plot(time_log, act_data[i, :], 'b-', linewidth=1.5, alpha=0.8, label='Actual')
        
        # Styling
        ax.set_ylabel(f"{dimension_labels[i]} {y_label}")
        ax.grid(True, linestyle=':', alpha=0.6)
        
        # Only add legend to the first subplot to avoid clutter
        if i == 0:
            ax.legend(loc='upper right')

    axes[-1].set_xlabel("Time (s)")
    fig.suptitle(title, fontsize=14)
    plt.tight_layout() # Adjusts layout to prevent overlap

    # 4. Save Logic
    if save_folder:
        os.makedirs(save_folder, exist_ok=True)
        # Create a safe filename from the title
        safe_name = title.replace(" ", "_").lower() + ".png"
        full_path = os.path.join(save_folder, safe_name)
        plt.savefig(full_path, dpi=300) # 300 DPI is standard for Thesis
        print(f"Plot saved to: {full_path}")
    
    plt.show()


def plot_interactive_comparison(
    time_log, 
    ref_data, 
    act_data,
    dimension_labels=None, 
    title="Trajectory Tracking Analysis", 
    y_label="Value",
    save_folder=None
):
    """
    Creates an interactive HTML plot to compare trajectories.
    Allows zooming, panning, and hovering over data points.
    """
    # plotly is optional: only this function needs it.
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots

    # 1. Standardization: Ensure data is (Dims, N)
    ref_data = _dims_by_time(ref_data, len(time_log))
    act_data = _dims_by_time(act_data, len(time_log))
        
    num_dims = ref_data.shape[0]
    
    if dimension_labels is None:
        dimension_labels = [f"Dim {i}" for i in range(num_dims)]

    # 2. Create Subplots (Vertical stack)
    fig = make_subplots(
        rows=num_dims, cols=1,
        shared_xaxes=True,
        vertical_spacing=0.1,
        subplot_titles=dimension_labels
    )

    # 3. Plotting Loop
    for i in range(num_dims):
        # Reference Line (Dashed Black)
        fig.add_trace(
            go.Scatter(
                x=time_log, y=ref_data[i, :],
                name=f"Ref {dimension_labels[i]}",
                line=dict(color='black', width=2, dash='dash'),
                legendgroup="ref",
                showlegend=(i==0) # Only show legend once
            ),
            row=i+1, col=1
        )

        # Actual Line (Solid Blue)
        fig.add_trace(
            go.Scatter(
                x=time_log, y=act_data[i, :],
                name=f"Act {dimension_labels[i]}",
                line=dict(color='blue', width=2),
                legendgroup="act",
                showlegend=(i==0)
            ),
            row=i+1, col=1
        )
        
        # Add Y-axis labels
        fig.update_yaxes(title_text=y_label, row=i+1, col=1)

    # 4. Layout Styling
    fig.update_layout(
        title=title,
        xaxis_title="Time (s)",
        height=300 * num_dims, # Adjust height based on number of plots
        template="plotly_white",
        hovermode="x unified" # Shows all values at a specific time when hovering
    )

    # 5. Show and Save
    if save_folder:
        os.makedirs(save_folder, exist_ok=True)
        filename = f"{title.replace(' ', '_').lower()}.html"
        full_path = os.path.join(save_folder, filename)
        # fig.write_html(full_path)
        # print(f"Interactive plot saved to: {full_path}")
        
    fig.show()

def PlotTracking(
    time_log, 
    ref_data, 
    act_data, 
    phase_codes, 
    dim_labels=['X', 'Y', 'Z'], 
    unit='m',
    title="Center of Mass Tracking",
    plot_interval=None,
    show_error=True,
    save_folder=None,
    labels = ["Actual", "Reference"],
    show_plot = True
):
    """
    Plots trajectory with optional error subplots and background phase shading.
    """
    # --- SAFETY FIXES ---
    phase_codes = np.array(phase_codes).flatten().astype(int)
    
    # 1. Standardization
    ref_data = _dims_by_time(ref_data, len(time_log))
    act_data = _dims_by_time(act_data, len(time_log))
    
    # 2. Data Slicing
    if plot_interval is not None:
        t_start, t_end = plot_interval
        idx_start = np.searchsorted(time_log, t_start)
        idx_end = np.searchsorted(time_log, t_end)
        
        time_log = time_log[idx_start:idx_end]
        ref_data = ref_data[:, idx_start:idx_end]
        act_data = act_data[:, idx_start:idx_end]
        phase_codes = phase_codes[idx_start:idx_end]
        
        if len(time_log) == 0:
            print(f"Warning: No data in interval {plot_interval}")
            return

    num_dims = len(dim_labels)
    
    # 3. Process Phases
    phase_segments = []
    # Map: 0=L Stance (Blue), 1=R Stance (Red), 2=Double (Gray)
    phase_map = {
        0: {'name': 'LS', 'color': '#d62728', 'alpha': 0.15},
        1: {'name': 'RS', 'color': '#1f77b4', 'alpha': 0.15},
        2: {'name': 'DS',   'color': '#7f7f7f', 'alpha': 0.25}
    }

    if len(phase_codes) > 0:
        current_code = phase_codes[0]
        start_idx = 0
        for i in range(1, len(phase_codes)):
            if phase_codes[i] != current_code:
                phase_segments.append({'start': time_log[start_idx], 'end': time_log[i], 'code': current_code})
                current_code = phase_codes[i]
                start_idx = i
        phase_segments.append({'start': time_log[start_idx], 'end': time_log[-1], 'code': current_code})

    # 4. Setup Figure based on show_error flag
    if show_error:
        # 2 plots per dimension (Trajectory + Error)
        nrows = num_dims * 2
        ratios = [3, 1] * num_dims
        figsize_h = 3.5 * num_dims
    else:
        # 1 plot per dimension (Trajectory only)
        nrows = num_dims
        ratios = [1] * num_dims
        figsize_h = 2.5 * num_dims

    fig, axes = plt.subplots(nrows, 1, figsize=(8, figsize_h), sharex=True,
                             gridspec_kw={'height_ratios': ratios})
    
    # Handle single dimension case where axes is not a list
    if nrows == 1: axes = [axes]

    for i in range(num_dims):
        # Determine which axes to use
        if show_error:
            ax_traj = axes[2*i]
            ax_err = axes[2*i+1]
        else:
            ax_traj = axes[i]
            ax_err = None

        # --- Plot Trajectory ---
        rmse = np.sqrt(np.mean((ref_data[i,:] - act_data[i,:])**2))
        
        ax_traj.plot(time_log, act_data[i, :], 'b-', linewidth=1.5, label=labels[0])
        ax_traj.plot(time_log, ref_data[i, :], 'k--', linewidth=1.5, label=labels[1])
        if unit:
            ax_traj.set_ylabel(f"${dim_labels[i]}$ [{unit}]")
        else:
            ax_traj.set_ylabel(f"${dim_labels[i]}$")
        ax_traj.grid(True, linestyle=':', alpha=0.6)

        # --- Plot Error (If enabled) ---
        if show_error and ax_err is not None:
            error = ref_data[i, :] - act_data[i, :]
            if unit=='m' or unit=='m/s':
                th = 0.5
                error[abs(error) > th] = 0.0
            ax_err.plot(time_log, error, 'r-', linewidth=1.0)
            ax_err.set_ylabel(f"Error [{unit}]")
            ax_err.grid(True, linestyle=':', alpha=0.6)
            
            max_err = np.max(np.abs(error)) * 1.1 if len(error) > 0 else 0.01
            if max_err > 1e-6: ax_err.set_ylim(-max_err, max_err)

        # --- Phase Shading ---
        for seg in phase_segments:
            p_info = phase_map.get(seg['code'], {'color': 'white', 'alpha': 0, 'name': ''})
            
            # Shade Trajectory
            ax_traj.axvspan(seg['start'], seg['end'], color=p_info['color'], alpha=p_info['alpha'], lw=0)
            
            # Shade Error (if it exists)
            if show_error and ax_err is not None:
                ax_err.axvspan(seg['start'], seg['end'], color=p_info['color'], alpha=p_info['alpha'], lw=0)
            
            # Label (Top plot only)
            if i == 0 and (seg['end'] - seg['start']) > 0.1:
                mid_t = (seg['start'] + seg['end']) / 2
                ax_traj.text(mid_t, ax_traj.get_ylim()[1], p_info['name'], 
                             ha='center', va='bottom', fontsize=8, color='black')

        if i == 0:
            ax_traj.legend(loc='upper right')

    axes[-1].set_xlabel("Time [s]")
    if plot_interval:
        if plot_interval[0] <= 0.02:
            title += f" (t= 0 s to {plot_interval[1]}s)"
        else:
            title += f" (t={plot_interval[0]}s to {plot_interval[1]}s)"
    # Increase 'y' slightly to push title up
    fig.suptitle(title, fontsize=16, y=1.02) 
    
    # Use tight_layout but leave space at top for title
    plt.tight_layout(rect=[0, 0, 1, 0.98])
    if save_folder:
        os.makedirs(save_folder, exist_ok=True)
        # Unique filename based on interval to avoid overwriting
        t_suffix = f"_t{int(plot_interval[0])}-{int(plot_interval[1])}" if plot_interval else ""
        filename = f"{title.replace(' ', '_').lower()}.pdf"
        full_path = os.path.join(save_folder, filename)
        plt.savefig(full_path, format='pdf', dpi=300, bbox_inches='tight')
        # print(f"Saved to: {full_path}")

    if show_plot:
        # print("Displaying plot...")
        plt.tight_layout()
        plt.show()


def PlotTrackingCompare(
    time_log, 
    ref_data, 
    act_data, # Can now be a single array OR a list of arrays: [data_A, data_B]
    phase_codes, 
    act_labels=None,  # Optional: ['Method A', 'Method B']
    act_colors=None,  # Optional: ['blue', 'green']
    dim_labels=['X', 'Y', 'Z'], 
    unit='m',
    title="Center of Mass Tracking Comparison",
    plot_interval=None,
    show_error=True,
    save_folder=None,
    show_plot = True,
    show_rms = True
):
    """
    Plots trajectory comparison for multiple 'Actual' datasets against one Reference.
    """
    # --- 1. PRE-PROCESS INPUTS (Handle Single vs. Multiple Data) ---
    # If act_data is a single array (not a list), wrap it in a list
    if not isinstance(act_data, list):
        act_data = [act_data]
    
    # Default Labels
    if act_labels is None:
        act_labels = [f"Actual {j+1}" for j in range(len(act_data))]
    
    # Default Colors (Matplotlib tableau colors)
    if act_colors is None:
        default_colors = ['#1f77b4', '#2ca02c', '#9467bd', '#ff7f0e'] # Blue, Green, Purple, Orange
        act_colors = default_colors[:len(act_data)]

    # Safety: Ensure Phase Codes are flat integers
    phase_codes = np.array(phase_codes).flatten().astype(int)
    
    # Standardization: Transpose if shape is (N, Dims) -> (Dims, N)
    ref_data = _dims_by_time(ref_data, len(time_log))
    for j in range(len(act_data)):
        act_data[j] = _dims_by_time(act_data[j], len(time_log))
    
    # --- 2. DATA SLICING ---
    if plot_interval is not None:
        t_start, t_end = plot_interval
        idx_start = np.searchsorted(time_log, t_start)
        idx_end = np.searchsorted(time_log, t_end)
        
        # Slice Time and Reference
        time_log = time_log[idx_start:idx_end]
        ref_data = ref_data[:, idx_start:idx_end]
        phase_codes = phase_codes[idx_start:idx_end]
        
        # Slice all Actual datasets
        for j in range(len(act_data)):
            act_data[j] = act_data[j][:, idx_start:idx_end]
        
        if len(time_log) == 0:
            print(f"Warning: No data in interval {plot_interval}")
            return

    num_dims = len(dim_labels)
    
    # --- 3. PROCESS PHASES ---
    phase_segments = []
    phase_map = {
        0: {'name': 'LS', 'color': '#d62728', 'alpha': 0.15},
        1: {'name': 'RS', 'color': '#1f77b4', 'alpha': 0.15},
        2: {'name': 'DS',   'color': '#7f7f7f', 'alpha': 0.25}
    }

    if len(phase_codes) > 0:
        current_code = phase_codes[0]
        start_idx = 0
        for i in range(1, len(phase_codes)):
            if phase_codes[i] != current_code:
                phase_segments.append({'start': time_log[start_idx], 'end': time_log[i], 'code': current_code})
                current_code = phase_codes[i]
                start_idx = i
        phase_segments.append({'start': time_log[start_idx], 'end': time_log[-1], 'code': current_code})

    # --- 4. SETUP FIGURE ---
    if show_error:
        nrows = num_dims * 2
        ratios = [3, 1] * num_dims
        figsize_h = 4.0 * num_dims # Slightly taller for clearer comparison
    else:
        nrows = num_dims
        ratios = [1] * num_dims
        figsize_h = 3.0 * num_dims

    fig, axes = plt.subplots(nrows, 1, figsize=(10, figsize_h), sharex=True,
                             gridspec_kw={'height_ratios': ratios})
    if nrows == 1: axes = [axes]

    for i in range(num_dims):
        if show_error:
            ax_traj = axes[2*i]
            ax_err = axes[2*i+1]
        else:
            ax_traj = axes[i]
            ax_err = None

        # --- PLOT TRAJECTORIES ---
       
        
        # 2. Plot All Actuals
        for j, data in enumerate(act_data):
            # Calculate RMSE for Legend
            rmse = np.sqrt(np.mean((ref_data[i,:] - data[i,:])**2))
            if show_rms:
                label_text = f"{act_labels[j]} (RMSE: {rmse:.4f})"
            else:
                label_text = f"{act_labels[j]}"
            ax_traj.plot(time_log, data[i, :], color=act_colors[j], linewidth=1.5, 
                         linestyle='-', alpha=0.9, label=label_text)
        # 1. Plot Reference (Black Dashed)
        ax_traj.plot(time_log, ref_data[i, :], 'k--', linewidth=1.5, label='Reference')
        ax_traj.set_ylabel(f"${dim_labels[i]}$ [{unit}]")
        ax_traj.grid(True, linestyle=':', alpha=0.6)

        # --- PLOT ERRORS ---
        if show_error and ax_err is not None:
            max_err_val = 0
            
            for j, data in enumerate(act_data):
                error = ref_data[i, :] - data[i, :]
                
                # Apply your threshold logic
                if unit=='m' or unit=='m/s':
                    th = 0.08
                    error[np.abs(error) > th] = 0.0
                
                # Track max error for Y-limit scaling
                current_max = np.max(np.abs(error))
                if current_max > max_err_val: max_err_val = current_max
                
                ax_err.plot(time_log, error, color=act_colors[j], linewidth=1.0, alpha=0.8)

            ax_err.set_ylabel(f"Error [{unit}]")
            ax_err.grid(True, linestyle=':', alpha=0.6)
            
            # Auto-scale Y-limits
            limit = max_err_val * 1.1 if max_err_val > 0 else 0.01
            if limit > 1e-6: ax_err.set_ylim(-limit, limit)

        # --- BACKGROUND SHADING ---
        for seg in phase_segments:
            p_info = phase_map.get(seg['code'], {'color': 'white', 'alpha': 0, 'name': ''})
            
            ax_traj.axvspan(seg['start'], seg['end'], color=p_info['color'], alpha=p_info['alpha'], lw=0)
            if show_error and ax_err is not None:
                ax_err.axvspan(seg['start'], seg['end'], color=p_info['color'], alpha=p_info['alpha'], lw=0)
            
            if i == 0 and (seg['end'] - seg['start']) > 0.05:
                mid_t = (seg['start'] + seg['end']) / 2
                ax_traj.text(mid_t, ax_traj.get_ylim()[1], p_info['name'], 
                             ha='center', va='bottom', fontsize=9, color='black')

        # Legend on top plot
        # if i == 0:
        ax_traj.legend(loc='upper left', framealpha=0.9)

    axes[-1].set_xlabel("Time [s]")
    if plot_interval:
        if plot_interval[0] <= 0.02:
            title += f" (t= 0 s to {plot_interval[1]}s)"
        else:
            title += f" (t={plot_interval[0]}s to {plot_interval[1]}s)"
    
    fig.suptitle(title, fontsize=16, y=1.02)
    plt.tight_layout(rect=[0, 0, 1, 0.98])
    
    if save_folder:
        os.makedirs(save_folder, exist_ok=True)
        filename = f"{title.split(' (')[0].replace(' ', '_').lower()}.pdf"
        full_path = os.path.join(save_folder, filename)
        plt.savefig(full_path, format='pdf', dpi=300, bbox_inches='tight')
        # print(f"Saved to: {full_path}")
    if show_plot:
        plt.show()

def PlotCoMZMP(
    time_log, 
    ZMP_data,
    CoM_data, 
    phase_codes, 
    dim_labels=['X', 'Y', 'Z'], 
    unit='m',
    title="Center of Mass Tracking",
    plot_interval=None,
    show_error=True,
    save_folder=None,
    labels = ["Actual", "Reference"],
    show_plot = True
):
    """
    Plots trajectory with optional error subplots and background phase shading.
    """
    # --- SAFETY FIXES ---
    phase_codes = np.array(phase_codes).flatten().astype(int)
    
    # 1. Standardization
    CoM_data = _dims_by_time(CoM_data, len(time_log))
    ZMP_data = _dims_by_time(ZMP_data, len(time_log))
    
    # 2. Data Slicing
    if plot_interval is not None:
        t_start, t_end = plot_interval
        idx_start = np.searchsorted(time_log, t_start)
        idx_end = np.searchsorted(time_log, t_end)
        
        time_log = time_log[idx_start:idx_end]
        CoM_data = CoM_data[:, idx_start:idx_end]
        ZMP_data = ZMP_data[:, idx_start:idx_end]
        phase_codes = phase_codes[idx_start:idx_end]
        
        if len(time_log) == 0:
            print(f"Warning: No data in interval {plot_interval}")
            return

    num_dims = len(dim_labels)
    
    # 3. Process Phases
    phase_segments = []
    # Map: 0=L Stance (Blue), 1=R Stance (Red), 2=Double (Gray)
    phase_map = {
        0: {'name': 'LS', 'color': '#d62728', 'alpha': 0.15},
        1: {'name': 'RS', 'color': '#1f77b4', 'alpha': 0.15},
        2: {'name': 'DS',   'color': '#7f7f7f', 'alpha': 0.25}
    }

    if len(phase_codes) > 0:
        current_code = phase_codes[0]
        start_idx = 0
        for i in range(1, len(phase_codes)):
            if phase_codes[i] != current_code:
                phase_segments.append({'start': time_log[start_idx], 'end': time_log[i], 'code': current_code})
                current_code = phase_codes[i]
                start_idx = i
        phase_segments.append({'start': time_log[start_idx], 'end': time_log[-1], 'code': current_code})

    # 4. Setup Figure based on show_error flag
    if show_error:
        # 2 plots per dimension (Trajectory + Error)
        nrows = num_dims * 2
        ratios = [3, 1] * num_dims
        figsize_h = 3.5 * num_dims
    else:
        # 1 plot per dimension (Trajectory only)
        nrows = num_dims
        ratios = [1] * num_dims
        figsize_h = 2.5 * num_dims

    fig, axes = plt.subplots(nrows, 1, figsize=(8, figsize_h), sharex=False,
                             gridspec_kw={'height_ratios': ratios})
    
    # Handle single dimension case where axes is not a list
    if nrows == 1: axes = [axes]

    for i in range(num_dims-1):
        # Determine which axes to use
        if show_error:
            ax_traj = axes[2*i]
            ax_err = axes[2*i+1]
        else:
            ax_traj = axes[i]
            ax_err = None

        # --- Plot Trajectory ---
        rmse = np.sqrt(np.mean((CoM_data[i,:] - ZMP_data[i,:])**2))
        
        ax_traj.plot(time_log, ZMP_data[i, :], label=f"{labels[0]}")
        ax_traj.plot(time_log, CoM_data[i, :], label=f"{labels[1]}")

        ax_traj.set_ylabel(f"${dim_labels[i]}$ [{unit}]")
        ax_traj.grid(True, linestyle=':', alpha=0.6)

        # --- Plot Error (If enabled) ---
        if show_error and ax_err is not None:
            error = CoM_data[i, :] - ZMP_data[i, :]
            if unit=='m' or unit=='m/s':
                th = 0.5
                error[abs(error) > th] = 0.0
            ax_err.plot(time_log, error, 'r-', linewidth=1.0)
            ax_err.set_ylabel(f"Error [{unit}]")
            ax_err.grid(True, linestyle=':', alpha=0.6)
            
            max_err = np.max(np.abs(error)) * 1.1 if len(error) > 0 else 0.01
            if max_err > 1e-6: ax_err.set_ylim(-max_err, max_err)

        # --- Phase Shading ---
        for seg in phase_segments:
            p_info = phase_map.get(seg['code'], {'color': 'white', 'alpha': 0, 'name': ''})
            
            # Shade Trajectory
            ax_traj.axvspan(seg['start'], seg['end'], color=p_info['color'], alpha=p_info['alpha'], lw=0)
            
            # Shade Error (if it exists)
            if show_error and ax_err is not None:
                ax_err.axvspan(seg['start'], seg['end'], color=p_info['color'], alpha=p_info['alpha'], lw=0)
            
            # Label (Top plot only)
            if i == 0 and (seg['end'] - seg['start']) > 0.1:
                mid_t = (seg['start'] + seg['end']) / 2
                ax_traj.text(mid_t, ax_traj.get_ylim()[1], p_info['name'], 
                             ha='center', va='bottom', fontsize=8, color='black')

        if i == 0:
            ax_traj.legend(loc='upper right')

    # axes[2].scatter(ZMP_data[0, :],ZMP_data[1, :],s=5, label=f'{labels[0]} xy ')
    # axes[2].scatter(CoM_data[0, :],CoM_data[1, :],s=5, label=f'{labels[1]} xy ')
    axes[2].plot(ZMP_data[0, :],ZMP_data[1, :], label=f'{labels[0]} xy ')
    axes[2].plot(CoM_data[0, :],CoM_data[1, :], label=f'{labels[1]} xy ')
    axes[-1].set_xlabel("x [m]")
    axes[-2].set_xlabel("Time [s]")
    axes[2].set_ylabel("y [m]")
    axes[2].grid(True, linestyle=':', alpha=0.6)
    axes[2].legend()
    if plot_interval:
        if plot_interval[0] <= 0.02:
            title += f" (t= 0 s to {plot_interval[1]}s)"
        else:
            title += f" (t={plot_interval[0]}s to {plot_interval[1]}s)"
    # Increase 'y' slightly to push title up
    fig.suptitle(title, fontsize=16, y=1.02) 
    
    # Use tight_layout but leave space at top for title
    plt.tight_layout(rect=[0, 0, 1, 0.98])
    if save_folder:
        os.makedirs(save_folder, exist_ok=True)
        # Unique filename based on interval to avoid overwriting
        t_suffix = f"_t{int(plot_interval[0])}-{int(plot_interval[1])}" if plot_interval else ""
        filename = f"{title.replace(' ', '_').lower()}.pdf"
        full_path = os.path.join(save_folder, filename)
        plt.savefig(full_path, format='pdf', dpi=300, bbox_inches='tight')
        # print(f"Saved to: {full_path}")

    if show_plot:
        plt.tight_layout()
        plt.show()

def PlotFootsteps( steps,
                   plot_1=None, 
                   plot_2=None, 
                   plot_3=None, 
                   plot_4=None,
                   title = "Footstep Plan Visualization",
                   initial_pose=[0.0,0.125,0.0], 
                   label_idx_offset=0,
                   save_folder=None,
                   show_plot=True,
                   support_size = [0.05, 0.05],
                   figsize=(16, 10),
                   dpi=600
                   ):
    """
    Visualizes the VOLTA plan with structured plot arguments. 
    Fixed facecolor/edgecolor conflict.
    """
    plt.rcParams['figure.dpi'] = 100 
    fig, ax_map = plt.subplots(figsize=figsize)

    ax_map.set_xlabel("X [m]", fontsize=12)
    ax_map.set_ylabel("Y [m]", fontsize=12)
    ax_map.axis('equal') 
    ax_map.grid(True, alpha=0.3)

    def DrawFoot(ax, x, y, theta, side, label_idx=None):
        width, height = support_size
        side_color = 'red' if side == 'lfoot' else 'blue'
        
        # FIX: Use facecolor instead of color to allow the black edgecolor to show
        rect = Rectangle((-width/2, -height/2), width, height, 
                            facecolor=side_color, 
                            alpha=0.4, 
                            linewidth=1.2, 
                            edgecolor='black')
        
        t = transforms.Affine2D().rotate(theta).translate(x, y) + ax.transData
        rect.set_transform(t)
        ax.add_patch(rect)
        
        ax.arrow(x, y, 0.15*np.cos(theta), 0.15*np.sin(theta), 
                    head_width=0.03, color='k', alpha=0.7)
        
        if label_idx is not None:
            ax.text(x, y, str(label_idx), color='white', 
                    ha='center', va='center', fontweight='bold', fontsize=10)

    # 1. Draw Step Plan
    for s in steps:
        DrawFoot(ax_map, s['pos'][0], s['pos'][1], s['ang'][2], s['foot_id'], 
                    label_idx=s['index']+label_idx_offset)

    # 2. Draw Connectivity Path
    xs = [initial_pose[0]] + [s['pos'][0] for s in steps]
    ys = [initial_pose[1]] + [s['pos'][1] for s in steps]
    ax_map.plot(xs, ys, 'k--', alpha=0.2, marker='o', markersize=4)

    # 3. Process Trajectories
    styles = [
        {'ls': '-',  'lw': 2.0, 'alpha': 0.8},
        {'ls': '-',  'lw': 2.0, 'alpha': 0.8},
        {'ls': '--', 'lw': 1.5, 'alpha': 0.7},
        {'ls': ':',  'lw': 1.5, 'alpha': 0.7},
    ]
    
    for i, p_data in enumerate([plot_1, plot_2, plot_3, plot_4]):
        if p_data is not None:
            x_vals, y_vals, label_str = p_data
            ax_map.plot(x_vals, y_vals, label=label_str, 
                        linestyle=styles[i]['ls'], 
                        linewidth=styles[i]['lw'], 
                        alpha=styles[i]['alpha'])

    ax_map.legend(loc='upper left', fontsize=10, framealpha=0.9)
    fig.suptitle(title, fontsize=20, y=1.02) 
    plt.tight_layout(rect=[0, 0, 1, 0.98])
    
    if save_folder:
        os.makedirs(save_folder, exist_ok=True)
        filename = f"{title.replace(' ', '_').lower()}.pdf"
        plt.savefig(os.path.join(save_folder, filename), format='pdf', dpi=dpi, bbox_inches='tight')

    if show_plot:
        plt.show()

# def PlotGRF(time_data, 
#              walking_phase=None,
#              plot_1=None, # Format: [data_array, "Label"]
#              plot_2=None, 
#              plot_3=None, 
#              plot_4=None,
#              plot_interval=[0, -1],
#              title="VOLTA: GROUND REACTION FORCES",
#              ylabel="Force [N]",
#              figsize=(16, 8),
#              dpi=600):
#     """
#     Plots GRF components with walking phase shading using structured [data, label] inputs.
#     """
#     # 1. Timing and Indexing
#     dt_inv = 100
#     start = int(plot_interval[0] * dt_inv)
#     stop = int(plot_interval[1] * dt_inv) if plot_interval[1] > 0 else len(time_data)
    
#     start, stop = max(0, start), min(len(time_data), stop)
#     t_slice = time_data[start:stop]

#     fig, ax = plt.subplots(figsize=figsize, dpi=dpi)

#     # 2. Walking Phase Shading
#     if walking_phase is not None:
#         phase_slice = walking_phase[start:stop]
#         change_indices = np.where(np.diff(phase_slice) != 0)[0]
#         boundaries = np.concatenate([[0], change_indices, [len(t_slice) - 1]])
        
#         for i in range(len(boundaries) - 1):
#             idx_s, idx_e = boundaries[i], boundaries[i+1]
#             p = phase_slice[idx_s + 1]
            
#             # Shading logic: 0=DS (Gray), 1=LSS (Red), 2=RSS (Blue)
#             color = 'gray' if p == 2 else ('red' if p == 0 else 'blue')
#             ax.axvspan(t_slice[idx_s], t_slice[idx_e], facecolor=color, alpha=0.1)

#     # 3. Process Plots [data, label]
#     # Styles cycle: Solid for primary, Dashed for reference/desired
#     styles = [
#         {'color': 'crimson',   'ls': '-',  'alpha': 0.9}, # Plot 1
#         {'color': 'royalblue', 'ls': '-',  'alpha': 0.9}, # Plot 2
#         {'color': 'darkred',   'ls': '--', 'alpha': 0.7}, # Plot 3
#         {'color': 'darkblue',  'ls': '--', 'alpha': 0.7}, # Plot 4
#     ]
    
#     plots = [plot_1, plot_2, plot_3, plot_4]
#     for i, p_info in enumerate(plots):
#         if p_info is not None:
#             data, label_str = p_info
#             # Slice the data to match time
#             ax.plot(t_slice, data[start:stop], 
#                     label=label_str, 
#                     color=styles[i]['color'], 
#                     linestyle=styles[i]['ls'], 
#                     alpha=styles[i]['alpha'],
#                     linewidth=2)

#     # 4. Styling
#     ax.set_title(title, fontsize=18, pad=20)
#     ax.set_xlabel('Time [s]', fontsize=12)
#     ax.set_ylabel(ylabel, fontsize=12)
#     ax.legend(loc='upper right', framealpha=0.9, ncol=2)
#     ax.grid(True, linestyle=':', alpha=0.5)
    
#     plt.tight_layout()
#     plt.show()

def PlotGRF(time_data, 
            walking_phase=None,
            plot_1=None, # Format: [data_array, "Label"]
            plot_2=None, 
            plot_3=None, 
            plot_4=None,
            plot_interval=[0, -1],
            title="VOLTA: GROUND REACTION FORCES",
            ylabel="Force [N]",
            figsize=(16, 8),
            dpi=600):
    """
    Plots GRF components with walking phase shading and top text labels (DS, LS, RS).
    """
    # 1. Timing and Indexing (Using your dt_inv = 100 downsample rate)
    dt_inv = 100
    start = int(plot_interval[0] * dt_inv)
    stop = int(plot_interval[1] * dt_inv) if plot_interval[1] > 0 else len(time_data)
    
    start, stop = max(0, start), min(len(time_data), stop)
    t_slice = time_data[start:stop]

    fig, ax = plt.subplots(figsize=figsize, dpi=dpi)

    # 2. Walking Phase Shading & Text Labels
    if walking_phase is not None:
        phase_slice = walking_phase[start:stop]
        change_indices = np.where(np.diff(phase_slice) != 0)[0]
        boundaries = np.concatenate([[0], change_indices, [len(t_slice) - 1]])
        
        for i in range(len(boundaries) - 1):
            idx_s, idx_e = boundaries[i], boundaries[i+1]
            p = phase_slice[idx_s + 1]
            
            # Map values to color and string labels based on your convention
            if p == 2:   # Double Support
                color, phase_str = 'gray', 'DS'
            elif p == 0: # Left Support
                color, phase_str = 'red', 'LS'
            else:        # Right Support
                color, phase_str = 'blue', 'RS'
                
            # Draw the background shading block
            t_start = t_slice[idx_s]
            t_end = t_slice[idx_e]
            ax.axvspan(t_start, t_end, facecolor=color, alpha=0.1)
            
            # --- ADD PHASE LABELS TO TOP ---
            # Compute center time of the current phase window
            t_mid = (t_start + t_end) / 2.0
            
            # transform=ax.get_xaxis_transform() places the Y coordinate relative to the axis boundaries.
            # Y=1.02 means 2% above the top axis frame line, keeping it perfectly aligned regardless of data spikes.
            ax.text(t_mid, 1.02, phase_str, 
                    transform=ax.get_xaxis_transform(),
                    ha='center', va='bottom', 
                    fontsize=11, fontweight='bold', color='#333333')

    # 3. Process Plots [data, label]
    styles = [
        {'color': 'crimson',   'ls': '-',  'alpha': 0.9}, # Plot 1
        {'color': 'royalblue', 'ls': '-',  'alpha': 0.9}, # Plot 2
        {'color': 'darkred',   'ls': '--', 'alpha': 0.7}, # Plot 3
        {'color': 'darkblue',  'ls': '--', 'alpha': 0.7}, # Plot 4
    ]
    
    plots = [plot_1, plot_2, plot_3, plot_4]
    for i, p_info in enumerate(plots):
        if p_info is not None:
            data, label_str = p_info
            ax.plot(t_slice, data[start:stop], 
                    label=label_str, 
                    color=styles[i]['color'], 
                    linestyle=styles[i]['ls'], 
                    alpha=styles[i]['alpha'],
                    linewidth=2)
# 4. Styling
    ax.set_title(title, fontsize=18, pad=30) # Increased padding to prevent overlap with the top text
    ax.set_xlabel('Time [s]', fontsize=12)
    ax.set_ylabel(ylabel, fontsize=12)
    ax.legend(loc='upper right', framealpha=0.9, ncol=2)
    ax.grid(True, linestyle=':', alpha=0.5)
    
    plt.tight_layout()
    plt.show()