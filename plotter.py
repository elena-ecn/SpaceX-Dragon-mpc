import matplotlib.pyplot as plt
import seaborn as sns


def plot_trajectories(x_history, u_history):
    """Plots state & control trajectories.

    Inputs:
      - x_history(np.ndarray): The state trajectories [6xN]
      - u_history(np.ndarray): The control trajectories [3xN]
    """

    # Plot state trajectories
    sns.set_theme()
    plt.figure()
    plt.plot(x_history[0, :], label="px")
    plt.plot(x_history[1, :], label="py")
    plt.plot(x_history[2, :], label="pz")
    plt.xlabel("N")
    plt.legend()
    plt.title("State trajectories - Positions")
    plt.savefig('images/positions.png')
    plt.show()

    plt.figure()
    plt.plot(x_history[3, :], label="vx")
    plt.plot(x_history[4, :], label="vy")
    plt.plot(x_history[5, :], label="vz")
    plt.xlabel("N")
    plt.legend()
    plt.title("State trajectories - Velocities")
    plt.savefig('images/velocities.png')
    plt.show()

    # Plot control trajectories
    plt.figure()
    plt.plot(u_history[0, :], label="ux")
    plt.plot(u_history[1, :], label="uy")
    plt.plot(u_history[2, :], label="uz")
    plt.xlabel("N")
    plt.legend()
    plt.title("Control trajectories")
    plt.savefig('images/controls.png')
    plt.show()
