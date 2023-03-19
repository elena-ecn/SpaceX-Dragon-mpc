# SpaceX-Dragon-mpc
MPC control for the SpaceX Dragon spacecraft as it rendezvous with the ISS.


<p align="center" width="100%">
    <img src="images/ISS.png" width="400">
    <br>The ISS and the SpaceX Dragon spacecraft 
</p>

<br>
The spacecraft is in circular orbit and its objective is to dock to the ISS. The simulation is implemented with a "sim to real gap" to account for unmodeled dynamics.

<br>

The MPC controller solves the following convex optimization problem :

$$ \begin{align*} 
\min_{x_{1:N},u_{1:N-1}} \quad & \sum_{i=1}^{N-1} \bigg[ \frac{1}{2} (x_i - \tilde{x}_{ref, i})^TQ({x}_i - \tilde{x}_{ref, i}) + \frac{1}{2} u_i^TRu_i \bigg] + \frac{1}{2}(x_N- \tilde{x}_{ref, N})^TQ_f
({x}_N- \tilde{x}_{ref, N})\\
 \text{st} \quad & x_1 = x_{\text{IC}} \\
 & x_{i+1} = A x_i + Bu_i \quad \text{for } i = 1,2,\ldots,N-1  \\
 & u_{min} \leq u_i \leq u_{max} \quad \text{for } i = 1,2,\ldots,N-1 \\
 & x_i[2] \leq x_{goal} [2]\quad \text{for } i = 1,2,\ldots,N
 \end{align*}$$


<p align="center" width="100%">
    <img src="images/spaceX.gif" width="400">
    <br>Docking with the MPC controller
</p>


<br>

State and control trajectories:

<br>

<p align="center" width="100%">
    <img src="images/positions.png" width="400">  
</p>

<p align="center" width="100%">
    <img src="images/velocities.png" width="400">  
</p>

<p align="center" width="100%">
    <img src="images/controls.png" width="400">  
</p>


<br>

Technologies
------------
The project was created with:
* Python 3.9.0
* numpy 1.24.2
* cvxpy 1.3.1
* matplotlib 3.6.2
* seaborn 0.12.2
* scipy 1.8.1

Installation
------------

To use this project, install it locally via:
```
git clone https://github.com/elena-ecn/SpaceX-Dragon-mpc.git
```

The dependencies can be installed by running:
```
pip install -r requirements.txt
```

To execute the code, run:
```
python3 main.py
```