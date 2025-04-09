# Control-of-Descent-Vehicle

This project simulates the powered descent of a vehicle, likely on a planetary body like Mars, using a user-defined controller. The simulation environment is implemented in MATLAB, allowing for the testing and analysis of different control strategies.

## Overview

The `DesignProblem01.m` script is the main entry point for running the simulation. It takes the name of a MATLAB function defining the control logic as a primary argument. Additionally, it accepts several optional parameters to customize the simulation, such as logging data, saving movies or snapshots, and setting initial conditions and references.

The simulation models the vehicle's dynamics, including its position (x, z), velocity (xdot, zdot), orientation (theta), and angular velocity (thetadot). The controller function receives sensor data (current state) and reference values (desired state) and outputs thrust commands for the vehicle's left and right thrusters.

**Key Features:**

* **Modular Controller Design:** Users can implement their own control algorithms by creating a MATLAB function that adheres to a defined interface.
* **Parameter Customization:** The simulation can be tailored through various optional parameters, allowing for different scenarios and analysis.
* **Data Logging:** Simulation data can be saved to a `.mat` file for post-simulation analysis.
* **Visualization:** The simulation can display a graphical representation of the vehicle's trajectory and state over time.
* **Movie and Snapshot Saving:** The simulation can record a movie or save a snapshot of the final state for documentation or sharing.

## Getting Started

1.  **MATLAB Environment:** Ensure you have MATLAB installed.
2.  **Controller Function:** Create a MATLAB function file (e.g., `MyController.m`) that defines your control logic. This function must have `initControlSystem` and `runControlSystem` sub-functions with specific input and output arguments as described within `DesignProblem01.m`.
3.  **Run the Simulation:** Execute the `DesignProblem01.m` script from the MATLAB command window, providing the name of your controller function as the first argument.

    ```matlab
    DesignProblem01('MyController');
    ```

## Controller Interface

Your controller function must define the following sub-functions:

* **`initControlSystem(parameters, data)`:** This function is called once at the beginning of the simulation. It allows you to initialize controller parameters and internal data structures.
    * **Inputs:**
        * `parameters`: A struct containing physical constants and simulation settings (e.g., `tStep`, `g`, `m`, `J`, `maxthrust`).
        * `data`: A struct for storing controller-specific data.
    * **Output:**
        * `data`: The (potentially modified) data struct.

* **`runControlSystem(sensors, references, parameters, data)`:** This function is called at each time step of the simulation. It implements your control algorithm.
    * **Inputs:**
        * `sensors`: A struct containing the current state of the vehicle (e.g., `t`, `x`, `xdot`, `z`, `zdot`, `theta`, `thetadot`).
        * `references`: A struct containing the desired state values (e.g., `x`, `xdot`, `z`, `zdot`, `theta`, `thetadot`).
        * `parameters`: The same parameters struct passed to `initControlSystem`.
        * `data`: The data struct updated in `initControlSystem` and previous calls to `runControlSystem`.
    * **Outputs:**
        * `actuators`: A struct containing the thrust commands for the left (`lthrust`) and right (`rthrust`) thrusters.
        * `data`: The (potentially modified) data struct for use in the next time step.

## Optional Parameters

You can customize the simulation by providing parameter-value pairs to the `DesignProblem01` function:

* `'team'`: A string specifying a team name to display on the figure window.
* `'datafile'`: A string specifying the filename for saving simulation data (e.g., `'data.mat'`).
* `'moviefile'`: A string specifying the filename for saving a movie of the simulation (e.g., `'movie.mp4'`).
* `'snapshotfile'`: A string specifying the filename for saving a PDF snapshot of the final simulation frame (e.g., `'snap.pdf'`).
* `'controllerdatatolog'`: A cell array of strings specifying fields in `controller.data` to log in the data file (if `'datafile'` is defined).
* `'reference'`: A 6x1 numerical matrix `[x; xdot; z; zdot; theta; thetadot]` defining constant reference values for the states. Default is `[0; 0; 0; 0; 0; 0]`.
* `'tStop'`: A positive scalar number specifying the simulation stop time (default is `30`).
* `'initial'`: A 6x1 numerical matrix `[x; xdot; z; zdot; theta; thetadot]` specifying the initial state values (default is `[0; 10; 4000; -50; 0.1; 0.01]`).
* `'display'`: A logical flag (`true` or `false`) to enable or disable the live simulation display (default is `true`).

**Example with optional parameters:**

```matlab
DesignProblem01('MyController', 'team', 'Awesome Team', 'datafile', 'my_run_data.mat', 'tStop', 60, 'reference', [10; 0; 100; 0; 0; 0], 'display', true);
```

## Understanding the Code

The `DesignProblem01.m` script orchestrates the simulation setup, execution loop, data management, and visualization. Here's a breakdown of its core functions:

* **`SetupSimulation(process)`:**
    * Initializes crucial simulation parameters.
    * Defines physical constants relevant to the simulated system.
    * Sets up the user-provided controller function.

* **`RunSimulation(process, controller)`:**
    * Manages the primary simulation loop.
    * At each time step, it calls the user-defined controller to obtain actuator commands.
    * Updates the state of the simulated vehicle based on these commands.

* **`UpdateProcess(process, controller)`:**
    * Integrates the vehicle's equations of motion.
    * Calculates the new state of the vehicle based on the applied thrust from the controller.

* **`GetSensors(process)`:**
    * Extracts the current state variables of the vehicle.
    * Prepares this data to be sent as "sensor" information to the controller.

* **`GetReferences(process)`:**
    * Retrieves the desired state values (reference signals) for the controller to track.

* **`GetInput(process, actuators)`:**
    * Takes the desired thrust commands from the controller.
    * Applies any constraints or limits (e.g., maximum thrust) to these commands before they are used in the simulation.

* **`UpdateFigure(process, controller, fig)`:**
    * Handles the real-time visualization of the simulation.
    * Updates the plot to show the vehicle's trajectory and state variables.

* **`UpdateDatalog(process, controller)`:**
    * Records simulation data at each time step.
    * Stores this data in memory for potential saving to a file.

## Included Test Code (`Test code:` section)

The script includes a section labeled "Test code:" that provides examples of how to use the simulation and explores the behavior of linear models:

* **Zero Feedback Linear Model:**
    * Simulates the open-loop response of a linearized version of the system.
    * Demonstrates the uncontrolled dynamics without any feedback.

* **State Feedback Linear Model:**
    * Simulates a closed-loop system where a linear state feedback controller (designed using MATLAB's `place` function) is used to control the linearized system.
    * Illustrates how feedback can be used to stabilize and control the system in a linear approximation.

* **Running Controllers through `DesignProblem01`:**
    * Shows how user-defined controller functions can be integrated into the main non-linear simulation environment provided by `DesignProblem01.m`.

* **Comparison of Initial Conditions:**
    * Examines how different starting states affect the system's response when controlled by the designed linear controller within the linear model.

**Important Note:** As highlighted in the comments of the test code, attempting to directly apply a linear controller (like the one designed in the test code) to the full non-linear simulation environment of `DesignProblem01` (using the generic `Controller` function interface) may not yield satisfactory results. This is because linear controllers are designed based on a simplified, locally accurate representation of the system's behavior and may not be effective when the system operates far from its linearization point or exhibits strong non-linear characteristics.
