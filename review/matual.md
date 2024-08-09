# matual
Review of `ACADOToolkitUser's Manual`


## 3. Optimal Control Problem (OCP)
ACADO solves OCP iteratively.

### 3.1. Rocket example
Default setting
* multiple-shooting discretization
* sequential quadratic programming (SQP) method
* Runge-Kutta integrator

### 3.2. Initialization
Set initial state. 3 ways.
1. Built-in auto-initialization (default)
    * ACADO uses constraints to initialize.
        * 2 bounds: To mean of constraints
        * 1 bound : To the bound
        * No constraint: To 0
2. From text file
    * ...
3. Use ACADO data structure
    * Needed class : `VariablesGrid`
    * Same with using text file, but hard-coding

### 3.3. Algorithmic options
How to overwrite the default settings
1. Tutorial code
    ```cpp
    algorithm.set(<Option Name>, <Option Value>)
    ```
    * Exception : the number of intervels
    * which are specified in the constructor of OCP
    * following the definition of the time interval.
2. Common Algorithmic options
    * ...

### 3.4. Storing the results
How to obtain and store the numerical results of optimization algorithm
1. In a text file
    * Easiest way
    * Similar to initializing via text file
2. In form of ACADO data structure
    * In form of `VariablesGrid`
    * Obtained results can be used directly in C++
3. By the logging functionality of ACADO
    * Set `LogRecord`.
    * Specify information to be logged.
    * After running, it can be used directly in C++.

### 3.5. DIfferential algebraic systems (DAE for eq.)
The model equation contains not only differential, but also algebraic states.
1. Mathematical formulation
2. Tutorial code for Semi-implicit DAEs

### 3.6 Discrete-time system
How to setup OCPs for discrete time systems.
1. Mathematical formulation
2. Implementation in ACADO syntax
    ```cpp
    f << next(s) == s + h*v
    ```

## 4. Multi-Objective Optimization (MOOCP)

### 4.1