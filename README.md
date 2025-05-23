

https://github.com/user-attachments/assets/79ef912e-795c-4a98-9af6-fd9f9d03a894


# Physics Based Particle Solver

## Prerequisites

- CMake (version 3.10 or higher)
- A C++ compiler (e.g., GCC, Clang, MSVC)

## Building the Project

1. Clone the repository:

    ```sh
    git clone https://github.com/heliumsneakers/Verlet-Physics-Solver.git
    cd Verlet-Physics-Solver
    ```

2. Initialize and update submodules:

    ```sh
    git submodule init
    git submodule update
    ```
    
NOTE: Must build raylib before building project: https://github.com/raysan5/raylib

3. Create a build directory and build the project:

    ```sh
    mkdir build
    cd build
    cmake ..
    make
    ```

4. Run the executable:

    ```sh
    ./Verlet
    ```

## License

This project is licensed under the MIT License.

