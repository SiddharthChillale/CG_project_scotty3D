# Scotty 3D 

This repo   conatins both MeshEdit tool as well as Pathtracer Tool

How to run :
1. Download the repo

2. Run the tool `Scotty3D.exe` that will be present under `RelWithDebInfo/`.

3. For mesh operations, import any triangular mesh model (e.g. `dof.dae`, `human.dae`, `teapot.dae`). Select an edge, face, vertex and perform the required operations.

4. For Path tracer, import `cbox_lambertian.dae` . GO to the Render tab. Start the render window and render. if you want to see rays, check the `Draw Rays` checkbox.

5. For better results, Replace the spheres inside the `cbox_lambertian.dae` with a `dodecahedron.dae` model (you might to scale the dodecahedron by 0.3). Run render.

7. Build according to build instructions given in https://cmu-graphics.github.io/Scotty3D/build/ for Linux, MacOS, and Windows OS.


