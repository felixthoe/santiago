import pinocchio as pin
import os
import numpy as np

# URDF path
cwd = os.getcwd()
urdf_path = os.path.join(cwd, 'gazebo_simulation', 'urdf', 'robot.urdf')
print(urdf_path)

# Build the model
model = pin.buildModelFromUrdf(urdf_path)
data = model.createData()

print(model)

# Set joint configurations
#q = pin.neutral(model)
q = np.array([1.5708, -0.3, 0.5708]) 
#v = np.zeros(model.nv)
v = np.array([0.5, -0.1, -0.2])
#a = np.zeros(model.nv)
a = np.array([0.1, -0.05, -0.15])

# Compute mass matrix
M = pin.crba(model, data, q)

# Compute gravity forces as a vector (G(q))
g = pin.computeGeneralizedGravity(model, data, q)

# Compute the Coriolis and centripetal coriforces matrix (C(q, v)) as a square matrix
C = pin.computeCoriolisMatrix(model, data, q, v)

# Print the matrices
print(f"Mass matrix M:\n{M}\n")
print(f"Gravity vector g(q) as a diagonal matrix:\n{g}\n")
print(f"Coriolis matrix C(q, v):\n{C}\n")

# Friction forces (example, zeros here)
f = np.zeros(model.nv)
print(f"Friction forces f:\n{f}\n")

# Generate LaTeX representation
def matrix_to_latex(matrix):
    rows = []
    for row in matrix:
        row_str = " & ".join([f"{elem:.2f}" for elem in row])  
        rows.append(f"{row_str}")
    return " \\\\ ".join(rows)

def vector_to_latex(vector):
    return " & ".join([f"{elem:.2f}" for elem in vector.flatten()])  

M_latex = matrix_to_latex(M)
g_latex = vector_to_latex(g)  
C_latex = matrix_to_latex(C) 
f_latex = vector_to_latex(f)  

latex_code = f"""
\\documentclass{{article}}
\\usepackage{{amsmath}}
\\begin{{document}}

The system dynamics equation is:

$$
M \\ddot{{q}} + C \\dot{{q}} + g + f = \\tau_{{\\text{{act}}}} + \\tau_h
$$

Where the matrices and vectors (for the neutral configuration) are populated as follows:

$$
M = \\left( \\begin{{array}}{{ccc}}
{M_latex}
\\end{{array}} \\right)
$$

$$
C = \\left( \\begin{{array}}{{ccc}}
{C_latex}
\\end{{array}} \\right)
$$

$$
g = \\left( \\begin{{array}}{{c}}
{g_latex}
\\end{{array}} \\right)
$$

$$
f = \\left( \\begin{{array}}{{c}}
{f_latex}
\\end{{array}} \\right)
$$

\\end{{document}}
"""

save_path = os.path.join(cwd, 'generated_files', 'dynamics_model.tex')
os.makedirs(os.path.dirname(save_path), exist_ok=True)
with open(save_path, 'w') as f:
    f.write(latex_code)