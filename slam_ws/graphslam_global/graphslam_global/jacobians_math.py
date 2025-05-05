import sympy as sp

# Define symbols
x, y, v, psi = sp.symbols('x y v psi')              # state variables
a, phi = sp.symbols('a phi')                        # control inputs
lf, lr = sp.symbols('l_f l_r', positive=True)       # vehicle parameters

# Define beta symbolically (independent of state variables)
beta = sp.Function('beta')(phi)

# Define state and control vectors
state = sp.Matrix([x, y, v, psi])
control = sp.Matrix([a, phi])

# Define dynamics f(x, u)
f = sp.Matrix([
    v * sp.cos(psi + beta),
    v * sp.sin(psi + beta),
    a,
    (v / lr) * sp.sin(beta)
])

# Jacobian of f with respect to the state vector x
A = f.jacobian(state)

# Now substitute actual expression for beta
# beta = arctan((lr * tan(phi)) / (lf + lr))
beta_expr = sp.atan((lr * sp.tan(phi)) / (lf + lr))

# Substitute beta back into A matrix
A_substituted = A.subs(beta, beta_expr)

# Simplify the resulting Jacobian
A_simplified = sp.simplify(A_substituted)

# Print the Jacobian
sp.pprint(A_simplified)