import numpy as np
from pysr import PySRRegressor, TemplateExpressionSpec

# Create data
X = np.random.randn(1000, 3)
y = np.sin(X[:, 0] + X[:, 1]) + X[:, 2]**2

# Define template: we want sin(f(x1, x2)) + g(x3)
template = TemplateExpressionSpec(
    function_symbols=["f", "g"],
    combine="((; f, g), (x1, x2, x3)) -> sin(f(x1, x2)) + g(x3)",
)

model = PySRRegressor(
    expression_spec=template,
    binary_operators=["+", "*", "-", "/"],
    unary_operators=["sin"],
    maxsize=10,
)
model.fit(X, y)
print(model)