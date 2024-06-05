"""
Matrix Operations
Author: Ameya Marakarkandy
Last updated: 05/06/2024

List of functions
1. scalar_multiply
2. matrix_multiply
3. cross_product_matrix

"""
def scalar_multiply(scalar, matrix):
    """
    Performs scalar multiplication with a matrix.
    
    Args:
    - scalar (int or float): The scalar value to multiply with.
    - matrix (list of lists): The input matrix.
    
    Returns:
    - list of lists: The result of scalar multiplication with the matrix.
    """
    return [[scalar * element for element in row] for row in matrix]

def matrix_multiply(A, B):
    result = [[0.0 for _ in range(len(B[0]))] for _ in range(len(A))]
    for i in range(len(A)):
        for j in range(len(B[0])):
            for k in range(len(B)):
                result[i][j] += A[i][k] * B[k][j]
    return result

def cross_product_matrix(vector):
    """
    Builds the cross product matrix of a vector.
    
    Args:
    - vector (list of floats): The input vector.
    
    Returns:
    - list of lists: The cross product matrix of the vector.
    """
    if len(vector) != 3:
        raise ValueError("Input vector must have exactly 3 components for cross product matrix.")
    
    # Extract vector components
    x, y, z = vector
    
    # Build the cross product matrix
    cross_matrix = [
        [0, -z[0], y[0]],
        [z[0], 0, -x[0]],
        [-y[0], x[0], 0]
    ]
    
    return cross_matrix