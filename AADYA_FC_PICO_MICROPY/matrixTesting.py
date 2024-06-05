import matrixOps

A = [[1,0,0],
     [0,0,0],
     [0,0,1]]

B = [[1,5,0],
     [0,1,0],
     [0,5,1]]

C = matrixOps.matrix_multiply(A,B)
D = matrixOps.scalar_multiply(2,A)

V = [[1],
     [2],
     [3]]


L= matrixOps.cross_product_matrix(V)
print(L)

