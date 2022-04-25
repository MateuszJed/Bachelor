import numpy as np
import scipy.io


mat = scipy.io.loadmat('K.mat')
# val = np.asarray(mat, dtype='double').tolist()


x = mat["K"].tolist()

x = np.asarray(x, dtype='double').tolist()

print(x)