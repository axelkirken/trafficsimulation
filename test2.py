import scipy.stats as st
import numpy as np

a = np.array([1,2,2,2,4,5])
print(st.mode(a, keepdims=False)[0])