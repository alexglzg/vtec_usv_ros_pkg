import numpy as np

loaded = np.load('weights.npz')

w1=loaded['w1']
b1=loaded['b1']
w2=loaded['w2']
b2=loaded['b2']
w3=loaded['w3']
b3=loaded['b3']
#print(np.size(b3))

state=np.ones(6)
#np.array([0.2,0.3,0.1,0.5,0.2,0.3])

h1 = np.tanh(np.matmul(state, w1) + b1)

h2 = np.tanh(np.matmul(np.concatenate([state, h1[0]]), w2) + b2)
action = np.matmul(np.concatenate([state, h1[0], h2[0]]), w3) + b3