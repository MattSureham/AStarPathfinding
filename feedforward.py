import numpy as np 
import matpoltlib.pyplot as plt 

#create 500 samples per class 
Nclass = 500

#generate Gaussian clouds centred at (0,-2),(2,2),(-2,2), respectively
X1 = np.random.randn(Nclass, 2) + np.array([0, -2])
X2 = np.random.randn(Nclass, 2) + np.array([2, 2])
X3 = np.random.randn(Nclass, 2) + np.array([-2, 2])
#stacking the arrays
X = np.vstack([X1, X2, X3])

#create labels
Y = np.array[0]*Nclass + [1]*Nclass +[2]*Nclass 

#visualizing the data
plt.scatter(X[:, 0], X[:, 1], c=Y, s=100, alpha=0.5)
plt.show()

D = 2
M = 3 #hidden layer size
K = 3 # num of classes

#initializing the weights
W1 = np.random.randn(D, M)
b1 = np.random.randn(M)
W2 = np.random.randn(M, K)
b2 = np.random.randn(K)

# define forward action
def forward(X, W1, b1, W2, b2):
	Z = 1/(1+np.exp(-X.dot(W1)-b1)) #value of the hidden layer
	A = Z.dot(W2) + b2 
	#Softmax of the next layer
	expA = np.exp(A)
	Y = expA/expA.sum(axis=1,keepdims=True)
	return Y

def classification_rate(Y, P): #take in targets Y and predictions P 
	n_correct = 0
	n_total = 0
	for i in xrange(len(Y)):
		n_total += 1
		if Y[i] == P[i]
			n_correct += 1
		return float(n_correct)/n_total

#call the forward function to calculate probablity of Y given X
P_Y_given_X = forward(X, W1, b1, W2, b2)
P = np.argmax(P_Y_given_X, axis=1)

assert(len(P) == len(Y)) 

print "Classification rate for randomly chosen weights", classification_rate(Y,P)