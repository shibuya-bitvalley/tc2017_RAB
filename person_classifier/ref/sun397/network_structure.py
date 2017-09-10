# python library
import numpy as np

# chainer library
import chainer
import chainer.functions as F
import chainer.links as L
from chainer.functions.evaluation import accuracy
from chainer.functions.loss import softmax_cross_entropy
from chainer import reporter


# networks
# model_2
class CNN_colorization(chainer.Chain):
    def __init__(self, train= True):
        super(CNN_colorization, self).__init__(
            conv1 = L.Convolution2D(1, 32, 3),
            conv2 = L.Convolution2D(32, 64, 3),
            conv3 = L.Convolution2D(64, 128, 3),
            conv4 = L.Convolution2D(128, 8, 3),
            l1 = L.Linear(14112, 8000),
            l2 = L.Linear(8000, 8000),
            lo = L.Linear(8000, 7500),
            bn1 = L.BatchNormalization(32),
            bn2 = L.BatchNormalization(64),
            bn3 = L.BatchNormalization(128),
            bn4 = L.BatchNormalization(8),
        )
        self.train = train

    def clear(self):
        self.loss = None
        self.accuracy = None
        self.h = None

    def forward(self, x):
        h = F.relu(self.bn1(self.conv1(x)))
        h = F.relu(self.bn2(self.conv2(h)))
        h = F.relu(self.bn3(self.conv3(h)))
        h = F.relu(self.bn4(self.conv4(h)))
        h = F.relu(self.l1(h))
        h = F.dropout(F.relu(self.l2(h)))
        y = self.lo(h)
        y = F.reshape(y, (y.shape[0], 3,50,50))
        return y

    def __call__(self, x, t):
        self.clear()
        y = self.forward(x)
        self.loss = F.mean_squared_error(y, t)
        reporter.report({'loss': self.loss}, self)
        return self.loss
