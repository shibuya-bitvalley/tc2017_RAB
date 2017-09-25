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
class CNN_thibault2(chainer.Chain):

    def __init__(self, train= True):
        super(CNN_thibault2, self).__init__(
            conv1 = L.Convolution2D(3, 6, 3, stride=1),
            conv2 = L.Convolution2D(6, 6, 3, pad=1),
            l1 = L.Linear(864, 432),
            l2 = L.Linear(432, 100),
            lo = L.Linear(100, 2),
            bn1 = L.BatchNormalization(6),
        )
        self.train = train

    def clear(self):
        self.loss = None
        self.accuracy = None
        self.h = None

    def forward(self, x):
        h = F.relu(self.conv1(x))
        h = F.max_pooling_2d(F.relu(h), 3, stride=2)
        h = F.relu(self.conv2(h))
        h = F.max_pooling_2d(F.relu(h), 3, stride=2)
        h = F.relu(self.l1(h))
        h = F.relu(self.l2(h))
        y = self.lo(h)
        return y

    def __call__(self, x, t):
        self.clear()
        y = self.forward(x)
        self.loss = F.softmax_cross_entropy(y, t)
        reporter.report({'loss': self.loss}, self)
        self.accuracy = accuracy.accuracy(y, t)
        reporter.report({'accuracy': self.accuracy}, self)
        return self.loss


# thibault_model, 2, 3, 4
class CNN_thibault(chainer.Chain):

    def __init__(self, train= True):
        super(CNN_thibault, self).__init__(
            conv1 = L.Convolution2D(3, 3, 3, stride=1),
            conv2 = L.Convolution2D(3, 3, 3, pad=1),
            l1 = L.Linear(432, 100),
            lo = L.Linear(100, 2),
            bn1 = L.BatchNormalization(3),
        )
        self.train = train

    def clear(self):
        self.loss = None
        self.accuracy = None
        self.h = None

    def forward(self, x):
        h = F.relu(self.conv1(x))
        h = F.max_pooling_2d(F.relu(h), 3, stride=2)
        h = F.relu(self.conv2(h))
        h = F.max_pooling_2d(F.relu(h), 3, stride=2)
        h = F.relu(self.l1(h))
        y = self.lo(h)
        return y

    def __call__(self, x, t):
        self.clear()
        y = self.forward(x)
        self.loss = F.softmax_cross_entropy(y, t)
        reporter.report({'loss': self.loss}, self)
        self.accuracy = accuracy.accuracy(y, t)
        reporter.report({'accuracy': self.accuracy}, self)
        return self.loss


# model_thibault with BatchNormalization
class CNN_classifier3(chainer.Chain):

    def __init__(self, train= True):
        super(CNN_classifier3, self).__init__(
            conv1 = L.Convolution2D(3, 3, 3, stride=1),
            conv2 = L.Convolution2D(3, 3, 3, pad=1),
            l1 = L.Linear(432, 100),
            lo = L.Linear(100, 2),
            bn1 = L.BatchNormalization(3),
        )
        self.train = train

    def clear(self):
        self.loss = None
        self.accuracy = None
        self.h = None

    def forward(self, x):
        h = F.relu(self.bn1(self.conv1(x)))
        h = F.max_pooling_2d(F.relu(h), 3, stride=2)
        h = F.relu(self.bn1(self.conv2(h)))
        h = F.max_pooling_2d(F.relu(h), 3, stride=2)
        h = F.relu(self.l1(h))
        y = self.lo(h)
        return y

    def __call__(self, x, t):
        self.clear()
        y = self.forward(x)
        self.loss = F.softmax_cross_entropy(y, t)
        reporter.report({'loss': self.loss}, self)
        self.accuracy = accuracy.accuracy(y, t)
        reporter.report({'accuracy': self.accuracy}, self)
        return self.loss


# first model
class CNN_classifier(chainer.Chain):

    def __init__(self, train= True):
        super(CNN_classifier, self).__init__(
            conv1 = L.Convolution2D(3, 32, 3, stride=1),
            conv2 = L.Convolution2D(32, 32, 3, pad=1),
            conv3 = L.Convolution2D(32, 64, 3, pad=1),
            conv4 = L.Convolution2D(64, 64, 3, pad=1),
            conv5 = L.Convolution2D(64, 64, 3, pad=1),
            conv6 = L.Convolution2D(64, 64, 3, pad=1),
            conv7 = L.Convolution2D(64, 128, 3, pad=1),
            conv8 = L.Convolution2D(128, 64, 3, pad=1),
            conv9 = L.Convolution2D(64, 128, 3, pad=1),
            l1 = L.Linear(18432, 1024),
            l2 = L.Linear(1024, 512),
            lo = L.Linear(512, 3),
            bn1 = L.BatchNormalization(32),
            bn2 = L.BatchNormalization(64),
            bn3 = L.BatchNormalization(128),
        )
        self.train = train

    def clear(self):
        self.loss = None
        self.accuracy = None
        self.h = None

    def forward(self, x):
        h = F.relu(self.bn1(self.conv1(x)))
        h = self.bn1(self.conv2(h))
        h = F.max_pooling_2d(F.relu(h), 3, stride=2)
        h = F.relu(self.bn2(self.conv3(h)))
        h = F.relu(self.bn2(self.conv4(h)))
        h = F.relu(self.bn2(self.conv5(h)))
        h = F.max_pooling_2d(F.relu(h), 3, stride=2)
        h = F.relu(self.bn2(self.conv6(h)))
        h = F.relu(self.bn3(self.conv7(h)))
        h = F.relu(self.bn2(self.conv8(h)))
        h = F.relu(self.bn3(self.conv9(h)))
        h = F.dropout(F.max_pooling_2d(F.relu(h), 2, stride=2))
        h = F.dropout(F.relu(self.l1(h)))
        h = F.dropout(F.relu(self.l2(h)))
        y = self.lo(h)
        return y

    def __call__(self, x, t):
        self.clear()
        y = self.forward(x)
        self.loss = F.softmax_cross_entropy(y, t)
        reporter.report({'loss': self.loss}, self)
        self.accuracy = accuracy.accuracy(y, t)
        reporter.report({'accuracy': self.accuracy}, self)
        return self.loss


class CNN_classifier_test(chainer.Chain):

    def __init__(self, train= True):
        super(CNN_classifier_test, self).__init__(
            conv1 = L.Convolution2D(3, 32, 3, stride=1),
            conv2 = L.Convolution2D(32, 32, 3, pad=1),
            conv3 = L.Convolution2D(32, 64, 3, pad=1),
            conv4 = L.Convolution2D(64, 64, 3, pad=1),
            conv5 = L.Convolution2D(64, 64, 3, pad=1),
            conv6 = L.Convolution2D(64, 64, 3, pad=1),
            conv7 = L.Convolution2D(64, 128, 3, pad=1),
            conv8 = L.Convolution2D(128, 64, 3, pad=1),
            conv9 = L.Convolution2D(64, 128, 3, pad=1),
            l1 = L.Linear(18432, 1024),
            l2 = L.Linear(1024, 512),
            lo = L.Linear(512, 3),
            bn1 = L.BatchNormalization(32),
            bn2 = L.BatchNormalization(64),
            bn3 = L.BatchNormalization(128),
        )
        self.train = train

    def clear(self):
        self.loss = None
        self.accuracy = None
        self.h = None

    def forward(self, x):
        h = F.relu(self.bn1(self.conv1(x)))
        h = self.bn1(self.conv2(h))
        h = F.max_pooling_2d(F.relu(h), 3, stride=2)
        h = F.relu(self.bn2(self.conv3(h)))
        h = F.relu(self.bn2(self.conv4(h)))
        h = F.relu(self.bn2(self.conv5(h)))
        h = F.max_pooling_2d(F.relu(h), 3, stride=2)
        h = F.relu(self.bn2(self.conv6(h)))
        h = F.relu(self.bn3(self.conv7(h)))
        h = F.relu(self.bn2(self.conv8(h)))
        h = F.relu(self.bn3(self.conv9(h)))
        h = F.max_pooling_2d(F.relu(h), 2, stride=2)
        h = F.relu(self.l1(h))
        h = F.relu(self.l2(h))
        y = self.lo(h)
        return y

    def __call__(self, x, t):
        self.clear()
        y = self.forward(x)
        self.loss = F.softmax_cross_entropy(y, t)
        reporter.report({'loss': self.loss}, self)
        self.accuracy = accuracy.accuracy(y, t)
        reporter.report({'accuracy': self.accuracy}, self)
        return self.loss
