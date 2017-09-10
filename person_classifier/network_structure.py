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
# good_result_3
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


# good_result_2
class CNN_classification_2(chainer.Chain):

    def __init__(self, train= True):
        super(CNN_classification_2, self).__init__(
            conv1 = L.Convolution2D(3, 32, 3, stride=1),
            conv2 = L.Convolution2D(32, 32, 3, pad=1),
            conv3 = L.Convolution2D(32, 64, 3, pad=1),
            conv4 = L.Convolution2D(64, 64, 3, pad=1),
            conv5 = L.Convolution2D(64, 64, 3, pad=1),
            conv6 = L.Convolution2D(64, 64, 3, pad=1),
            conv7 = L.Convolution2D(64, 64, 3, pad=1),
            conv8 = L.Convolution2D(64, 128, 3, pad=1),
            l1 = L.Linear(2048, 1024),
            l2 = L.Linear(1024, 512),
            lo = L.Linear(512, 10),
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
        h = F.max_pooling_2d(F.relu(h), 3, stride=2)
        h = F.relu(self.bn2(self.conv5(h)))
        h = F.relu(self.bn2(self.conv6(h)))
        h = F.relu(self.bn2(self.conv7(h)))
        h = F.relu(self.bn3(self.conv8(h)))
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


#  good_result_1
class CNN_classification_1(chainer.Chain):

    def __init__(self, train= True):
        super(CNN_classification_1, self).__init__(
            conv1 = L.Convolution2D(3, 32, 3, stride=1),
            bn1 = L.BatchNormalization(32),
            conv2 = L.Convolution2D(32, 64, 3, pad=1),
            bn2 = L.BatchNormalization(64),
            conv3 = L.Convolution2D(64, 64, 3, pad=1),
            conv4 = L.Convolution2D(64, 64, 3, pad=1),
            conv5 = L.Convolution2D(64, 128, 3, pad=1),
            l1 = L.Linear(2048, 1024),
            l2 = L.Linear(1024, 1024),
            lo = L.Linear(1024, 10),
        )
        self.train = train

    def clear(self):
        self.loss = None
        self.accuracy = None
        self.h = None

    def forward(self, x):
        h = self.bn1(self.conv1(x))
        h = F.max_pooling_2d(F.relu(h), 3, stride=2)
        h = self.bn2(self.conv2(h))
        h = F.max_pooling_2d(F.relu(h), 3, stride=2)
        h = F.relu(self.conv3(h))
        h = F.relu(self.conv4(h))
        h = F.relu(self.conv5(h))
        h = F.max_pooling_2d(F.relu(h), 2, stride=2)
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


#  the first network
class CNN_classification_0(chainer.Chain):

    def __init__(self, train= True):
        super(CNN_classification_0, self).__init__(
            conv1 = L.Convolution2D(3, 64, 3, stride=1, pad=2),
            conv2 = L.Convolution2D(64, 64, 3, stride=1, pad=2),
            conv3 = L.Convolution2D(64, 128, 3, stride=1),
            l1 = L.Linear(2048, 1000),
            lo = L.Linear(1000, 10),
        )
        self.train = train

    def clear(self):
        self.loss = None
        self.accuracy = None
        self.h = None

    def forward(self, x):
        h = self.conv1(x)
        h = F.max_pooling_2d(h,2,stride = 2)
        h = self.conv2(h)
        h = F.max_pooling_2d(h,2,stride = 2)
        h = self.conv3(h)
        h = F.max_pooling_2d(h,2,stride = 2)
        h = F.dropout(F.relu(self.l1(h)))
        h = self.lo(h)
        return h

    def __call__(self, x, t):
        self.clear()
        h = self.forward(x)
        self.loss = F.softmax_cross_entropy(h, t)
        reporter.report({'loss': self.loss}, self)
        self.accuracy = accuracy.accuracy(h, t)
        reporter.report({'accuracy': self.accuracy}, self)
        return self.loss
