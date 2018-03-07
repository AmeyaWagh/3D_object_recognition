from keras.preprocessing.image import ImageDataGenerator
from keras.models import Sequential
from keras.layers.core import Dense, Dropout, Activation, Flatten
from keras.layers.convolutional import Conv3D,Convolution3D, MaxPooling3D

from keras.optimizers import SGD, RMSprop
from keras.utils import np_utils, generic_utils
from sklearn.preprocessing import LabelBinarizer
from termprinter import *
from keras.models import model_from_json
import numpy as np
import os

class CNNModel:
    def __init__(self,nb_classes=5,ip_shape=(1, 40, 40, 40)):
        self.encoder = LabelBinarizer()
        self.CNN_model(nb_classes=nb_classes,ip_shape=ip_shape)    

    def CNN_model(self,nb_classes = 5,ip_shape=(1, 40, 40, 40)):
        self.model = Sequential()
        self.model.add(
            Conv3D(32, (5,5,5), 
            strides=(1, 1, 1), padding='valid', data_format="channels_first",input_shape=ip_shape,
            activation='relu', dilation_rate=(1, 1, 1) ))
        self.model.add(
            Conv3D(32, (5,5,5), 
            strides=(1, 1, 1), padding='valid', data_format="channels_first",
            # activation='relu'))
            activation='sigmoid'))
        
        self.model.add(MaxPooling3D(pool_size=(2,2,2)))
        self.model.add(Dropout(0.1))
        
        self.model.add(
            Conv3D(32, (3,3,3), 
            strides=(1, 1, 1), padding='valid', data_format="channels_first",
            # activation='relu'))
            activation='sigmoid'))
        self.model.add(
            Conv3D(32, (3,3,3), 
            strides=(1, 1, 1), padding='valid', data_format="channels_first",
            activation='relu'))

        self.model.add(MaxPooling3D(pool_size=(2,2,2)))
        self.model.add(Dropout(0.1))
        
        self.model.add(
            Conv3D(32, (3,3,3), 
            strides=(1, 1, 1), padding='valid', data_format="channels_first",
            activation='relu'))
        self.model.add(
            Conv3D(32, (3,3,3), 
            strides=(1, 1, 1), padding='valid', data_format="channels_first",
            activation='relu'))

        self.model.add(MaxPooling3D(pool_size=(2,2,2)))
        self.model.add(Dropout(0.1))
        

        self.model.add(Flatten())
        # self.model.add(Dense(128, init='normal', activation='relu'))
        self.model.add(Dense(128, kernel_initializer='normal', activation='relu'))
        # self.model.add(Dropout(0.5))
        # self.model.add(Dense(nb_classes,init='normal'))
        self.model.add(Dense(nb_classes,kernel_initializer='normal'))
        self.model.add(Activation('softmax'))
        # self.model.compile(loss='categorical_crossentropy', optimizer='RMSprop', metrics=['mse', 'accuracy'])
        self.model.compile(loss='categorical_crossentropy', optimizer='adam', metrics=['mse', 'accuracy'])
        # return self.model
    
    def train(self,data,labels):
        labels = self.encoder.fit_transform(labels)
        hist = self.model.fit(data,labels,batch_size=10, epochs = 10,shuffle=True)  
        
    def test(self,data,labels): 
        labels = self.encoder.fit_transform(labels)
        score = self.model.evaluate(data,labels,batch_size=10)
        print('**********************************************')
        print(OKGREEN+"[{d[0]}]{d[3]}   [{d[1]}]{d[4]}  [{d[2]}]{d[5]}".format(d=self.model.metrics_names+score) + ENDC)

    def predict(self,data,labelsSet):
        _idx = np.argmax(self.model.predict(data))
        return labelsSet[_idx],self.model.predict_proba(data)[0][_idx]

    def save_model(self,base_path="."):
        # serialize model to JSON
        json_path = os.path.join(base_path,'model.json')
        print(OKGREEN+json_path+ENDC)
        model_json = self.model.to_json()
        with open(json_path, "w") as json_file:
            json_file.write(model_json)
        # serialize weights to HDF5
        weight_file = os.path.join(base_path,"model.h5")
        print(OKGREEN+weight_file+ENDC)
        self.model.save_weights(weight_file)
        print("Saved model to disk")

    def load_model(self,base_path="."):
        # load json and create model
        json_path = os.path.join(base_path,'model.json')
        print(OKGREEN+json_path+ENDC)
        json_file = open(json_path, 'r')
        loaded_model_json = json_file.read()
        json_file.close()
        
        self.model = model_from_json(loaded_model_json)
        # load weights into new model
        weight_file = os.path.join(base_path,"model.h5")
        print(OKGREEN+weight_file+ENDC)
        self.model.load_weights(weight_file)
        print(OKGREEN+"Loaded model from disk"+ENDC)