import torch
from torch import nn

#preprocessing data
class interet_segmentation():
    def __init__(self,):
        super().__init__()

    def skin_color_detection():
        pass

    def image_cropping():
        '''
        INPUT: segmented image with unwanted noise resion I_M*N
        OUTPUT: Noise-free image I_Denoised
        '''
        pass

class pre_processing():
    def __init__(self,):
        super().__init__()

    def rescaling():
        pass
    
    def normalization():
        pass

#feature extraction
class feature_extraction():
    def __init__(self,):
        super().__init__()
    
    def PCA():
        pass

#classification
class KNeighorsClassifier():
    def __init__(self, k=5,p=0,device='cuda',log_interval=100,log=True):
        '''
        k: the number of neighbour
        p: the order of distance
            p=0: manhattan distance
            p=1: euclidean distance
        '''
        super().__init__()
        self.k=k
        self.p=p
        self.device = device
        self.log_interval=log_interval
        self.log = log

    def fit(self, train_features, train_labels):
         self.train_features = train_features
         self.train_labels = train_labels
        
    def predict(self,test_features):
        num_features = test_features.shape[0]
        test_labels = torch.zeros((num_features),device = self.device,dtype = torch.float)
        # 
        for test_index in range(num_features):
            test_feature = test_features[test_index]
            distances = torch.norm(self.train_features - test_feature,dim=1,p=self.p)
            # get the nearest neighbor
            indexes = torch.topk(distances,self.k,largest = False)
            # get the class of the neighbor [val,ind]
            classes = torch.gather(self.train_labels,0,indexes)
            # the class val: char -> int
            mode_ind = torch.mode(classes)[1]

            test_labels[test_index] = self.train_labels[mode_ind]

            if self.log:
                if(test_index % self.log_inerval == 0):
                    print("Currently predicting at test_index = %d" % test_index)

        return test_labels
