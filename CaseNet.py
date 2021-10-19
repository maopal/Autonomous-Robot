"""Dilated ResNet"""
import math
import torch
import torch.utils.model_zoo as model_zoo
import torch.nn as nn



class block(nn.Module):
    def __init__(self, in_channels, out_channels, identity_downsample=None, stride=1, dilation=1):
        super(block,self).__init__()
        self.expansion = 4
        self.conv1 = nn.Conv2d(in_channels, out_channels, kernel_size=1, stride=1, padding=0)
        self.bn1 = nn.BatchNorm2d(out_channels)
        self.conv2 = nn.Conv2d(out_channels, out_channels, kernel_size=3, stride=stride, padding=dilation, dilation=dilation)
        self.bn2 = nn.BatchNorm2d(out_channels)
        self.conv3 = nn.Conv2d(out_channels, out_channels*self.expansion, kernel_size=1, stride=1, padding=0)
        self.bn3 = nn.BatchNorm2d(out_channels*self.expansion)
        self.relu = nn.ReLU()
        self.identity_downsample = identity_downsample
        
    def forward(self, x):
         identity = x
         x = self.conv1(x)
         x = self.bn1(x)
         x = self.relu(x)
         x = self.conv2(x)
         x = self.bn2(x)
         x = self.relu(x)
         x = self.conv3(x)
         x = self.bn3(x)
         
         if self.identity_downsample is not None: 
             identity = self.identity_downsample(identity)
         x += identity
         x = self.relu(x)
         return x
     
        
class CaseNet(nn.Module):
    def __init__(self, block, layers, image_channels, num_classes):
        super(CaseNet, self).__init__()
        self.in_channels = 64
        self.conv1 = nn.Conv2d(image_channels, 64, kernel_size=7, stride=1, padding=3) 
        self.bn1 = nn.BatchNorm2d(64)
        self.relu = nn.ReLU()
        self.maxpool = nn.MaxPool2d(kernel_size=3, stride=2, padding=1)
        
        self.layer1 = self._make_layer(block, layers[0], out_channels=64,  dilation=2, stride=1)
        self.layer2 = self._make_layer(block, layers[1], out_channels=128, dilation=2, stride=2)
        self.layer3 = self._make_layer(block, layers[2], out_channels=256, dilation=2, stride=2)
        self.layer4 = self._make_layer(block, layers[3], out_channels=512, dilation=4, stride=1) 
        
        
        """CHANGE TO INTERPOLATION-BILINEAR UP SAMPLE [IF NEEDED]"""
        self.side1 = nn.Conv2d(64, 1, 1)
        self.side2 = nn.Sequential(nn.Conv2d(256, 1, 1, bias=True),
                                   nn.ConvTranspose2d(1, 1, 4, stride=2, padding=1, bias=False))
        self.side3 = nn.Sequential(nn.Conv2d(512, 1, 1, bias=True),
                                   nn.ConvTranspose2d(1, 1, 8, stride=4, padding=2, bias=False))
        self.side5 = nn.Sequential(nn.Conv2d(2048, num_classes, 1, bias=True),
                                   nn.ConvTranspose2d(num_classes, num_classes, 16, stride=8, padding=4, bias=False))
        
        
    def forward(self, x):
        x = self.conv1(x)
        x = self.bn1(x)
        c1 = self.relu(x)
        x = self.maxpool(c1)
        
        c2 = self.layer1(x)
        c3 = self.layer2(c2)
        c4 = self.layer3(c3)
        c5 = self.layer4(c4)
        
        side1 = self.side1(c1)
        side2 = self.side2(c2)
        side3 = self.side3(c3)
        side5 = self.side5(c5)
        
        return side1, side2, side3, side5
        
        
    def _make_layer(self, block, num_residual_blocks, out_channels, stride, dilation=1):
        identity_downsample = None
        layers = []
        
        if stride != 1 or self.in_channels != out_channels * 4:
            identity_downsample = nn.Sequential(nn.Conv2d(self.in_channels, out_channels*4, kernel_size=1, stride=stride, padding=0),
                                                nn.BatchNorm2d(out_channels*4)) 
        
        layers.append(block(self.in_channels, out_channels, identity_downsample,stride, dilation))
        self.in_channels = out_channels*4
        
        for i in range(num_residual_blocks -1):
            layers.append(block(self.in_channels, out_channels, dilation=dilation)) 
            
        return nn.Sequential(*layers)
            
     
def casenet(img_channel=3, num_classes=10):
    return CaseNet(block, [3,4,23,3], img_channel, num_classes)

def test_CaseNet():
    net = casenet()
    x = torch.randn(2,3,224,224)
    for i in range(5):
     y = net(x)[i].to('cpu')
     print(y.shape)
    
test_CaseNet()



"""
class Upsample(nn.Module):   
    def __init__(self, num_classes, sides=[]):
        super(Upsample, self).__init__()
        self.side1 = nn.Conv2d(64, 1, 1)
        self.side2 = nn.Sequential(nn.Conv2d(256, 1, 1, bias=True),
                                   nn.ConvTranspose2d(1, 1, 4, stride=2, padding=1, bias=False))
        self.side3 = nn.Sequential(nn.Conv2d(512, 1, 1, bias=True),
                                   nn.ConvTranspose2d(1, 1, 8, stride=4, padding=2, bias=False))
        self.side5 = nn.Sequential(nn.Conv2d(2048, num_classes, 1, bias=True),
                                   nn.ConvTranspose2d(num_classes, num_classes, 16, stride=8, padding=4, bias=False))
        self.sides = sides
    def forward(self, x):
        # import pdb #hy added
        # pdb.set_trace() #hy added

        side1 = self.side1(self.sides[0])
        side2 = self.side2(self.sides[1])
        side3 = self.side3(self.sides[2])
        side5 = self.side5(self.sides[3])
        
        return side1, side2, side3, side5
    
""" 


