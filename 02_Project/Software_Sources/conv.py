#==============================================================================
#
# Title: conv.py
# Author: Seongmin Hong
#
#==============================================================================

import PIL.Image as pilimg
import numpy as np
import time

#==============================================================================
# Conv2D

def conv2d(feature, kernel):

  of     = feature.copy()
  if_pad = np.pad(feature, ((1,1),(1,1)), 'constant', constant_values=0)

  k_list = kernel.reshape(-1,1)

  for i in range(if_pad.shape[0]):
    for j in range(if_pad.shape[1]):
      if (i!=0 and i!=if_pad.shape[0]-1 and j!=0 and j!=if_pad.shape[1]-1):
        if_slice     = if_pad[i-1:i+2, j-1:j+2]
        if_list      = if_slice.reshape(1,-1)
        if_dot       = np.dot(if_list, k_list)
        of[i-1, j-1] = if_dot

  return of

#==============================================================================
# Random Kernel

def random_kernel(bitwidth=8, scale=1, use_signed=1):

  kernel = np.random.randn(3,3,3) ## 평균이 0, 표준편차가 1
  kernel = kernel*scale
  kernel = np.where(kernel>1, 0, kernel)
  kernel = np.where(kernel<-1, 0, kernel)
  kernel = np.around(kernel*(2**bitwidth)) ## 표현하고 싶은 비트 크기로 곱하고 정수로 반올림
  kernel = kernel.astype(np.int32)

  if(use_signed==0):
    kernel = np.where(kernel<0, 0, kernel)

  return kernel

#==============================================================================
# Fixed point Quantization

def fixed_quan(array, bitwidth=8):

  array = np.where(array<0, (2**bitwidth)+array, array)
  array = np.where(array>=(2**bitwidth), (2**bitwidth)-1, array)

  return array

#==============================================================================
# ReLU

def relu(array):

  array = np.where(array<0, 0, array)

  return array

#==============================================================================
# Main Computation

img = pilimg.open('./dataset/sample.bmp')
img = img.resize((128,128))
img.save('./dataset/input_feature.bmp')

#------------------------------------------------------------------------------

pix_frac = 8
kernel0_frac = 6
kernel1_frac = 6
kernel2_frac = 6

pix = np.array(img, dtype=np.int32) 

kernel0 = random_kernel(kernel0_frac, 0.05, 0)
kernel1 = random_kernel(kernel1_frac,    2, 0)
kernel2 = random_kernel(kernel2_frac,    1, 1)

#------------------------------------------------------------------------------

start_time = time.time()

pix_r = pix[:,:,0]
pix_g = pix[:,:,1]
pix_b = pix[:,:,2]
kernel0_r = kernel0[:,:,0]
kernel0_g = kernel0[:,:,1]
kernel0_b = kernel0[:,:,2]
kernel1_r = kernel1[:,:,0]
kernel1_g = kernel1[:,:,1]
kernel1_b = kernel1[:,:,2]
kernel2_r = kernel2[:,:,0]
kernel2_g = kernel2[:,:,1]
kernel2_b = kernel2[:,:,2]

#------------------------------------------------------------------------------

pix_conv0_r = conv2d(pix_r, kernel0_r)
pix_conv0_g = conv2d(pix_g, kernel0_g)
pix_conv0_b = conv2d(pix_b, kernel0_b)
pix_conv1_r = conv2d(pix_r, kernel1_r)
pix_conv1_g = conv2d(pix_g, kernel1_g)
pix_conv1_b = conv2d(pix_b, kernel1_b)
pix_conv2_r = conv2d(pix_r, kernel2_r)
pix_conv2_g = conv2d(pix_g, kernel2_g)
pix_conv2_b = conv2d(pix_b, kernel2_b)

pix_conv_sum0 = relu((pix_conv0_r + pix_conv0_g + pix_conv0_b) >> kernel0_frac)
pix_conv_sum1 = relu((pix_conv1_r + pix_conv1_g + pix_conv1_b) >> kernel1_frac)
pix_conv_sum2 = relu((pix_conv2_r + pix_conv2_g + pix_conv2_b) >> kernel2_frac)

end_time = time.time()

print("Execution Time (s): ", end_time-start_time)

#------------------------------------------------------------------------------

pix_uint8           = fixed_quan(pix, 8)
kernel0_uint8       = fixed_quan(kernel0, 8)
kernel1_uint8       = fixed_quan(kernel1, 8)
kernel2_uint8       = fixed_quan(kernel2, 8)
pix_conv_sum0_uint8 = fixed_quan(pix_conv_sum0, 8)
pix_conv_sum1_uint8 = fixed_quan(pix_conv_sum1, 8)
pix_conv_sum2_uint8 = fixed_quan(pix_conv_sum2, 8)

pilimg.fromarray(pix_uint8.astype(np.uint8)).save('./dataset/input_feature.bmp')

pilimg.fromarray(kernel0_uint8.astype(np.uint8)).save('./dataset/kernel0.bmp')
pilimg.fromarray(kernel1_uint8.astype(np.uint8)).save('./dataset/kernel1.bmp')
pilimg.fromarray(kernel2_uint8.astype(np.uint8)).save('./dataset/kernel2.bmp')

pilimg.fromarray(pix_conv_sum0_uint8.astype(np.uint8)).save('./dataset/output_feature0.bmp')
pilimg.fromarray(pix_conv_sum1_uint8.astype(np.uint8)).save('./dataset/output_feature1.bmp')
pilimg.fromarray(pix_conv_sum2_uint8.astype(np.uint8)).save('./dataset/output_feature2.bmp')

print("Files Generation Done")





#==============================================================================
# Debug

# pix_frac = 8
# kernel_frac = 6

# pix = np.array( [[[10, 10, 10], [10, 10, 10], [10, 10, 10], [10, 10, 10]],
#                  [[10, 10, 10], [10, 10, 10], [10, 10, 10], [10, 10, 10]],
#                  [[10, 10, 10], [10, 10, 10], [10, 10, 10], [10, 10, 10]],
#                  [[10, 10, 10], [10, 10, 10], [10, 10, 10], [10, 10, 10]]] , dtype=np.int32)

# kernel = np.array( [[[ 16,  16, -16], [16, -16, 16], [ 16,  16, -16]], 
#                     [[ 16, -16,  16], [16,  16, 16], [ 16, -16,  16]], 
                    # [[ 16,  16, -16], [16, -16, 16], [ 16,  16, -16]]] , dtype=np.int32) 

#------------------------------------------------------------------------------

# pix_r = pix[:,:,0]
# pix_g = pix[:,:,1]
# pix_b = pix[:,:,2]
# kernel_r = kernel[:,:,0]
# kernel_g = kernel[:,:,1]
# kernel_b = kernel[:,:,2]

# #------------------------------------------------------------------------------

# pix_conv_r = conv2d(pix_r, kernel_r)
# pix_conv_g = conv2d(pix_g, kernel_g)
# pix_conv_b = conv2d(pix_b, kernel_b)

# pix_conv_sum = relu((pix_conv_r + pix_conv_g + pix_conv_b) >> kernel_frac)

# #------------------------------------------------------------------------------

# pix_uint8          = fixed_quan(pix, 8)
# kernel_uint8       = fixed_quan(kernel, 8)
# pix_conv_sum_uint8 = fixed_quan(pix_conv_sum, 8)

# pilimg.fromarray(pix_uint8.astype(np.uint8)).save('./dataset/debug_input_feature.bmp')

# pilimg.fromarray(kernel_uint8.astype(np.uint8)).save('./dataset/debug_kernel0.bmp')
# pilimg.fromarray(kernel_uint8.astype(np.uint8)).save('./dataset/debug_kernel1.bmp')
# pilimg.fromarray(kernel_uint8.astype(np.uint8)).save('./dataset/debug_kernel2.bmp')

# pilimg.fromarray(pix_conv_sum_uint8.astype(np.uint8)).save('./dataset/debug_output_feature0.bmp')
# pilimg.fromarray(pix_conv_sum_uint8.astype(np.uint8)).save('./dataset/debug_output_feature1.bmp')
# pilimg.fromarray(pix_conv_sum_uint8.astype(np.uint8)).save('./dataset/debug_output_feature2.bmp')

# print("Debug Files Generation Done")

