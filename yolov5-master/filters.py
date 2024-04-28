from machine import ADC
import time
# 
# a0=ADC(0)
# 
# def acq(freq,s):
#     d=0
#     while d<s:
#         num=a0.read()/1024
#         print(num)
#         time.sleep(freq)
#         d+=1
#         
# acq(0.5,1000)






a0=ADC(0)
a = [0, 0, 0, 0]
weight = [0.4,0.3,0.2,0.1]
weight_Q = [3,2,1]
d_array=[0,0]
while True:
    data = a0.read()  # 读取AD值
    for i in range(0, 3):
        a[i + 1] = a[i]  # 窗口移动
    a[0] = data
    # 滑动平均
    sum = 0
    ave1 = 0
    for j in a:
        sum += j
    ave1 = sum / 4
    # 加权平均
    ave2 = 0
    for j in range(0, 3):
        ave2 += a[j] * weight[j]
    # 加权滑动平均
    ave3=0
    sum_Q=0    
    for j in weight_Q:
        sum_Q += j      
    for j in range(0, 2):
        ave3 += a[j] * weight_Q[j]
    ave3=ave3/sum_Q
    
    print("{},{},{},{}".format(data, ave1, ave2, ave3))
    
    
    
    
    
    
#    print("{},{}".format(data, ave2))
#   print("{}".format(ave2))

    # 差值队列
#     
#     d_array[0]=d_array[1]
#     d_array[1]=ave2
# 
#     print(d_array[0]-d_array[1])
# 
    time.sleep(0.01)
# 
# 





     