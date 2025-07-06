import math
class CentroidTracking():
    def __init__(self,maxDisappeared=5,maxobjectID=8):
        self.maxDisappeared = maxDisappeared  # 达到最大连续帧数删除
        self.maxobjectID = maxobjectID  # 追踪的最大目标数量
        self.ObjectNum = 0         #目标数量
        self.objectrects =[]       #目标信息，坐标框信息
        self.objectCentroids =[]   #目标信息，质心坐标信息
        self.objectIDs =[]         #目标ID，跟目标信息对应,ID从1开始
        self.objectlost =[]        #目标消失次数，跟目标信息对应

    # 增加新对象函数
    def register(self,centroid,rect):
        self.objectCentroids.append(centroid)     #保存当前目标质心到元组中
        self.objectIDs.append(self.ObjectNum)     #新目标ID取最大值
        self.objectrects.append(rect)             #保存目标框信息
        self.objectlost.append(0)                 #Id位置的目标丢失帧数为0
        self.ObjectNum = self.ObjectNum+1         #数量加1

    # 移除对象函数,移除之后需要重新排列ID号
    def dergister(self, objectID):
        self.ObjectNum = self.ObjectNum-1         #ID数量减1
        del self.objectCentroids[objectID]        #删除目标位置信息
        del self.objectlost[objectID]             #删除丢失信息
        del self.objectIDs[objectID]              #删除ID
        del self.objectrects[objectID]            #删除坐标框
        if self.ObjectNum>0: #防止指针越界
           for i in range(0,self.ObjectNum):       # 移除对象函数,移除之后需要重新排列ID号
              self.objectIDs[i]=i

    def getobjID(self):
        return self.objectIDs

    def qsort(self,unsort_list,Order_list):
        COLOR_NUM = len(unsort_list)
        for i in range(0,COLOR_NUM):
            j=i+1
            for j in range(0,COLOR_NUM-1):
                if unsort_list[i]<unsort_list[j] :
                    unsort_list[i],unsort_list[j] = unsort_list[j],unsort_list[i]
                    Order_list[i],Order_list[j] = Order_list[j],Order_list[i]
                j+=1
            i+=1
    def euclidean_distance(self,p1, p2):
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
    # 更新函数
    def update(self,iputrects):
        if len(iputrects) == 0 and len(self.objectIDs)>0: #当前帧没有目标
            for i in range(0,len(self.objectIDs)):
                self.objectlost[i]=self.objectlost[i]+1
            if(len(self.objectIDs)):
                for i in range(len(self.objectIDs)-1,-1,-1):#从大到小遍历
                    if self.objectlost[i] > self.maxDisappeared:
                        self.dergister(i) #删除丢失目标
            print('objectrect',self.objectrects)
            print('objectID',self.objectIDs)
            return  self.objectrects
        # 存储质心坐标
        inputCentroids = []
        for (i, (startX, startY, rectW, rectH)) in enumerate(iputrects):
            cX = int((startX +rectW/2.0))
            cY = int((startY + rectH/2.0))
            inputCentroids.append((cX, cY))
        #print(len(inputCentroids))
        #print(inputCentroids)
        if len(self.objectCentroids) == 0:
            for i in range(0, len(inputCentroids)):
                if self.ObjectNum>=self.maxobjectID: break
                self.register(inputCentroids[i],iputrects[i])
        else:
            #print(len(self.objectCentroids))
            # 计算距离排序
            objID=[]
            Centroidsdis=[]
            CentroidID=[]
            if(self.ObjectNum>=len(inputCentroids)):#没有新增目标
                for i in range(0,len(inputCentroids)):
                    Centroidsdis=[]
                    CentroidID=[]
                    iputrect=iputrects[i]
                    inputCentroid=inputCentroids[i] #新目标
                    #print(inputCentroid)
                    for j in range(0,self.ObjectNum):
                        objCentroid=self.objectCentroids[j]#旧目标
                        #print(objCentroid)
                        D=self.euclidean_distance(inputCentroid,objCentroid)
                        Centroidsdis.append(D)         #D是无序距离值
                        CentroidID.append(j)           #j是有序的下标
                    self.qsort(Centroidsdis,CentroidID)    #欧式距离排序
                    self.objectCentroids[CentroidID[0]]=inputCentroid #更新质心
                    self.objectrects[CentroidID[0]]=iputrect#更新目标矩形信息
                    self.objectlost[CentroidID[0]]=0#更新目标丢失计数
                    objID.append(CentroidID[0])#记录有更新的ID
                objectsid=[]
                for id in self.objectIDs:#将id取出
                    objectsid.append(id)
                if(len(objectsid)>len(objID)):#存在未更新的ID
                    #print(objID)#对未更新的目标框进行lost累计
                    #print(self.objectIDs)#对未更新的目标框进行lost累计
                    for i in range(0,len(objID)):
                        iid=objID[i]
                        for j in range(0,len(objectsid)):
                            if(objectsid[j]==iid):
                                del objectsid[j]#去除有更新的id，剩下的就是未更新的
                                break
                    #print(objectsid)
                    #print(len(objectsid))
                    if (len(objectsid)>0):
                        for i in range(0,len(objectsid)):
                               self.objectlost[objectsid[i]]=self.objectlost[objectsid[i]]+1
                    if(len(self.objectIDs)):
                        for i in range(len(self.objectIDs)-1,-1,-1): #从大到小遍历
                            if self.objectlost[i] > self.maxDisappeared:
                                self.dergister(i) #删除丢失目标

                del objectsid
                del objID
                #print(self.objectlost)

            else:#旧目标比新目标少（有新增目标）
                for i in range(0,self.ObjectNum):
                    objCentroid=self.objectCentroids[i]         #旧目标
                    Centroidsdis=[]
                    CentroidID=[]
                    for j in range(0,len(inputCentroids)):
                        inputCentroid=inputCentroids[j]         #新目标
                        D=self.euclidean_distance(inputCentroid, objCentroid)
                        Centroidsdis.append(D)                  #D是无序距离值
                        CentroidID.append(j)                    #j是有序的下标
                    self.qsort(Centroidsdis,CentroidID)         #欧式距离排序
                    self.objectCentroids[i]=inputCentroids[CentroidID[0]] #更新质心
                    self.objectrects[i]=iputrects[CentroidID[0]]#更新目标矩形信息
                    self.objectlost[i]=0                        #更新目标丢失计数
                    del inputCentroids[CentroidID[0]]           #去除已更新质心
                    del iputrects[CentroidID[0]]                #去除已更新坐标框
                    del Centroidsdis
                    del CentroidID
                if len(inputCentroids)>0:
                    for i in range(0,len(inputCentroids)):
                        if self.ObjectNum>=self.maxobjectID: break
                        self.register(inputCentroids[i],iputrects[i]) #注册新目标
        del iputrects
        del inputCentroids
        #print('objectrect',self.objectrects)
        #print('objectID',self.objectIDs)
        return self.objectrects
