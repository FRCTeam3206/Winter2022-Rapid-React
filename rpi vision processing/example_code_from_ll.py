import cv2
import numpy as np

# global variables go here:
testVar = 0

# To change a global variable inside a function,
# re-declare it the global keyword
def incrementTestVar():
    global testVar
    testVar = testVar + 1
    if testVar == 100:
        print("test")
    if testVar >= 200:
        print("print")
        testVar = 0

def drawDecorations(image):
    cv2.putText(image, 
        'Limelight python script!', 
        (0, 230), 
        cv2.FONT_HERSHEY_SIMPLEX, 
        .5, (0, 255, 0), 1, cv2.LINE_AA)
def overlappingRectangles(rect1,rect2):
    x = max(rect1[0], rect2[0])
    y = max(rect1[1], rect2[1])
    w = min(rect1[0] + rect1[2], rect2[0] + rect2[2]) - x
    h = min(rect1[1] + rect1[3], rect2[1] + rect2[3]) - y
    return w>0 and h>0
def scaleRectangle(rect,x_scale,y_scale):
    rect[0]-=(rect[2]*x_scale-rect[2])/2
    rect[1]-=(rect[3]*y_scale-rect[3])/2
    rect[2]*=x_scale
    rect[3]*=y_scale
    return rect
def drawRectangle(image,rect,color):
    cv2.rectangle(image,(int(rect[0]),int(rect[1])),(int(rect[0]+rect[2]),int(rect[1]+rect[3])),color,2)
def contour_from_rect(rect):
    pt1=np.array([[rect[0],rect[1]]],dtype=np.int32)
    pt2=np.array([[rect[0]+rect[2],rect[1]]],dtype=np.int32)
    pt3=np.array([[rect[0],rect[1]+rect[3]]],dtype=np.int32)
    pt4=np.array([[rect[0]+rect[2],rect[1]+rect[3]]],dtype=np.int32)
    return np.array([pt1,pt2,pt3,pt4],dtype=np.int32)
# runPipeline() is called every frame by Limelight's backend.
desired_num_contours_check=7
first=True
def cluster_by_pairs(val,cluster,pairs):
    for pair in pairs:
        if pair[0]==val and not pair[1] in cluster:
            cluster.append(pair[1])
            cluster_by_pairs(pair[1],cluster,pairs)
        if pair[1]==val and not pair[0] in cluster:
            cluster.append(pair[0])
            cluster_by_pairs(pair[0],cluster,pairs)
    return cluster
def min_enclosing_rectangle(rects):
    x=500
    y=500
    r=0
    b=0
    for rect in rects:
        x=min(x,rect[0])
        y=min(y,rect[1])
        r=max(r,rect[0]+rect[2])
        b=max(b,rect[1]+rect[2])
    return [x,y,r-x,b-y]
def runPipeline(image, llrobot):
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    img_threshold = cv2.inRange(img_hsv, (60, 70, 70), (85, 255, 255))
   
    contours, _ = cv2.findContours(img_threshold, 
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  
    largestContour = np.array([[]])
    llpython = [0,0,0,0,0,0,0,0]
    x=320/2
    y=240/2
    width=0
    height=0
    size_sim_k=4
    if len(contours) > 0:
        num_contours_check=min(desired_num_contours_check,len(contours))
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        rects=list()
        for contour in contours[:num_contours_check]:
            x,y,w,h=cv2.boundingRect(contour)
            rects.append([x,y,w,h])
        for rect in rects:
            drawRectangle(image,rect,(255,0,255))
        cluster_ids=list()
        for i in range(num_contours_check):
            cluster_ids.append(i)
        pairs=list()
        for i in range(num_contours_check):
            for j in range(num_contours_check):
                filters=list()
                filters.append(overlappingRectangles(scaleRectangle(rects[i][:],3,1.5),scaleRectangle(rects[j][:],3,1.5)))
                filters.append(cv2.matchShapes(contour_from_rect(rects[i]),contour_from_rect(rects[j]),3,0.0)<.6)
                area_rat=rects[i][2]*rects[i][3]/rects[j][2]/rects[j][3]
                filters.append(area_rat>(1/size_sim_k) and area_rat<size_sim_k)
                ave_h=(rects[i][3]+rects[j][3]<2)
                dh=abs(rects[i][1]-rects[j][1])
                #filters.append(rects[i][1]==rects[j][1] or (dh<5*ave_h))
                if not i==j:
                    failed=False
                    for filter_ in filters:
                        if not filter_:
                            failed=True
                            break
                    if not failed:
                        pairs.append([i,j])
        clusters=list()
        for i in range(num_contours_check):
            cluster=list()
            cluster_by_pairs(i,cluster,pairs)
            if len(cluster)>2:
                clusters.append(cluster)
        #sets the size of each rectangle to the smallest rectangle containing all contours in it's cluster
        cluster_rects=list()
        for cluster in clusters:
            sub_rects=list()
            for rect in cluster:
                sub_rects.append(rects[rect])
            cluster_rects.append(sub_rects)
        enclosing_cluster_rects=list()
        #print(cluster_rects)
        for i,rects in enumerate(cluster_rects):
            enclosing_cluster_rects.append(min_enclosing_rectangle(rects))
        largest_area=0
        largest_contour_id=-1
        num_in_largest_cluster=1
        for i,rect in enumerate(enclosing_cluster_rects):
            if rect[2]>0 and rect[3]>0 and rect[2]*rect[3]>largest_area:
                largest_area=rect[2]*rect[3]
                largest_contour_id=i
        if largest_contour_id>=0:
            drawRectangle(image,enclosing_cluster_rects[largest_contour_id],(255,0,0))
            #cluster rects is the bounding rectangle of each identified cluster
            #cv2.rectangle(image,(x_val,y1),(x_val+10,y1+10),(255,255,255),2)
            x=enclosing_cluster_rects[largest_contour_id][0]
            y=enclosing_cluster_rects[largest_contour_id][1]
            width=enclosing_cluster_rects[largest_contour_id][2]
            height=enclosing_cluster_rects[largest_contour_id][3]
            #print(enclosing_cluster_rects)
        llpython = [1,x,y,width,height,9,8,7] 
    # incrementTestVar()
    # drawDecorations(image)
    
    # make sure to return a contour,
    # an image to stream,
    # and optionally an array of up to 8 values for the "llpython"
    # networktables array
    #global first
    #if first:
        #print(contour_from_rect(enclosing_cluster_rects[largest_contour_id]))
        #first=False
    #print(x)
    return contour_from_rect([x,y,width,height]), image, llpython