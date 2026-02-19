#!/usr/bin/python3
# -*- coding: utf-8 -*- 

# https://russianblogs.com/article/1808360908/ 
# Используйте визуальные методы для измерения координат цели в мировой системе координат
# Сначала оцените положение камеры, а затем вычислите положение центральной точки маркера цели в мировой системе координат.
# Инструкции: 
# 1. Калибровка камеры,
# 2. Поместите в пространство более 4 опорных координат и укажите информацию об этих точках в программе, включая ID и мировые координаты
# 3. Цель для тестирования помечается маркером, а маркерID этих точек указывается в программе
# 4. Снимите видео и убедитесь, что 4 маркера находятся в поле зрения.
# 5. Запустите программу для обработки видеокадров
# CR@ Guofeng, mailto:gf@gfshen.cn
# 
# ------ История версий ---
# ---V1.0
# № --- 19 июля 2019 г.
# # Первоначальная запись
 
 
 
import numpy as np
import cv2
import cv2.aruco as aruco

def estimateCameraPose(cameraMtx, dist, refMarkerArray,corners,markerIDs):
    '''
         В соответствии с маркером контрольной точки, решить вектор вращения камеры rvecs и вектор перевода tvecs, (solvePnP () реализация)
         И преобразовать rvecs в вывод матрицы вращения (через Rodrigues ())
         Входные данные:
                 матрица внутренних параметров cameraMtx,
                 коэффициент искажения.
                 Текущий обработанный кадр изображения,
                 Контрольная точка refMarkerArray для позиционирования мировой системы координат. Тип словаря Py, требуется len (refMarkerArray)> = 3, формат: {ID: [X, Y, Z], ID: [X, Y, Z] ..}
                 угловые, вывод функции deteMarkers ()
                 markerIDs, вывод функции deteMarkers ()
         Вывод: матрица вращения rMatrix, вектор перевода tVecs
    '''
    marker_count = len(refMarkerArray)
    if marker_count <4: # не более четырех маркерных досок
        raise RuntimeError('at least 3 pair of points required when invoking solvePnP')
   
 
    corners=corners; ids=markerIDs
    print('ids:\n')
    print(ids)
    print('corners:\n')
    print(corners)
 
    objectPoints=[]
    imagePoints=[]
         # Проверьте, обнаружены ли все ожидаемые маркеры
         если len (ids)! = 0: обнаружен маркер #, сохраните мировые координаты маркера в objectPoints и создайте соответствующий список координат плоскости изображения imagePoints
        print('------detected ref markers----')
                 для диапазона i (len (ids)): # перемещение идентификатора обнаруженного маркера,
                         если ids [i] [0] в refMarkerArray: # Если это метка контрольной точки, извлеките координаты изображения контрольной точки, использованные для построения входных данных solvePnP ()
 
                print('id:\n ' + str(ids[i][0]))
                print('cornors: \n '+ str(corners[i][0]))
                objectPoints.append(refMarkerArray[ ids[i][0] ])
                                 imagePoints.append (corners [i] [0] [0] .tolist ()) # Извлечь верхнюю левую точку маркера
        objectPoints=np.array(objectPoints)
        imagePoints=np.array(imagePoints)
            
        print('------------------------------\n')
        print('objectPoints:\n'+str(objectPoints))
        print('imagePoints:\n'+str(imagePoints))
        pass
    else:
        return False, None, None
 
         # Если обнаруженные контрольные точки больше 3, вы можете решить положение камеры
    if len(objectPoints)>=4:
                 # Требуется минимум 4 балла
        retval, rvec, tvec = cv2.solvePnP(objectPoints, imagePoints, cameraMtx, dist)
        rMatrix,jacobian = cv2.Rodrigues(rvec)
        return True, rMatrix, tvec
    else:
        return False, None, None
 
 
         # возврат стоимости
    #return rMatrix=[], tVecs=[]
 
 
 
def detectTarget(cameraMatrix, dist, rMatrix, tvec, targetMarker, corners, markerIDs,zWorld = 0.0):
    '''
         Измерьте положение центра маркера цели в мировой системе координат
         Войти:
         Вывод:
                 Список, равный длине идентификатора маркера, включая целевые координаты, определенные положением, и None не обнаружен. Например, [None, [x2, y2, z2]]
    '''
    if rMatrix==[]:
        return
    targets_count=len(targetMarker)
    if targets_count == 0:
        raise Exception('targets empty, areyou dou?')
 
         # Создайте список того же размера, что и targetMarker, чтобы сохранить мировые координаты цели, полученные решением
    targetsWorldPoint=[None] * targets_count
 
         для i в диапазоне (len (markerIDs)): # Обход обнаруженного идентификатора маркера,
        markerIDThisIterate = markerIDs[i][0]
                 если markerIDThisIterate в targetMarker: # Если это идентификатор целевого маркера
                         # Получить индекс текущего обработанного маркера в targetMarker, используемого для заполнения targetWorldPoint
            targetIndex = targetMarker.index(markerIDThisIterate)
        else:
            continue
                 # Рассчитать координаты изображения центра маркера
        markerCenter = corners[i][0].sum(0)/4.0
                 # Коррекция искажения, преобразование в систему координат камеры, получение (u, v, 1)
        #https://stackoverflow.com/questions/39394785/opencv-get-3d-coordinates-from-2d
        markerCenterIdeal=cv2.undistortPoints(markerCenter.reshape([1,-1,2]),cameraMatrix,dist)
        markerCameraCoodinate=np.append(markerCenterIdeal[0][0],[1])
        print('++++++++markerCameraCoodinate')
        print(markerCameraCoodinate)
 
                 # Маркерные координаты преобразуются из камеры в мировые координаты
        markerWorldCoodinate = np.linalg.inv(rMatrix).dot((markerCameraCoodinate-tvec.reshape(3)) )
        print('++++++++markerworldCoodinate')
        print(markerWorldCoodinate)
                 # Преобразовать начало координат камеры в мировую систему координат
        originWorldCoodinate = np.linalg.inv(rMatrix).dot((np.array([0, 0, 0.0])-tvec.reshape(3)) )
                 # Две точки определяют прямую линию (x-x0) / (x0-x1) = (y-y0) / (y0-y1) = (z-z0) / (z0-z1) 
                 # Когда z = 0, вычислите x, y
        delta = originWorldCoodinate-markerWorldCoodinate
        #zWorld = 0.0
        xWorld = (zWorld-originWorldCoodinate[2])/delta[2] * delta[0] + originWorldCoodinate[0]
        yWorld = (zWorld-originWorldCoodinate[2])/delta[2] * delta[1] + originWorldCoodinate[1]
        targetsWorldPoint[targetIndex]=[xWorld,yWorld,zWorld]
        
        print('-=-=-=\n Target Position '+ str(targetsWorldPoint[targetIndex]) )
        pass
    return targetsWorldPoint
 
 
 
 
 
if __name__ == '__main__':
    frame = cv2.imread('./inputImage2.bmp')
    try:
        npzfile = np.load('./calibrateDataMi5.npz')
        mtx = npzfile['mtx']
        dist = npzfile['dist']
    except IOError:
        raise Exception('cant find calibration data, do that first')
    
         # Сохраните информацию о контрольной точке, она будет обновлена ​​после обнаружения.
    rMatrix=[]
    tvec=[]
    #######
 
         # Процесс видео 
    cv2.namedWindow('image',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('image', 1280,720)
    cv2.imshow("image",frame)
 
    ##process and measure target position
         # 0.1 Укажите идентификатор маркера и мировые координаты контрольной точки
    # [[marker ID, X, Y, Z]..]
    refMarkerArray={   \
        0: [4.0, 6.0, 0.0], \
        1: [4.0, 2.0, 0.0], \
        2: [2.0, 2.0, 0.0], \
        3: [2.0, 6.0, 0.0], \
    }
         # 0.2 Укажите целевой идентификатор маркера
    targetMarker =[10,11]
 
         # 1. Оцените позу камеры 
    #1.1 detect aruco markers
    img_gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()
   
    
    corners, ids, rejectedImgPoints = aruco.detectMarkers(img_gray, aruco_dict, parameters=parameters)
    aruco.drawDetectedMarkers(img_gray, corners) #Draw A square around the markers
    cv2.namedWindow('detect',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('detect', 1280,720)
    cv2.imshow("detect",img_gray)
    #1.2 estimate camera pose
    gotCameraPose, rMatrixTemp, tvecTemp = estimateCameraPose(mtx, dist, refMarkerArray,corners,ids)
 
    #1.3 updata R, T to static value 
    if gotCameraPose: 
        rMatrix = rMatrixTemp
        tvec = tvecTemp
        print('rMatrix\n'+str(rMatrixTemp))
        print('tvec\n'+str(tvecTemp))
 
         # 2. Рассчитать координаты мировой системы координат по целевому маркеру
    detectTarget(mtx, dist, rMatrix, tvec, targetMarker, corners, ids)
 
    
    '''
    if ( cv2.waitKey(10) & 0xFF ) == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
    '''
    
    #cap.release()
    cv2.destroyAllWindows()
