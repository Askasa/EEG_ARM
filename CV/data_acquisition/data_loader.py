import cv2 as cv
import os

camera = cv.VideoCapture(0)
if not camera.isOpened():
    print("The camera is not Opening .... Exiting")
    exit()

Labels = ["A","B","C"]
test_path = 'C:/github_project/demo/demo-repo/CV/data/test/'
for label in Labels:
    test_path = 'C:/github_project/demo/demo-repo/CV/data/test/'
    if not os.path.exists(test_path+label):
        os.mkdir(test_path+label)


for folder in Labels:
    count = 0
    print("Press 'p' to start the data collection for ",folder)

    userinput = input()
    if(userinput != 'p'):
        print("Wrong Imput ....")
        exit()

    while count<2:
        status,frame = camera.read()
        if not status:
            print("Frame is not been captured .. Exiting ..")
            break
        gray = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
        cv.imshow("Vedio Windo",gray)
        gray = cv.resize(gray,(192,112))

        cv.imwrite(test_path+folder+'/img'+str(count)+'.png',gray)
        count = count + 1
        if(cv.waitKey(1) == ord('q')):
            break
camera.release()
cv.destroyAllWindows()