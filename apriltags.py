from dt_apriltags import Detector
import cv2
import numpy as np
import matplotlib.pyplot as plt
from pymavlink import mavutil
import sys
import signal
from pid import PID
import os
from time import sleep
from scipy.spatial.transform import Rotation as R


cameraMatrix = np.array([ 353.571428571, 0, 320, 0, 353.571428571, 180, 0, 0, 1]).reshape((3,3))
camera_params = (cameraMatrix[0, 0], cameraMatrix[1, 1], cameraMatrix[0, 2], cameraMatrix[1, 2])
at_detector = Detector(families='tag36h11',
                        nthreads=1,
                        quad_decimate=1.0,
                        quad_sigma=0.0,
                        refine_edges=1,
                        decode_sharpening=0.25,
                        debug=0)



def getTag(frame):
    # video.set(cv2.CAP_PROP_POS_FRAMES, frameNumber)

    img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = at_detector.detect(img, True, camera_params, tag_size=0.1)

        # for idx in range(len(tag.corners)):
        #     cv2.line(frame, tuple(tag.corners[idx - 1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))
        #     cv2.circle(frame, (x, y), 50, (0, 0, 255), 2)
    return tags

def getTagCenter(tags):
    detected_tags = []
    for tag in tags:
        (cX, cY) = (int(tag.center[0]), int(tag.center[1]))
        detected_tags.append([cX, cY])
    return detected_tags

def drawTag(frame, height, width, centerY, centerX):
    tags = getTag(frame)
    print("gottags")

    for tag in tags:
        (ptA, ptB, ptC, ptD) = tag.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))
        print("get square")

        # draw the bounding box of the AprilTag detection
        cv2.line(frame, ptA, ptB, (0, 255, 0), interfaceLine)
        cv2.line(frame, ptB, ptC, (0, 255, 0), interfaceLine)
        cv2.line(frame, ptC, ptD, (0, 255, 0), interfaceLine)
        cv2.line(frame, ptD, ptA, (0, 255, 0), interfaceLine)
        print("draw square")

        # draw circle in center of tag
        (cX, cY) = (int(tag.center[0]), int(tag.center[1]))
        cv2.circle(frame, (cX, cY), dotRadius, (0, 0, 255), -1)
        print("drawpoing")

        # draws line between center and circle
        cv2.line(frame, (cX, cY), (centerX, centerY), (255, 0, 0), interfaceLine)
        cv2.putText(frame, f"({centerX}, {centerY + 20})", (centerX, centerY), 0, fontSize, (255, 0, 0), 3)
        cv2.putText(frame, f"({cX}, {cY})", (cX, cY + 20), 0, fontSize, (255, 0, 0), 3)
        print("drawfunnyline")

        # draws line between center and circle
        cv2.line(frame, (cX, cY), (cX, centerY), (255, 100, 0), interfaceLine) # center of the detected tag, then extend a line down for the vertical component
        cv2.line(frame, (cX, cY), (centerX, cY), (255, 100, 0), interfaceLine) #center of the detected tag, then extend a line horizontal for the horizontal component
        print("moredrawfunnies")

        
        errorPercentY = (centerY - cY)/height * 100
        errorPercentX = (centerX - cX)/width * 100
        vertLabel = (0, int(centerY/2))
        horizLabel = (0, int(centerY/2-30))
        print("makevar")
        # angles = getTagAngles(tags)

        cv2.putText(frame, f'Vertical Distance Percentage: {round(errorPercentY, 3)}%', vertLabel, 0, fontSize, (255, 0, 255), 3)
        cv2.putText(frame, f'Lateral Distance Percentage: {round(errorPercentX, 3)}%', horizLabel, 0, fontSize, (255, 0, 255), 3)
        print("text...")
        # cv2.putText(frame, f'Angle (deg): {angles}', (0, int(centerY/2-60)), 0, fontSize, (255, 0, 255), 3)
    return frame

def getTagDistance(tags):
    distance = tags[0].pose_t

    return distance


def tagVideo(vid):
    global interfaceLine, fontSize, dotRadius
    interfaceLine = int(input("Enter line thickness (px): "))
    fontSize = float(input("Enter font scale (float): "))
    dotRadius = int(input("Enter radius of dot: "))

    w = int(vid.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(vid.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(vid.get(cv2.CAP_PROP_FPS))

    centerY = int(h/2)
    centerX = int(w/2)

    print(fps)
    output_video = cv2.VideoWriter('output_video.avi', cv2.VideoWriter_fourcc(*'XVID'), 16, (w, h))
    # video.release() #Save video to disk.
    # total_frames = []
    # Capture frame-by-frame

    count = 1
    ret, frame = vid.read()

    # change comments for set number of frames
    while True:
    # while True:
        if ret:
            output_video.write(drawTag(frame, h, w, centerY, centerX))
            print(ret)
            print(f"Frame: {count}")
            count += 1
            ret, frame = vid.read()
            print("afterRet1")
        else:
            ret, frame = vid.read()
            print("chceckingoforhflnet")
            if ret:
                output_video.write(drawTag(frame, h, w, centerY, centerX))
                print(ret)
                print(f"Frame: {count}")
                count += 1
                ret, frame = vid.read()
                print("afterRet2")
            else: break

        # change comments for set number of frames
        # if count > 200:
        #     break

    output_video.release()



def writeImages(vid, start = 1, images = 10):
    dirname = 'vidFrames'
    if not os.path.exists(dirname):
        os.mkdir(dirname)
    
    if (start + images) > vid.get(cv2.CAP_PROP_FRAME_COUNT):
        raise Exception("Number of frames must not exceed number of frames in video.")

    count = 1
    ret, frame = vid.read()

    # change comments for set number of frames
    while True:
        if ret:
            print(ret)
            print(f"Frame: {count}")
            if count >= start and count < (start + images):
                cv2.imwrite(os.path.join(dirname, str(count)+".jpg"), frame)
            count += 1
            ret, frame = vid.read()
        else:
            ret, frame = vid.read()   
            if ret:
                if count >= start and count < (start + images):
                    cv2.imwrite(os.path.join(dirname, str(count)+".jpg"), frame)
                print(ret)
                print(f"Frame: {count}")
                count += 1
                ret, frame = vid.read()
            else: break
        if count > (start + images): break

def createFeed(vid):
    count = 1
    ret, frame = vid.read()

    # change comments for set number of frames
    while True:
        if ret:
            print(ret)
            print(f"Frame: {count}")
            cv2.imwrite('feed.jpg', frame)
            count += 1
            ret, frame = vid.read()
        else:
            ret, frame = vid.read()   
            if ret:
                cv2.imwrite('feed.jpg', frame)
                print(ret)
                print(f"Frame: {count}")
                count += 1
                ret, frame = vid.read()
            else: break
        sleep(0.2)




def getTagAngles(tags):
    angles = []
    for tag in tags:
        r = R.from_matrix(tag.pose_R)
        r = r.as_euler('zyx', degrees=True)
        angles.append(r[1])
        print(f"Angles: {r[1]}")
    return angles


    

def sizeV(video):
    width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))

    middle_x = width/2
    middle_y = height/2

    return middle_x, middle_y, width, height

def sizeF(frame):
    width = frame.shape[1]
    height = frame.shape[0]

    middle_x = int(width/2)
    middle_y = int(height/2)

    return middle_x, middle_y, width, height

    

def main(video):
    #mav = mavutil.mavlink_connection("udpin:0.0.0.0:14550")

    x = 0

    pid_vertical = PID(K_p=1, K_i=0.0, K_d=-0.7, integral_limit=1)
    pid_horizontal = PID(K_p=2, K_i=0.0, K_d=-0.7, integral_limit=1)

    try:
        video = cv2.VideoCapture(video)
        middle_x, middle_y, width, height = sizeV(video)

        while True:
            # Read the current coordinates from the AprilTag detector
            ret, frame = video.read()
            detected_tags = []
            if ret:
                tags = getTag(frame)
                detected_tags = getTagCenter(tags)
            else:  
                continue
            # if not detected_tags:
            #     print("No tags found in frame", x)
            #     break

            # For simplicity, assume only one tag is detected in each frame
            print (f"Frame: {x}")

            if len(detected_tags) != 0:
                tagX, tagY = detected_tags[0]
                print(detected_tags)

                distance = getTagDistance(tags)
                print(distance)

                getTagAngles(tags)

                # Calculate percent error from the desired middle coordinates
                error_y = round((middle_y - tagY)/height * 100, 6)
                error_x = round((middle_x - tagX)/width * 100, 6)

                print(f"Error X: {error_x}%")
                print(f"Error Y: {error_y}%")

                # TODO: set vertical_power and lateral_power here
                # Update the PID controllers and get the output
                vertical_power = int(pid_vertical.update(error_y))
                lateral_power = int(pid_horizontal.update(error_x))

                print("Output X:", lateral_power)
                print("Output Y:", vertical_power)

            else:
                vertical_power = 0
                lateral_power = 0
                print("Error Y: No tag detected.")
                print("Error X: No tag detected.")

            x += 1
            
    except KeyboardInterrupt:
        print("Interrupted by user.")
        # finally:
        # Stop the vehicle's movement when the program ends
        # set_vertical_power(mav, 0)
        # set_rc_channel_pwm(mav, 6, pwm=1500)


if __name__ == "__main__":
    # vida = cv2.VideoCapture('AprilTagTest.mkv')
    vida = 'AprilTagTest.mkv'
    # tagVideo(vida)
    # writeImages(vida, 3773, 4)
    # createFeed(vida)
    main(vida)
    