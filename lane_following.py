import cv2
import numpy as np
import matplotlib.pyplot as plt
import lane_detection
from apriltags import drawTag, getTag, getTagCenter


def get_lane_center(img, lanes):
    """Find the center of the lane closest to the middle of a list of given lanes
    args: 
        img (image path or np.ndarray): img that lanes come from
        lanes (list): list of lanes [[[x, x, x, x], [x, x, x, x]], ...]
    return: 
        (list): [centerSlope, centerIntercept(x), centerIntercept(y)]
        """
    
    if not isinstance(img, np.ndarray):
        img = cv2.imread(img)
    # find slopes and intercepts of all lines in the lanes
    cenSlopes = []
    cenInters = []
    cenIntersy = []
    if lanes is not None:
        for lane in lanes:
            slope, intercept = lane_detection.get_slopes_intercepts(img, lane)
            cenSl = 1/((1/slope[0] + 1/slope[1])/2) # center slope float
            cenInt = (intercept[0] + intercept[1])/2 # center int float
            cenInty = ((lane[0][1]-lane[0][0]*slope[0])+(lane[1][1]-lane[1][0]*slope[1]))/2
            cenSlopes.append(cenSl) # list of center slopes of all lanes
            cenInters.append(cenInt) # list of center slopes of all intercepts
            cenIntersy.append(cenInty)

            cenInters.sort()
            cenInter = cenInters[0]
            for num in cenInters:
                if abs(num - img.shape[1]/2) < abs(cenInter - img.shape[1]/2):
                    cenInter = num
                if num > img.shape[1]/2:
                    break

            # cenInter = sorted(cenInters, key=lambda x: abs(img[1]/2 - x))[0] # center slope that is closest to the center (but I don't think this works) 
        index = cenInters.index(cenInter)
        cv2.putText(img, f'number of lanes: {len(lanes)}', (20, 390), 0, 0.3, (255, 0, 255), 3)
        slInters = [cenSlopes[index], cenInters[index], cenIntersy[index]]
    return slInters


# for just lanes
def videoDetection(vid):
    output_video = cv2.VideoWriter('output_video.avi', cv2.VideoWriter_fourcc(*'XVID'), 30, (1912, 535))
    # video.release() #Save video to disk.
    # total_frames = []
    # Capture frame-by-frame

    count = 1
    ret, frame = vid.read()
    while count <= 500:
        ret, frame = vid.read()
        if ret:
            resized = (cv2.resize(frame, (1912, 1069)))
            h = resized.shape[0]
            w = resized.shape[1]
            againResized = resized[int(h/2) : h, 0 : w]
        
            lines = lane_detection.detect_lines(againResized, 50, 70, 3, 200, 10)
            if lines is not None:
                lanes = lane_detection.detect_lanes(againResized, lines)   
            else:
                lanes = []
            if len(lanes) != 0:
                slInt = get_lane_center(againResized, lanes)
                draw_lane_center(againResized, slInt) 

            againResized = lane_detection.draw_lanes(againResized, lanes)
            # total_frames.append(frame)
            output_video.write(againResized)

        print(ret)
        print(f"Frame: {count}")
        count += 1
    output_video.release()



# video detection for both tags and lines
def videoDetectionTL(vid):
    w = int(vid.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(vid.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(vid.get(cv2.CAP_PROP_FPS))

    output_video = cv2.VideoWriter('output_video.avi', cv2.VideoWriter_fourcc(*'XVID'), fps, (w, h))
    # video.release() #Save video to disk.
    # total_frames = []
    # Capture frame-by-frame


    centerY = int(h/2)
    centerX = int(w/2)

    count = 1
    ret, frame = vid.read()
    while True:
        if ret:

            tags = getTag(frame)
            detected_tags = getTagCenter(tags)
            
            if len(detected_tags) != 0:
                frame = drawTag(frame, h, w, centerY, centerX)
                output_video.write(frame)
                ret, frame = vid.read()

            else:
                lines = lane_detection.detect_lines(frame, 50, 70, 3, 200, 10)
                if lines is not None:
                    lanes = lane_detection.detect_lanes(frame, lines)   
                else:
                    lanes = []
                if len(lanes) != 0:
                    slInt = get_lane_center(frame, lanes)
                    draw_lane_center(frame, slInt) 
                output_video.write(frame)
                ret, frame = vid.read()
        else:
            ret, frame = vid.read()
            if ret:

                tags = getTag(frame)
                detected_tags = getTagCenter(tags)
                
                if len(detected_tags) != 0:
                    frame = drawTag(frame, h, w, centerY, centerX)
                    output_video.write(frame)
                    ret, frame = vid.read()

                else:
                    lines = lane_detection.detect_lines(frame, 50, 70, 3, 200, 10)
                    if lines is not None:
                        lanes = lane_detection.detect_lanes(frame, lines)   
                    else:
                        lanes = []
                    if len(lanes) != 0:
                        slInt = get_lane_center(frame, lanes)
                        draw_lane_center(frame, slInt) 
                output_video.write(frame)
                ret, frame = vid.read()
            else: break
                

            frame = lane_detection.draw_lanes(frame, lanes)
            # total_frames.append(frame)


        print(ret)
        print(f"Frame: {count}")
        count += 1
    output_video.release()




def recommend_angle(slope):
    laneAngle = np.arctan(slope)
    return laneAngle
    

def videoDetectionFrames(vid, framesVid):
    output_video = cv2.VideoWriter('output_video.avi', cv2.VideoWriter_fourcc(*'XVID'), 30, (1912, 535))
    # video.release() #Save video to disk.
    # total_frames = []
    # Capture frame-by-frame

    count = 1
    while True:
        ret, frame = vid.read()
        if ret:
            resized = (cv2.resize(frame, (1912, 1069)))
            h = resized.shape[0]
            w = resized.shape[1]
            againResized = resized[int(h/2) : h, 0 : w]
        
            lines = lane_detection.detect_lines(againResized, 50, 50, 3, 100, 10)
            if lines is not None:
                lanes = lane_detection.detect_lanes(againResized, lines)   
            else:
                lanes = []
            if len(lanes) != 0:
                slInt = get_lane_center(againResized, lanes)
                
                draw_lane_center(againResized, slInt) 

            againResized = lane_detection.draw_lanes(againResized, lanes)
            # total_frames.append(frame)
            output_video.write(againResized)

        print(ret)
        print(f"Frame: {count}")
        count += 1
        if count > framesVid: break
    output_video.release()



def draw_lane_center(img, slInt):
    if not isinstance(img, np.ndarray):
        image = cv2.imread(img)
    else:
        image = img
    
    height = img.shape[0]

    if slInt is not None:
        slope = slInt[0]
        intercept = slInt[1]
        angle = np.degrees(recommend_angle(slope))
        if slope == 0:
            slope = 0.0000000001
            cv2.line(image, (0, int(slInt[2])), (img.shape[1], int(slInt[2])), (180, 0, 255), 1)
            cv2.putText(image, f'Angle: {round(angle, 3)}', (20, 300), 0, 0.3, (255, 0, 255), 3)
            cv2.putText(image, f'Center X-intercept: {round(intercept, 3)}', (20, 330), 0, 0.3, (255, 0, 255), 3)
            cv2.putText(image, f'Center Y-intercept: {round(slInt[2], 3)}', (20, 360), 0, 0.3, (255, 0, 255), 3)
        else: 
            x2 = int((0 - (height - slope * intercept)) / slope)
            cv2.line(image, (int(intercept), height), (x2, 0), (180, 0, 255), 1)
            cv2.putText(image, f'Angle: {round(angle, 3)}', (20, 300), 0, 1, (255, 0, 255), 3)
            cv2.putText(image, f'Center X-intercept: {round(intercept, 3)}', (20, 330), 0, 0.3, (255, 0, 255), 3)
            cv2.putText(image, f'Center Y-intercept: {round(slInt[2], 3)}', (20, 360), 0, 0.3, (255, 0, 255), 3)

    return image

# def recommend_direction(img, center, slope): #not used
#     """Recommends the direction that the AUV should move in order to follow a lane
#     args:
#         center (int or float): center point of intercepts
#         slope (int or float): average slope
#     return:
#         (list): list of suggested strafe and turn direction"""
    
#     # width = img.shape[1] / 2
#     # if center >= (width - 50) and center <= (width + 50):
#     #     direction =  "forward"
#     # elif center > 1220:
#     #     direction = "right"
#     # else:
#     #     direction = "left"

#     # if slope > 2.5:
#     #     turn = "left"
#     # if slope < 2.5:
#     #     turn = "right"
#     # else: 
#     #     turn = "don't turn"
#     # return [direction, turn]
#     width = img.shape[1] / 2
#     if center >= (width - 20) and center <= (width + 20) and abs(1/slope) <= 0.1:
#         direction =  "drive forward"
#         return[direction]
#     elif center > (width + 20):
#         direction = "strafe right"
#         return[direction]
#     elif center < (width - 20):
#         direction = "strafe left"
#         return[direction]
#     if center >= (width - 20) and center <= (width + 20) and slope <= -0.5:
#         direction = "turn right"
#     if center >= (width - 20) and center <= (width + 20) and slope >= 0.5: 
#         direction = "turn left"
#     return[direction]


if __name__ == "__main__":
    image = cv2.imread('lanes.png')
    img = cv2.resize(image, (1912, 1069))
    lines = lane_detection.detect_lines(img, 30, 100, 3, 229, 13)
    lanes = lane_detection.detect_lanes(img, lines)
    center = get_lane_center(img, lanes)
    # action = recommend_direction(img, center[0], center[1])
    print(f"Possible lines: {lines}") # [[1415, 531, 1676, 563], [514, 1047, 699, 738], [1441, 573, 1674, 618], [712, 1068, 765, 839], [557, 973, 706, 725]]
    print(f"Possible lanes: {lanes}") # [[[514, 1047, 699, 738], [712, 1068, 765, 839]]]
    print(f"Center slope and intercept: {center}") # [-2.9955124936257014, 606.2985189581832]
    # print(f"Recommended action: {action}")