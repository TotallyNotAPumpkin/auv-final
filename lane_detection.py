import cv2
import numpy as np
import matplotlib.pyplot as plt

def detect_lines(img, threshold1 = 50, threshold2 = 150, apertureSize = 3, minLineLength = 100, maxLineGap = 10):
    """Detects if lines are present in an image (pool).
    args:
        img (image path or np.ndarray): image of pool to detect lines from
        threshold1 (int): lower threshold to detect lines (default 50)
        threshold2 (int): upper threshold to detect lines (default 150)
        apertureSize (odd int): amount of details cv2 will be taking (default 3)
        minLineLength (int): the minimum line length of lines detected (default 100)
        maxLineGap (int): the maximum line gap of lines detected (default 10)
    returns:
        (list): list of points [[x1, y2, x2, y2], ...] that correspond to detected lines
    """
    if not isinstance(img, np.ndarray):
        img = cv2.imread(img)
    grayimg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # convert to grayscale
    # grayCon = cv2.addWeighted(gray, 2, gray, 0, 0)
    imgblur = cv2.blur(grayimg, [10, 10])
    (thresh, im_bw) = cv2.threshold(imgblur, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    edges = cv2.Canny(im_bw, threshold1, threshold2, apertureSize=apertureSize) # detect edges
    lines = cv2.HoughLinesP(
                    edges,
                    1,
                    np.pi/180,
                    100,
                    minLineLength=minLineLength,
                    maxLineGap=maxLineGap,
            ) # detect lines

    lineList = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            linexy = [x1, y1, x2, y2]
            cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            lineList.append(linexy)
    return lineList

def draw_lines(img, lines, color = (0, 255, 0)):
    """Returns an image with specified lines drawn on
    args:
        img (image path or np.ndarray): image that lines are drawn on
        lines (list): list of points [[x1, y2, x2, y2], ...] of the lines being drawn
        color (tuple): (x, x, x)
    returns:
        (np.ndarray): image with lines drawn on
    """
    if not isinstance(img, np.ndarray):
        img = cv2.imread(img)
    for line in lines:
        cv2.line(img, (line[0], line[1]), (line[2], line[3]), color, 3) 
    return img

def get_slopes_intercepts(img, lines):
    """Returns slopes and bottom x-intercepts of given lines in an image
    args: 
        img (image path or np.ndarray): image that lines reside in
        lines (list): list of points [[x1, y2, x2, y2], ...] of the lines
    returns:
        (list, list): 2 lists of the slopes and intercepts of the specified lines
        """
    if not isinstance(img, np.ndarray):
        img = cv2.imread(img)
    slopes = []
    intercepts = []
    height = img.shape[0]
    for line in lines:
        if line[0] == line[2]:
            line[0] += 0.000000001
        slope = (line[1] - line[3]) / (line[0] - line[2])
        slopes.append(slope)
        if slope == 0:
            slope = 0.0000000001
        intercepts.append((height-line[1])/slope + line[0])
    return slopes, intercepts

def detect_lanes(imageInput, lines):
    """Detects lanes from given lines and an image.
    args:
        imageInput (image path or np.ndarray): image that lines originate from
        lines (list): list of points [[x1, y2, x2, y2], ...] of the lines
    return:
        (list): list of possible lanes, each lane containing 2 lines with points - [[[x, x, x, x], [x, x, x, x]], ...]
    """
    if not isinstance(imageInput, np.ndarray):
        imageInput = cv2.imread(imageInput)
    img = cv2.cvtColor(imageInput, cv2.COLOR_BGR2GRAY)
    slopes, intercepts = get_slopes_intercepts(img, lines)
    lineDict = dict(sorted(zip(intercepts, slopes)))
    height = img.shape[0]
    possibleLanes = []

    # finding lines with similar slopes and intercepts
    for i in range(1, len(lineDict)):
        # checks if slopes and intercepts are similar
        intercept1, intercept2 = list(lineDict)[i-1], list(lineDict)[i]
        slope1, slope2 = list(lineDict.values())[i-1], list(lineDict.values())[i]
        if abs(intercept1 - intercept2) < 500 and abs(intercept1 - intercept2) > 50 and abs(slope1 - slope2) < 6 and abs(slope1 - slope2) > 0.03:
            # # find the fuckin in between line darkness
            # centerM2 = int((intercept1 + intercept2) / 2 - 2)
            # averageDark = 0
            # for pixel in range(centerM2, (centerM2 + 5)):
            #     colorValue = img[(height-2)][pixel]
            #     averageDark += colorValue
            # if (averageDark/5) <= 75:
            # finds indices of lines list that correspond to lanes
            index1, index2 = intercepts.index(list(lineDict)[i-1]), intercepts.index(list(lineDict)[i])
            line1, line2 = lines[index1], lines[index2]
            possibleLanes.append([line1, line2])
        elif abs(intercept1 - intercept2) < 500 and abs(intercept1 - intercept2) > 50 and abs(-slope1 - slope2) < 6:
            index1, index2 = intercepts.index(list(lineDict)[i-1]), intercepts.index(list(lineDict)[i])
            line1, line2 = lines[index1], lines[index2]
            possibleLanes.append([line1, line2])
    return possibleLanes


def draw_lanes(img, lanes):
    """draws specified lanes on a given image.
    args:
        img (image path or np.ndarray): image that lanes are drawn on
        lanes (list): list of possible lanes, each lane containing 2 lines with points - [[[x, x, x, x], [x, x, x, x]], ...]
    return:
        (np.ndarray): image with specified lanes drawn on
        """
    if not isinstance(img, np.ndarray):
        image = cv2.imread(img)
    else:
        image = img
    if lanes != []:
        for lane in lanes:
            for line in lane:
                cv2.line(image, (line[0], line[1]), (line[2], line[3]), (0, 255, 255), 4)
    return image

# cv2.line(image, start_point, end_point, color, thickness) 
if __name__ == "__main__":
    image = cv2.imread('lanes.png')
    img = cv2.resize(image, (1912, 1069))
    print(img.shape)
    lines = detect_lines(img, 30, 100, 3, 229, 13)
    lanes = detect_lanes(img, lines)
    print(f"Possible lines: {lines}")
    print(f"Possible lanes: {lanes}")