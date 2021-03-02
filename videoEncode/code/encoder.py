import os
import shutil
import numpy as np
np.set_printoptions(suppress=True)

from adaptation.optimalController import *
from supervision.mandatoryController import *
from supervision.deviationDetector import *
from supervision.switcher import *

def generateJPG2MP4(videoFile):
    # delete old folder original
    if os.path.exists("../original/" + str(os.path.basename(videoFile).split(".")[0])):
        try:
            shutil.rmtree("../original/" + str(os.path.basename(videoFile).split(".")[0]))
        except OSError as e:
            print(e)
        else:
            print("folder original is deleted successfully")

    # make orignal folder and divide video into frames
    os.makedirs("../original/" + str(os.path.basename(videoFile).split(".")[0]))

    # generate original video stream
    os.system("mplayer -vo jpeg:quality=100:outdir=../original/" + str(os.path.basename(videoFile).split(".")[0]) + " " + str(videoFile))

def convert(path_in, path_out, quality):
    command = "magick convert -quality " + str(quality) + " "
    command += path_in + ' ' + path_out
    os.system(command)

def encode(videoFile):

    # delete old folder compressed and make new folder compressed/videoFile
    if os.path.exists("../compressed"):
        try:
            shutil.rmtree("../compressed")
        except OSError as e:
            print(e)
        else:
            print("folder compressed is deleted successfully\n")
    os.makedirs("../compressed/" + str(os.path.basename(videoFile).split(".")[0]))

    # define input and output directory
    inputDir = "../original/" + str(os.path.basename(videoFile).split(".")[0])
    outputDir = "../compressed/" + str(os.path.basename(videoFile).split(".")[0])
    print("inputDir: {}, outputDir: {}\n".format(inputDir, outputDir))

    startFrame = 101
    endFrame = len(os.listdir(inputDir))
    print("startFrame: {}, endFrame: {}\n".format(startFrame, endFrame))

    if not os.path.exists("result/"):
        os.makedirs("result/")

    logFile = "result/" + str(os.path.basename(inputDir)) + ".txt"
    print("logFile: {}\n".format(logFile))
    log = open(logFile, 'w+')

    # initialize controllers
    optCtrl = OptimalController()
    mandCtrl = MandatoryController()

    # initialize supervision
    detector = DeviationDetector()
    switcher = Switcher()

    # main loop
    quality = 30
    print("quality: {}\n".format(quality))

    for frame in range(startFrame, endFrame+1):
        framename = str(frame).zfill(8) + ".jpg"
        img_in = inputDir + "/" + framename
        img_out = outputDir + "/" + framename

        # measure frame size
        convert(img_in, img_out, quality)
        originalSize = os.path.getsize(img_in)/1024  # y(k)
        compressedSize = os.path.getsize(img_out)/1024  # y(k)

        # supervision
        if switcher.getSwithMode()==0:
            alarm = detector.detector(quality, originalSize, compressedSize) # u(k-1),a(k),y(k)
            print(detector.getBMean(), detector.getBCovariance(), alarm)

            if alarm==1:
                switcher.setSwitchMode(alarm)

        if switcher.getSwithMode()==0:
            # compute new control parameter
            quality = optCtrl.PIControl(quality, compressedSize) # u(k)
        else:
            # compute new control parameter
            quality = mandCtrl.control(quality, compressedSize)  # u(k)

        # Save trace
        outline = str(frame) + "," + str(quality) + "," + str(originalSize) + "," + str(compressedSize) + "\n"
        log.write(outline)
        log.flush()

        print("{}, {}, {}, {}".format(frame, quality, originalSize, compressedSize))

    log.close()

def main():
    videoFile = "../mp4/negativeVideo.mp4"
    generateJPG2MP4(videoFile)
    encode(videoFile)


if __name__ == "__main__":
    main()