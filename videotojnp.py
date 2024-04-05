from moviepy.editor import VideoFileClip
import os 

def extract_frames(movie,times,imgdir):
    if not os.path.exists(imgdir):
        os.makedirs(imgdir)
    # apply clip tool on the video
    clip = VideoFileClip(movie)
    for t in times:
        # save images to the folder with frame number as file name
        imgpath = os.path.join(imgdir, '{}.png'.format(int(t*clip.fps)))
        clip.save_frame(imgpath,t)

#video file in the folder
movie = '3.mp4'

# image folder for image output
imgdir = './pngs3'
clip = VideoFileClip(movie)
times = [i/clip.fps for i in range(int(clip.fps*clip.duration))]
extract_frames(movie,times,imgdir)  
    