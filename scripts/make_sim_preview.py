"""Encode saved simulator viewer frames at their recorded capture times."""
import argparse
from pathlib import Path
import av
from PIL import Image

p=argparse.ArgumentParser();p.add_argument('directory',type=Path);a=p.parse_args()
frames=sorted(a.directory.glob('frame_*.jpg'))
if not frames:raise RuntimeError('No recorded simulator frames')
times=[float(x.stem.removeprefix('frame_')) for x in frames]
size=Image.open(frames[0]).size
output=a.directory/'simulation_preview.mp4'
container=av.open(str(output),'w');stream=container.add_stream('libx264',rate=5)
stream.width,stream.height=size;stream.pix_fmt='yuv420p'
i=0
for tick in range(int(times[-1]*5)+1):
    while i+1<len(frames) and times[i+1]<=tick/5:i+=1
    with Image.open(frames[i]) as image:
        frame=av.VideoFrame.from_image(image.convert('RGB'))
    for packet in stream.encode(frame):container.mux(packet)
for packet in stream.encode():container.mux(packet)
container.close()
print(output)
