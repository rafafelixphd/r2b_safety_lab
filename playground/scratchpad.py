for filename in $(ls -l /Users/rafaelfelix/.cache/huggingface/lerobot//rafafelixphd/dataset-trial-1/videos/observation.images.front/chunk-000/); do
    ffmpeg -i $filename -filter:v "crop=1724:808:0:0, scale=224:224" -c:v libx264 -crf 18 observation.images.front_left.mp4
done


/Users/rafaelfelix/.cache/huggingface/lerobot/rafafelixphd/dataset-trial-2/videos/observation.images.front.left/chunk-000/
/Users/rafaelfelix/.cache/huggingface/lerobot/rafafelixphd/dataset-trial-2/videos/observation.images.front.right/chunk-000/