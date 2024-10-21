import os

for file_number in range(12):
    os.rename(f'/home/dhruvagarwal/imgs/frame{file_number:04}.jpg', f'/home/dhruvagarwal/imgs/Pic_{file_number}.jpg')