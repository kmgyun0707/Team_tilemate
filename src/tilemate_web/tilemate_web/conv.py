import os
import subprocess
from tkinter import Tk, filedialog

def convert_gif_to_mp4():
    # GUI 창 숨기기
    root = Tk()
    root.withdraw()

    # GIF 파일 선택
    file_path = filedialog.askopenfilename(
        title="GIF 파일 선택",
        filetypes=[("GIF Files", "*.webm")]
    )

    if not file_path:
        print("파일을 선택하지 않았습니다.")
        return

    base, _ = os.path.splitext(file_path)
    output_path = base + "_converted.mp4"

    cmd = [
        "ffmpeg",
        "-i", file_path,
        "-vf", "scale=1280:-2",
        "-r", "30",
        "-pix_fmt", "yuv420p",
        output_path
    ]

    print("변환 시작...")
    subprocess.run(cmd)
    print("완료:", output_path)

if __name__ == "__main__":
    convert_gif_to_mp4()