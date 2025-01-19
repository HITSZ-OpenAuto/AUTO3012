from PIL import Image

def read_img():
    im = Image.open("eai_map_imu.pgm")    # 读取文件
    im.save("output_image.png")
    print(im.size)   # 输出图片大小

if __name__ == "__main__":
    read_img()     # 调用read_img()