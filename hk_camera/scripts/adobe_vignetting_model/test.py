import cv2
import numpy as np
import matplotlib.pyplot as plt

# === 生成增益掩膜 ===
def generate_vignetting_mask(shape, a, b, c):
    h, w = shape
    y, x = np.indices((h, w))
    cx, cy = w / 2, h / 2
    r = np.sqrt((x - cx)**2 + (y - cy)**2)
    r_norm = r / np.max(r)
    mask = 1.0 + a * r_norm**2 + b * r_norm**4 + c * r_norm**6
    return mask

# === 应用渐晕校正 ===
def apply_correction(img, a, b, c):
    mask = generate_vignetting_mask(img.shape[:2], a, b, c)
    img_f = img.astype(np.float32)
    corrected = img_f / mask
    corrected = np.clip(corrected, 0, 255)
    return corrected.astype(np.uint8)

# === 保存图像对比与直方图 ===
def save_comparison_and_histogram(original, corrected):
    # 拼接图像（灰度）
    combined_img = np.hstack((original, corrected))

    # 创建标题区域（40像素高的灰度条，白底）
    h, w = original.shape
    title_bar = np.ones((40, w * 2), dtype=np.uint8) * 0  # 黑色背景

    # 添加白色文字（灰度图需用灰度值）
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(title_bar, "Original", (int(w * 0.25) - 50, 25), font, 0.8, 255, 2, cv2.LINE_AA)
    cv2.putText(title_bar, "Corrected", (int(w * 0.75) - 50, 25), font, 0.8, 255, 2, cv2.LINE_AA)

    # 垂直拼接标题与图像
    final_image = np.vstack((title_bar, combined_img))

    # 保存图像
    cv2.imwrite("comparison.png", final_image)

    # 保存直方图
    import matplotlib.pyplot as plt
    plt.figure(figsize=(10, 5))
    plt.hist(original.ravel(), bins=256, range=(0, 256), alpha=0.5, label='Original', color='blue')
    plt.hist(corrected.ravel(), bins=256, range=(0, 256), alpha=0.5, label='Corrected', color='red')
    plt.title("Histogram Comparison")
    plt.xlabel("Pixel Intensity")
    plt.ylabel("Count")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig("histogram.png")
    plt.close()

# === 滑动条回调函数 ===
def update(val=0):
    a = cv2.getTrackbarPos("a x1000", "Corrected") / -1000.0
    b = cv2.getTrackbarPos("b x1000", "Corrected") / -1000.0
    c = cv2.getTrackbarPos("c x1000", "Corrected") / -1000.0
    corrected = apply_correction(img, a, b, c)
    cv2.imshow("Corrected", corrected)
    save_comparison_and_histogram(img, corrected)
    print(f"Saved comparison.png and histogram.png (a={a:.3f}, b={b:.3f}, c={c:.3f})")

# === 读取图像 ===
img = cv2.imread("image_left.png", cv2.IMREAD_GRAYSCALE)
if img is None:
    raise FileNotFoundError("图像文件未找到，请确保存在 'image_left.png'")

# === 创建窗口与滑动条 ===
cv2.namedWindow("Corrected", cv2.WINDOW_NORMAL)
cv2.createTrackbar("a x1000", "Corrected", 100, 2000, update)
cv2.createTrackbar("b x1000", "Corrected", 60, 2000, update)
cv2.createTrackbar("c x1000", "Corrected", 20, 2000, update)

# 初始化显示
update()

print("请拖动滑动条调节渐晕校正参数（按 Esc 键退出）")

# 事件循环
while True:
    if cv2.waitKey(10) == 27:  # Esc 退出
        break

cv2.destroyAllWindows()
