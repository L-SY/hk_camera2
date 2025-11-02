import cv2
import numpy as np
import glob, os

def extract_and_save_squares(input_folder, output_folder):
    os.makedirs(output_folder, exist_ok=True)
    for path in glob.glob(os.path.join(input_folder, "*.png")):
        img = cv2.imread(path)
        h, w = img.shape[:2]

        # 1. 预处理：灰度 + 中值滤波 + CLAHE
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur = cv2.medianBlur(gray, 5)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        gray = clahe.apply(blur)

        # 2. 自适应阈值提取白色边框
        thr = cv2.adaptiveThreshold(gray,255,
                     cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                     cv2.THRESH_BINARY,11,2)
        # 3. 形态学开闭消除小噪声
        k = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
        clean = cv2.morphologyEx(thr, cv2.MORPH_CLOSE, k)
        clean = cv2.morphologyEx(clean, cv2.MORPH_OPEN, k)

        # 4. 轮廓检测
        cnts, _ = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        squares = []
        for cnt in cnts:
            area = cv2.contourArea(cnt)
            if area < (h*w*0.01):  # 过滤太小
                continue
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.01*peri, True)
            if len(approx) == 4:
                # 检查长宽比
                pts = approx.reshape(4,2)
                xs = pts[:,0]; ys = pts[:,1]
                rw = xs.max()-xs.min(); rh = ys.max()-ys.min()
                ar = rw / max(rh,1)
                if 0.8 < ar < 1.2:
                    squares.append((area, pts))

        # 5. 选最大或按面积排序
        if not squares:
            print(f"No square in {os.path.basename(path)}")
            continue
        squares.sort(key=lambda x: -x[0])
        pts = squares[0][1].astype(np.float32)

        # 6. 透视变换 + 旋转
        def order_pts(pts):
            rect = np.zeros((4,2),dtype=np.float32)
            s = pts.sum(axis=1)
            rect[0] = pts[np.argmin(s)]; rect[2] = pts[np.argmax(s)]
            diff = np.diff(pts,axis=1)
            rect[1] = pts[np.argmin(diff)]; rect[3] = pts[np.argmax(diff)]
            return rect
        rect = order_pts(pts)
        (tl,tr,br,bl) = rect
        wA = np.linalg.norm(br-bl); wB = np.linalg.norm(tr-tl)
        hA = np.linalg.norm(tr-br); hB = np.linalg.norm(tl-bl)
        maxW, maxH = int(max(wA,wB)), int(max(hA,hB))
        dst = np.array([[0,0],[maxW-1,0],[maxW-1,maxH-1],[0,maxH-1]],dtype=np.float32)
        M = cv2.getPerspectiveTransform(rect,dst)
        warp = cv2.warpPerspective(img,M,(maxW,maxH))
        if warp.shape[0] > warp.shape[1]:
            warp = cv2.rotate(warp, cv2.ROTATE_90_CLOCKWISE)

        # 保存
        fn = os.path.splitext(os.path.basename(path))[0] + "_crop.png"
        cv2.imwrite(os.path.join(output_folder, fn), warp)
        print(f"Saved {fn}")


# 使用示例：
extract_and_save_squares("/home/yang/Desktop/MPP/PParound", "/home/yang/Desktop/MPP/crop")
