import cv2

img1 = cv2.imread("img1.png") 
img2 = cv2.imread("img2.png")
brisk = cv2.BRISK_create()

kp1, descr1 = brisk.detectAndCompute(img1, None)
kp2, descr2 = brisk.detectAndCompute(img2, None)

matcher = cv2.BFMatcher_create(cv2.NORM_HAMMING) 
matches = matcher.match(descr1, descr2)

img3 = cv2.drawMatches(img1, kp1, img2, kp2, matches[:100], None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
# img3 = cv2.resize(img3, None, fx=0.5, fy=0.5)
cv2.imshow("", img3)
cv2.waitKey(0)
