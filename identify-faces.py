import cv2

cascade_file = "haarcascade_frontalface_default.xml"
haar_cascade = cv2.CascadeClassifier(cascade_file)

file_name = 'nick.jpg'

img = cv2.imread(file_name, 0)

gray_img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

faces = haar_cascade.detectMultiScale(gray_img, 1.1, 4)

i = 0

for x, y, w, h in faces:

    cropped_image = img[y : y + h, x : x + w]
    target_file_name = file_name + '_face' + str(i) + '.jpg'
    cv2.imwrite(target_file_name, cropped_image)
    i += 1