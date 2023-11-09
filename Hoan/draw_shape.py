import cv2

img = cv2.imread('Hoan/blue_blank.jpg')

print(img.shape)

# Draw line
start_point = (0, 0)
end_point = (100,100)
color = (0, 255, 0)
thickness = 5

img = cv2.line(img, start_point, end_point, color, thickness)

# Draw circle
center = (100, 100)
r = 30
circle_color = (0,0,255)
circle_thickness = 3

img = cv2.circle(img, center, r, circle_color, circle_thickness)

# Draw rectangle
rect_start_point = (0, 0)
rect_end_point = (150, 100)
rect_color = (0, 255, 100)
rect_thickness = 5

img = cv2.rectangle(img, rect_start_point, rect_end_point, rect_color, rect_thickness)

# Draw Text
font = cv2.FONT_HERSHEY_SIMPLEX
org = ( 30, 30)
fontscale = 1
text_color = (0,0,255)
text_thickness = 3

img = cv2.putText(img, 'Xin chao cac ban',org, font,fontscale, text_color, text_thickness ) 



cv2.imshow('Draw shape', img)

if cv2.waitKey(0) & 0xff == 27:
    cv2.destroyAllWindows()