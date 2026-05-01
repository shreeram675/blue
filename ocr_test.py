import easyocr
import cv2

reader = easyocr.Reader(['en'])

img = cv2.imread('test_blueprint.jpg')
results = reader.readtext(img)

print("\n=== OCR RESULTS ===")
for (bbox, text, confidence) in results:
    print(f"Text: '{text}' | Confidence: {confidence:.2f}")