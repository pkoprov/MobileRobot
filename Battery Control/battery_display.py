from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import ImageDraw, ImageFont, Image
import time

# Setup
serial = i2c(port=0, address=0x3C)
device = ssd1306(serial)

font = ImageFont.load_default()

def get_voltage():
    # Replace with real voltage reading if needed
    return round(11.7 + 0.3 * (time.time() % 1), 2)

while True:
    with Image.new("1", device.size) as image:
        draw = ImageDraw.Draw(image)
        voltage = get_voltage()
        draw.text((0, 0), "Battery Voltage", font=font, fill=255)
        draw.text((0, 15), f"{voltage} V", font=font, fill=255)
        device.display(image)
    time.sleep(1)
