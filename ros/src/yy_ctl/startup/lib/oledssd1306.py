#!/usr/bin/env python3
# coding: utf-8
'''
  Демонстрационная программа работы с OLED
  ssd1315
  https://robotclass.ru/tutorials/raspberry-pi-oled-python/

  10.08.2024
  LP 11.08.2024
'''
import time
import board
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306

class lcd:
    #initializes objects and lcd
    # ADDRESS - OLED Address
    def __init__(self, ADDRESS=0x3c):
        # разрешение дисплея
        WIDTH = 128
        HEIGHT = 64

        # инициализация шины I2C и дисплея с адресом 0x3D
        i2c = board.I2C()
        self.oled = adafruit_ssd1306.SSD1306_I2C(WIDTH, HEIGHT, i2c, addr=ADDRESS)

        # заливаем экран черным цветом
        self.oled.fill(0)
        self.oled.show()

        # будем работать с буфером экрана из библиотеки PIL
        # первый параметр - цветность дисплея, для монохромного - 1
        self.image = Image.new("1", (self.oled.width, self.oled.height))

        # draw будет нашим карандашом, которым мы будем рисовать на экране
        self.draw = ImageDraw.Draw(self.image)

        # загружаем стандартный шрифт
        self.font = ImageFont.load_default()

    def lcd_clear(self):
        self.draw.rectangle( (0, 0, self.oled.width - 1, self.oled.height - 1), outline=0,  fill=0)

    def lcd_display_string(self, text, row):
        (font_width, font_height) = self.font.getsize(text)
        x1 = 5
        y1 = (row-1)*font_height
        x2 = x1+font_width
        y2 = y1+font_height
        self.draw.rectangle( (x1, y1, x2, y2), outline=0,  fill=0)
        self.draw.text( (x1, y1), text, font=self.font, fill=255)

        # выводим буфер изображения на дисплей
        self.oled.image(self.image)
        self.oled.show()
