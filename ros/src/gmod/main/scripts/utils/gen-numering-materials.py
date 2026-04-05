#!/usr/bin/env python3
# coding: utf-8

"""
Формирователь файла с описанием материала для нумерованных кубиков
Предполагается, что для каждого кода ArUco должен быть свой файл png с соответствующим именем
(0.png, 1.png и т.п.)

В итоге получим > numering_marker.material

LP 29.02.2024
"""

smat = \
'''
material numering_marker_{}
{{
  technique
  {{
    pass
    {{
      texture_unit
      {{
        texture {}{}.png
        scale 1 1
      }}
    }}
  }}
}}'''

f=open("numering_marker.material", "w")
path="../textures/"
for i in range(50+1):
    print(smat.format(i, path, i), file=f)
f.close()