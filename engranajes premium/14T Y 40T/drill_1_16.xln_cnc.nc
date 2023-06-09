(G-CODE GENERATED BY FLATCAM v8.994 - www.flatcam.org - Version Date: 2020/11/7)

(Name: drill_1_16.xln_cnc)
(Type: G-code from Geometry)
(Units: MM)

(Created on Friday, 07 April 2023 at 16:05)

(This preprocessor is the default preprocessor used by FlatCAM.)
(It is made to work with MACH3 compatible motion controllers.)


(TOOLS DIAMETER: )
(Tool: 1 -> Dia: 2.5)
(Tool: 2 -> Dia: 3.1)

(FEEDRATE Z: )
(Tool: 1 -> Feedrate: 10.0)
(Tool: 2 -> Feedrate: 10.0)

(FEEDRATE RAPIDS: )
(Tool: 1 -> Feedrate Rapids: 1500)
(Tool: 2 -> Feedrate Rapids: 1500)

(Z_CUT: )
(Tool: 1 -> Z_Cut: -1.95)
(Tool: 2 -> Z_Cut: -1.95)

(Tools Offset: )
(Tool: 1 -> Offset Z: 0.0)
(Tool: 2 -> Offset Z: 0.0)

(Z_MOVE: )
(Tool: 1 -> Z_Move: 5.0)
(Tool: 2 -> Z_Move: 5.0)

(Z Toolchange: 8.0 mm)
(X,Y Toolchange: 0.0000, 0.0000 mm)
(Z Start: None mm)
(Z End: 10.0 mm)
(X,Y End: None mm)
(Steps per circle: 64)
(Preprocessor Excellon: default)

(X range:  -14.0500 ...   14.0500  mm)
(Y range:   -7.5500 ...    7.2500  mm)

(Spindle Speed: 0 RPM)
G21
G90
G94

G01 F10.00

M5
G00 Z8.0000
T1
G00 X0.0000 Y0.0000                
M6
(MSG, Change to Tool Dia = 2.5000 ||| Total drills for tool T1 = 8)
M0
G00 Z8.0000

G01 F10.00
M03
G00 X0.0000 Y6.0000
G01 Z-1.9500
G01 Z0
G00 Z5.0000
G00 X5.1940 Y3.0020
G01 Z-1.9500
G01 Z0
G00 Z5.0000
G00 X12.5000 Y-6.0000
G01 Z-1.9500
G01 Z0
G00 Z5.0000
G00 X5.1970 Y-2.9950
G01 Z-1.9500
G01 Z0
G00 Z5.0000
G00 X0.0000 Y-6.0000
G01 Z-1.9500
G01 Z0
G00 Z5.0000
G00 X-5.1940 Y-2.9990
G01 Z-1.9500
G01 Z0
G00 Z5.0000
G00 X-12.5000 Y-6.0000
G01 Z-1.9500
G01 Z0
G00 Z5.0000
G00 X-5.1970 Y2.9990
G01 Z-1.9500
G01 Z0
G00 Z5.0000
M05
G00 Z10.00


