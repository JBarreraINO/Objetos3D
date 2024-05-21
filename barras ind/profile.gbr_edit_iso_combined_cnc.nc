(G-CODE GENERATED BY FLATCAM v8.994 - www.flatcam.org - Version Date: 2020/11/7)

(Name: profile.gbr_edit_iso_combined_cnc)
(Type: G-code from Geometry)
(Units: MM)

(Created on Wednesday, 05 April 2023 at 22:21)

(This preprocessor is the default preprocessor used by FlatCAM.)
(It is made to work with MACH3 compatible motion controllers.)

(TOOL DIAMETER: 3.175 mm)
(Feedrate_XY: 10.0 mm/min)
(Feedrate_Z: 10.0 mm/min)
(Feedrate rapids 1500.0 mm/min)

(Z_Cut: -1.75 mm)
(DepthPerCut: 0.8 mm <=>3 passes)
(Z_Move: 10.0 mm)
(Z Start: None mm)
(Z End: 15.0 mm)
(X,Y End: None mm)
(Steps per circle: 64)
(Preprocessor Geometry: default)

(X range:  -19.8646 ...   10.9746  mm)
(Y range:   -8.2146 ...    8.2146  mm)

(Spindle Speed: 0.0 RPM)
G21
G90
G94

G01 F10.00

M5
G00 Z15.0000
G00 X0.0000 Y0.0000
T2
M6    
(MSG, Change to Tool Dia = 3.1750)
M0
G00 Z15.0000

M03
G01 F10.00
G00 X-1.4854 Y-0.0366
G01 F10.00
G01 Z-0.8000
G01 F10.00
G01 X-1.4854 Y0.0366
G01 X-1.4819 Y0.1093
G01 X-1.4747 Y0.1816
G01 X-1.4640 Y0.2540
G01 X-1.4498 Y0.3257
G01 X-1.4321 Y0.3963
G01 X-1.4108 Y0.4663
G01 X-1.3862 Y0.5351
G01 X-1.3584 Y0.6022
G01 X-1.3274 Y0.6678
G01 X-1.2930 Y0.7321
G01 X-1.2553 Y0.7949
G01 X-1.2147 Y0.8558
G01 X-1.1714 Y0.9142
G01 X-1.1251 Y0.9706
G01 X-1.0762 Y1.0246
G01 X-1.0246 Y1.0762
G01 X-0.9706 Y1.1251
G01 X-0.9142 Y1.1714
G01 X-0.8558 Y1.2147
G01 X-0.7949 Y1.2553
G01 X-0.7321 Y1.2930
G01 X-0.6678 Y1.3274
G01 X-0.6022 Y1.3584
G01 X-0.5351 Y1.3862
G01 X-0.4663 Y1.4108
G01 X-0.3963 Y1.4321
G01 X-0.3257 Y1.4498
G01 X-0.2540 Y1.4640
G01 X-0.1816 Y1.4747
G01 X-0.1093 Y1.4819
G01 X-0.0366 Y1.4854
G01 X0.0366 Y1.4854
G01 X0.1093 Y1.4819
G01 X0.1816 Y1.4747
G01 X0.2540 Y1.4640
G01 X0.3257 Y1.4498
G01 X0.3963 Y1.4321
G01 X0.4663 Y1.4108
G01 X0.5351 Y1.3862
G01 X0.6022 Y1.3584
G01 X0.6678 Y1.3274
G01 X0.7321 Y1.2930
G01 X0.7949 Y1.2553
G01 X0.8558 Y1.2147
G01 X0.9142 Y1.1714
G01 X0.9706 Y1.1251
G01 X1.0246 Y1.0762
G01 X1.0762 Y1.0246
G01 X1.1251 Y0.9706
G01 X1.1714 Y0.9142
G01 X1.2147 Y0.8558
G01 X1.2553 Y0.7949
G01 X1.2930 Y0.7321
G01 X1.3274 Y0.6678
G01 X1.3584 Y0.6022
G01 X1.3862 Y0.5351
G01 X1.4108 Y0.4663
G01 X1.4321 Y0.3963
G01 X1.4498 Y0.3257
G01 X1.4640 Y0.2540
G01 X1.4747 Y0.1816
G01 X1.4819 Y0.1093
G01 X1.4854 Y0.0366
G01 X1.4854 Y-0.0366
G01 X1.4819 Y-0.1093
G01 X1.4747 Y-0.1816
G01 X1.4640 Y-0.2540
G01 X1.4498 Y-0.3257
G01 X1.4321 Y-0.3963
G01 X1.4108 Y-0.4663
G01 X1.3862 Y-0.5351
G01 X1.3584 Y-0.6022
G01 X1.3274 Y-0.6678
G01 X1.2930 Y-0.7321
G01 X1.2553 Y-0.7949
G01 X1.2147 Y-0.8558
G01 X1.1714 Y-0.9142
G01 X1.1251 Y-0.9706
G01 X1.0762 Y-1.0246
G01 X1.0246 Y-1.0762
G01 X0.9706 Y-1.1251
G01 X0.9142 Y-1.1714
G01 X0.8558 Y-1.2147
G01 X0.7949 Y-1.2553
G01 X0.7321 Y-1.2930
G01 X0.6678 Y-1.3274
G01 X0.6022 Y-1.3584
G01 X0.5351 Y-1.3862
G01 X0.4663 Y-1.4108
G01 X0.3963 Y-1.4321
G01 X0.3257 Y-1.4498
G01 X0.2540 Y-1.4640
G01 X0.1816 Y-1.4747
G01 X0.1093 Y-1.4819
G01 X0.0366 Y-1.4854
G01 X-0.0366 Y-1.4854
G01 X-0.1093 Y-1.4819
G01 X-0.1816 Y-1.4747
G01 X-0.2540 Y-1.4640
G01 X-0.3257 Y-1.4498
G01 X-0.3963 Y-1.4321
G01 X-0.4663 Y-1.4108
G01 X-0.5351 Y-1.3862
G01 X-0.6022 Y-1.3584
G01 X-0.6678 Y-1.3274
G01 X-0.7321 Y-1.2930
G01 X-0.7949 Y-1.2553
G01 X-0.8558 Y-1.2147
G01 X-0.9142 Y-1.1714
G01 X-0.9706 Y-1.1251
G01 X-1.0246 Y-1.0762
G01 X-1.0762 Y-1.0246
G01 X-1.1251 Y-0.9706
G01 X-1.1714 Y-0.9142
G01 X-1.2147 Y-0.8558
G01 X-1.2553 Y-0.7949
G01 X-1.2930 Y-0.7321
G01 X-1.3274 Y-0.6678
G01 X-1.3584 Y-0.6022
G01 X-1.3862 Y-0.5351
G01 X-1.4108 Y-0.4663
G01 X-1.4321 Y-0.3963
G01 X-1.4498 Y-0.3257
G01 X-1.4640 Y-0.2540
G01 X-1.4747 Y-0.1816
G01 X-1.4819 Y-0.1093
G01 X-1.4854 Y-0.0366
G00 X-1.4854 Y-0.0366
G01 F10.00
G01 Z-1.6000
G01 F10.00
G01 X-1.4854 Y0.0366
G01 X-1.4819 Y0.1093
G01 X-1.4747 Y0.1816
G01 X-1.4640 Y0.2540
G01 X-1.4498 Y0.3257
G01 X-1.4321 Y0.3963
G01 X-1.4108 Y0.4663
G01 X-1.3862 Y0.5351
G01 X-1.3584 Y0.6022
G01 X-1.3274 Y0.6678
G01 X-1.2930 Y0.7321
G01 X-1.2553 Y0.7949
G01 X-1.2147 Y0.8558
G01 X-1.1714 Y0.9142
G01 X-1.1251 Y0.9706
G01 X-1.0762 Y1.0246
G01 X-1.0246 Y1.0762
G01 X-0.9706 Y1.1251
G01 X-0.9142 Y1.1714
G01 X-0.8558 Y1.2147
G01 X-0.7949 Y1.2553
G01 X-0.7321 Y1.2930
G01 X-0.6678 Y1.3274
G01 X-0.6022 Y1.3584
G01 X-0.5351 Y1.3862
G01 X-0.4663 Y1.4108
G01 X-0.3963 Y1.4321
G01 X-0.3257 Y1.4498
G01 X-0.2540 Y1.4640
G01 X-0.1816 Y1.4747
G01 X-0.1093 Y1.4819
G01 X-0.0366 Y1.4854
G01 X0.0366 Y1.4854
G01 X0.1093 Y1.4819
G01 X0.1816 Y1.4747
G01 X0.2540 Y1.4640
G01 X0.3257 Y1.4498
G01 X0.3963 Y1.4321
G01 X0.4663 Y1.4108
G01 X0.5351 Y1.3862
G01 X0.6022 Y1.3584
G01 X0.6678 Y1.3274
G01 X0.7321 Y1.2930
G01 X0.7949 Y1.2553
G01 X0.8558 Y1.2147
G01 X0.9142 Y1.1714
G01 X0.9706 Y1.1251
G01 X1.0246 Y1.0762
G01 X1.0762 Y1.0246
G01 X1.1251 Y0.9706
G01 X1.1714 Y0.9142
G01 X1.2147 Y0.8558
G01 X1.2553 Y0.7949
G01 X1.2930 Y0.7321
G01 X1.3274 Y0.6678
G01 X1.3584 Y0.6022
G01 X1.3862 Y0.5351
G01 X1.4108 Y0.4663
G01 X1.4321 Y0.3963
G01 X1.4498 Y0.3257
G01 X1.4640 Y0.2540
G01 X1.4747 Y0.1816
G01 X1.4819 Y0.1093
G01 X1.4854 Y0.0366
G01 X1.4854 Y-0.0366
G01 X1.4819 Y-0.1093
G01 X1.4747 Y-0.1816
G01 X1.4640 Y-0.2540
G01 X1.4498 Y-0.3257
G01 X1.4321 Y-0.3963
G01 X1.4108 Y-0.4663
G01 X1.3862 Y-0.5351
G01 X1.3584 Y-0.6022
G01 X1.3274 Y-0.6678
G01 X1.2930 Y-0.7321
G01 X1.2553 Y-0.7949
G01 X1.2147 Y-0.8558
G01 X1.1714 Y-0.9142
G01 X1.1251 Y-0.9706
G01 X1.0762 Y-1.0246
G01 X1.0246 Y-1.0762
G01 X0.9706 Y-1.1251
G01 X0.9142 Y-1.1714
G01 X0.8558 Y-1.2147
G01 X0.7949 Y-1.2553
G01 X0.7321 Y-1.2930
G01 X0.6678 Y-1.3274
G01 X0.6022 Y-1.3584
G01 X0.5351 Y-1.3862
G01 X0.4663 Y-1.4108
G01 X0.3963 Y-1.4321
G01 X0.3257 Y-1.4498
G01 X0.2540 Y-1.4640
G01 X0.1816 Y-1.4747
G01 X0.1093 Y-1.4819
G01 X0.0366 Y-1.4854
G01 X-0.0366 Y-1.4854
G01 X-0.1093 Y-1.4819
G01 X-0.1816 Y-1.4747
G01 X-0.2540 Y-1.4640
G01 X-0.3257 Y-1.4498
G01 X-0.3963 Y-1.4321
G01 X-0.4663 Y-1.4108
G01 X-0.5351 Y-1.3862
G01 X-0.6022 Y-1.3584
G01 X-0.6678 Y-1.3274
G01 X-0.7321 Y-1.2930
G01 X-0.7949 Y-1.2553
G01 X-0.8558 Y-1.2147
G01 X-0.9142 Y-1.1714
G01 X-0.9706 Y-1.1251
G01 X-1.0246 Y-1.0762
G01 X-1.0762 Y-1.0246
G01 X-1.1251 Y-0.9706
G01 X-1.1714 Y-0.9142
G01 X-1.2147 Y-0.8558
G01 X-1.2553 Y-0.7949
G01 X-1.2930 Y-0.7321
G01 X-1.3274 Y-0.6678
G01 X-1.3584 Y-0.6022
G01 X-1.3862 Y-0.5351
G01 X-1.4108 Y-0.4663
G01 X-1.4321 Y-0.3963
G01 X-1.4498 Y-0.3257
G01 X-1.4640 Y-0.2540
G01 X-1.4747 Y-0.1816
G01 X-1.4819 Y-0.1093
G01 X-1.4854 Y-0.0366
G00 X-1.4854 Y-0.0366
G01 F10.00
G01 Z-1.7500
G01 F10.00
G01 X-1.4854 Y0.0366
G01 X-1.4819 Y0.1093
G01 X-1.4747 Y0.1816
G01 X-1.4640 Y0.2540
G01 X-1.4498 Y0.3257
G01 X-1.4321 Y0.3963
G01 X-1.4108 Y0.4663
G01 X-1.3862 Y0.5351
G01 X-1.3584 Y0.6022
G01 X-1.3274 Y0.6678
G01 X-1.2930 Y0.7321
G01 X-1.2553 Y0.7949
G01 X-1.2147 Y0.8558
G01 X-1.1714 Y0.9142
G01 X-1.1251 Y0.9706
G01 X-1.0762 Y1.0246
G01 X-1.0246 Y1.0762
G01 X-0.9706 Y1.1251
G01 X-0.9142 Y1.1714
G01 X-0.8558 Y1.2147
G01 X-0.7949 Y1.2553
G01 X-0.7321 Y1.2930
G01 X-0.6678 Y1.3274
G01 X-0.6022 Y1.3584
G01 X-0.5351 Y1.3862
G01 X-0.4663 Y1.4108
G01 X-0.3963 Y1.4321
G01 X-0.3257 Y1.4498
G01 X-0.2540 Y1.4640
G01 X-0.1816 Y1.4747
G01 X-0.1093 Y1.4819
G01 X-0.0366 Y1.4854
G01 X0.0366 Y1.4854
G01 X0.1093 Y1.4819
G01 X0.1816 Y1.4747
G01 X0.2540 Y1.4640
G01 X0.3257 Y1.4498
G01 X0.3963 Y1.4321
G01 X0.4663 Y1.4108
G01 X0.5351 Y1.3862
G01 X0.6022 Y1.3584
G01 X0.6678 Y1.3274
G01 X0.7321 Y1.2930
G01 X0.7949 Y1.2553
G01 X0.8558 Y1.2147
G01 X0.9142 Y1.1714
G01 X0.9706 Y1.1251
G01 X1.0246 Y1.0762
G01 X1.0762 Y1.0246
G01 X1.1251 Y0.9706
G01 X1.1714 Y0.9142
G01 X1.2147 Y0.8558
G01 X1.2553 Y0.7949
G01 X1.2930 Y0.7321
G01 X1.3274 Y0.6678
G01 X1.3584 Y0.6022
G01 X1.3862 Y0.5351
G01 X1.4108 Y0.4663
G01 X1.4321 Y0.3963
G01 X1.4498 Y0.3257
G01 X1.4640 Y0.2540
G01 X1.4747 Y0.1816
G01 X1.4819 Y0.1093
G01 X1.4854 Y0.0366
G01 X1.4854 Y-0.0366
G01 X1.4819 Y-0.1093
G01 X1.4747 Y-0.1816
G01 X1.4640 Y-0.2540
G01 X1.4498 Y-0.3257
G01 X1.4321 Y-0.3963
G01 X1.4108 Y-0.4663
G01 X1.3862 Y-0.5351
G01 X1.3584 Y-0.6022
G01 X1.3274 Y-0.6678
G01 X1.2930 Y-0.7321
G01 X1.2553 Y-0.7949
G01 X1.2147 Y-0.8558
G01 X1.1714 Y-0.9142
G01 X1.1251 Y-0.9706
G01 X1.0762 Y-1.0246
G01 X1.0246 Y-1.0762
G01 X0.9706 Y-1.1251
G01 X0.9142 Y-1.1714
G01 X0.8558 Y-1.2147
G01 X0.7949 Y-1.2553
G01 X0.7321 Y-1.2930
G01 X0.6678 Y-1.3274
G01 X0.6022 Y-1.3584
G01 X0.5351 Y-1.3862
G01 X0.4663 Y-1.4108
G01 X0.3963 Y-1.4321
G01 X0.3257 Y-1.4498
G01 X0.2540 Y-1.4640
G01 X0.1816 Y-1.4747
G01 X0.1093 Y-1.4819
G01 X0.0366 Y-1.4854
G01 X-0.0366 Y-1.4854
G01 X-0.1093 Y-1.4819
G01 X-0.1816 Y-1.4747
G01 X-0.2540 Y-1.4640
G01 X-0.3257 Y-1.4498
G01 X-0.3963 Y-1.4321
G01 X-0.4663 Y-1.4108
G01 X-0.5351 Y-1.3862
G01 X-0.6022 Y-1.3584
G01 X-0.6678 Y-1.3274
G01 X-0.7321 Y-1.2930
G01 X-0.7949 Y-1.2553
G01 X-0.8558 Y-1.2147
G01 X-0.9142 Y-1.1714
G01 X-0.9706 Y-1.1251
G01 X-1.0246 Y-1.0762
G01 X-1.0762 Y-1.0246
G01 X-1.1251 Y-0.9706
G01 X-1.1714 Y-0.9142
G01 X-1.2147 Y-0.8558
G01 X-1.2553 Y-0.7949
G01 X-1.2930 Y-0.7321
G01 X-1.3274 Y-0.6678
G01 X-1.3584 Y-0.6022
G01 X-1.3862 Y-0.5351
G01 X-1.4108 Y-0.4663
G01 X-1.4321 Y-0.3963
G01 X-1.4498 Y-0.3257
G01 X-1.4640 Y-0.2540
G01 X-1.4747 Y-0.1816
G01 X-1.4819 Y-0.1093
G01 X-1.4854 Y-0.0366
G00 Z10.0000
G00 X-19.8639 Y-0.0544
G01 F10.00
G01 Z-0.8000
G01 F10.00
G01 X-19.8629 Y-0.0848
G01 X-19.8382 Y-0.6504
G01 X-19.8363 Y-0.6850
G01 X-19.8327 Y-0.7305
G01 X-19.8293 Y-0.7650
G01 X-19.8252 Y-0.7994
G01 X-19.7513 Y-1.3608
G01 X-19.7464 Y-1.3950
G01 X-19.7408 Y-1.4292
G01 X-19.7325 Y-1.4741
G01 X-19.7253 Y-1.5080
G01 X-19.6028 Y-2.0607
G01 X-19.5950 Y-2.0945
G01 X-19.5864 Y-2.1281
G01 X-19.5741 Y-2.1722
G01 X-19.5641 Y-2.2054
G01 X-19.3938 Y-2.7454
G01 X-19.3830 Y-2.7783
G01 X-19.3677 Y-2.8214
G01 X-19.3555 Y-2.8538
G01 X-19.3426 Y-2.8859
G01 X-19.1259 Y-3.4090
G01 X-19.1123 Y-3.4408
G01 X-19.0981 Y-3.4723
G01 X-19.0784 Y-3.5136
G01 X-19.0628 Y-3.5444
G01 X-18.8014 Y-4.0466
G01 X-18.7850 Y-4.0772
G01 X-18.7625 Y-4.1170
G01 X-18.7449 Y-4.1468
G01 X-18.7265 Y-4.1762
G01 X-18.4223 Y-4.6537
G01 X-18.4034 Y-4.6827
G01 X-18.3775 Y-4.7204
G01 X-18.3573 Y-4.7486
G01 X-18.3365 Y-4.7763
G01 X-17.9918 Y-5.2255
G01 X-17.9705 Y-5.2527
G01 X-17.9485 Y-5.2795
G01 X-17.9188 Y-5.3143
G01 X-17.8957 Y-5.3401
G01 X-17.5132 Y-5.7575
G01 X-17.4895 Y-5.7827
G01 X-17.4653 Y-5.8075
G01 X-17.4327 Y-5.8395
G01 X-17.4075 Y-5.8632
G01 X-16.9901 Y-6.2457
G01 X-16.9643 Y-6.2688
G01 X-16.9380 Y-6.2913
G01 X-16.9027 Y-6.3204
G01 X-16.8755 Y-6.3418
G01 X-16.4263 Y-6.6865
G01 X-16.3986 Y-6.7073
G01 X-16.3613 Y-6.7339
G01 X-16.3327 Y-6.7534
G01 X-16.3037 Y-6.7723
G01 X-15.8262 Y-7.0765
G01 X-15.7968 Y-7.0948
G01 X-15.7574 Y-7.1180
G01 X-15.7272 Y-7.1350
G01 X-15.6966 Y-7.1514
G01 X-15.1944 Y-7.4128
G01 X-15.1636 Y-7.4284
G01 X-15.1223 Y-7.4481
G01 X-15.0908 Y-7.4623
G01 X-15.0590 Y-7.4759
G01 X-14.5359 Y-7.6926
G01 X-14.5038 Y-7.7055
G01 X-14.4610 Y-7.7215
G01 X-14.4283 Y-7.7330
G01 X-14.3954 Y-7.7438
G01 X-13.8554 Y-7.9141
G01 X-13.8222 Y-7.9241
G01 X-13.7888 Y-7.9335
G01 X-13.7445 Y-7.9450
G01 X-13.7107 Y-7.9528
G01 X-13.1580 Y-8.0753
G01 X-13.1241 Y-8.0825
G01 X-13.0901 Y-8.0888
G01 X-13.0450 Y-8.0964
G01 X-13.0108 Y-8.1013
G01 X-12.4494 Y-8.1752
G01 X-12.4150 Y-8.1793
G01 X-12.3805 Y-8.1827
G01 X-12.3350 Y-8.1863
G01 X-12.3004 Y-8.1882
G01 X-11.7175 Y-8.2136
G01 X-11.6600 Y-8.2146
G01 X2.7700 Y-8.2146
G01 X2.8046 Y-8.2142
G01 X2.8448 Y-8.2129
G01 X3.4104 Y-8.1882
G01 X3.4450 Y-8.1863
G01 X3.4905 Y-8.1827
G01 X3.5250 Y-8.1793
G01 X3.5594 Y-8.1752
G01 X4.1208 Y-8.1013
G01 X4.1550 Y-8.0964
G01 X4.1892 Y-8.0908
G01 X4.2341 Y-8.0825
G01 X4.2680 Y-8.0753
G01 X4.8207 Y-7.9528
G01 X4.8545 Y-7.9450
G01 X4.8988 Y-7.9335
G01 X4.9322 Y-7.9241
G01 X4.9654 Y-7.9141
G01 X5.5054 Y-7.7438
G01 X5.5383 Y-7.7330
G01 X5.5710 Y-7.7215
G01 X5.6138 Y-7.7055
G01 X5.6459 Y-7.6926
G01 X6.1690 Y-7.4759
G01 X6.2008 Y-7.4623
G01 X6.2424 Y-7.4434
G01 X6.2736 Y-7.4284
G01 X6.3044 Y-7.4128
G01 X6.8066 Y-7.1514
G01 X6.8372 Y-7.1350
G01 X6.8674 Y-7.1180
G01 X6.9068 Y-7.0948
G01 X6.9362 Y-7.0765
G01 X7.4137 Y-6.7723
G01 X7.4427 Y-6.7534
G01 X7.4713 Y-6.7339
G01 X7.5086 Y-6.7073
G01 X7.5363 Y-6.6865
G01 X7.9855 Y-6.3418
G01 X8.0127 Y-6.3204
G01 X8.0395 Y-6.2985
G01 X8.0743 Y-6.2688
G01 X8.1001 Y-6.2457
G01 X8.5175 Y-5.8632
G01 X8.5427 Y-5.8395
G01 X8.5753 Y-5.8075
G01 X8.5995 Y-5.7827
G01 X8.6232 Y-5.7575
G01 X9.0057 Y-5.3401
G01 X9.0288 Y-5.3143
G01 X9.0585 Y-5.2795
G01 X9.0804 Y-5.2527
G01 X9.1018 Y-5.2255
G01 X9.4465 Y-4.7763
G01 X9.4673 Y-4.7486
G01 X9.4875 Y-4.7204
G01 X9.5134 Y-4.6827
G01 X9.5323 Y-4.6537
G01 X9.8365 Y-4.1762
G01 X9.8548 Y-4.1468
G01 X9.8725 Y-4.1170
G01 X9.8950 Y-4.0772
G01 X9.9114 Y-4.0466
G01 X10.1728 Y-3.5444
G01 X10.1884 Y-3.5136
G01 X10.2081 Y-3.4723
G01 X10.2223 Y-3.4408
G01 X10.2359 Y-3.4090
G01 X10.4526 Y-2.8859
G01 X10.4655 Y-2.8538
G01 X10.4777 Y-2.8214
G01 X10.4930 Y-2.7783
G01 X10.5038 Y-2.7454
G01 X10.6741 Y-2.2054
G01 X10.6841 Y-2.1722
G01 X10.6935 Y-2.1388
G01 X10.7050 Y-2.0945
G01 X10.7128 Y-2.0607
G01 X10.8353 Y-1.5080
G01 X10.8425 Y-1.4741
G01 X10.8488 Y-1.4401
G01 X10.8564 Y-1.3950
G01 X10.8613 Y-1.3608
G01 X10.9352 Y-0.7994
G01 X10.9393 Y-0.7650
G01 X10.9437 Y-0.7195
G01 X10.9463 Y-0.6850
G01 X10.9482 Y-0.6504
G01 X10.9729 Y-0.0848
G01 X10.9742 Y-0.0446
G01 X10.9746 Y0.0100
G01 X10.9741 Y0.0502
G01 X10.9729 Y0.0848
G01 X10.9482 Y0.6504
G01 X10.9463 Y0.6850
G01 X10.9437 Y0.7195
G01 X10.9393 Y0.7650
G01 X10.9352 Y0.7994
G01 X10.8613 Y1.3608
G01 X10.8564 Y1.3950
G01 X10.8508 Y1.4292
G01 X10.8425 Y1.4741
G01 X10.8353 Y1.5080
G01 X10.7128 Y2.0607
G01 X10.7050 Y2.0945
G01 X10.6964 Y2.1281
G01 X10.6841 Y2.1722
G01 X10.6741 Y2.2054
G01 X10.5038 Y2.7454
G01 X10.4930 Y2.7783
G01 X10.4777 Y2.8214
G01 X10.4655 Y2.8538
G01 X10.4526 Y2.8859
G01 X10.2359 Y3.4090
G01 X10.2223 Y3.4408
G01 X10.2081 Y3.4723
G01 X10.1884 Y3.5136
G01 X10.1728 Y3.5444
G01 X9.9114 Y4.0466
G01 X9.8950 Y4.0772
G01 X9.8725 Y4.1170
G01 X9.8548 Y4.1468
G01 X9.8365 Y4.1762
G01 X9.5323 Y4.6537
G01 X9.5134 Y4.6827
G01 X9.4875 Y4.7204
G01 X9.4673 Y4.7486
G01 X9.4465 Y4.7763
G01 X9.1018 Y5.2255
G01 X9.0804 Y5.2527
G01 X9.0585 Y5.2795
G01 X9.0288 Y5.3143
G01 X9.0057 Y5.3401
G01 X8.6232 Y5.7575
G01 X8.5995 Y5.7827
G01 X8.5753 Y5.8075
G01 X8.5427 Y5.8395
G01 X8.5175 Y5.8632
G01 X8.1001 Y6.2457
G01 X8.0743 Y6.2688
G01 X8.0480 Y6.2913
G01 X8.0127 Y6.3204
G01 X7.9855 Y6.3418
G01 X7.5363 Y6.6865
G01 X7.5086 Y6.7073
G01 X7.4713 Y6.7339
G01 X7.4427 Y6.7534
G01 X7.4137 Y6.7723
G01 X6.9362 Y7.0765
G01 X6.9068 Y7.0948
G01 X6.8674 Y7.1180
G01 X6.8372 Y7.1350
G01 X6.8066 Y7.1514
G01 X6.3044 Y7.4128
G01 X6.2736 Y7.4284
G01 X6.2323 Y7.4481
G01 X6.2008 Y7.4623
G01 X6.1690 Y7.4759
G01 X5.6459 Y7.6926
G01 X5.6138 Y7.7055
G01 X5.5710 Y7.7215
G01 X5.5383 Y7.7330
G01 X5.5054 Y7.7438
G01 X4.9654 Y7.9141
G01 X4.9322 Y7.9241
G01 X4.8988 Y7.9335
G01 X4.8545 Y7.9450
G01 X4.8207 Y7.9528
G01 X4.2680 Y8.0753
G01 X4.2341 Y8.0825
G01 X4.2001 Y8.0888
G01 X4.1550 Y8.0964
G01 X4.1208 Y8.1013
G01 X3.5594 Y8.1752
G01 X3.5250 Y8.1793
G01 X3.4905 Y8.1827
G01 X3.4450 Y8.1863
G01 X3.4104 Y8.1882
G01 X2.8275 Y8.2136
G01 X2.7700 Y8.2146
G01 X-11.6600 Y8.2146
G01 X-11.6946 Y8.2142
G01 X-11.7348 Y8.2129
G01 X-12.3004 Y8.1882
G01 X-12.3350 Y8.1863
G01 X-12.3805 Y8.1827
G01 X-12.4150 Y8.1793
G01 X-12.4494 Y8.1752
G01 X-13.0108 Y8.1013
G01 X-13.0450 Y8.0964
G01 X-13.0792 Y8.0908
G01 X-13.1241 Y8.0825
G01 X-13.1580 Y8.0753
G01 X-13.7107 Y7.9528
G01 X-13.7445 Y7.9450
G01 X-13.7781 Y7.9364
G01 X-13.8222 Y7.9241
G01 X-13.8554 Y7.9141
G01 X-14.3954 Y7.7438
G01 X-14.4283 Y7.7330
G01 X-14.4610 Y7.7215
G01 X-14.5038 Y7.7055
G01 X-14.5359 Y7.6926
G01 X-15.0590 Y7.4759
G01 X-15.0908 Y7.4623
G01 X-15.1324 Y7.4434
G01 X-15.1636 Y7.4284
G01 X-15.1944 Y7.4128
G01 X-15.6966 Y7.1514
G01 X-15.7272 Y7.1350
G01 X-15.7574 Y7.1180
G01 X-15.7968 Y7.0948
G01 X-15.8262 Y7.0765
G01 X-16.3037 Y6.7723
G01 X-16.3327 Y6.7534
G01 X-16.3613 Y6.7339
G01 X-16.3986 Y6.7073
G01 X-16.4263 Y6.6865
G01 X-16.8755 Y6.3418
G01 X-16.9027 Y6.3204
G01 X-16.9295 Y6.2985
G01 X-16.9643 Y6.2688
G01 X-16.9901 Y6.2457
G01 X-17.4075 Y5.8632
G01 X-17.4327 Y5.8395
G01 X-17.4653 Y5.8075
G01 X-17.4895 Y5.7827
G01 X-17.5132 Y5.7575
G01 X-17.8957 Y5.3401
G01 X-17.9188 Y5.3143
G01 X-17.9485 Y5.2795
G01 X-17.9705 Y5.2527
G01 X-17.9918 Y5.2255
G01 X-18.3365 Y4.7763
G01 X-18.3573 Y4.7486
G01 X-18.3775 Y4.7204
G01 X-18.4034 Y4.6827
G01 X-18.4223 Y4.6537
G01 X-18.7265 Y4.1762
G01 X-18.7449 Y4.1468
G01 X-18.7680 Y4.1074
G01 X-18.7850 Y4.0772
G01 X-18.8014 Y4.0466
G01 X-19.0628 Y3.5444
G01 X-19.0784 Y3.5136
G01 X-19.0981 Y3.4723
G01 X-19.1123 Y3.4408
G01 X-19.1259 Y3.4090
G01 X-19.3426 Y2.8859
G01 X-19.3555 Y2.8538
G01 X-19.3677 Y2.8214
G01 X-19.3830 Y2.7783
G01 X-19.3938 Y2.7454
G01 X-19.5641 Y2.2054
G01 X-19.5741 Y2.1722
G01 X-19.5835 Y2.1388
G01 X-19.5950 Y2.0945
G01 X-19.6028 Y2.0607
G01 X-19.7253 Y1.5080
G01 X-19.7325 Y1.4741
G01 X-19.7388 Y1.4401
G01 X-19.7464 Y1.3950
G01 X-19.7513 Y1.3608
G01 X-19.8252 Y0.7994
G01 X-19.8293 Y0.7650
G01 X-19.8337 Y0.7195
G01 X-19.8363 Y0.6850
G01 X-19.8382 Y0.6504
G01 X-19.8629 Y0.0848
G01 X-19.8645 Y0.0273
G01 X-19.8646 Y-0.0100
G01 X-19.8639 Y-0.0544
G00 X-19.8639 Y-0.0544
G01 F10.00
G01 Z-1.6000
G01 F10.00
G01 X-19.8629 Y-0.0848
G01 X-19.8382 Y-0.6504
G01 X-19.8363 Y-0.6850
G01 X-19.8327 Y-0.7305
G01 X-19.8293 Y-0.7650
G01 X-19.8252 Y-0.7994
G01 X-19.7513 Y-1.3608
G01 X-19.7464 Y-1.3950
G01 X-19.7408 Y-1.4292
G01 X-19.7325 Y-1.4741
G01 X-19.7253 Y-1.5080
G01 X-19.6028 Y-2.0607
G01 X-19.5950 Y-2.0945
G01 X-19.5864 Y-2.1281
G01 X-19.5741 Y-2.1722
G01 X-19.5641 Y-2.2054
G01 X-19.3938 Y-2.7454
G01 X-19.3830 Y-2.7783
G01 X-19.3677 Y-2.8214
G01 X-19.3555 Y-2.8538
G01 X-19.3426 Y-2.8859
G01 X-19.1259 Y-3.4090
G01 X-19.1123 Y-3.4408
G01 X-19.0981 Y-3.4723
G01 X-19.0784 Y-3.5136
G01 X-19.0628 Y-3.5444
G01 X-18.8014 Y-4.0466
G01 X-18.7850 Y-4.0772
G01 X-18.7625 Y-4.1170
G01 X-18.7449 Y-4.1468
G01 X-18.7265 Y-4.1762
G01 X-18.4223 Y-4.6537
G01 X-18.4034 Y-4.6827
G01 X-18.3775 Y-4.7204
G01 X-18.3573 Y-4.7486
G01 X-18.3365 Y-4.7763
G01 X-17.9918 Y-5.2255
G01 X-17.9705 Y-5.2527
G01 X-17.9485 Y-5.2795
G01 X-17.9188 Y-5.3143
G01 X-17.8957 Y-5.3401
G01 X-17.5132 Y-5.7575
G01 X-17.4895 Y-5.7827
G01 X-17.4653 Y-5.8075
G01 X-17.4327 Y-5.8395
G01 X-17.4075 Y-5.8632
G01 X-16.9901 Y-6.2457
G01 X-16.9643 Y-6.2688
G01 X-16.9380 Y-6.2913
G01 X-16.9027 Y-6.3204
G01 X-16.8755 Y-6.3418
G01 X-16.4263 Y-6.6865
G01 X-16.3986 Y-6.7073
G01 X-16.3613 Y-6.7339
G01 X-16.3327 Y-6.7534
G01 X-16.3037 Y-6.7723
G01 X-15.8262 Y-7.0765
G01 X-15.7968 Y-7.0948
G01 X-15.7574 Y-7.1180
G01 X-15.7272 Y-7.1350
G01 X-15.6966 Y-7.1514
G01 X-15.1944 Y-7.4128
G01 X-15.1636 Y-7.4284
G01 X-15.1223 Y-7.4481
G01 X-15.0908 Y-7.4623
G01 X-15.0590 Y-7.4759
G01 X-14.5359 Y-7.6926
G01 X-14.5038 Y-7.7055
G01 X-14.4610 Y-7.7215
G01 X-14.4283 Y-7.7330
G01 X-14.3954 Y-7.7438
G01 X-13.8554 Y-7.9141
G01 X-13.8222 Y-7.9241
G01 X-13.7888 Y-7.9335
G01 X-13.7445 Y-7.9450
G01 X-13.7107 Y-7.9528
G01 X-13.1580 Y-8.0753
G01 X-13.1241 Y-8.0825
G01 X-13.0901 Y-8.0888
G01 X-13.0450 Y-8.0964
G01 X-13.0108 Y-8.1013
G01 X-12.4494 Y-8.1752
G01 X-12.4150 Y-8.1793
G01 X-12.3805 Y-8.1827
G01 X-12.3350 Y-8.1863
G01 X-12.3004 Y-8.1882
G01 X-11.7175 Y-8.2136
G01 X-11.6600 Y-8.2146
G01 X2.7700 Y-8.2146
G01 X2.8046 Y-8.2142
G01 X2.8448 Y-8.2129
G01 X3.4104 Y-8.1882
G01 X3.4450 Y-8.1863
G01 X3.4905 Y-8.1827
G01 X3.5250 Y-8.1793
G01 X3.5594 Y-8.1752
G01 X4.1208 Y-8.1013
G01 X4.1550 Y-8.0964
G01 X4.1892 Y-8.0908
G01 X4.2341 Y-8.0825
G01 X4.2680 Y-8.0753
G01 X4.8207 Y-7.9528
G01 X4.8545 Y-7.9450
G01 X4.8988 Y-7.9335
G01 X4.9322 Y-7.9241
G01 X4.9654 Y-7.9141
G01 X5.5054 Y-7.7438
G01 X5.5383 Y-7.7330
G01 X5.5710 Y-7.7215
G01 X5.6138 Y-7.7055
G01 X5.6459 Y-7.6926
G01 X6.1690 Y-7.4759
G01 X6.2008 Y-7.4623
G01 X6.2424 Y-7.4434
G01 X6.2736 Y-7.4284
G01 X6.3044 Y-7.4128
G01 X6.8066 Y-7.1514
G01 X6.8372 Y-7.1350
G01 X6.8674 Y-7.1180
G01 X6.9068 Y-7.0948
G01 X6.9362 Y-7.0765
G01 X7.4137 Y-6.7723
G01 X7.4427 Y-6.7534
G01 X7.4713 Y-6.7339
G01 X7.5086 Y-6.7073
G01 X7.5363 Y-6.6865
G01 X7.9855 Y-6.3418
G01 X8.0127 Y-6.3204
G01 X8.0395 Y-6.2985
G01 X8.0743 Y-6.2688
G01 X8.1001 Y-6.2457
G01 X8.5175 Y-5.8632
G01 X8.5427 Y-5.8395
G01 X8.5753 Y-5.8075
G01 X8.5995 Y-5.7827
G01 X8.6232 Y-5.7575
G01 X9.0057 Y-5.3401
G01 X9.0288 Y-5.3143
G01 X9.0585 Y-5.2795
G01 X9.0804 Y-5.2527
G01 X9.1018 Y-5.2255
G01 X9.4465 Y-4.7763
G01 X9.4673 Y-4.7486
G01 X9.4875 Y-4.7204
G01 X9.5134 Y-4.6827
G01 X9.5323 Y-4.6537
G01 X9.8365 Y-4.1762
G01 X9.8548 Y-4.1468
G01 X9.8725 Y-4.1170
G01 X9.8950 Y-4.0772
G01 X9.9114 Y-4.0466
G01 X10.1728 Y-3.5444
G01 X10.1884 Y-3.5136
G01 X10.2081 Y-3.4723
G01 X10.2223 Y-3.4408
G01 X10.2359 Y-3.4090
G01 X10.4526 Y-2.8859
G01 X10.4655 Y-2.8538
G01 X10.4777 Y-2.8214
G01 X10.4930 Y-2.7783
G01 X10.5038 Y-2.7454
G01 X10.6741 Y-2.2054
G01 X10.6841 Y-2.1722
G01 X10.6935 Y-2.1388
G01 X10.7050 Y-2.0945
G01 X10.7128 Y-2.0607
G01 X10.8353 Y-1.5080
G01 X10.8425 Y-1.4741
G01 X10.8488 Y-1.4401
G01 X10.8564 Y-1.3950
G01 X10.8613 Y-1.3608
G01 X10.9352 Y-0.7994
G01 X10.9393 Y-0.7650
G01 X10.9437 Y-0.7195
G01 X10.9463 Y-0.6850
G01 X10.9482 Y-0.6504
G01 X10.9729 Y-0.0848
G01 X10.9742 Y-0.0446
G01 X10.9746 Y0.0100
G01 X10.9741 Y0.0502
G01 X10.9729 Y0.0848
G01 X10.9482 Y0.6504
G01 X10.9463 Y0.6850
G01 X10.9437 Y0.7195
G01 X10.9393 Y0.7650
G01 X10.9352 Y0.7994
G01 X10.8613 Y1.3608
G01 X10.8564 Y1.3950
G01 X10.8508 Y1.4292
G01 X10.8425 Y1.4741
G01 X10.8353 Y1.5080
G01 X10.7128 Y2.0607
G01 X10.7050 Y2.0945
G01 X10.6964 Y2.1281
G01 X10.6841 Y2.1722
G01 X10.6741 Y2.2054
G01 X10.5038 Y2.7454
G01 X10.4930 Y2.7783
G01 X10.4777 Y2.8214
G01 X10.4655 Y2.8538
G01 X10.4526 Y2.8859
G01 X10.2359 Y3.4090
G01 X10.2223 Y3.4408
G01 X10.2081 Y3.4723
G01 X10.1884 Y3.5136
G01 X10.1728 Y3.5444
G01 X9.9114 Y4.0466
G01 X9.8950 Y4.0772
G01 X9.8725 Y4.1170
G01 X9.8548 Y4.1468
G01 X9.8365 Y4.1762
G01 X9.5323 Y4.6537
G01 X9.5134 Y4.6827
G01 X9.4875 Y4.7204
G01 X9.4673 Y4.7486
G01 X9.4465 Y4.7763
G01 X9.1018 Y5.2255
G01 X9.0804 Y5.2527
G01 X9.0585 Y5.2795
G01 X9.0288 Y5.3143
G01 X9.0057 Y5.3401
G01 X8.6232 Y5.7575
G01 X8.5995 Y5.7827
G01 X8.5753 Y5.8075
G01 X8.5427 Y5.8395
G01 X8.5175 Y5.8632
G01 X8.1001 Y6.2457
G01 X8.0743 Y6.2688
G01 X8.0480 Y6.2913
G01 X8.0127 Y6.3204
G01 X7.9855 Y6.3418
G01 X7.5363 Y6.6865
G01 X7.5086 Y6.7073
G01 X7.4713 Y6.7339
G01 X7.4427 Y6.7534
G01 X7.4137 Y6.7723
G01 X6.9362 Y7.0765
G01 X6.9068 Y7.0948
G01 X6.8674 Y7.1180
G01 X6.8372 Y7.1350
G01 X6.8066 Y7.1514
G01 X6.3044 Y7.4128
G01 X6.2736 Y7.4284
G01 X6.2323 Y7.4481
G01 X6.2008 Y7.4623
G01 X6.1690 Y7.4759
G01 X5.6459 Y7.6926
G01 X5.6138 Y7.7055
G01 X5.5710 Y7.7215
G01 X5.5383 Y7.7330
G01 X5.5054 Y7.7438
G01 X4.9654 Y7.9141
G01 X4.9322 Y7.9241
G01 X4.8988 Y7.9335
G01 X4.8545 Y7.9450
G01 X4.8207 Y7.9528
G01 X4.2680 Y8.0753
G01 X4.2341 Y8.0825
G01 X4.2001 Y8.0888
G01 X4.1550 Y8.0964
G01 X4.1208 Y8.1013
G01 X3.5594 Y8.1752
G01 X3.5250 Y8.1793
G01 X3.4905 Y8.1827
G01 X3.4450 Y8.1863
G01 X3.4104 Y8.1882
G01 X2.8275 Y8.2136
G01 X2.7700 Y8.2146
G01 X-11.6600 Y8.2146
G01 X-11.6946 Y8.2142
G01 X-11.7348 Y8.2129
G01 X-12.3004 Y8.1882
G01 X-12.3350 Y8.1863
G01 X-12.3805 Y8.1827
G01 X-12.4150 Y8.1793
G01 X-12.4494 Y8.1752
G01 X-13.0108 Y8.1013
G01 X-13.0450 Y8.0964
G01 X-13.0792 Y8.0908
G01 X-13.1241 Y8.0825
G01 X-13.1580 Y8.0753
G01 X-13.7107 Y7.9528
G01 X-13.7445 Y7.9450
G01 X-13.7781 Y7.9364
G01 X-13.8222 Y7.9241
G01 X-13.8554 Y7.9141
G01 X-14.3954 Y7.7438
G01 X-14.4283 Y7.7330
G01 X-14.4610 Y7.7215
G01 X-14.5038 Y7.7055
G01 X-14.5359 Y7.6926
G01 X-15.0590 Y7.4759
G01 X-15.0908 Y7.4623
G01 X-15.1324 Y7.4434
G01 X-15.1636 Y7.4284
G01 X-15.1944 Y7.4128
G01 X-15.6966 Y7.1514
G01 X-15.7272 Y7.1350
G01 X-15.7574 Y7.1180
G01 X-15.7968 Y7.0948
G01 X-15.8262 Y7.0765
G01 X-16.3037 Y6.7723
G01 X-16.3327 Y6.7534
G01 X-16.3613 Y6.7339
G01 X-16.3986 Y6.7073
G01 X-16.4263 Y6.6865
G01 X-16.8755 Y6.3418
G01 X-16.9027 Y6.3204
G01 X-16.9295 Y6.2985
G01 X-16.9643 Y6.2688
G01 X-16.9901 Y6.2457
G01 X-17.4075 Y5.8632
G01 X-17.4327 Y5.8395
G01 X-17.4653 Y5.8075
G01 X-17.4895 Y5.7827
G01 X-17.5132 Y5.7575
G01 X-17.8957 Y5.3401
G01 X-17.9188 Y5.3143
G01 X-17.9485 Y5.2795
G01 X-17.9705 Y5.2527
G01 X-17.9918 Y5.2255
G01 X-18.3365 Y4.7763
G01 X-18.3573 Y4.7486
G01 X-18.3775 Y4.7204
G01 X-18.4034 Y4.6827
G01 X-18.4223 Y4.6537
G01 X-18.7265 Y4.1762
G01 X-18.7449 Y4.1468
G01 X-18.7680 Y4.1074
G01 X-18.7850 Y4.0772
G01 X-18.8014 Y4.0466
G01 X-19.0628 Y3.5444
G01 X-19.0784 Y3.5136
G01 X-19.0981 Y3.4723
G01 X-19.1123 Y3.4408
G01 X-19.1259 Y3.4090
G01 X-19.3426 Y2.8859
G01 X-19.3555 Y2.8538
G01 X-19.3677 Y2.8214
G01 X-19.3830 Y2.7783
G01 X-19.3938 Y2.7454
G01 X-19.5641 Y2.2054
G01 X-19.5741 Y2.1722
G01 X-19.5835 Y2.1388
G01 X-19.5950 Y2.0945
G01 X-19.6028 Y2.0607
G01 X-19.7253 Y1.5080
G01 X-19.7325 Y1.4741
G01 X-19.7388 Y1.4401
G01 X-19.7464 Y1.3950
G01 X-19.7513 Y1.3608
G01 X-19.8252 Y0.7994
G01 X-19.8293 Y0.7650
G01 X-19.8337 Y0.7195
G01 X-19.8363 Y0.6850
G01 X-19.8382 Y0.6504
G01 X-19.8629 Y0.0848
G01 X-19.8645 Y0.0273
G01 X-19.8646 Y-0.0100
G01 X-19.8639 Y-0.0544
G00 X-19.8639 Y-0.0544
G01 F10.00
G01 Z-1.7500
G01 F10.00
G01 X-19.8629 Y-0.0848
G01 X-19.8382 Y-0.6504
G01 X-19.8363 Y-0.6850
G01 X-19.8327 Y-0.7305
G01 X-19.8293 Y-0.7650
G01 X-19.8252 Y-0.7994
G01 X-19.7513 Y-1.3608
G01 X-19.7464 Y-1.3950
G01 X-19.7408 Y-1.4292
G01 X-19.7325 Y-1.4741
G01 X-19.7253 Y-1.5080
G01 X-19.6028 Y-2.0607
G01 X-19.5950 Y-2.0945
G01 X-19.5864 Y-2.1281
G01 X-19.5741 Y-2.1722
G01 X-19.5641 Y-2.2054
G01 X-19.3938 Y-2.7454
G01 X-19.3830 Y-2.7783
G01 X-19.3677 Y-2.8214
G01 X-19.3555 Y-2.8538
G01 X-19.3426 Y-2.8859
G01 X-19.1259 Y-3.4090
G01 X-19.1123 Y-3.4408
G01 X-19.0981 Y-3.4723
G01 X-19.0784 Y-3.5136
G01 X-19.0628 Y-3.5444
G01 X-18.8014 Y-4.0466
G01 X-18.7850 Y-4.0772
G01 X-18.7625 Y-4.1170
G01 X-18.7449 Y-4.1468
G01 X-18.7265 Y-4.1762
G01 X-18.4223 Y-4.6537
G01 X-18.4034 Y-4.6827
G01 X-18.3775 Y-4.7204
G01 X-18.3573 Y-4.7486
G01 X-18.3365 Y-4.7763
G01 X-17.9918 Y-5.2255
G01 X-17.9705 Y-5.2527
G01 X-17.9485 Y-5.2795
G01 X-17.9188 Y-5.3143
G01 X-17.8957 Y-5.3401
G01 X-17.5132 Y-5.7575
G01 X-17.4895 Y-5.7827
G01 X-17.4653 Y-5.8075
G01 X-17.4327 Y-5.8395
G01 X-17.4075 Y-5.8632
G01 X-16.9901 Y-6.2457
G01 X-16.9643 Y-6.2688
G01 X-16.9380 Y-6.2913
G01 X-16.9027 Y-6.3204
G01 X-16.8755 Y-6.3418
G01 X-16.4263 Y-6.6865
G01 X-16.3986 Y-6.7073
G01 X-16.3613 Y-6.7339
G01 X-16.3327 Y-6.7534
G01 X-16.3037 Y-6.7723
G01 X-15.8262 Y-7.0765
G01 X-15.7968 Y-7.0948
G01 X-15.7574 Y-7.1180
G01 X-15.7272 Y-7.1350
G01 X-15.6966 Y-7.1514
G01 X-15.1944 Y-7.4128
G01 X-15.1636 Y-7.4284
G01 X-15.1223 Y-7.4481
G01 X-15.0908 Y-7.4623
G01 X-15.0590 Y-7.4759
G01 X-14.5359 Y-7.6926
G01 X-14.5038 Y-7.7055
G01 X-14.4610 Y-7.7215
G01 X-14.4283 Y-7.7330
G01 X-14.3954 Y-7.7438
G01 X-13.8554 Y-7.9141
G01 X-13.8222 Y-7.9241
G01 X-13.7888 Y-7.9335
G01 X-13.7445 Y-7.9450
G01 X-13.7107 Y-7.9528
G01 X-13.1580 Y-8.0753
G01 X-13.1241 Y-8.0825
G01 X-13.0901 Y-8.0888
G01 X-13.0450 Y-8.0964
G01 X-13.0108 Y-8.1013
G01 X-12.4494 Y-8.1752
G01 X-12.4150 Y-8.1793
G01 X-12.3805 Y-8.1827
G01 X-12.3350 Y-8.1863
G01 X-12.3004 Y-8.1882
G01 X-11.7175 Y-8.2136
G01 X-11.6600 Y-8.2146
G01 X2.7700 Y-8.2146
G01 X2.8046 Y-8.2142
G01 X2.8448 Y-8.2129
G01 X3.4104 Y-8.1882
G01 X3.4450 Y-8.1863
G01 X3.4905 Y-8.1827
G01 X3.5250 Y-8.1793
G01 X3.5594 Y-8.1752
G01 X4.1208 Y-8.1013
G01 X4.1550 Y-8.0964
G01 X4.1892 Y-8.0908
G01 X4.2341 Y-8.0825
G01 X4.2680 Y-8.0753
G01 X4.8207 Y-7.9528
G01 X4.8545 Y-7.9450
G01 X4.8988 Y-7.9335
G01 X4.9322 Y-7.9241
G01 X4.9654 Y-7.9141
G01 X5.5054 Y-7.7438
G01 X5.5383 Y-7.7330
G01 X5.5710 Y-7.7215
G01 X5.6138 Y-7.7055
G01 X5.6459 Y-7.6926
G01 X6.1690 Y-7.4759
G01 X6.2008 Y-7.4623
G01 X6.2424 Y-7.4434
G01 X6.2736 Y-7.4284
G01 X6.3044 Y-7.4128
G01 X6.8066 Y-7.1514
G01 X6.8372 Y-7.1350
G01 X6.8674 Y-7.1180
G01 X6.9068 Y-7.0948
G01 X6.9362 Y-7.0765
G01 X7.4137 Y-6.7723
G01 X7.4427 Y-6.7534
G01 X7.4713 Y-6.7339
G01 X7.5086 Y-6.7073
G01 X7.5363 Y-6.6865
G01 X7.9855 Y-6.3418
G01 X8.0127 Y-6.3204
G01 X8.0395 Y-6.2985
G01 X8.0743 Y-6.2688
G01 X8.1001 Y-6.2457
G01 X8.5175 Y-5.8632
G01 X8.5427 Y-5.8395
G01 X8.5753 Y-5.8075
G01 X8.5995 Y-5.7827
G01 X8.6232 Y-5.7575
G01 X9.0057 Y-5.3401
G01 X9.0288 Y-5.3143
G01 X9.0585 Y-5.2795
G01 X9.0804 Y-5.2527
G01 X9.1018 Y-5.2255
G01 X9.4465 Y-4.7763
G01 X9.4673 Y-4.7486
G01 X9.4875 Y-4.7204
G01 X9.5134 Y-4.6827
G01 X9.5323 Y-4.6537
G01 X9.8365 Y-4.1762
G01 X9.8548 Y-4.1468
G01 X9.8725 Y-4.1170
G01 X9.8950 Y-4.0772
G01 X9.9114 Y-4.0466
G01 X10.1728 Y-3.5444
G01 X10.1884 Y-3.5136
G01 X10.2081 Y-3.4723
G01 X10.2223 Y-3.4408
G01 X10.2359 Y-3.4090
G01 X10.4526 Y-2.8859
G01 X10.4655 Y-2.8538
G01 X10.4777 Y-2.8214
G01 X10.4930 Y-2.7783
G01 X10.5038 Y-2.7454
G01 X10.6741 Y-2.2054
G01 X10.6841 Y-2.1722
G01 X10.6935 Y-2.1388
G01 X10.7050 Y-2.0945
G01 X10.7128 Y-2.0607
G01 X10.8353 Y-1.5080
G01 X10.8425 Y-1.4741
G01 X10.8488 Y-1.4401
G01 X10.8564 Y-1.3950
G01 X10.8613 Y-1.3608
G01 X10.9352 Y-0.7994
G01 X10.9393 Y-0.7650
G01 X10.9437 Y-0.7195
G01 X10.9463 Y-0.6850
G01 X10.9482 Y-0.6504
G01 X10.9729 Y-0.0848
G01 X10.9742 Y-0.0446
G01 X10.9746 Y0.0100
G01 X10.9741 Y0.0502
G01 X10.9729 Y0.0848
G01 X10.9482 Y0.6504
G01 X10.9463 Y0.6850
G01 X10.9437 Y0.7195
G01 X10.9393 Y0.7650
G01 X10.9352 Y0.7994
G01 X10.8613 Y1.3608
G01 X10.8564 Y1.3950
G01 X10.8508 Y1.4292
G01 X10.8425 Y1.4741
G01 X10.8353 Y1.5080
G01 X10.7128 Y2.0607
G01 X10.7050 Y2.0945
G01 X10.6964 Y2.1281
G01 X10.6841 Y2.1722
G01 X10.6741 Y2.2054
G01 X10.5038 Y2.7454
G01 X10.4930 Y2.7783
G01 X10.4777 Y2.8214
G01 X10.4655 Y2.8538
G01 X10.4526 Y2.8859
G01 X10.2359 Y3.4090
G01 X10.2223 Y3.4408
G01 X10.2081 Y3.4723
G01 X10.1884 Y3.5136
G01 X10.1728 Y3.5444
G01 X9.9114 Y4.0466
G01 X9.8950 Y4.0772
G01 X9.8725 Y4.1170
G01 X9.8548 Y4.1468
G01 X9.8365 Y4.1762
G01 X9.5323 Y4.6537
G01 X9.5134 Y4.6827
G01 X9.4875 Y4.7204
G01 X9.4673 Y4.7486
G01 X9.4465 Y4.7763
G01 X9.1018 Y5.2255
G01 X9.0804 Y5.2527
G01 X9.0585 Y5.2795
G01 X9.0288 Y5.3143
G01 X9.0057 Y5.3401
G01 X8.6232 Y5.7575
G01 X8.5995 Y5.7827
G01 X8.5753 Y5.8075
G01 X8.5427 Y5.8395
G01 X8.5175 Y5.8632
G01 X8.1001 Y6.2457
G01 X8.0743 Y6.2688
G01 X8.0480 Y6.2913
G01 X8.0127 Y6.3204
G01 X7.9855 Y6.3418
G01 X7.5363 Y6.6865
G01 X7.5086 Y6.7073
G01 X7.4713 Y6.7339
G01 X7.4427 Y6.7534
G01 X7.4137 Y6.7723
G01 X6.9362 Y7.0765
G01 X6.9068 Y7.0948
G01 X6.8674 Y7.1180
G01 X6.8372 Y7.1350
G01 X6.8066 Y7.1514
G01 X6.3044 Y7.4128
G01 X6.2736 Y7.4284
G01 X6.2323 Y7.4481
G01 X6.2008 Y7.4623
G01 X6.1690 Y7.4759
G01 X5.6459 Y7.6926
G01 X5.6138 Y7.7055
G01 X5.5710 Y7.7215
G01 X5.5383 Y7.7330
G01 X5.5054 Y7.7438
G01 X4.9654 Y7.9141
G01 X4.9322 Y7.9241
G01 X4.8988 Y7.9335
G01 X4.8545 Y7.9450
G01 X4.8207 Y7.9528
G01 X4.2680 Y8.0753
G01 X4.2341 Y8.0825
G01 X4.2001 Y8.0888
G01 X4.1550 Y8.0964
G01 X4.1208 Y8.1013
G01 X3.5594 Y8.1752
G01 X3.5250 Y8.1793
G01 X3.4905 Y8.1827
G01 X3.4450 Y8.1863
G01 X3.4104 Y8.1882
G01 X2.8275 Y8.2136
G01 X2.7700 Y8.2146
G01 X-11.6600 Y8.2146
G01 X-11.6946 Y8.2142
G01 X-11.7348 Y8.2129
G01 X-12.3004 Y8.1882
G01 X-12.3350 Y8.1863
G01 X-12.3805 Y8.1827
G01 X-12.4150 Y8.1793
G01 X-12.4494 Y8.1752
G01 X-13.0108 Y8.1013
G01 X-13.0450 Y8.0964
G01 X-13.0792 Y8.0908
G01 X-13.1241 Y8.0825
G01 X-13.1580 Y8.0753
G01 X-13.7107 Y7.9528
G01 X-13.7445 Y7.9450
G01 X-13.7781 Y7.9364
G01 X-13.8222 Y7.9241
G01 X-13.8554 Y7.9141
G01 X-14.3954 Y7.7438
G01 X-14.4283 Y7.7330
G01 X-14.4610 Y7.7215
G01 X-14.5038 Y7.7055
G01 X-14.5359 Y7.6926
G01 X-15.0590 Y7.4759
G01 X-15.0908 Y7.4623
G01 X-15.1324 Y7.4434
G01 X-15.1636 Y7.4284
G01 X-15.1944 Y7.4128
G01 X-15.6966 Y7.1514
G01 X-15.7272 Y7.1350
G01 X-15.7574 Y7.1180
G01 X-15.7968 Y7.0948
G01 X-15.8262 Y7.0765
G01 X-16.3037 Y6.7723
G01 X-16.3327 Y6.7534
G01 X-16.3613 Y6.7339
G01 X-16.3986 Y6.7073
G01 X-16.4263 Y6.6865
G01 X-16.8755 Y6.3418
G01 X-16.9027 Y6.3204
G01 X-16.9295 Y6.2985
G01 X-16.9643 Y6.2688
G01 X-16.9901 Y6.2457
G01 X-17.4075 Y5.8632
G01 X-17.4327 Y5.8395
G01 X-17.4653 Y5.8075
G01 X-17.4895 Y5.7827
G01 X-17.5132 Y5.7575
G01 X-17.8957 Y5.3401
G01 X-17.9188 Y5.3143
G01 X-17.9485 Y5.2795
G01 X-17.9705 Y5.2527
G01 X-17.9918 Y5.2255
G01 X-18.3365 Y4.7763
G01 X-18.3573 Y4.7486
G01 X-18.3775 Y4.7204
G01 X-18.4034 Y4.6827
G01 X-18.4223 Y4.6537
G01 X-18.7265 Y4.1762
G01 X-18.7449 Y4.1468
G01 X-18.7680 Y4.1074
G01 X-18.7850 Y4.0772
G01 X-18.8014 Y4.0466
G01 X-19.0628 Y3.5444
G01 X-19.0784 Y3.5136
G01 X-19.0981 Y3.4723
G01 X-19.1123 Y3.4408
G01 X-19.1259 Y3.4090
G01 X-19.3426 Y2.8859
G01 X-19.3555 Y2.8538
G01 X-19.3677 Y2.8214
G01 X-19.3830 Y2.7783
G01 X-19.3938 Y2.7454
G01 X-19.5641 Y2.2054
G01 X-19.5741 Y2.1722
G01 X-19.5835 Y2.1388
G01 X-19.5950 Y2.0945
G01 X-19.6028 Y2.0607
G01 X-19.7253 Y1.5080
G01 X-19.7325 Y1.4741
G01 X-19.7388 Y1.4401
G01 X-19.7464 Y1.3950
G01 X-19.7513 Y1.3608
G01 X-19.8252 Y0.7994
G01 X-19.8293 Y0.7650
G01 X-19.8337 Y0.7195
G01 X-19.8363 Y0.6850
G01 X-19.8382 Y0.6504
G01 X-19.8629 Y0.0848
G01 X-19.8645 Y0.0273
G01 X-19.8646 Y-0.0100
G01 X-19.8639 Y-0.0544
G00 Z10.0000
M05
G00 Z10.0000
G00 Z15.00


