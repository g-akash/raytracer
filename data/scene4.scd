(Material matRED
	(color 0.75 0.25 0.25 )
	(ma 0.5 ) (ml 0.6 ) (ms 0.1 ) (msp 150 ) (msm 1 ) (mt 0 ) (mtn 1 ))
(Material matBLUE
	(color 0.25 0.25 0.75 )
	(ma 0.5 ) (ml 0.6 ) (ms 0.1 ) (msp 20 ) (msm 1 ) (mt 0 ) (mtn 1 ))
(Material matGREEN
	(color 0.25 0.75 0.25 )
	(ma 0.5 ) (ml 0.6 ) (ms 0.1 ) (msp 150 ) (msm 1 ) (mt 0 ) (mtn 1 ))
(Material matWHITE
	(color 0.75 0.75 0.75 )
	(ma 0.5 ) (ml 0.6 ) (ms 0.1 ) (msp 40 ) (msm 1 ) (mt 0 ) (mtn 1 ))
(Material matGlass
	(color 0.75 0.75 0.75 )
	(ma 0.0 ) (ml 0.2 ) (ms 0.9 ) (msp 40 ) (msm 1 ) (mt 0 ) (mtn 1))
(Light amb (type 0 ) (color 0.2 0.2 0.2 ) )
(Light pt1 (type 2 ) (color 20.8 20.8 20.8 )  (falloff 1 )  (deaddistance 0.1 ) )
(Camera cam
	(perspective 1 )
	(l -1 ) (r 1 ) (b -1 ) (t 1 ) (n 3 )) #no far plane
(Sphere spTOP (radius 100000.0 ) (material matGREEN ))
(Sphere spWALL (radius 100000.0 ) (material matWHITE ))
(Sphere spLEFT (radius 100000.0 ) (material matBLUE ))
(Sphere spRIGHT (radius 100000.0 ) (material matRED ))
(Sphere spGlass (radius 4 ) (material matGlass ))
(Material helixMat0
	(color 0.0 1.0 0.335001069639 )
	(ma 0.4 ) (ml 0.8 ) (ms 0.5 ) (msp 50 ) (msm 0.5 ) (mt 0 ) (mtn 1 ))
(Sphere sp_0 (radius 1.5 ) (material helixMat0 ))
(Material helixMat1
	(color 0.025 0.975 0.717533103556 )
	(ma 0.4 ) (ml 0.8 ) (ms 0.5 ) (msp 50 ) (msm 0.5 ) (mt 0 ) (mtn 1 ))
(Sphere sp_1 (radius 1.5 ) (material helixMat1 ))
(Material helixMat2
	(color 0.05 0.95 0.549281656954 )
	(ma 0.4 ) (ml 0.8 ) (ms 0.5 ) (msp 50 ) (msm 0.5 ) (mt 0 ) (mtn 1 ))
(Sphere sp_2 (radius 1.5 ) (material helixMat2 ))
(G awesome
	(I cam1 cam )
	(I dir1 dir (Xform 0 0 -0.5 0
		0 0 -0.5 0
		0 0 0.5 0
		0 0 0 0 ) )
	(I amb1 amb )
	(I pt1_I pt1 (T 2 2 -45 ) )
	(I s_0 sp_0 (T 0.0 6.0 -100 ) )
	(I spGlass_I spGlass (T 0 0 -22 ) )
	(I spB_I spWALL (T 0 -100009.0 -3 ) )
	(I spT_I spTOP (T 0 100009.0 -3 ) )
	(I spF_I spWALL (T 0 0 -100100.0 ) )
	(I spN_I spWALL (T 0 0 100000.0 ) )
	(I spL_I spLEFT (T -100009.0 0 -3 ) )
	(I spR_I spRIGHT (T 100009.0 0 -3 ) )
)
(Render awesome )
