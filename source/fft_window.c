float hanning_1024[1024] = {
0,9.430769E-06,3.772272E-05,8.487479E-05,0.0001508852,0.0002357514,0.0003394703,0.0004620379,0.0006034497,0.0007637002,
0.0009427834,0.001140693,0.00135742,0.001592958,0.001847298,0.002120429,0.002412342,0.002723026,0.003052468,0.003400657,
0.003767578,0.00415322,0.004557566,0.004980602,0.005422311,0.005882678,0.006361684,0.006859311,0.007375542,0.007910355,
0.008463732,0.009035652,0.009626091,0.01023503,0.01086244,0.01150831,0.0121726,0.0128553,0.01355637,0.01427579,
0.01501354,0.01576957,0.01654388,0.01733643,0.01814718,0.01897611,0.01982318,0.02068837,0.02157164,0.02247296,
0.02339229,0.0243296,0.02528485,0.02625801,0.02724904,0.02825791,0.02928457,0.03032899,0.03139113,0.03247094,
0.03356839,0.03468344,0.03581603,0.03696614,0.03813372,0.03931872,0.04052109,0.0417408,0.0429778,0.04423204,
0.04550347,0.04679205,0.04809771,0.04942043,0.05076015,0.05211682,0.05349037,0.05488078,0.05628797,0.0577119,
0.05915251,0.06060976,0.06208358,0.06357393,0.06508072,0.06660394,0.0681435,0.06969935,0.07127143,0.07285969,
0.07446406,0.07608448,0.07772089,0.07937323,0.08104144,0.08272546,0.08442521,0.08614065,0.08787169,0.08961828,
0.09138035,0.09315784,0.09495067,0.09675878,0.09858211,0.1004206,0.1022741,0.1041427,0.1060261,0.1079245,
0.1098376,0.1117655,0.1137079,0.115665,0.1176366,0.1196226,0.1216229,0.1236375,0.1256663,0.1277092,
0.1297662,0.1318371,0.133922,0.1360206,0.138133,0.140259,0.1423986,0.1445517,0.1467181,0.148898,
0.151091,0.1532972,0.1555165,0.1577488,0.159994,0.1622521,0.1645228,0.1668063,0.1691023,0.1714107,
0.1737316,0.1760648,0.1784102,0.1807677,0.1831373,0.1855188,0.1879122,0.1903174,0.1927342,0.1951627,
0.1976026,0.2000539,0.2025166,0.2049905,0.2074755,0.2099715,0.2124785,0.2149964,0.2175249,0.2200642,
0.222614,0.2251742,0.2277449,0.2303258,0.2329168,0.235518,0.2381291,0.2407501,0.2433809,0.2460214,
0.2486714,0.2513309,0.2539999,0.256678,0.2593654,0.2620618,0.2647673,0.2674816,0.2702046,0.2729363,
0.2756766,0.2784254,0.2811825,0.2839479,0.2867214,0.289503,0.2922925,0.2950898,0.2978949,0.3007076,
0.3035278,0.3063554,0.3091904,0.3120325,0.3148817,0.3177379,0.320601,0.3234709,0.3263474,0.3292304,
0.3321199,0.3350158,0.3379178,0.340826,0.3437402,0.3466602,0.3495861,0.3525176,0.3554547,0.3583972,
0.3613451,0.3642983,0.3672565,0.3702197,0.3731879,0.3761608,0.3791384,0.3821205,0.3851071,0.3880981,
0.3910932,0.3940925,0.3970957,0.4001029,0.4031138,0.4061283,0.4091465,0.412168,0.4151928,0.4182209,
0.421252,0.4242861,0.4273231,0.4303628,0.4334051,0.4364499,0.4394972,0.4425467,0.4455984,0.4486521,
0.4517078,0.4547653,0.4578245,0.4608853,0.4639475,0.4670112,0.470076,0.473142,0.476209,0.479277,
0.4823456,0.485415,0.4884849,0.4915553,0.4946259,0.4976968,0.5007678,0.5038387,0.5069094,0.50998,
0.5130501,0.5161198,0.5191888,0.5222571,0.5253246,0.5283911,0.5314565,0.5345208,0.5375838,0.5406453,
0.5437053,0.5467637,0.5498203,0.552875,0.5559278,0.5589784,0.5620267,0.5650728,0.5681164,0.5711574,
0.5741957,0.5772313,0.5802639,0.5832936,0.58632,0.5893432,0.592363,0.5953794,0.5983921,0.6014012,
0.6044064,0.6074076,0.6104048,0.613398,0.6163867,0.6193711,0.622351,0.6253263,0.6282968,0.6312625,
0.6342233,0.637179,0.6401294,0.6430747,0.6460145,0.6489488,0.6518776,0.6548005,0.6577176,0.6606289,
0.663534,0.6664329,0.6693256,0.6722119,0.6750917,0.6779649,0.6808314,0.683691,0.6865438,0.6893895,
0.692228,0.6950593,0.6978832,0.7006997,0.7035086,0.7063098,0.7091032,0.7118888,0.7146664,0.7174358,
0.7201971,0.72295,0.7256946,0.7284306,0.731158,0.7338767,0.7365866,0.7392875,0.7419794,0.7446622,
0.7473358,0.75,0.7526548,0.7553,0.7579357,0.7605616,0.7631777,0.7657838,0.7683799,0.7709659,
0.7735417,0.7761072,0.7786622,0.7812068,0.7837407,0.7862639,0.7887763,0.7912778,0.7937684,0.7962478,
0.7987161,0.8011732,0.8036188,0.806053,0.8084757,0.8108867,0.8132859,0.8156734,0.818049,0.8204126,
0.822764,0.8251033,0.8274304,0.8297451,0.8320473,0.834337,0.8366141,0.8388786,0.8411302,0.8433689,
0.8455948,0.8478075,0.8500072,0.8521936,0.8543668,0.8565266,0.8586729,0.8608057,0.8629249,0.8650304,
0.8671222,0.8692001,0.871264,0.873314,0.8753499,0.8773715,0.879379,0.8813722,0.883351,0.8853154,
0.8872651,0.8892003,0.8911208,0.8930265,0.8949175,0.8967935,0.8986545,0.9005005,0.9023315,0.9041472,
0.9059477,0.9077328,0.9095026,0.911257,0.9129958,0.914719,0.9164266,0.9181185,0.9197946,0.9214549,
0.9230993,0.9247277,0.9263402,0.9279364,0.9295166,0.9310806,0.9326283,0.9341597,0.9356747,0.9371733,
0.9386554,0.9401209,0.9415699,0.9430022,0.9444177,0.9458165,0.9471985,0.9485636,0.9499118,0.951243,
0.9525573,0.9538544,0.9551344,0.9563972,0.9576429,0.9588712,0.9600823,0.9612759,0.9624522,0.9636111,
0.9647524,0.9658763,0.9669825,0.9680712,0.9691421,0.9701954,0.971231,0.9722487,0.9732487,0.9742308,
0.975195,0.9761413,0.9770696,0.97798,0.9788722,0.9797465,0.9806026,0.9814406,0.9822605,0.9830621,
0.9838455,0.9846107,0.9853576,0.9860862,0.9867964,0.9874883,0.9881619,0.9888169,0.9894536,0.9900718,
0.9906715,0.9912526,0.9918153,0.9923594,0.9928849,0.9933918,0.9938802,0.9943498,0.9948009,0.9952332,
0.995647,0.996042,0.9964182,0.9967758,0.9971146,0.9974347,0.997736,0.9980185,0.9982822,0.9985272,
0.9987533,0.9989606,0.9991491,0.9993188,0.9994696,0.9996016,0.9997147,0.999809,0.9998845,0.9999411,
0.9999788,0.9999976,0.9999976,0.9999788,0.9999411,0.9998845,0.999809,0.9997147,0.9996016,0.9994696,
0.9993188,0.9991491,0.9989606,0.9987533,0.9985272,0.9982822,0.9980185,0.997736,0.9974347,0.9971146,
0.9967758,0.9964182,0.996042,0.995647,0.9952332,0.9948009,0.9943498,0.9938802,0.9933918,0.9928849,
0.9923594,0.9918153,0.9912526,0.9906715,0.9900718,0.9894536,0.9888169,0.9881619,0.9874883,0.9867964,
0.9860862,0.9853576,0.9846107,0.9838455,0.9830621,0.9822605,0.9814406,0.9806026,0.9797465,0.9788722,
0.97798,0.9770696,0.9761413,0.975195,0.9742308,0.9732487,0.9722487,0.971231,0.9701954,0.9691421,
0.9680712,0.9669825,0.9658763,0.9647524,0.9636111,0.9624522,0.9612759,0.9600823,0.9588712,0.9576429,
0.9563972,0.9551344,0.9538544,0.9525573,0.951243,0.9499118,0.9485636,0.9471985,0.9458165,0.9444177,
0.9430022,0.9415699,0.9401209,0.9386554,0.9371733,0.9356747,0.9341597,0.9326283,0.9310806,0.9295166,
0.9279364,0.9263402,0.9247277,0.9230993,0.9214549,0.9197946,0.9181185,0.9164266,0.914719,0.9129958,
0.911257,0.9095026,0.9077328,0.9059477,0.9041472,0.9023315,0.9005005,0.8986545,0.8967935,0.8949175,
0.8930265,0.8911208,0.8892003,0.8872651,0.8853154,0.883351,0.8813722,0.879379,0.8773715,0.8753499,
0.873314,0.871264,0.8692001,0.8671222,0.8650304,0.8629249,0.8608057,0.8586729,0.8565266,0.8543668,
0.8521936,0.8500072,0.8478075,0.8455948,0.8433689,0.8411302,0.8388786,0.8366141,0.834337,0.8320473,
0.8297451,0.8274304,0.8251033,0.822764,0.8204126,0.818049,0.8156734,0.8132859,0.8108867,0.8084757,
0.806053,0.8036188,0.8011732,0.7987161,0.7962478,0.7937684,0.7912778,0.7887763,0.7862639,0.7837407,
0.7812068,0.7786622,0.7761072,0.7735417,0.7709659,0.7683799,0.7657838,0.7631777,0.7605616,0.7579357,
0.7553,0.7526548,0.75,0.7473358,0.7446622,0.7419794,0.7392875,0.7365866,0.7338767,0.731158,
0.7284306,0.7256946,0.72295,0.7201971,0.7174358,0.7146664,0.7118888,0.7091032,0.7063098,0.7035086,
0.7006997,0.6978832,0.6950593,0.692228,0.6893895,0.6865438,0.683691,0.6808314,0.6779649,0.6750917,
0.6722119,0.6693256,0.6664329,0.663534,0.6606289,0.6577176,0.6548005,0.6518776,0.6489488,0.6460145,
0.6430747,0.6401294,0.637179,0.6342233,0.6312625,0.6282968,0.6253263,0.622351,0.6193711,0.6163867,
0.613398,0.6104048,0.6074076,0.6044064,0.6014012,0.5983921,0.5953794,0.592363,0.5893432,0.58632,
0.5832936,0.5802639,0.5772313,0.5741957,0.5711574,0.5681164,0.5650728,0.5620267,0.5589784,0.5559278,
0.552875,0.5498203,0.5467637,0.5437053,0.5406453,0.5375838,0.5345208,0.5314565,0.5283911,0.5253246,
0.5222571,0.5191888,0.5161198,0.5130501,0.50998,0.5069094,0.5038387,0.5007678,0.4976968,0.4946259,
0.4915553,0.4884849,0.485415,0.4823456,0.479277,0.476209,0.473142,0.470076,0.4670112,0.4639475,
0.4608853,0.4578245,0.4547653,0.4517078,0.4486521,0.4455984,0.4425467,0.4394972,0.4364499,0.4334051,
0.4303628,0.4273231,0.4242861,0.421252,0.4182209,0.4151928,0.412168,0.4091465,0.4061283,0.4031138,
0.4001029,0.3970957,0.3940925,0.3910932,0.3880981,0.3851071,0.3821205,0.3791384,0.3761608,0.3731879,
0.3702197,0.3672565,0.3642983,0.3613451,0.3583972,0.3554547,0.3525176,0.3495861,0.3466602,0.3437402,
0.340826,0.3379178,0.3350158,0.3321199,0.3292304,0.3263474,0.3234709,0.320601,0.3177379,0.3148817,
0.3120325,0.3091904,0.3063554,0.3035278,0.3007076,0.2978949,0.2950898,0.2922925,0.289503,0.2867214,
0.2839479,0.2811825,0.2784254,0.2756766,0.2729363,0.2702046,0.2674816,0.2647673,0.2620618,0.2593654,
0.256678,0.2539999,0.2513309,0.2486714,0.2460214,0.2433809,0.2407501,0.2381291,0.235518,0.2329168,
0.2303258,0.2277449,0.2251742,0.222614,0.2200642,0.2175249,0.2149964,0.2124785,0.2099715,0.2074755,
0.2049905,0.2025166,0.2000539,0.1976026,0.1951627,0.1927342,0.1903174,0.1879122,0.1855188,0.1831373,
0.1807677,0.1784102,0.1760648,0.1737316,0.1714107,0.1691023,0.1668063,0.1645228,0.1622521,0.159994,
0.1577488,0.1555165,0.1532972,0.151091,0.148898,0.1467181,0.1445517,0.1423986,0.140259,0.138133,
0.1360206,0.133922,0.1318371,0.1297662,0.1277092,0.1256663,0.1236375,0.1216229,0.1196226,0.1176366,
0.115665,0.1137079,0.1117655,0.1098376,0.1079245,0.1060261,0.1041427,0.1022741,0.1004206,0.09858211,
0.09675878,0.09495067,0.09315784,0.09138035,0.08961828,0.08787169,0.08614065,0.08442521,0.08272546,0.08104144,
0.07937323,0.07772089,0.07608448,0.07446406,0.07285969,0.07127143,0.06969935,0.0681435,0.06660394,0.06508072,
0.06357393,0.06208358,0.06060976,0.05915251,0.0577119,0.05628797,0.05488078,0.05349037,0.05211682,0.05076015,
0.04942043,0.04809771,0.04679205,0.04550347,0.04423204,0.0429778,0.0417408,0.04052109,0.03931872,0.03813372,
0.03696614,0.03581603,0.03468344,0.03356839,0.03247094,0.03139113,0.03032899,0.02928457,0.02825791,0.02724904,
0.02625801,0.02528485,0.0243296,0.02339229,0.02247296,0.02157164,0.02068837,0.01982318,0.01897611,0.01814718,
0.01733643,0.01654388,0.01576957,0.01501354,0.01427579,0.01355637,0.0128553,0.0121726,0.01150831,0.01086244,
0.01023503,0.009626091,0.009035652,0.008463732,0.007910355,0.007375542,0.006859311,0.006361684,0.005882678,0.005422311,
0.004980602,0.004557566,0.00415322,0.003767578,0.003400657,0.003052468,0.002723026,0.002412342,0.002120429,0.001847298,
0.001592958,0.00135742,0.001140693,0.0009427834,0.0007637002,0.0006034497,0.0004620379,0.0003394703,0.0002357514,0.0001508852,
8.487479E-05,3.772272E-05,9.430769E-06,0,};