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

float hanning_2048[2048] = {
0,2.355395E-06,9.421557E-06,2.119842E-05,3.768587E-05,5.888376E-05,8.479189E-05,0.00011541,0.0001507378,0.000190775,
0.0002355212,0.0002849759,0.0003391388,0.0003980092,0.0004615867,0.0005298706,0.0006028604,0.0006805552,0.0007629544,0.0008500572,
0.0009418628,0.00103837,0.001139579,0.001245487,0.001356095,0.001471401,0.001591403,0.001716102,0.001845495,0.001979581,
0.00211836,0.002261829,0.002409988,0.002562835,0.002720368,0.002882587,0.003049489,0.003221074,0.003397339,0.003578282,
0.003763903,0.003954199,0.004149168,0.00434881,0.004553121,0.0047621,0.004975745,0.005194054,0.005417024,0.005644655,
0.005876943,0.006113886,0.006355483,0.00660173,0.006852626,0.007108169,0.007368355,0.007633183,0.007902649,0.008176751,
0.008455488,0.008738856,0.009026852,0.009319474,0.009616719,0.009918584,0.01022507,0.01053616,0.01085187,0.01117219,
0.01149711,0.01182664,0.01216076,0.01249948,0.01284279,0.0131907,0.01354319,0.01390026,0.01426191,0.01462814,
0.01499894,0.01537431,0.01575425,0.01613875,0.01652781,0.01692143,0.01731959,0.0177223,0.01812956,0.01854136,
0.01895769,0.01937855,0.01980395,0.02023386,0.0206683,0.02110725,0.02155072,0.02199869,0.02245117,0.02290815,
0.02336962,0.02383558,0.02430603,0.02478096,0.02526036,0.02574424,0.02623259,0.0267254,0.02722267,0.0277244,
0.02823057,0.02874119,0.02925625,0.02977574,0.03029967,0.03082802,0.03136079,0.03189797,0.03243957,0.03298557,
0.03353597,0.03409077,0.03464995,0.03521352,0.03578147,0.03635379,0.03693049,0.03751154,0.03809695,0.03868671,
0.03928082,0.03987927,0.04048206,0.04108917,0.04170061,0.04231637,0.04293643,0.04356081,0.04418948,0.04482245,
0.04545971,0.04610125,0.04674706,0.04739715,0.0480515,0.04871011,0.04937297,0.05004008,0.05071142,0.051387,
0.05206681,0.05275083,0.05343907,0.05413152,0.05482817,0.05552901,0.05623404,0.05694325,0.05765663,0.05837419,
0.0590959,0.05982177,0.06055178,0.06128594,0.06202422,0.06276664,0.06351317,0.06426382,0.06501857,0.06577742,
0.06654036,0.06730739,0.06807849,0.06885366,0.0696329,0.07041618,0.07120351,0.07199489,0.07279029,0.07358973,
0.07439318,0.07520064,0.0760121,0.07682756,0.077647,0.07847042,0.07929782,0.08012918,0.08096449,0.08180375,
0.08264696,0.08349409,0.08434515,0.08520012,0.086059,0.08692179,0.08778846,0.08865902,0.08953345,0.09041175,
0.09129391,0.09217992,0.09306978,0.09396346,0.09486097,0.0957623,0.09666744,0.09757638,0.09848911,0.09940562,
0.1003259,0.10125,0.1021778,0.1031093,0.1040446,0.1049836,0.1059264,0.1068729,0.107823,0.1087769,
0.1097344,0.1106956,0.1116605,0.1126291,0.1136013,0.1145771,0.1155566,0.1165397,0.1175264,0.1185167,
0.1195106,0.1205081,0.1215092,0.1225138,0.123522,0.1245337,0.125549,0.1265678,0.1275901,0.1286159,
0.1296452,0.1306781,0.1317143,0.1327541,0.1337973,0.134844,0.1358941,0.1369477,0.1380046,0.139065,
0.1401288,0.141196,0.1422665,0.1433404,0.1444177,0.1454983,0.1465823,0.1476696,0.1487602,0.1498541,
0.1509514,0.1520519,0.1531557,0.1542627,0.155373,0.1564866,0.1576034,0.1587234,0.1598467,0.1609731,
0.1621028,0.1632356,0.1643716,0.1655107,0.1666531,0.1677985,0.1689471,0.1700988,0.1712536,0.1724115,
0.1735725,0.1747366,0.1759037,0.1770739,0.1782471,0.1794234,0.1806026,0.1817849,0.1829702,0.1841585,
0.1853497,0.186544,0.1877411,0.1889412,0.1901443,0.1913502,0.1925591,0.1937709,0.1949855,0.196203,
0.1974234,0.1986467,0.1998727,0.2011016,0.2023334,0.2035679,0.2048052,0.2060453,0.2072882,0.2085338,
0.2097822,0.2110333,0.2122871,0.2135436,0.2148029,0.2160648,0.2173294,0.2185967,0.2198666,0.2211391,
0.2224143,0.2236921,0.2249725,0.2262555,0.2275411,0.2288292,0.2301199,0.2314131,0.2327089,0.2340072,
0.235308,0.2366112,0.237917,0.2392252,0.2405359,0.2418491,0.2431646,0.2444826,0.245803,0.2471258,
0.248451,0.2497785,0.2511084,0.2524407,0.2537752,0.2551121,0.2564513,0.2577928,0.2591366,0.2604827,
0.261831,0.2631815,0.2645343,0.2658893,0.2672465,0.2686059,0.2699675,0.2713312,0.2726971,0.2740652,
0.2754353,0.2768076,0.278182,0.2795585,0.280937,0.2823177,0.2837003,0.2850851,0.2864718,0.2878605,
0.2892513,0.290644,0.2920387,0.2934354,0.294834,0.2962345,0.297637,0.2990413,0.3004476,0.3018557,
0.3032658,0.3046776,0.3060913,0.3075068,0.3089242,0.3103433,0.3117642,0.3131869,0.3146114,0.3160376,
0.3174655,0.3188952,0.3203266,0.3217596,0.3231944,0.3246308,0.3260688,0.3275085,0.3289499,0.3303928,
0.3318373,0.3332835,0.3347311,0.3361804,0.3376312,0.3390835,0.3405373,0.3419926,0.3434495,0.3449078,
0.3463675,0.3478287,0.3492914,0.3507555,0.3522209,0.3536878,0.355156,0.3566256,0.3580966,0.3595689,
0.3610425,0.3625174,0.3639936,0.3654711,0.3669499,0.3684299,0.3699112,0.3713937,0.3728773,0.3743622,
0.3758483,0.3773355,0.378824,0.3803135,0.3818042,0.383296,0.3847889,0.3862828,0.3877779,0.389274,
0.3907711,0.3922693,0.3937685,0.3952687,0.3967699,0.398272,0.3997751,0.4012792,0.4027841,0.40429,
0.4057968,0.4073045,0.4088131,0.4103225,0.4118327,0.4133438,0.4148557,0.4163685,0.417882,0.4193963,
0.4209113,0.4224271,0.4239436,0.4254608,0.4269787,0.4284973,0.4300166,0.4315366,0.4330572,0.4345784,
0.4361002,0.4376227,0.4391457,0.4406693,0.4421935,0.4437182,0.4452434,0.4467692,0.4482954,0.4498222,
0.4513494,0.4528771,0.4544052,0.4559337,0.4574627,0.4589921,0.4605218,0.462052,0.4635824,0.4651133,
0.4666444,0.4681759,0.4697076,0.4712397,0.472772,0.4743046,0.4758374,0.4773705,0.4789037,0.4804372,
0.4819708,0.4835047,0.4850386,0.4865727,0.488107,0.4896413,0.4911758,0.4927103,0.4942449,0.4957795,
0.4973142,0.4988489,0.5003837,0.5019184,0.5034531,0.5049878,0.5065224,0.508057,0.5095915,0.5111259,
0.5126601,0.5141944,0.5157284,0.5172623,0.518796,0.5203296,0.5218629,0.5233961,0.524929,0.5264617,
0.5279942,0.5295264,0.5310583,0.5325899,0.5341212,0.5356522,0.5371829,0.5387132,0.5402431,0.5417727,
0.5433018,0.5448306,0.5463589,0.5478868,0.5494143,0.5509412,0.5524678,0.5539938,0.5555192,0.5570442,
0.5585687,0.5600926,0.5616159,0.5631386,0.5646607,0.5661823,0.5677032,0.5692235,0.5707431,0.572262,
0.5737803,0.5752979,0.5768148,0.5783309,0.5798463,0.581361,0.5828749,0.584388,0.5859003,0.5874118,
0.5889225,0.5904323,0.5919413,0.5934494,0.5949567,0.596463,0.5979685,0.599473,0.6009766,0.6024792,
0.6039808,0.6054816,0.6069812,0.6084799,0.6099776,0.6114742,0.6129698,0.6144643,0.6159577,0.6174501,
0.6189413,0.6204314,0.6219204,0.6234082,0.6248949,0.6263804,0.6278647,0.6293477,0.6308296,0.6323103,
0.6337897,0.6352678,0.6367447,0.6382203,0.6396945,0.6411675,0.6426391,0.6441094,0.6455783,0.6470459,
0.648512,0.6499768,0.6514401,0.6529021,0.6543625,0.6558216,0.6572791,0.6587352,0.6601898,0.6616428,
0.6630944,0.6645445,0.6659929,0.6674398,0.6688851,0.6703289,0.671771,0.6732115,0.6746504,0.6760876,
0.6775232,0.6789571,0.6803893,0.6818199,0.6832486,0.6846757,0.686101,0.6875246,0.6889464,0.6903665,
0.6917847,0.6932012,0.6946158,0.6960285,0.6974395,0.6988485,0.7002558,0.7016611,0.7030645,0.704466,
0.7058656,0.7072632,0.7086589,0.7100526,0.7114444,0.7128341,0.7142218,0.7156076,0.7169912,0.7183729,
0.7197525,0.72113,0.7225055,0.7238788,0.72525,0.7266191,0.7279861,0.7293509,0.7307136,0.7320741,
0.7334324,0.7347885,0.7361424,0.7374941,0.7388434,0.7401906,0.7415355,0.7428782,0.7442185,0.7455566,
0.7468923,0.7482257,0.7495568,0.7508855,0.7522119,0.7535359,0.7548575,0.7561767,0.7574934,0.7588078,
0.7601197,0.7614292,0.7627362,0.7640407,0.7653428,0.7666423,0.7679393,0.7692338,0.7705258,0.7718152,
0.773102,0.7743863,0.775668,0.7769471,0.7782236,0.7794975,0.7807687,0.7820373,0.7833033,0.7845665,
0.7858271,0.787085,0.7883402,0.7895926,0.7908424,0.7920893,0.7933336,0.7945751,0.7958138,0.7970497,
0.7982829,0.7995132,0.8007407,0.8019653,0.8031871,0.8044061,0.8056222,0.8068354,0.8080457,0.8092531,
0.8104576,0.8116592,0.8128578,0.8140535,0.8152462,0.816436,0.8176228,0.8188066,0.8199874,0.8211651,
0.8223399,0.8235116,0.8246803,0.8258458,0.8270084,0.8281679,0.8293242,0.8304775,0.8316276,0.8327746,
0.8339185,0.8350592,0.8361968,0.8373312,0.8384625,0.8395905,0.8407153,0.841837,0.8429554,0.8440706,
0.8451825,0.8462912,0.8473967,0.8484988,0.8495977,0.8506932,0.8517855,0.8528745,0.8539601,0.8550424,
0.8561214,0.857197,0.8582692,0.859338,0.8604035,0.8614656,0.8625243,0.8635795,0.8646314,0.8656797,
0.8667247,0.8677662,0.8688042,0.8698388,0.8708699,0.8718975,0.8729215,0.8739421,0.8749591,0.8759726,
0.8769826,0.877989,0.8789918,0.8799911,0.8809868,0.8819789,0.8829674,0.8839523,0.8849336,0.8859112,
0.8868853,0.8878556,0.8888224,0.8897854,0.8907448,0.8917005,0.8926525,0.8936008,0.8945454,0.8954864,
0.8964235,0.8973569,0.8982866,0.8992125,0.9001347,0.9010531,0.9019677,0.9028786,0.9037856,0.9046888,
0.9055883,0.9064839,0.9073756,0.9082636,0.9091477,0.9100279,0.9109042,0.9117767,0.9126453,0.9135101,
0.914371,0.9152278,0.9160809,0.91693,0.9177752,0.9186164,0.9194537,0.920287,0.9211164,0.9219418,
0.9227632,0.9235806,0.9243941,0.9252036,0.9260091,0.9268105,0.9276079,0.9284013,0.9291906,0.929976,
0.9307572,0.9315344,0.9323075,0.9330766,0.9338416,0.9346025,0.9353593,0.936112,0.9368606,0.9376051,
0.9383454,0.9390817,0.9398137,0.9405417,0.9412655,0.9419851,0.9427006,0.9434119,0.944119,0.944822,
0.9455207,0.9462152,0.9469056,0.9475917,0.9482736,0.9489513,0.9496248,0.950294,0.950959,0.9516197,
0.9522762,0.9529284,0.9535764,0.9542201,0.9548594,0.9554946,0.9561254,0.9567519,0.9573742,0.9579921,
0.9586056,0.9592149,0.9598199,0.9604205,0.9610168,0.9616087,0.9621963,0.9627795,0.9633584,0.9639329,
0.964503,0.9650688,0.9656302,0.9661872,0.9667398,0.967288,0.9678318,0.9683712,0.9689062,0.9694367,
0.9699628,0.9704846,0.9710019,0.9715147,0.9720231,0.972527,0.9730265,0.9735216,0.9740121,0.9744983,
0.9749799,0.9754571,0.9759297,0.976398,0.9768617,0.9773209,0.9777756,0.9782258,0.9786716,0.9791128,
0.9795495,0.9799817,0.9804093,0.9808325,0.9812511,0.9816651,0.9820746,0.9824796,0.9828801,0.9832759,
0.9836673,0.9840541,0.9844363,0.9848139,0.9851871,0.9855555,0.9859195,0.9862788,0.9866337,0.9869838,
0.9873294,0.9876705,0.9880069,0.9883387,0.9886659,0.9889885,0.9893066,0.98962,0.9899287,0.9902329,
0.9905325,0.9908274,0.9911177,0.9914034,0.9916844,0.9919609,0.9922327,0.9924998,0.9927623,0.9930202,
0.9932734,0.993522,0.9937659,0.9940051,0.9942398,0.9944698,0.9946951,0.9949157,0.9951317,0.995343,
0.9955496,0.9957516,0.9959489,0.9961416,0.9963295,0.9965128,0.9966914,0.9968653,0.9970345,0.9971991,
0.997359,0.9975142,0.9976647,0.9978105,0.9979516,0.9980881,0.9982198,0.9983468,0.9984692,0.9985868,
0.9986998,0.998808,0.9989116,0.9990104,0.9991046,0.9991941,0.9992788,0.9993589,0.9994342,0.9995049,
0.9995708,0.999632,0.9996885,0.9997404,0.9997874,0.9998298,0.9998675,0.9999005,0.9999288,0.9999523,
0.9999712,0.9999853,0.9999947,0.9999994,0.9999994,0.9999947,0.9999853,0.9999712,0.9999523,0.9999288,
0.9999005,0.9998675,0.9998298,0.9997874,0.9997404,0.9996885,0.999632,0.9995708,0.9995049,0.9994342,
0.9993589,0.9992788,0.9991941,0.9991046,0.9990104,0.9989116,0.998808,0.9986998,0.9985868,0.9984692,
0.9983468,0.9982198,0.9980881,0.9979516,0.9978105,0.9976647,0.9975142,0.997359,0.9971991,0.9970345,
0.9968653,0.9966914,0.9965128,0.9963295,0.9961416,0.9959489,0.9957516,0.9955496,0.995343,0.9951317,
0.9949157,0.9946951,0.9944698,0.9942398,0.9940051,0.9937659,0.993522,0.9932734,0.9930202,0.9927623,
0.9924998,0.9922327,0.9919609,0.9916844,0.9914034,0.9911177,0.9908274,0.9905325,0.9902329,0.9899287,
0.98962,0.9893066,0.9889885,0.9886659,0.9883387,0.9880069,0.9876705,0.9873294,0.9869838,0.9866337,
0.9862788,0.9859195,0.9855555,0.9851871,0.9848139,0.9844363,0.9840541,0.9836673,0.9832759,0.9828801,
0.9824796,0.9820746,0.9816651,0.9812511,0.9808325,0.9804093,0.9799817,0.9795495,0.9791128,0.9786716,
0.9782258,0.9777756,0.9773209,0.9768617,0.976398,0.9759297,0.9754571,0.9749799,0.9744983,0.9740121,
0.9735216,0.9730265,0.972527,0.9720231,0.9715147,0.9710019,0.9704846,0.9699628,0.9694367,0.9689062,
0.9683712,0.9678318,0.967288,0.9667398,0.9661872,0.9656302,0.9650688,0.964503,0.9639329,0.9633584,
0.9627795,0.9621963,0.9616087,0.9610168,0.9604205,0.9598199,0.9592149,0.9586056,0.9579921,0.9573742,
0.9567519,0.9561254,0.9554946,0.9548594,0.9542201,0.9535764,0.9529284,0.9522762,0.9516197,0.950959,
0.950294,0.9496248,0.9489513,0.9482736,0.9475917,0.9469056,0.9462152,0.9455207,0.944822,0.944119,
0.9434119,0.9427006,0.9419851,0.9412655,0.9405417,0.9398137,0.9390817,0.9383454,0.9376051,0.9368606,
0.936112,0.9353593,0.9346025,0.9338416,0.9330766,0.9323075,0.9315344,0.9307572,0.929976,0.9291906,
0.9284013,0.9276079,0.9268105,0.9260091,0.9252036,0.9243941,0.9235806,0.9227632,0.9219418,0.9211164,
0.920287,0.9194537,0.9186164,0.9177752,0.91693,0.9160809,0.9152278,0.914371,0.9135101,0.9126453,
0.9117767,0.9109042,0.9100279,0.9091477,0.9082636,0.9073756,0.9064839,0.9055883,0.9046888,0.9037856,
0.9028786,0.9019677,0.9010531,0.9001347,0.8992125,0.8982866,0.8973569,0.8964235,0.8954864,0.8945454,
0.8936008,0.8926525,0.8917005,0.8907448,0.8897854,0.8888224,0.8878556,0.8868853,0.8859112,0.8849336,
0.8839523,0.8829674,0.8819789,0.8809868,0.8799911,0.8789918,0.877989,0.8769826,0.8759726,0.8749591,
0.8739421,0.8729215,0.8718975,0.8708699,0.8698388,0.8688042,0.8677662,0.8667247,0.8656797,0.8646314,
0.8635795,0.8625243,0.8614656,0.8604035,0.859338,0.8582692,0.857197,0.8561214,0.8550424,0.8539601,
0.8528745,0.8517855,0.8506932,0.8495977,0.8484988,0.8473967,0.8462912,0.8451825,0.8440706,0.8429554,
0.841837,0.8407153,0.8395905,0.8384625,0.8373312,0.8361968,0.8350592,0.8339185,0.8327746,0.8316276,
0.8304775,0.8293242,0.8281679,0.8270084,0.8258458,0.8246803,0.8235116,0.8223399,0.8211651,0.8199874,
0.8188066,0.8176228,0.816436,0.8152462,0.8140535,0.8128578,0.8116592,0.8104576,0.8092531,0.8080457,
0.8068354,0.8056222,0.8044061,0.8031871,0.8019653,0.8007407,0.7995132,0.7982829,0.7970497,0.7958138,
0.7945751,0.7933336,0.7920893,0.7908424,0.7895926,0.7883402,0.787085,0.7858271,0.7845665,0.7833033,
0.7820373,0.7807687,0.7794975,0.7782236,0.7769471,0.775668,0.7743863,0.773102,0.7718152,0.7705258,
0.7692338,0.7679393,0.7666423,0.7653428,0.7640407,0.7627362,0.7614292,0.7601197,0.7588078,0.7574934,
0.7561767,0.7548575,0.7535359,0.7522119,0.7508855,0.7495568,0.7482257,0.7468923,0.7455566,0.7442185,
0.7428782,0.7415355,0.7401906,0.7388434,0.7374941,0.7361424,0.7347885,0.7334324,0.7320741,0.7307136,
0.7293509,0.7279861,0.7266191,0.72525,0.7238788,0.7225055,0.72113,0.7197525,0.7183729,0.7169912,
0.7156076,0.7142218,0.7128341,0.7114444,0.7100526,0.7086589,0.7072632,0.7058656,0.704466,0.7030645,
0.7016611,0.7002558,0.6988485,0.6974395,0.6960285,0.6946158,0.6932012,0.6917847,0.6903665,0.6889464,
0.6875246,0.686101,0.6846757,0.6832486,0.6818199,0.6803893,0.6789571,0.6775232,0.6760876,0.6746504,
0.6732115,0.671771,0.6703289,0.6688851,0.6674398,0.6659929,0.6645445,0.6630944,0.6616428,0.6601898,
0.6587352,0.6572791,0.6558216,0.6543625,0.6529021,0.6514401,0.6499768,0.648512,0.6470459,0.6455783,
0.6441094,0.6426391,0.6411675,0.6396945,0.6382203,0.6367447,0.6352678,0.6337897,0.6323103,0.6308296,
0.6293477,0.6278647,0.6263804,0.6248949,0.6234082,0.6219204,0.6204314,0.6189413,0.6174501,0.6159577,
0.6144643,0.6129698,0.6114742,0.6099776,0.6084799,0.6069812,0.6054816,0.6039808,0.6024792,0.6009766,
0.599473,0.5979685,0.596463,0.5949567,0.5934494,0.5919413,0.5904323,0.5889225,0.5874118,0.5859003,
0.584388,0.5828749,0.581361,0.5798463,0.5783309,0.5768148,0.5752979,0.5737803,0.572262,0.5707431,
0.5692235,0.5677032,0.5661823,0.5646607,0.5631386,0.5616159,0.5600926,0.5585687,0.5570442,0.5555192,
0.5539938,0.5524678,0.5509412,0.5494143,0.5478868,0.5463589,0.5448306,0.5433018,0.5417727,0.5402431,
0.5387132,0.5371829,0.5356522,0.5341212,0.5325899,0.5310583,0.5295264,0.5279942,0.5264617,0.524929,
0.5233961,0.5218629,0.5203296,0.518796,0.5172623,0.5157284,0.5141944,0.5126601,0.5111259,0.5095915,
0.508057,0.5065224,0.5049878,0.5034531,0.5019184,0.5003837,0.4988489,0.4973142,0.4957795,0.4942449,
0.4927103,0.4911758,0.4896413,0.488107,0.4865727,0.4850386,0.4835047,0.4819708,0.4804372,0.4789037,
0.4773705,0.4758374,0.4743046,0.472772,0.4712397,0.4697076,0.4681759,0.4666444,0.4651133,0.4635824,
0.462052,0.4605218,0.4589921,0.4574627,0.4559337,0.4544052,0.4528771,0.4513494,0.4498222,0.4482954,
0.4467692,0.4452434,0.4437182,0.4421935,0.4406693,0.4391457,0.4376227,0.4361002,0.4345784,0.4330572,
0.4315366,0.4300166,0.4284973,0.4269787,0.4254608,0.4239436,0.4224271,0.4209113,0.4193963,0.417882,
0.4163685,0.4148557,0.4133438,0.4118327,0.4103225,0.4088131,0.4073045,0.4057968,0.40429,0.4027841,
0.4012792,0.3997751,0.398272,0.3967699,0.3952687,0.3937685,0.3922693,0.3907711,0.389274,0.3877779,
0.3862828,0.3847889,0.383296,0.3818042,0.3803135,0.378824,0.3773355,0.3758483,0.3743622,0.3728773,
0.3713937,0.3699112,0.3684299,0.3669499,0.3654711,0.3639936,0.3625174,0.3610425,0.3595689,0.3580966,
0.3566256,0.355156,0.3536878,0.3522209,0.3507555,0.3492914,0.3478287,0.3463675,0.3449078,0.3434495,
0.3419926,0.3405373,0.3390835,0.3376312,0.3361804,0.3347311,0.3332835,0.3318373,0.3303928,0.3289499,
0.3275085,0.3260688,0.3246308,0.3231944,0.3217596,0.3203266,0.3188952,0.3174655,0.3160376,0.3146114,
0.3131869,0.3117642,0.3103433,0.3089242,0.3075068,0.3060913,0.3046776,0.3032658,0.3018557,0.3004476,
0.2990413,0.297637,0.2962345,0.294834,0.2934354,0.2920387,0.290644,0.2892513,0.2878605,0.2864718,
0.2850851,0.2837003,0.2823177,0.280937,0.2795585,0.278182,0.2768076,0.2754353,0.2740652,0.2726971,
0.2713312,0.2699675,0.2686059,0.2672465,0.2658893,0.2645343,0.2631815,0.261831,0.2604827,0.2591366,
0.2577928,0.2564513,0.2551121,0.2537752,0.2524407,0.2511084,0.2497785,0.248451,0.2471258,0.245803,
0.2444826,0.2431646,0.2418491,0.2405359,0.2392252,0.237917,0.2366112,0.235308,0.2340072,0.2327089,
0.2314131,0.2301199,0.2288292,0.2275411,0.2262555,0.2249725,0.2236921,0.2224143,0.2211391,0.2198666,
0.2185967,0.2173294,0.2160648,0.2148029,0.2135436,0.2122871,0.2110333,0.2097822,0.2085338,0.2072882,
0.2060453,0.2048052,0.2035679,0.2023334,0.2011016,0.1998727,0.1986467,0.1974234,0.196203,0.1949855,
0.1937709,0.1925591,0.1913502,0.1901443,0.1889412,0.1877411,0.186544,0.1853497,0.1841585,0.1829702,
0.1817849,0.1806026,0.1794234,0.1782471,0.1770739,0.1759037,0.1747366,0.1735725,0.1724115,0.1712536,
0.1700988,0.1689471,0.1677985,0.1666531,0.1655107,0.1643716,0.1632356,0.1621028,0.1609731,0.1598467,
0.1587234,0.1576034,0.1564866,0.155373,0.1542627,0.1531557,0.1520519,0.1509514,0.1498541,0.1487602,
0.1476696,0.1465823,0.1454983,0.1444177,0.1433404,0.1422665,0.141196,0.1401288,0.139065,0.1380046,
0.1369477,0.1358941,0.134844,0.1337973,0.1327541,0.1317143,0.1306781,0.1296452,0.1286159,0.1275901,
0.1265678,0.125549,0.1245337,0.123522,0.1225138,0.1215092,0.1205081,0.1195106,0.1185167,0.1175264,
0.1165397,0.1155566,0.1145771,0.1136013,0.1126291,0.1116605,0.1106956,0.1097344,0.1087769,0.107823,
0.1068729,0.1059264,0.1049836,0.1040446,0.1031093,0.1021778,0.10125,0.1003259,0.09940562,0.09848911,
0.09757638,0.09666744,0.0957623,0.09486097,0.09396346,0.09306978,0.09217992,0.09129391,0.09041175,0.08953345,
0.08865902,0.08778846,0.08692179,0.086059,0.08520012,0.08434515,0.08349409,0.08264696,0.08180375,0.08096449,
0.08012918,0.07929782,0.07847042,0.077647,0.07682756,0.0760121,0.07520064,0.07439318,0.07358973,0.07279029,
0.07199489,0.07120351,0.07041618,0.0696329,0.06885366,0.06807849,0.06730739,0.06654036,0.06577742,0.06501857,
0.06426382,0.06351317,0.06276664,0.06202422,0.06128594,0.06055178,0.05982177,0.0590959,0.05837419,0.05765663,
0.05694325,0.05623404,0.05552901,0.05482817,0.05413152,0.05343907,0.05275083,0.05206681,0.051387,0.05071142,
0.05004008,0.04937297,0.04871011,0.0480515,0.04739715,0.04674706,0.04610125,0.04545971,0.04482245,0.04418948,
0.04356081,0.04293643,0.04231637,0.04170061,0.04108917,0.04048206,0.03987927,0.03928082,0.03868671,0.03809695,
0.03751154,0.03693049,0.03635379,0.03578147,0.03521352,0.03464995,0.03409077,0.03353597,0.03298557,0.03243957,
0.03189797,0.03136079,0.03082802,0.03029967,0.02977574,0.02925625,0.02874119,0.02823057,0.0277244,0.02722267,
0.0267254,0.02623259,0.02574424,0.02526036,0.02478096,0.02430603,0.02383558,0.02336962,0.02290815,0.02245117,
0.02199869,0.02155072,0.02110725,0.0206683,0.02023386,0.01980395,0.01937855,0.01895769,0.01854136,0.01812956,
0.0177223,0.01731959,0.01692143,0.01652781,0.01613875,0.01575425,0.01537431,0.01499894,0.01462814,0.01426191,
0.01390026,0.01354319,0.0131907,0.01284279,0.01249948,0.01216076,0.01182664,0.01149711,0.01117219,0.01085187,
0.01053616,0.01022507,0.009918584,0.009616719,0.009319474,0.009026852,0.008738856,0.008455488,0.008176751,0.007902649,
0.007633183,0.007368355,0.007108169,0.006852626,0.00660173,0.006355483,0.006113886,0.005876943,0.005644655,0.005417024,
0.005194054,0.004975745,0.0047621,0.004553121,0.00434881,0.004149168,0.003954199,0.003763903,0.003578282,0.003397339,
0.003221074,0.003049489,0.002882587,0.002720368,0.002562835,0.002409988,0.002261829,0.00211836,0.001979581,0.001845495,
0.001716102,0.001591403,0.001471401,0.001356095,0.001245487,0.001139579,0.00103837,0.0009418628,0.0008500572,0.0007629544,
0.0006805552,0.0006028604,0.0005298706,0.0004615867,0.0003980092,0.0003391388,0.0002849759,0.0002355212,0.000190775,0.0001507378,
0.00011541,8.479189E-05,5.888376E-05,3.768587E-05,2.119842E-05,9.421557E-06,2.355395E-06,0,};