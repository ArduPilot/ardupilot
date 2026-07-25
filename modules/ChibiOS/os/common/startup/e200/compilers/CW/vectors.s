/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    vectors.s
 * @brief   SPC56x vectors table.
 *
 * @addtogroup PPC_CW_CORE
 * @{
 */

#define _FROM_ASM_
#include "ppcparams.h"

#if defined(VECTORS_RENAMING)
#include "isrs.h"
#endif

#if !defined(__DOXYGEN__)

        .global     vector0,    vector1,    vector2,    vector3
#if PPC_NUM_VECTORS > 4
        .global     vector4,    vector5,    vector6,    vector7
#endif
#if PPC_NUM_VECTORS > 8
        .global     vector8,    vector9,    vector10,   vector11
#endif
#if PPC_NUM_VECTORS > 12
        .global     vector12,   vector13,   vector14,   vector15
#endif
#if PPC_NUM_VECTORS > 16
        .global     vector16,   vector17,   vector18,   vector19
#endif
#if PPC_NUM_VECTORS > 20
        .global     vector20,   vector21,   vector22,   vector23
#endif
#if PPC_NUM_VECTORS > 24
        .global     vector24,   vector25,   vector26,   vector27
#endif
#if PPC_NUM_VECTORS > 28
        .global     vector28,   vector29,   vector30,   vector31
#endif
#if PPC_NUM_VECTORS > 32
        .global     vector32,   vector33,   vector34,   vector35
#endif
#if PPC_NUM_VECTORS > 36
        .global     vector36,   vector37,   vector38,   vector39
#endif
#if PPC_NUM_VECTORS > 40
        .global     vector40,   vector41,   vector42,   vector43
#endif
#if PPC_NUM_VECTORS > 44
        .global     vector44,   vector45,   vector46,   vector47
#endif
#if PPC_NUM_VECTORS > 48
        .global     vector48,   vector49,   vector50,   vector51
#endif
#if PPC_NUM_VECTORS > 52
        .global     vector52,   vector53,   vector54,   vector55
#endif
#if PPC_NUM_VECTORS > 56
        .global     vector56,   vector57,   vector58,   vector59
#endif
#if PPC_NUM_VECTORS > 60
        .global     vector60,   vector61,   vector62,   vector63
#endif
#if PPC_NUM_VECTORS > 64
        .global     vector64,   vector65,   vector66,   vector67
#endif
#if PPC_NUM_VECTORS > 68
        .global     vector68,   vector69,   vector70,   vector71
#endif
#if PPC_NUM_VECTORS > 72
        .global     vector72,   vector73,   vector74,   vector75
#endif
#if PPC_NUM_VECTORS > 76
        .global     vector76,   vector77,   vector78,   vector79
#endif
#if PPC_NUM_VECTORS > 80
        .global     vector80,   vector81,   vector82,   vector83
#endif
#if PPC_NUM_VECTORS > 84
        .global     vector84,   vector85,   vector86,   vector87
#endif
#if PPC_NUM_VECTORS > 88
        .global     vector88,   vector89,   vector90,   vector91
#endif
#if PPC_NUM_VECTORS > 92
        .global     vector92,   vector93,   vector94,   vector95
#endif
#if PPC_NUM_VECTORS > 96
        .global     vector96,   vector97,   vector98,   vector99
#endif
#if PPC_NUM_VECTORS > 100
        .global     vector100,  vector101,  vector102,  vector103
#endif
#if PPC_NUM_VECTORS > 104
        .global     vector104,  vector105,  vector106,  vector107
#endif
#if PPC_NUM_VECTORS > 108
        .global     vector108,  vector109,  vector110,  vector111
#endif
#if PPC_NUM_VECTORS > 112
        .global     vector112,  vector113,  vector114,  vector115
#endif
#if PPC_NUM_VECTORS > 116
        .global     vector116,  vector117,  vector118,  vector119
#endif
#if PPC_NUM_VECTORS > 120
        .global     vector120,  vector121,  vector122,  vector123
#endif
#if PPC_NUM_VECTORS > 124
        .global     vector124,  vector125,  vector126,  vector127
#endif
#if PPC_NUM_VECTORS > 128
        .global     vector128,  vector129,  vector130,  vector131
#endif
#if PPC_NUM_VECTORS > 132
        .global     vector132,  vector133,  vector134,  vector135
#endif
#if PPC_NUM_VECTORS > 136
        .global     vector136,  vector137,  vector138,  vector139
#endif
#if PPC_NUM_VECTORS > 140
        .global     vector140,  vector141,  vector142,  vector143
#endif
#if PPC_NUM_VECTORS > 144
        .global     vector144,  vector145,  vector146,  vector147
#endif
#if PPC_NUM_VECTORS > 148
        .global     vector148,  vector149,  vector150,  vector151
#endif
#if PPC_NUM_VECTORS > 152
        .global     vector152,  vector153,  vector154,  vector155
#endif
#if PPC_NUM_VECTORS > 156
        .global     vector156,  vector157,  vector158,  vector159
#endif
#if PPC_NUM_VECTORS > 160
        .global     vector160,  vector161,  vector162,  vector163
#endif
#if PPC_NUM_VECTORS > 164
        .global     vector164,  vector165,  vector166,  vector167
#endif
#if PPC_NUM_VECTORS > 168
        .global     vector168,  vector169,  vector170,  vector171
#endif
#if PPC_NUM_VECTORS > 172
        .global     vector172,  vector173,  vector174,  vector175
#endif
#if PPC_NUM_VECTORS > 176
        .global     vector176,  vector177,  vector178,  vector179
#endif
#if PPC_NUM_VECTORS > 180
        .global     vector180,  vector181,  vector182,  vector183
#endif
#if PPC_NUM_VECTORS > 184
        .global     vector184,  vector185,  vector186,  vector187
#endif
#if PPC_NUM_VECTORS > 188
        .global     vector188,  vector189,  vector190,  vector191
#endif
#if PPC_NUM_VECTORS > 192
        .global     vector192,  vector193,  vector194,  vector195
#endif
#if PPC_NUM_VECTORS > 196
        .global     vector196,  vector197,  vector198,  vector199
#endif
#if PPC_NUM_VECTORS > 200
        .global     vector200,  vector201,  vector202,  vector203
#endif
#if PPC_NUM_VECTORS > 204
        .global     vector204,  vector205,  vector206,  vector207
#endif
#if PPC_NUM_VECTORS > 208
        .global     vector208,  vector209,  vector210,  vector211
#endif
#if PPC_NUM_VECTORS > 212
        .global     vector212,  vector213,  vector214,  vector215
#endif
#if PPC_NUM_VECTORS > 216
        .global     vector216,  vector217,  vector218,  vector219
#endif
#if PPC_NUM_VECTORS > 220
        .global     vector220,  vector221,  vector222,  vector223
#endif
#if PPC_NUM_VECTORS > 224
        .global     vector224,  vector225,  vector226,  vector227
#endif
#if PPC_NUM_VECTORS > 228
        .global     vector228,  vector229,  vector230,  vector231
#endif
#if PPC_NUM_VECTORS > 232
        .global     vector232,  vector233,  vector234,  vector235
#endif
#if PPC_NUM_VECTORS > 236
        .global     vector236,  vector237,  vector238,  vector239
#endif
#if PPC_NUM_VECTORS > 240
        .global     vector240,  vector241,  vector242,  vector243
#endif
#if PPC_NUM_VECTORS > 244
        .global     vector244,  vector245,  vector246,  vector247
#endif
#if PPC_NUM_VECTORS > 248
        .global     vector248,  vector249,  vector250,  vector251
#endif
#if PPC_NUM_VECTORS > 252
        .global     vector252,  vector253,  vector254,  vector255
#endif
#if PPC_NUM_VECTORS > 256
        .global     vector256,  vector257,  vector258,  vector259
#endif
#if PPC_NUM_VECTORS > 260
        .global     vector260,  vector261,  vector262,  vector263
#endif
#if PPC_NUM_VECTORS > 264
        .global     vector264,  vector265,  vector266,  vector267
#endif
#if PPC_NUM_VECTORS > 268
        .global     vector268,  vector269,  vector270,  vector271
#endif
#if PPC_NUM_VECTORS > 272
        .global     vector272,  vector273,  vector274,  vector275
#endif
#if PPC_NUM_VECTORS > 276
        .global     vector276,  vector277,  vector278,  vector279
#endif
#if PPC_NUM_VECTORS > 280
        .global     vector280,  vector281,  vector282,  vector283
#endif
#if PPC_NUM_VECTORS > 284
        .global     vector284,  vector285,  vector286,  vector287
#endif
#if PPC_NUM_VECTORS > 288
        .global     vector288,  vector289,  vector290,  vector291
#endif
#if PPC_NUM_VECTORS > 292
        .global     vector292,  vector293,  vector294,  vector295
#endif
#if PPC_NUM_VECTORS > 296
        .global     vector296,  vector297,  vector298,  vector299
#endif
#if PPC_NUM_VECTORS > 300
        .global     vector300,  vector301,  vector302,  vector303
#endif
#if PPC_NUM_VECTORS > 304
        .global     vector304,  vector305,  vector306,  vector307
#endif
#if PPC_NUM_VECTORS > 308
        .global     vector308,  vector309,  vector310,  vector311
#endif
#if PPC_NUM_VECTORS > 312
        .global     vector312,  vector313,  vector314,  vector315
#endif
#if PPC_NUM_VECTORS > 316
        .global     vector316,  vector317,  vector318,  vector319
#endif
#if PPC_NUM_VECTORS > 320
        .global     vector320,  vector321,  vector322,  vector323
#endif
#if PPC_NUM_VECTORS > 324
        .global     vector324,  vector325,  vector326,  vector327
#endif
#if PPC_NUM_VECTORS > 328
        .global     vector328,  vector329,  vector330,  vector331
#endif
#if PPC_NUM_VECTORS > 332
        .global     vector332,  vector333,  vector334,  vector335
#endif
#if PPC_NUM_VECTORS > 336
        .global     vector336,  vector337,  vector338,  vector339
#endif
#if PPC_NUM_VECTORS > 340
        .global     vector340,  vector341,  vector342,  vector343
#endif
#if PPC_NUM_VECTORS > 344
        .global     vector344,  vector345,  vector346,  vector347
#endif
#if PPC_NUM_VECTORS > 348
        .global     vector348,  vector349,  vector350,  vector351
#endif
#if PPC_NUM_VECTORS > 352
        .global     vector352,  vector353,  vector354,  vector355
#endif
#if PPC_NUM_VECTORS > 356
        .global     vector356,  vector357,  vector358,  vector359
#endif
#if PPC_NUM_VECTORS > 360
        .global     vector360,  vector361,  vector362,  vector363
#endif
#if PPC_NUM_VECTORS > 364
        .global     vector364,  vector365,  vector366,  vector367
#endif
#if PPC_NUM_VECTORS > 368
        .global     vector368,  vector369,  vector370,  vector371
#endif
#if PPC_NUM_VECTORS > 372
        .global     vector372,  vector373,  vector374,  vector375
#endif
#if PPC_NUM_VECTORS > 376
        .global     vector376,  vector377,  vector378,  vector379
#endif
#if PPC_NUM_VECTORS > 380
        .global     vector380,  vector381,  vector382,  vector383
#endif
#if PPC_NUM_VECTORS > 384
        .global     vector384,  vector385,  vector386,  vector387
#endif
#if PPC_NUM_VECTORS > 388
        .global     vector388,  vector389,  vector390,  vector391
#endif
#if PPC_NUM_VECTORS > 392
        .global     vector392,  vector393,  vector394,  vector395
#endif
#if PPC_NUM_VECTORS > 396
        .global     vector396,  vector397,  vector398,  vector399
#endif
#if PPC_NUM_VECTORS > 400
        .global     vector400,  vector401,  vector402,  vector403
#endif
#if PPC_NUM_VECTORS > 404
        .global     vector404,  vector405,  vector406,  vector407
#endif
#if PPC_NUM_VECTORS > 408
        .global     vector408,  vector409,  vector410,  vector411
#endif
#if PPC_NUM_VECTORS > 412
        .global     vector412,  vector413,  vector414,  vector415
#endif
#if PPC_NUM_VECTORS > 416
        .global     vector416,  vector417,  vector418,  vector419
#endif
#if PPC_NUM_VECTORS > 420
        .global     vector420,  vector421,  vector422,  vector423
#endif
#if PPC_NUM_VECTORS > 424
        .global     vector424,  vector425,  vector426,  vector427
#endif
#if PPC_NUM_VECTORS > 428
        .global     vector428,  vector429,  vector430,  vector431
#endif
#if PPC_NUM_VECTORS > 432
        .global     vector432,  vector433,  vector434,  vector435
#endif
#if PPC_NUM_VECTORS > 436
        .global     vector436,  vector437,  vector438,  vector439
#endif
#if PPC_NUM_VECTORS > 440
        .global     vector440,  vector441,  vector442,  vector443
#endif
#if PPC_NUM_VECTORS > 444
        .global     vector444,  vector445,  vector446,  vector447
#endif
#if PPC_NUM_VECTORS > 448
        .global     vector448,  vector449,  vector450,  vector451
#endif
#if PPC_NUM_VECTORS > 452
        .global     vector452,  vector453,  vector454,  vector455
#endif
#if PPC_NUM_VECTORS > 456
        .global     vector456,  vector457,  vector458,  vector459
#endif
#if PPC_NUM_VECTORS > 460
        .global     vector460,  vector461,  vector462,  vector463
#endif
#if PPC_NUM_VECTORS > 464
        .global     vector464,  vector465,  vector466,  vector467
#endif
#if PPC_NUM_VECTORS > 468
        .global     vector468,  vector469,  vector470,  vector471
#endif
#if PPC_NUM_VECTORS > 472
        .global     vector472,  vector473,  vector474,  vector475
#endif
#if PPC_NUM_VECTORS > 476
        .global     vector476,  vector477,  vector478,  vector479
#endif
#if PPC_NUM_VECTORS > 480
        .global     vector480,  vector481,  vector482,  vector483
#endif
#if PPC_NUM_VECTORS > 484
        .global     vector484,  vector485,  vector486,  vector487
#endif
#if PPC_NUM_VECTORS > 488
        .global     vector488,  vector489,  vector490,  vector491
#endif
#if PPC_NUM_VECTORS > 492
        .global     vector492,  vector493,  vector494,  vector495
#endif
#if PPC_NUM_VECTORS > 496
        .global     vector496,  vector497,  vector498,  vector499
#endif
#if PPC_NUM_VECTORS > 500
        .global     vector500,  vector501,  vector502,  vector503
#endif
#if PPC_NUM_VECTORS > 504
        .global     vector504,  vector505,  vector506,  vector507
#endif
#if PPC_NUM_VECTORS > 508
        .global     vector508,  vector509,  vector510,  vector511
#endif
#if PPC_NUM_VECTORS > 512
        .global     vector512,  vector513,  vector514,  vector515
#endif
#if PPC_NUM_VECTORS > 516
        .global     vector516,  vector517,  vector518,  vector519
#endif
#if PPC_NUM_VECTORS > 520
        .global     vector520,  vector521,  vector522,  vector523
#endif
#if PPC_NUM_VECTORS > 524
        .global     vector524,  vector525,  vector526,  vector527
#endif
#if PPC_NUM_VECTORS > 528
        .global     vector528,  vector529,  vector530,  vector531
#endif
#if PPC_NUM_VECTORS > 532
        .global     vector532,  vector533,  vector534,  vector535
#endif
#if PPC_NUM_VECTORS > 536
        .global     vector536,  vector537,  vector538,  vector539
#endif
#if PPC_NUM_VECTORS > 540
        .global     vector540,  vector541,  vector542,  vector543
#endif
#if PPC_NUM_VECTORS > 544
        .global     vector544,  vector545,  vector546,  vector547
#endif
#if PPC_NUM_VECTORS > 548
        .global     vector548,  vector549,  vector550,  vector551
#endif
#if PPC_NUM_VECTORS > 552
        .global     vector552,  vector553,  vector554,  vector555
#endif
#if PPC_NUM_VECTORS > 556
        .global     vector556,  vector557,  vector558,  vector559
#endif
#if PPC_NUM_VECTORS > 560
        .global     vector560,  vector561,  vector562,  vector563
#endif
#if PPC_NUM_VECTORS > 564
        .global     vector564,  vector565,  vector566,  vector567
#endif
#if PPC_NUM_VECTORS > 568
        .global     vector568,  vector569,  vector570,  vector571
#endif
#if PPC_NUM_VECTORS > 572
        .global     vector572,  vector573,  vector574,  vector575
#endif
#if PPC_NUM_VECTORS > 576
        .global     vector576,  vector577,  vector578,  vector579
#endif
#if PPC_NUM_VECTORS > 580
        .global     vector580,  vector581,  vector582,  vector583
#endif
#if PPC_NUM_VECTORS > 584
        .global     vector584,  vector585,  vector586,  vector587
#endif
#if PPC_NUM_VECTORS > 588
        .global     vector588,  vector589,  vector590,  vector591
#endif
#if PPC_NUM_VECTORS > 592
        .global     vector592,  vector593,  vector594,  vector595
#endif
#if PPC_NUM_VECTORS > 596
        .global     vector596,  vector597,  vector598,  vector599
#endif
#if PPC_NUM_VECTORS > 600
        .global     vector600,  vector601,  vector602,  vector603
#endif
#if PPC_NUM_VECTORS > 604
        .global     vector604,  vector605,  vector606,  vector607
#endif
#if PPC_NUM_VECTORS > 608
        .global     vector608,  vector609,  vector610,  vector611
#endif
#if PPC_NUM_VECTORS > 612
        .global     vector612,  vector613,  vector614,  vector615
#endif
#if PPC_NUM_VECTORS > 616
        .global     vector616,  vector617,  vector618,  vector619
#endif
#if PPC_NUM_VECTORS > 620
        .global     vector620,  vector621,  vector622,  vector623
#endif
#if PPC_NUM_VECTORS > 624
        .global     vector624,  vector625,  vector626,  vector627
#endif
#if PPC_NUM_VECTORS > 628
        .global     vector628,  vector629,  vector630,  vector631
#endif
#if PPC_NUM_VECTORS > 632
        .global     vector632,  vector633,  vector634,  vector635
#endif
#if PPC_NUM_VECTORS > 636
        .global     vector636,  vector637,  vector638,  vector639
#endif
#if PPC_NUM_VECTORS > 640
        .global     vector640,  vector641,  vector642,  vector643
#endif
#if PPC_NUM_VECTORS > 644
        .global     vector644,  vector645,  vector646,  vector647
#endif
#if PPC_NUM_VECTORS > 648
        .global     vector648,  vector649,  vector650,  vector651
#endif
#if PPC_NUM_VECTORS > 652
        .global     vector652,  vector653,  vector654,  vector655
#endif
#if PPC_NUM_VECTORS > 656
        .global     vector656,  vector657,  vector658,  vector659
#endif
#if PPC_NUM_VECTORS > 660
        .global     vector660,  vector661,  vector662,  vector663
#endif
#if PPC_NUM_VECTORS > 664
        .global     vector664,  vector665,  vector666,  vector667
#endif
#if PPC_NUM_VECTORS > 668
        .global     vector668,  vector669,  vector670,  vector671
#endif
#if PPC_NUM_VECTORS > 672
        .global     vector672,  vector673,  vector674,  vector675
#endif
#if PPC_NUM_VECTORS > 676
        .global     vector676,  vector677,  vector678,  vector679
#endif
#if PPC_NUM_VECTORS > 680
        .global     vector680,  vector681,  vector682,  vector683
#endif
#if PPC_NUM_VECTORS > 684
        .global     vector684,  vector685,  vector686,  vector687
#endif
#if PPC_NUM_VECTORS > 688
        .global     vector688,  vector689,  vector690,  vector691
#endif
#if PPC_NUM_VECTORS > 692
        .global     vector692,  vector693,  vector694,  vector695
#endif
#if PPC_NUM_VECTORS > 696
        .global     vector696,  vector697,  vector698,  vector699
#endif
#if PPC_NUM_VECTORS > 700
        .global     vector700,  vector701,  vector702,  vector703
#endif
#if PPC_NUM_VECTORS > 704
        .global     vector704,  vector705,  vector706,  vector707
#endif
#if PPC_NUM_VECTORS > 708
        .global     vector708,  vector709,  vector710,  vector711
#endif
#if PPC_NUM_VECTORS > 712
        .global     vector712,  vector713,  vector714,  vector715
#endif
#if PPC_NUM_VECTORS > 716
        .global     vector716,  vector717,  vector718,  vector719
#endif
#if PPC_NUM_VECTORS > 720
        .global     vector720,  vector721,  vector722,  vector723
#endif
#if PPC_NUM_VECTORS > 724
        .global     vector724,  vector725,  vector726,  vector727
#endif
#if PPC_NUM_VECTORS > 728
        .global     vector728,  vector729,  vector730,  vector731
#endif
#if PPC_NUM_VECTORS > 732
        .global     vector732,  vector733,  vector734,  vector735
#endif
#if PPC_NUM_VECTORS > 736
        .global     vector736,  vector737,  vector738,  vector739
#endif
#if PPC_NUM_VECTORS > 740
        .global     vector740,  vector741,  vector742,  vector743
#endif
#if PPC_NUM_VECTORS > 744
        .global     vector744,  vector745,  vector746,  vector747
#endif
#if PPC_NUM_VECTORS > 748
        .global     vector748,  vector749,  vector750,  vector751
#endif
#if PPC_NUM_VECTORS > 752
        .global     vector752,  vector753,  vector754,  vector755
#endif
#if PPC_NUM_VECTORS > 756
        .global     vector756,  vector757,  vector758,  vector759
#endif
#if PPC_NUM_VECTORS > 760
        .global     vector760,  vector761,  vector762,  vector763
#endif
#if PPC_NUM_VECTORS > 764
        .global     vector764,  vector765,  vector766,  vector767
#endif
#if PPC_NUM_VECTORS > 768
        .global     vector768,  vector769,  vector770,  vector771
#endif
#if PPC_NUM_VECTORS > 772
        .global     vector772,  vector773,  vector774,  vector775
#endif
#if PPC_NUM_VECTORS > 776
        .global     vector776,  vector777,  vector778,  vector779
#endif
#if PPC_NUM_VECTORS > 780
        .global     vector780,  vector781,  vector782,  vector783
#endif
#if PPC_NUM_VECTORS > 784
        .global     vector784,  vector785,  vector786,  vector787
#endif
#if PPC_NUM_VECTORS > 788
        .global     vector788,  vector789,  vector790,  vector791
#endif
#if PPC_NUM_VECTORS > 792
        .global     vector792,  vector793,  vector794,  vector795
#endif
#if PPC_NUM_VECTORS > 796
        .global     vector796,  vector797,  vector798,  vector799
#endif
#if PPC_NUM_VECTORS > 800
        .global     vector800,  vector801,  vector802,  vector803
#endif
#if PPC_NUM_VECTORS > 804
        .global     vector804,  vector805,  vector806,  vector807
#endif
#if PPC_NUM_VECTORS > 808
        .global     vector808,  vector809,  vector810,  vector811
#endif
#if PPC_NUM_VECTORS > 812
        .global     vector812,  vector813,  vector814,  vector815
#endif
#if PPC_NUM_VECTORS > 816
        .global     vector816,  vector817,  vector818,  vector819
#endif
#if PPC_NUM_VECTORS > 820
        .global     vector820,  vector821,  vector822,  vector823
#endif
#if PPC_NUM_VECTORS > 824
        .global     vector824,  vector825,  vector826,  vector827
#endif
#if PPC_NUM_VECTORS > 828
        .global     vector828,  vector829,  vector830,  vector831
#endif
#if PPC_NUM_VECTORS > 832
        .global     vector832,  vector833,  vector834,  vector835
#endif
#if PPC_NUM_VECTORS > 836
        .global     vector836,  vector837,  vector838,  vector839
#endif
#if PPC_NUM_VECTORS > 840
        .global     vector840,  vector841,  vector842,  vector843
#endif
#if PPC_NUM_VECTORS > 844
        .global     vector844,  vector845,  vector846,  vector847
#endif
#if PPC_NUM_VECTORS > 848
        .global     vector848,  vector849,  vector850,  vector851
#endif
#if PPC_NUM_VECTORS > 852
        .global     vector852,  vector853,  vector854,  vector855
#endif
#if PPC_NUM_VECTORS > 856
        .global     vector856,  vector857,  vector858,  vector859
#endif
#if PPC_NUM_VECTORS > 860
        .global     vector860,  vector861,  vector862,  vector863
#endif
#if PPC_NUM_VECTORS > 864
        .global     vector864,  vector865,  vector866,  vector867
#endif
#if PPC_NUM_VECTORS > 868
        .global     vector868,  vector869,  vector870,  vector871
#endif
#if PPC_NUM_VECTORS > 872
        .global     vector872,  vector873,  vector874,  vector875
#endif
#if PPC_NUM_VECTORS > 876
        .global     vector876,  vector877,  vector878,  vector879
#endif
#if PPC_NUM_VECTORS > 880
        .global     vector880,  vector881,  vector882,  vector883
#endif
#if PPC_NUM_VECTORS > 884
        .global     vector884,  vector885,  vector886,  vector887
#endif
#if PPC_NUM_VECTORS > 888
        .global     vector888,  vector889,  vector890,  vector891
#endif
#if PPC_NUM_VECTORS > 892
        .global     vector892,  vector893,  vector894,  vector895
#endif
#if PPC_NUM_VECTORS > 896
        .global     vector896,  vector897,  vector898,  vector899
#endif
#if PPC_NUM_VECTORS > 900
        .global     vector900,  vector901,  vector902,  vector903
#endif
#if PPC_NUM_VECTORS > 904
        .global     vector904,  vector905,  vector906,  vector907
#endif
#if PPC_NUM_VECTORS > 908
        .global     vector908,  vector909,  vector910,  vector911
#endif
#if PPC_NUM_VECTORS > 912
        .global     vector912,  vector913,  vector914,  vector915
#endif
#if PPC_NUM_VECTORS > 916
        .global     vector916,  vector917,  vector918,  vector919
#endif
#if PPC_NUM_VECTORS > 920
        .global     vector920,  vector921,  vector922,  vector923
#endif
#if PPC_NUM_VECTORS > 924
        .global     vector924,  vector925,  vector926,  vector927
#endif
#if PPC_NUM_VECTORS > 928
        .global     vector928,  vector929,  vector930,  vector931
#endif
#if PPC_NUM_VECTORS > 932
        .global     vector932,  vector933,  vector934,  vector935
#endif
#if PPC_NUM_VECTORS > 936
        .global     vector936,  vector937,  vector938,  vector939
#endif
#if PPC_NUM_VECTORS > 940
        .global     vector940,  vector941,  vector942,  vector943
#endif
#if PPC_NUM_VECTORS > 944
        .global     vector944,  vector945,  vector946,  vector947
#endif
#if PPC_NUM_VECTORS > 948
        .global     vector948,  vector949,  vector950,  vector951
#endif
#if PPC_NUM_VECTORS > 952
        .global     vector952,  vector953,  vector954,  vector955
#endif
#if PPC_NUM_VECTORS > 956
        .global     vector956,  vector957,  vector958,  vector959
#endif
#if PPC_NUM_VECTORS > 960
        .global     vector960,  vector961,  vector962,  vector963
#endif
#if PPC_NUM_VECTORS > 964
        .global     vector964,  vector965,  vector966,  vector967
#endif
#if PPC_NUM_VECTORS > 968
        .global     vector968,  vector969,  vector970,  vector971
#endif
#if PPC_NUM_VECTORS > 972
        .global     vector972,  vector973,  vector974,  vector975
#endif
#if PPC_NUM_VECTORS > 976
        .global     vector976,  vector977,  vector978,  vector979
#endif
#if PPC_NUM_VECTORS > 980
        .global     vector980,  vector981,  vector982,  vector983
#endif
#if PPC_NUM_VECTORS > 984
        .global     vector984,  vector985,  vector986,  vector987
#endif
#if PPC_NUM_VECTORS > 988
        .global     vector988,  vector989,  vector990,  vector991
#endif
#if PPC_NUM_VECTORS > 992
        .global     vector992,  vector993,  vector994,  vector995
#endif
#if PPC_NUM_VECTORS > 996
        .global     vector996,  vector997,  vector998,  vector999
#endif
#if PPC_NUM_VECTORS > 1000
        .global     vector1000, vector1001, vector1002, vector1003
#endif
#if PPC_NUM_VECTORS > 1004
        .global     vector1004, vector1005, vector1006, vector1007
#endif
#if PPC_NUM_VECTORS > 1008
        .global     vector1008, vector1009, vector1010, vector1011
#endif
#if PPC_NUM_VECTORS > 1012
        .global     vector1012, vector1013, vector1014, vector1015
#endif
#if PPC_NUM_VECTORS > 1016
        .global     vector1016, vector1017, vector1018, vector1019
#endif
#if PPC_NUM_VECTORS > 1020
        .global     vector1020, vector1021, vector1022, vector1023
#endif

        /* Software vectors table. The vectors are accessed from the IVOR4
           handler only. In order to declare an interrupt handler just create
           a function withe the same name of a vector, the symbol will
           override the weak symbol declared here.*/
        .section    .vectors
        .globl      _vectors
_vectors:
        .long       vector0,    vector1,    vector2,    vector3
#if PPC_NUM_VECTORS > 4
        .long       vector4,    vector5,    vector6,    vector7
#endif
#if PPC_NUM_VECTORS > 8
        .long       vector8,    vector9,    vector10,   vector11
#endif
#if PPC_NUM_VECTORS > 12
        .long       vector12,   vector13,   vector14,   vector15
#endif
#if PPC_NUM_VECTORS > 16
        .long       vector16,   vector17,   vector18,   vector19
#endif
#if PPC_NUM_VECTORS > 20
        .long       vector20,   vector21,   vector22,   vector23
#endif
#if PPC_NUM_VECTORS > 24
        .long       vector24,   vector25,   vector26,   vector27
#endif
#if PPC_NUM_VECTORS > 28
        .long       vector28,   vector29,   vector30,   vector31
#endif
#if PPC_NUM_VECTORS > 32
        .long       vector32,   vector33,   vector34,   vector35
#endif
#if PPC_NUM_VECTORS > 36
        .long       vector36,   vector37,   vector38,   vector39
#endif
#if PPC_NUM_VECTORS > 40
        .long       vector40,   vector41,   vector42,   vector43
#endif
#if PPC_NUM_VECTORS > 44
        .long       vector44,   vector45,   vector46,   vector47
#endif
#if PPC_NUM_VECTORS > 48
        .long       vector48,   vector49,   vector50,   vector51
#endif
#if PPC_NUM_VECTORS > 52
        .long       vector52,   vector53,   vector54,   vector55
#endif
#if PPC_NUM_VECTORS > 56
        .long       vector56,   vector57,   vector58,   vector59
#endif
#if PPC_NUM_VECTORS > 60
        .long       vector60,   vector61,   vector62,   vector63
#endif
#if PPC_NUM_VECTORS > 64
        .long       vector64,   vector65,   vector66,   vector67
#endif
#if PPC_NUM_VECTORS > 68
        .long       vector68,   vector69,   vector70,   vector71
#endif
#if PPC_NUM_VECTORS > 72
        .long       vector72,   vector73,   vector74,   vector75
#endif
#if PPC_NUM_VECTORS > 76
        .long       vector76,   vector77,   vector78,   vector79
#endif
#if PPC_NUM_VECTORS > 80
        .long       vector80,   vector81,   vector82,   vector83
#endif
#if PPC_NUM_VECTORS > 84
        .long       vector84,   vector85,   vector86,   vector87
#endif
#if PPC_NUM_VECTORS > 88
        .long       vector88,   vector89,   vector90,   vector91
#endif
#if PPC_NUM_VECTORS > 92
        .long       vector92,   vector93,   vector94,   vector95
#endif
#if PPC_NUM_VECTORS > 96
        .long       vector96,   vector97,   vector98,   vector99
#endif
#if PPC_NUM_VECTORS > 100
        .long       vector100,  vector101,  vector102,  vector103
#endif
#if PPC_NUM_VECTORS > 104
        .long       vector104,  vector105,  vector106,  vector107
#endif
#if PPC_NUM_VECTORS > 108
        .long       vector108,  vector109,  vector110,  vector111
#endif
#if PPC_NUM_VECTORS > 112
        .long       vector112,  vector113,  vector114,  vector115
#endif
#if PPC_NUM_VECTORS > 116
        .long       vector116,  vector117,  vector118,  vector119
#endif
#if PPC_NUM_VECTORS > 120
        .long       vector120,  vector121,  vector122,  vector123
#endif
#if PPC_NUM_VECTORS > 124
        .long       vector124,  vector125,  vector126,  vector127
#endif
#if PPC_NUM_VECTORS > 128
        .long       vector128,  vector129,  vector130,  vector131
#endif
#if PPC_NUM_VECTORS > 132
        .long       vector132,  vector133,  vector134,  vector135
#endif
#if PPC_NUM_VECTORS > 136
        .long       vector136,  vector137,  vector138,  vector139
#endif
#if PPC_NUM_VECTORS > 140
        .long       vector140,  vector141,  vector142,  vector143
#endif
#if PPC_NUM_VECTORS > 144
        .long       vector144,  vector145,  vector146,  vector147
#endif
#if PPC_NUM_VECTORS > 148
        .long       vector148,  vector149,  vector150,  vector151
#endif
#if PPC_NUM_VECTORS > 152
        .long       vector152,  vector153,  vector154,  vector155
#endif
#if PPC_NUM_VECTORS > 156
        .long       vector156,  vector157,  vector158,  vector159
#endif
#if PPC_NUM_VECTORS > 160
        .long       vector160,  vector161,  vector162,  vector163
#endif
#if PPC_NUM_VECTORS > 164
        .long       vector164,  vector165,  vector166,  vector167
#endif
#if PPC_NUM_VECTORS > 168
        .long       vector168,  vector169,  vector170,  vector171
#endif
#if PPC_NUM_VECTORS > 172
        .long       vector172,  vector173,  vector174,  vector175
#endif
#if PPC_NUM_VECTORS > 176
        .long       vector176,  vector177,  vector178,  vector179
#endif
#if PPC_NUM_VECTORS > 180
        .long       vector180,  vector181,  vector182,  vector183
#endif
#if PPC_NUM_VECTORS > 184
        .long       vector184,  vector185,  vector186,  vector187
#endif
#if PPC_NUM_VECTORS > 188
        .long       vector188,  vector189,  vector190,  vector191
#endif
#if PPC_NUM_VECTORS > 192
        .long       vector192,  vector193,  vector194,  vector195
#endif
#if PPC_NUM_VECTORS > 196
        .long       vector196,  vector197,  vector198,  vector199
#endif
#if PPC_NUM_VECTORS > 200
        .long       vector200,  vector201,  vector202,  vector203
#endif
#if PPC_NUM_VECTORS > 204
        .long       vector204,  vector205,  vector206,  vector207
#endif
#if PPC_NUM_VECTORS > 208
        .long       vector208,  vector209,  vector210,  vector211
#endif
#if PPC_NUM_VECTORS > 212
        .long       vector212,  vector213,  vector214,  vector215
#endif
#if PPC_NUM_VECTORS > 216
        .long       vector216,  vector217,  vector218,  vector219
#endif
#if PPC_NUM_VECTORS > 220
        .long       vector220,  vector221,  vector222,  vector223
#endif
#if PPC_NUM_VECTORS > 224
        .long       vector224,  vector225,  vector226,  vector227
#endif
#if PPC_NUM_VECTORS > 228
        .long       vector228,  vector229,  vector230,  vector231
#endif
#if PPC_NUM_VECTORS > 232
        .long       vector232,  vector233,  vector234,  vector235
#endif
#if PPC_NUM_VECTORS > 236
        .long       vector236,  vector237,  vector238,  vector239
#endif
#if PPC_NUM_VECTORS > 240
        .long       vector240,  vector241,  vector242,  vector243
#endif
#if PPC_NUM_VECTORS > 244
        .long       vector244,  vector245,  vector246,  vector247
#endif
#if PPC_NUM_VECTORS > 248
        .long       vector248,  vector249,  vector250,  vector251
#endif
#if PPC_NUM_VECTORS > 252
        .long       vector252,  vector253,  vector254,  vector255
#endif
#if PPC_NUM_VECTORS > 256
        .long       vector256,  vector257,  vector258,  vector259
#endif
#if PPC_NUM_VECTORS > 260
        .long       vector260,  vector261,  vector262,  vector263
#endif
#if PPC_NUM_VECTORS > 264
        .long       vector264,  vector265,  vector266,  vector267
#endif
#if PPC_NUM_VECTORS > 268
        .long       vector268,  vector269,  vector270,  vector271
#endif
#if PPC_NUM_VECTORS > 272
        .long       vector272,  vector273,  vector274,  vector275
#endif
#if PPC_NUM_VECTORS > 276
        .long       vector276,  vector277,  vector278,  vector279
#endif
#if PPC_NUM_VECTORS > 280
        .long       vector280,  vector281,  vector282,  vector283
#endif
#if PPC_NUM_VECTORS > 284
        .long       vector284,  vector285,  vector286,  vector287
#endif
#if PPC_NUM_VECTORS > 288
        .long       vector288,  vector289,  vector290,  vector291
#endif
#if PPC_NUM_VECTORS > 292
        .long       vector292,  vector293,  vector294,  vector295
#endif
#if PPC_NUM_VECTORS > 296
        .long       vector296,  vector297,  vector298,  vector299
#endif
#if PPC_NUM_VECTORS > 300
        .long       vector300,  vector301,  vector302,  vector303
#endif
#if PPC_NUM_VECTORS > 304
        .long       vector304,  vector305,  vector306,  vector307
#endif
#if PPC_NUM_VECTORS > 308
        .long       vector308,  vector309,  vector310,  vector311
#endif
#if PPC_NUM_VECTORS > 312
        .long       vector312,  vector313,  vector314,  vector315
#endif
#if PPC_NUM_VECTORS > 316
        .long       vector316,  vector317,  vector318,  vector319
#endif
#if PPC_NUM_VECTORS > 320
        .long       vector320,  vector321,  vector322,  vector323
#endif
#if PPC_NUM_VECTORS > 324
        .long       vector324,  vector325,  vector326,  vector327
#endif
#if PPC_NUM_VECTORS > 328
        .long       vector328,  vector329,  vector330,  vector331
#endif
#if PPC_NUM_VECTORS > 332
        .long       vector332,  vector333,  vector334,  vector335
#endif
#if PPC_NUM_VECTORS > 336
        .long       vector336,  vector337,  vector338,  vector339
#endif
#if PPC_NUM_VECTORS > 340
        .long       vector340,  vector341,  vector342,  vector343
#endif
#if PPC_NUM_VECTORS > 344
        .long       vector344,  vector345,  vector346,  vector347
#endif
#if PPC_NUM_VECTORS > 348
        .long       vector348,  vector349,  vector350,  vector351
#endif
#if PPC_NUM_VECTORS > 352
        .long       vector352,  vector353,  vector354,  vector355
#endif
#if PPC_NUM_VECTORS > 356
        .long       vector356,  vector357,  vector358,  vector359
#endif
#if PPC_NUM_VECTORS > 360
        .long       vector360,  vector361,  vector362,  vector363
#endif
#if PPC_NUM_VECTORS > 364
        .long       vector364,  vector365,  vector366,  vector367
#endif
#if PPC_NUM_VECTORS > 368
        .long       vector368,  vector369,  vector370,  vector371
#endif
#if PPC_NUM_VECTORS > 372
        .long       vector372,  vector373,  vector374,  vector375
#endif
#if PPC_NUM_VECTORS > 376
        .long       vector376,  vector377,  vector378,  vector379
#endif
#if PPC_NUM_VECTORS > 380
        .long       vector380,  vector381,  vector382,  vector383
#endif
#if PPC_NUM_VECTORS > 384
        .long       vector384,  vector385,  vector386,  vector387
#endif
#if PPC_NUM_VECTORS > 388
        .long       vector388,  vector389,  vector390,  vector391
#endif
#if PPC_NUM_VECTORS > 392
        .long       vector392,  vector393,  vector394,  vector395
#endif
#if PPC_NUM_VECTORS > 396
        .long       vector396,  vector397,  vector398,  vector399
#endif
#if PPC_NUM_VECTORS > 400
        .long       vector400,  vector401,  vector402,  vector403
#endif
#if PPC_NUM_VECTORS > 404
        .long       vector404,  vector405,  vector406,  vector407
#endif
#if PPC_NUM_VECTORS > 408
        .long       vector408,  vector409,  vector410,  vector411
#endif
#if PPC_NUM_VECTORS > 412
        .long       vector412,  vector413,  vector414,  vector415
#endif
#if PPC_NUM_VECTORS > 416
        .long       vector416,  vector417,  vector418,  vector419
#endif
#if PPC_NUM_VECTORS > 420
        .long       vector420,  vector421,  vector422,  vector423
#endif
#if PPC_NUM_VECTORS > 424
        .long       vector424,  vector425,  vector426,  vector427
#endif
#if PPC_NUM_VECTORS > 428
        .long       vector428,  vector429,  vector430,  vector431
#endif
#if PPC_NUM_VECTORS > 432
        .long       vector432,  vector433,  vector434,  vector435
#endif
#if PPC_NUM_VECTORS > 436
        .long       vector436,  vector437,  vector438,  vector439
#endif
#if PPC_NUM_VECTORS > 440
        .long       vector440,  vector441,  vector442,  vector443
#endif
#if PPC_NUM_VECTORS > 444
        .long       vector444,  vector445,  vector446,  vector447
#endif
#if PPC_NUM_VECTORS > 448
        .long       vector448,  vector449,  vector450,  vector451
#endif
#if PPC_NUM_VECTORS > 452
        .long       vector452,  vector453,  vector454,  vector455
#endif
#if PPC_NUM_VECTORS > 456
        .long       vector456,  vector457,  vector458,  vector459
#endif
#if PPC_NUM_VECTORS > 460
        .long       vector460,  vector461,  vector462,  vector463
#endif
#if PPC_NUM_VECTORS > 464
        .long       vector464,  vector465,  vector466,  vector467
#endif
#if PPC_NUM_VECTORS > 468
        .long       vector468,  vector469,  vector470,  vector471
#endif
#if PPC_NUM_VECTORS > 472
        .long       vector472,  vector473,  vector474,  vector475
#endif
#if PPC_NUM_VECTORS > 476
        .long       vector476,  vector477,  vector478,  vector479
#endif
#if PPC_NUM_VECTORS > 480
        .long       vector480,  vector481,  vector482,  vector483
#endif
#if PPC_NUM_VECTORS > 484
        .long       vector484,  vector485,  vector486,  vector487
#endif
#if PPC_NUM_VECTORS > 488
        .long       vector488,  vector489,  vector490,  vector491
#endif
#if PPC_NUM_VECTORS > 492
        .long       vector492,  vector493,  vector494,  vector495
#endif
#if PPC_NUM_VECTORS > 496
        .long       vector496,  vector497,  vector498,  vector499
#endif
#if PPC_NUM_VECTORS > 500
        .long       vector500,  vector501,  vector502,  vector503
#endif
#if PPC_NUM_VECTORS > 504
        .long       vector504,  vector505,  vector506,  vector507
#endif
#if PPC_NUM_VECTORS > 508
        .long       vector508,  vector509,  vector510,  vector511
#endif
#if PPC_NUM_VECTORS > 512
        .long       vector512,  vector513,  vector514,  vector515
#endif
#if PPC_NUM_VECTORS > 516
        .long       vector516,  vector517,  vector518,  vector519
#endif
#if PPC_NUM_VECTORS > 520
        .long       vector520,  vector521,  vector522,  vector523
#endif
#if PPC_NUM_VECTORS > 524
        .long       vector524,  vector525,  vector526,  vector527
#endif
#if PPC_NUM_VECTORS > 528
        .long       vector528,  vector529,  vector530,  vector531
#endif
#if PPC_NUM_VECTORS > 532
        .long       vector532,  vector533,  vector534,  vector535
#endif
#if PPC_NUM_VECTORS > 536
        .long       vector536,  vector537,  vector538,  vector539
#endif
#if PPC_NUM_VECTORS > 540
        .long       vector540,  vector541,  vector542,  vector543
#endif
#if PPC_NUM_VECTORS > 544
        .long       vector544,  vector545,  vector546,  vector547
#endif
#if PPC_NUM_VECTORS > 548
        .long       vector548,  vector549,  vector550,  vector551
#endif
#if PPC_NUM_VECTORS > 552
        .long       vector552,  vector553,  vector554,  vector555
#endif
#if PPC_NUM_VECTORS > 556
        .long       vector556,  vector557,  vector558,  vector559
#endif
#if PPC_NUM_VECTORS > 560
        .long       vector560,  vector561,  vector562,  vector563
#endif
#if PPC_NUM_VECTORS > 564
        .long       vector564,  vector565,  vector566,  vector567
#endif
#if PPC_NUM_VECTORS > 568
        .long       vector568,  vector569,  vector570,  vector571
#endif
#if PPC_NUM_VECTORS > 572
        .long       vector572,  vector573,  vector574,  vector575
#endif
#if PPC_NUM_VECTORS > 576
        .long       vector576,  vector577,  vector578,  vector579
#endif
#if PPC_NUM_VECTORS > 580
        .long       vector580,  vector581,  vector582,  vector583
#endif
#if PPC_NUM_VECTORS > 584
        .long       vector584,  vector585,  vector586,  vector587
#endif
#if PPC_NUM_VECTORS > 588
        .long       vector588,  vector589,  vector590,  vector591
#endif
#if PPC_NUM_VECTORS > 592
        .long       vector592,  vector593,  vector594,  vector595
#endif
#if PPC_NUM_VECTORS > 596
        .long       vector596,  vector597,  vector598,  vector599
#endif
#if PPC_NUM_VECTORS > 600
        .long       vector600,  vector601,  vector602,  vector603
#endif
#if PPC_NUM_VECTORS > 604
        .long       vector604,  vector605,  vector606,  vector607
#endif
#if PPC_NUM_VECTORS > 608
        .long       vector608,  vector609,  vector610,  vector611
#endif
#if PPC_NUM_VECTORS > 612
        .long       vector612,  vector613,  vector614,  vector615
#endif
#if PPC_NUM_VECTORS > 616
        .long       vector616,  vector617,  vector618,  vector619
#endif
#if PPC_NUM_VECTORS > 620
        .long       vector620,  vector621,  vector622,  vector623
#endif
#if PPC_NUM_VECTORS > 624
        .long       vector624,  vector625,  vector626,  vector627
#endif
#if PPC_NUM_VECTORS > 628
        .long       vector628,  vector629,  vector630,  vector631
#endif
#if PPC_NUM_VECTORS > 632
        .long       vector632,  vector633,  vector634,  vector635
#endif
#if PPC_NUM_VECTORS > 636
        .long       vector636,  vector637,  vector638,  vector639
#endif
#if PPC_NUM_VECTORS > 640
        .long       vector640,  vector641,  vector642,  vector643
#endif
#if PPC_NUM_VECTORS > 644
        .long       vector644,  vector645,  vector646,  vector647
#endif
#if PPC_NUM_VECTORS > 648
        .long       vector648,  vector649,  vector650,  vector651
#endif
#if PPC_NUM_VECTORS > 652
        .long       vector652,  vector653,  vector654,  vector655
#endif
#if PPC_NUM_VECTORS > 656
        .long       vector656,  vector657,  vector658,  vector659
#endif
#if PPC_NUM_VECTORS > 660
        .long       vector660,  vector661,  vector662,  vector663
#endif
#if PPC_NUM_VECTORS > 664
        .long       vector664,  vector665,  vector666,  vector667
#endif
#if PPC_NUM_VECTORS > 668
        .long       vector668,  vector669,  vector670,  vector671
#endif
#if PPC_NUM_VECTORS > 672
        .long       vector672,  vector673,  vector674,  vector675
#endif
#if PPC_NUM_VECTORS > 676
        .long       vector676,  vector677,  vector678,  vector679
#endif
#if PPC_NUM_VECTORS > 680
        .long       vector680,  vector681,  vector682,  vector683
#endif
#if PPC_NUM_VECTORS > 684
        .long       vector684,  vector685,  vector686,  vector687
#endif
#if PPC_NUM_VECTORS > 688
        .long       vector688,  vector689,  vector690,  vector691
#endif
#if PPC_NUM_VECTORS > 692
        .long       vector692,  vector693,  vector694,  vector695
#endif
#if PPC_NUM_VECTORS > 696
        .long       vector696,  vector697,  vector698,  vector699
#endif
#if PPC_NUM_VECTORS > 700
        .long       vector700,  vector701,  vector702,  vector703
#endif
#if PPC_NUM_VECTORS > 704
        .long       vector704,  vector705,  vector706,  vector707
#endif
#if PPC_NUM_VECTORS > 708
        .long       vector708,  vector709,  vector710,  vector711
#endif
#if PPC_NUM_VECTORS > 712
        .long       vector712,  vector713,  vector714,  vector715
#endif
#if PPC_NUM_VECTORS > 716
        .long       vector716,  vector717,  vector718,  vector719
#endif
#if PPC_NUM_VECTORS > 720
        .long       vector720,  vector721,  vector722,  vector723
#endif
#if PPC_NUM_VECTORS > 724
        .long       vector724,  vector725,  vector726,  vector727
#endif
#if PPC_NUM_VECTORS > 728
        .long       vector728,  vector729,  vector730,  vector731
#endif
#if PPC_NUM_VECTORS > 732
        .long       vector732,  vector733,  vector734,  vector735
#endif
#if PPC_NUM_VECTORS > 736
        .long       vector736,  vector737,  vector738,  vector739
#endif
#if PPC_NUM_VECTORS > 740
        .long       vector740,  vector741,  vector742,  vector743
#endif
#if PPC_NUM_VECTORS > 744
        .long       vector744,  vector745,  vector746,  vector747
#endif
#if PPC_NUM_VECTORS > 748
        .long       vector748,  vector749,  vector750,  vector751
#endif
#if PPC_NUM_VECTORS > 752
        .long       vector752,  vector753,  vector754,  vector755
#endif
#if PPC_NUM_VECTORS > 756
        .long       vector756,  vector757,  vector758,  vector759
#endif
#if PPC_NUM_VECTORS > 760
        .long       vector760,  vector761,  vector762,  vector763
#endif
#if PPC_NUM_VECTORS > 764
        .long       vector764,  vector765,  vector766,  vector767
#endif
#if PPC_NUM_VECTORS > 768
        .long       vector768,  vector769,  vector770,  vector771
#endif
#if PPC_NUM_VECTORS > 772
        .long       vector772,  vector773,  vector774,  vector775
#endif
#if PPC_NUM_VECTORS > 776
        .long       vector776,  vector777,  vector778,  vector779
#endif
#if PPC_NUM_VECTORS > 780
        .long       vector780,  vector781,  vector782,  vector783
#endif
#if PPC_NUM_VECTORS > 784
        .long       vector784,  vector785,  vector786,  vector787
#endif
#if PPC_NUM_VECTORS > 788
        .long       vector788,  vector789,  vector790,  vector791
#endif
#if PPC_NUM_VECTORS > 792
        .long       vector792,  vector793,  vector794,  vector795
#endif
#if PPC_NUM_VECTORS > 796
        .long       vector796,  vector797,  vector798,  vector799
#endif
#if PPC_NUM_VECTORS > 800
        .long       vector800,  vector801,  vector802,  vector803
#endif
#if PPC_NUM_VECTORS > 804
        .long       vector804,  vector805,  vector806,  vector807
#endif
#if PPC_NUM_VECTORS > 808
        .long       vector808,  vector809,  vector810,  vector811
#endif
#if PPC_NUM_VECTORS > 812
        .long       vector812,  vector813,  vector814,  vector815
#endif
#if PPC_NUM_VECTORS > 816
        .long       vector816,  vector817,  vector818,  vector819
#endif
#if PPC_NUM_VECTORS > 820
        .long       vector820,  vector821,  vector822,  vector823
#endif
#if PPC_NUM_VECTORS > 824
        .long       vector824,  vector825,  vector826,  vector827
#endif
#if PPC_NUM_VECTORS > 828
        .long       vector828,  vector829,  vector830,  vector831
#endif
#if PPC_NUM_VECTORS > 832
        .long       vector832,  vector833,  vector834,  vector835
#endif
#if PPC_NUM_VECTORS > 836
        .long       vector836,  vector837,  vector838,  vector839
#endif
#if PPC_NUM_VECTORS > 840
        .long       vector840,  vector841,  vector842,  vector843
#endif
#if PPC_NUM_VECTORS > 844
        .long       vector844,  vector845,  vector846,  vector847
#endif
#if PPC_NUM_VECTORS > 848
        .long       vector848,  vector849,  vector850,  vector851
#endif
#if PPC_NUM_VECTORS > 852
        .long       vector852,  vector853,  vector854,  vector855
#endif
#if PPC_NUM_VECTORS > 856
        .long       vector856,  vector857,  vector858,  vector859
#endif
#if PPC_NUM_VECTORS > 860
        .long       vector860,  vector861,  vector862,  vector863
#endif
#if PPC_NUM_VECTORS > 864
        .long       vector864,  vector865,  vector866,  vector867
#endif
#if PPC_NUM_VECTORS > 868
        .long       vector868,  vector869,  vector870,  vector871
#endif
#if PPC_NUM_VECTORS > 872
        .long       vector872,  vector873,  vector874,  vector875
#endif
#if PPC_NUM_VECTORS > 876
        .long       vector876,  vector877,  vector878,  vector879
#endif
#if PPC_NUM_VECTORS > 880
        .long       vector880,  vector881,  vector882,  vector883
#endif
#if PPC_NUM_VECTORS > 884
        .long       vector884,  vector885,  vector886,  vector887
#endif
#if PPC_NUM_VECTORS > 888
        .long       vector888,  vector889,  vector890,  vector891
#endif
#if PPC_NUM_VECTORS > 892
        .long       vector892,  vector893,  vector894,  vector895
#endif
#if PPC_NUM_VECTORS > 896
        .long       vector896,  vector897,  vector898,  vector899
#endif
#if PPC_NUM_VECTORS > 900
        .long       vector900,  vector901,  vector902,  vector903
#endif
#if PPC_NUM_VECTORS > 904
        .long       vector904,  vector905,  vector906,  vector907
#endif
#if PPC_NUM_VECTORS > 908
        .long       vector908,  vector909,  vector910,  vector911
#endif
#if PPC_NUM_VECTORS > 912
        .long       vector912,  vector913,  vector914,  vector915
#endif
#if PPC_NUM_VECTORS > 916
        .long       vector916,  vector917,  vector918,  vector919
#endif
#if PPC_NUM_VECTORS > 920
        .long       vector920,  vector921,  vector922,  vector923
#endif
#if PPC_NUM_VECTORS > 924
        .long       vector924,  vector925,  vector926,  vector927
#endif
#if PPC_NUM_VECTORS > 928
        .long       vector928,  vector929,  vector930,  vector931
#endif
#if PPC_NUM_VECTORS > 932
        .long       vector932,  vector933,  vector934,  vector935
#endif
#if PPC_NUM_VECTORS > 936
        .long       vector936,  vector937,  vector938,  vector939
#endif
#if PPC_NUM_VECTORS > 940
        .long       vector940,  vector941,  vector942,  vector943
#endif
#if PPC_NUM_VECTORS > 944
        .long       vector944,  vector945,  vector946,  vector947
#endif
#if PPC_NUM_VECTORS > 948
        .long       vector948,  vector949,  vector950,  vector951
#endif
#if PPC_NUM_VECTORS > 952
        .long       vector952,  vector953,  vector954,  vector955
#endif
#if PPC_NUM_VECTORS > 956
        .long       vector956,  vector957,  vector958,  vector959
#endif
#if PPC_NUM_VECTORS > 960
        .long       vector960,  vector961,  vector962,  vector963
#endif
#if PPC_NUM_VECTORS > 964
        .long       vector964,  vector965,  vector966,  vector967
#endif
#if PPC_NUM_VECTORS > 968
        .long       vector968,  vector969,  vector970,  vector971
#endif
#if PPC_NUM_VECTORS > 972
        .long       vector972,  vector973,  vector974,  vector975
#endif
#if PPC_NUM_VECTORS > 976
        .long       vector976,  vector977,  vector978,  vector979
#endif
#if PPC_NUM_VECTORS > 980
        .long       vector980,  vector981,  vector982,  vector983
#endif
#if PPC_NUM_VECTORS > 984
        .long       vector984,  vector985,  vector986,  vector987
#endif
#if PPC_NUM_VECTORS > 988
        .long       vector988,  vector989,  vector990,  vector991
#endif
#if PPC_NUM_VECTORS > 992
        .long       vector992,  vector993,  vector994,  vector995
#endif
#if PPC_NUM_VECTORS > 996
        .long       vector996,  vector997,  vector998,  vector999
#endif
#if PPC_NUM_VECTORS > 1000
        .long       vector1000, vector1001, vector1002, vector1003
#endif
#if PPC_NUM_VECTORS > 1004
        .long       vector1004, vector1005, vector1006, vector1007
#endif
#if PPC_NUM_VECTORS > 1008
        .long       vector1008, vector1009, vector1010, vector1011
#endif
#if PPC_NUM_VECTORS > 1012
        .long       vector1012, vector1013, vector1014, vector1015
#endif
#if PPC_NUM_VECTORS > 1016
        .long       vector1016, vector1017, vector1018, vector1019
#endif
#if PPC_NUM_VECTORS > 1020
        .long       vector1020, vector1021, vector1022, vector1023
#endif

#endif /* !defined(__DOXYGEN__) */

/** @} */
