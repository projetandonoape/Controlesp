
const uint16_t PROGMEM gama[] = {
   0, 32, 45, 55, 64, 72, 78, 85, 90, 96,101,106,111,115,120,124,
 128,132,136,139,143,147,150,153,157,160,163,166,169,172,175,178,
 181,184,186,189,192,195,197,200,202,205,207,210,212,215,217,219,
 222,224,226,228,231,233,235,237,239,241,244,246,248,250,252,254,
 256,258,260,262,264,266,268,270,271,273,275,277,279,281,282,284,
 286,288,290,291,293,295,297,298,300,302,303,305,307,308,310,312,
 313,315,317,318,320,321,323,325,326,328,329,331,332,334,335,337,
 338,340,341,343,344,346,347,349,350,352,353,355,356,358,359,360,
 362,363,365,366,367,369,370,372,373,374,376,377,378,380,381,382,
 384,385,386,388,389,390,392,393,394,396,397,398,399,401,402,403,
 405,406,407,408,410,411,412,413,415,416,417,418,419,421,422,423,
 424,426,427,428,429,430,431,433,434,435,436,437,439,440,441,442,
 443,444,445,447,448,449,450,451,452,453,455,456,457,458,459,460,
 461,462,463,465,466,467,468,469,470,471,472,473,474,475,477,478,
 479,480,481,482,483,484,485,486,487,488,489,490,491,492,493,494,
 495,497,498,499,500,501,502,503,504,505,506,507,508,509,510,511,
 512,513,514,515,516,517,518,519,520,521,522,523,524,525,526,527,
 527,528,529,530,531,532,533,534,535,536,537,538,539,540,541,542,
 543,544,545,546,547,547,548,549,550,551,552,553,554,555,556,557,
 558,559,559,560,561,562,563,564,565,566,567,568,569,569,570,571,
 572,573,574,575,576,577,577,578,579,580,581,582,583,584,585,585,
 586,587,588,589,590,591,591,592,593,594,595,596,597,598,598,599,
 600,601,602,603,603,604,605,606,607,608,609,609,610,611,612,613,
 614,614,615,616,617,618,619,619,620,621,622,623,623,624,625,626,
 627,628,628,629,630,631,632,632,633,634,635,636,636,637,638,639,
 640,640,641,642,643,644,644,645,646,647,648,648,649,650,651,652,
 652,653,654,655,655,656,657,658,659,659,660,661,662,662,663,664,
 665,666,666,667,668,669,669,670,671,672,672,673,674,675,675,676,
 677,678,678,679,680,681,681,682,683,684,684,685,686,687,687,688,
 689,690,690,691,692,693,693,694,695,696,696,697,698,699,699,700,
 701,701,702,703,704,704,705,706,707,707,708,709,709,710,711,712,
 712,713,714,714,715,716,717,717,718,719,719,720,721,722,722,723,
 724,724,725,726,727,727,728,729,729,730,731,731,732,733,734,734,
 735,736,736,737,738,738,739,740,740,741,742,743,743,744,745,745,
 746,747,747,748,749,749,750,751,751,752,753,754,754,755,756,756,
 757,758,758,759,760,760,761,762,762,763,764,764,765,766,766,767,
 768,768,769,770,770,771,772,772,773,774,774,775,776,776,777,778,
 778,779,780,780,781,781,782,783,783,784,785,785,786,787,787,788,
 789,789,790,791,791,792,793,793,794,794,795,796,796,797,798,798,
 799,800,800,801,802,802,803,803,804,805,805,806,807,807,808,809,
 809,810,810,811,812,812,813,814,814,815,815,816,817,817,818,819,
 819,820,820,821,822,822,823,824,824,825,825,826,827,827,828,829,
 829,830,830,831,832,832,833,833,834,835,835,836,836,837,838,838,
 839,840,840,841,841,842,843,843,844,844,845,846,846,847,847,848,
 849,849,850,850,851,852,852,853,853,854,855,855,856,856,857,858,
 858,859,859,860,861,861,862,862,863,864,864,865,865,866,867,867,
 868,868,869,869,870,871,871,872,872,873,874,874,875,875,876,877,
 877,878,878,879,879,880,881,881,882,882,883,883,884,885,885,886,
 886,887,888,888,889,889,890,890,891,892,892,893,893,894,894,895,
 896,896,897,897,898,898,899,900,900,901,901,902,902,903,904,904,
 905,905,906,906,907,907,908,909,909,910,910,911,911,912,913,913,
 914,914,915,915,916,916,917,918,918,919,919,920,920,921,921,922,
 923,923,924,924,925,925,926,926,927,928,928,929,929,930,930,931,
 931,932,932,933,934,934,935,935,936,936,937,937,938,939,939,940,
 940,941,941,942,942,943,943,944,944,945,946,946,947,947,948,948,
 949,949,950,950,951,952,952,953,953,954,954,955,955,956,956,957,
 957,958,958,959,960,960,961,961,962,962,963,963,964,964,965,965,
 966,966,967,967,968,969,969,970,970,971,971,972,972,973,973,974,
 974,975,975,976,976,977,977,978,979,979,980,980,981,981,982,982,
 983,983,984,984,985,985,986,986,987,987,988,988,989,989,990,990,
 991,992,992,993,993,994,994,995,995,996,996,997,997,998,998,999,
 999,1000,1000,1001,1001,1002,1002,1003,1003,1004,1004,1005,1005,1006,1006,1007,
 1007,1008,1008,1009,1009,1010,1010,1011,1011,1012,1012,1013,1013,1014,1014,1015,
 1015,1016,1016,1017,1017,1018,1018,1019,1019,1020,1020,1021,1021,1022,1022,1023 };
