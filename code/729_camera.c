/************************************
 * 729_CODE
 * 729_camera.c
 * version 1
 ************************************/

#include "zf_common_headfile.h"
#define uint16_t uint16
#define uint8_t uint8
#define mt9v03x_image_dvp mt9v03x_image

//#define Ture                       //是否为普通惯导，定义为普通，无定义为特殊
//#define Have                     //库有无电磁，定义为有电磁，无定义为没有电磁

extern uint16 dl1a_distance_mm;
extern uint16 ADC_Filter[3];
extern int16 Adc_Err;
extern float Angle_z_temp;
extern int16 Encoder_Go;

image_t img_raw = DEF_IMAGE(NULL, MT9V03X_W, MT9V03X_H);
float thres;                      //全图局部阈值

bool start = 0;                  //开始标志
int16 go_strat = 0;               //出库标志,1表示正在出库
int code_mode = 1;                //速度方案，1：稳定代码  2：加速代码
int16 crs_turn = 0;               //十字左右直走标准，0直，1左，2右
int16 gara_turn = 1;              //入库方向，0，不判断，1左，2右
int16 open_in_start = 1;          //断路惯导标志,0为正常，1为启用惯导
uint16 barr_turn = 1;             //1左过障碍，2右过障碍

int16 flag_ramp = 0;              //坡道标志，决定误差取决于那几行
uint16 cen_black = 0;             //中线位置为黑色计数
uint16 cen_black_up = 0;          //中线上端位置为黑色计数
uint16 flag_cen_false = 0;        //中线错误标志
uint16 undo = 10;                  //中线从没有计算电磁，到开始计算电磁次数，20次以上代表长时间计算了电磁
uint16 cir_undo = 10;             //环岛。。。。。
uint16 crs_undo = 10;             //十字....

//注意：所有与图像相关的行下标i都是从远到近对应0~MT9V03X_H-1，因此循环一般从i=MT9V03X_H-1开始

int16 leftside[MT9V03X_DVP_H];      //左边界j值数组，0表示左边界丢失
int16 rightside[MT9V03X_DVP_H];     //右边界j值数组，MT9V03X_W-1表示右边界丢失
int16 centerline[MT9V03X_DVP_H];    //中心线数组
int16 roweff_flag[MT9V03X_DVP_H];   //行有效标志位，（初步类型判断）0表示行左右边界均存在，1表示左边界缺失，2表示行右边界缺失，3表示行左右边界均缺失，-1表示该行无效
//（完整类型判断）0表示双边界均存在，1表示左边界只是看不见了但不是突变缺失，2表示有边界看不见了但不是突变缺失，3表示双边界都没有，但中间都是赛道，一般用于十字路口判断
//4表示三岔的发散类型，5表示左边界（相对于正常左边界）突变缺失，6表示右边界（相对于正常右边界）突变缺失，7表示左边界向内突变，8表示右边界向内突变，-1表示该行无效（其中7和8只在环岛内使用）

//左边界标准数组，表示当车模处于赛道正中间时看到的左边界数组，用来计算当车模偏离赛道中心时的偏差值
int16 ls_std[MT9V03X_DVP_H];

//右边界标准数组，表示当车模处于赛道正中间时看到的右边界数组，用来计算当车模偏离赛道中心时的偏差值，以上两个数组数据靠实测取得
int16 rs_std[MT9V03X_DVP_H];

image_parameter image_para;          //图像参数，内容见定义，包括位置偏差与赛道类型
int16 img_mode = 0;                  //图像识别模数，0表示正常模式，1表示环岛预识别模式，2表示环岛模式，3表示障碍模式,4入库模式,5断路模式,6十字模式，7坡道模式
int16 row_width[MT9V03X_DVP_H];      //各行宽度数组
uint16 real_distans[MT9V03X_DVP_H];  //每一行所对应得到现实距离

//边界分段判断
int16 part_type[MT9V03X_DVP_H];      //分段类型
int16 part_index[MT9V03X_DVP_H];     //分段终点下标
int16 part_num;                      //分段数
int16 part_sum[8];                   //各类型分段数和
int16 part_sum__1;                   //-1分段树

int16 road_len=0;                    //赛道长到的I行

float offset_menb[3] = {0,0,0};         //误差记录
int16 straight_distain[3] = {107,107,107}; //三次直道距离
int16 ave_distain = 107;                 //平均距离
float offest_cam = 0;                   //摄像头当前误差
float offest_K = 0;                     //图像曲线斜率

//拐点位置
uint16 right_turn_down[2];
uint16 left_turn_down[2];
uint16 left_turn_up[2];
uint16 right_turn_up[2];
//拐点标志位
uint16 youxia_guai = 0;
uint16 zuoxia_guai = 0;
uint16 zuoshang_guai = 0;
uint16 youshang_guai = 0;
//左右边界cos值[100,0]~[1,0]
uint16 right_cos[MT9V03X_DVP_H];
uint16 left_cos[MT9V03X_DVP_H];
uint16 right_cos_ave = 0;
uint16 left_cos_ave = 0;
uint16 right_line_min[2];
uint16 left_line_max[2];

//环岛特殊变量
int16 CIR_W_S3 = 0;                  //环岛S3阶段的距离
float CIR_W_S3_max = 35;             //环岛S3阶段的误差最大值
float offset_S5 = 0;                 //环岛S5阶段的开环偏差值
float weight_S5 = 0;                 //环岛S5阶段的开环偏差累加计数，offset_S5/weight_S5为真正的偏差值
image_type last_type=0;               //图像上一帧的类型，用于判断环岛第二阶段
uint16 CIR_END_T = 0;                //环岛结束计时，用于开启环岛识别
uint16 CIR_ENABLE = 1;               //环岛识别开启标志
int16 cirsud_i;                      //环岛入环时的特殊点i坐标，特殊点为环岛外边界与直道边界的交点（此交点有两个，此处指入环时的点）
int16 S3S4_flag = 0;                 //环岛S3到S4阶段的标志位，当值为1时说明从S3跳到S4的那一帧
float s3_encode = 0;
uint16 cir_num = 0;                  //环岛运行次数

//十字特殊变量
int16 crs_t = 0;                     //十字运行时间限定
float offest_crs_S3;                 //十字S3阶段的需要积累偏差
float weight_crs_S3 = 1;             //十字S3阶段的需要积累偏差加计数
uint16 crs_num = 0;                  //十字运行次数
uint16 crs_turn_row[5];              //十字转向数组,0直，1左，2右

//障碍特殊变量
float Barr_S2_Z = 0;                  //s1跳变到s2的z轴数据
float offest_quike = 55;              //障碍误差快
float offest_smell = 55;              //障碍误差慢
int32 Barr_Encode = 0;                //障碍编码器计数
uint16 Barr_2_flag = 0;

//库特殊变量
int32 Gara_Encode = 0;                //入库编码器积分
int32 Go_Encode = 0;                  //有电磁出库编码器积分

//出库特殊变量
int16 go_turn = 0;
int16 go_t = 0;                       //刚开机会抖

//断路特殊变量
int16 open_t = 0;                     //断路看到白线运行时间限定
float goal_z = -40;                    //惯导的目标角度
int32 Open_Encode = 0;                //断路编码器计数
int32 Open_End_Encode = 0;           //出断路编码器积分
int16 flag_end = 0;                  //出断路标志
int16 end_turn = 0;                  //惯导断路进入道路方向
float OPpen_Encode = 0;              //惯导编码器值
float goal_angle[5];
float goal_encode[5];


int16 CURSE_OFFSET_1 = 10;             //弯道的固定偏差，用于在弯道时调整车模位置


//-------------------------------------------------------------------------------------------------------------------
//  函数功能：图像标准初始化
//  参数：void
//  返回值：int16
//-------------------------------------------------------------------------------------------------------------------
int16 image_process_init(void)
{
    /* 高度 29  前瞻80
     * ls_std[0] = 61; ls_std[1] = 61; ls_std[2] = 60; ls_std[3] = 60; ls_std[4] = 59;
    ls_std[5] = 56; ls_std[6] = 55; ls_std[7] = 55; ls_std[8] = 54; ls_std[9] = 53;
    ls_std[10] = 53; ls_std[11] = 52; ls_std[12] = 51; ls_std[13] = 50; ls_std[14] = 50;
    ls_std[15] = 49; ls_std[16] = 48; ls_std[17] = 48; ls_std[18] = 48; ls_std[19] = 48;
    ls_std[20] = 47; ls_std[21] = 44; ls_std[22] = 44; ls_std[23] = 44; ls_std[24] = 43;
    ls_std[25] = 42; ls_std[26] = 42; ls_std[27] = 42; ls_std[28] = 41; ls_std[29] = 38;
    ls_std[30] = 38; ls_std[31] = 38; ls_std[32] = 37; ls_std[33] = 36; ls_std[34] = 35;
    ls_std[35] = 35; ls_std[36] = 34; ls_std[37] = 33; ls_std[38] = 34; ls_std[39] = 35;
    ls_std[40] = 31; ls_std[41] = 30; ls_std[42] = 30; ls_std[43] = 32; ls_std[44] = 32;
    ls_std[45] = 31; ls_std[46] = 31; ls_std[47] = 32; ls_std[48] = 32; ls_std[49] = 33;

    rs_std[0] = 123; rs_std[1] = 124; rs_std[2] = 126; rs_std[3] = 127; rs_std[4] = 129;
    rs_std[5] = 129; rs_std[6] = 129; rs_std[7] = 131; rs_std[8] = 131; rs_std[9] = 131;
    rs_std[10] = 132; rs_std[11] = 133; rs_std[12] = 134; rs_std[13] = 134; rs_std[14] = 134;
    rs_std[15] = 135; rs_std[16] = 136; rs_std[17] = 138; rs_std[18] = 138; rs_std[19] = 138;
    rs_std[20] = 140; rs_std[21] = 140; rs_std[22] = 140; rs_std[23] = 140; rs_std[24] = 141;
    rs_std[25] = 142; rs_std[26] = 144; rs_std[27] = 144; rs_std[28] = 146; rs_std[29] = 146;
    rs_std[30] = 146; rs_std[31] = 146; rs_std[32] = 146; rs_std[33] = 149; rs_std[34] = 149;
    rs_std[35] = 150; rs_std[36] = 151; rs_std[37] = 152; rs_std[38] = 152; rs_std[39] = 152;
    rs_std[40] = 153; rs_std[41] = 154; rs_std[42] = 156; rs_std[43] = 156; rs_std[44] = 156;
    rs_std[45] = 158; rs_std[46] = 158; rs_std[47] = 158; rs_std[48] = 160; rs_std[49] = 160;*/


    /*高度 22 前瞻 80
    ls_std[0] = 66; ls_std[1] = 65; ls_std[2] = 65; ls_std[3] = 62; ls_std[4] = 62;
    ls_std[5] = 62; ls_std[6] = 59; ls_std[7] = 59; ls_std[8] = 59; ls_std[9] = 58;
    ls_std[10] = 55; ls_std[11] = 54; ls_std[12] = 53; ls_std[13] = 53; ls_std[14] = 53;
    ls_std[15] = 52; ls_std[16] = 49; ls_std[17] = 49; ls_std[18] = 46; ls_std[19] = 46;
    ls_std[20] = 47; ls_std[21] = 43; ls_std[22] = 43; ls_std[23] = 43; ls_std[24] = 40;
    ls_std[25] = 40; ls_std[26] = 40; ls_std[27] = 37; ls_std[28] = 37; ls_std[29] = 37;
    ls_std[30] = 34; ls_std[31] = 34; ls_std[32] = 34; ls_std[33] = 31; ls_std[34] = 31;
    ls_std[35] = 31; ls_std[36] = 28; ls_std[37] = 28; ls_std[38] = 28; ls_std[39] = 25;
    ls_std[40] = 25; ls_std[41] = 24; ls_std[42] = 21; ls_std[43] = 21; ls_std[44] = 21;
    ls_std[45] = 18; ls_std[46] = 18; ls_std[47] = 17; ls_std[48] = 16; ls_std[49] = 16;

    rs_std[0] = 130; rs_std[1] = 131; rs_std[2] = 131; rs_std[3] = 134; rs_std[4] = 134;
    rs_std[5] = 134; rs_std[6] = 137; rs_std[7] = 137; rs_std[8] = 137; rs_std[9] = 139;
    rs_std[10] = 139; rs_std[11] = 141; rs_std[12] = 140; rs_std[13] = 143; rs_std[14] = 143;
    rs_std[15] = 145; rs_std[16] = 145; rs_std[17] = 145; rs_std[18] = 148; rs_std[19] = 148;
    rs_std[20] = 148; rs_std[21] = 151; rs_std[22] = 151; rs_std[23] = 151; rs_std[24] = 154;
    rs_std[25] = 154; rs_std[26] = 154; rs_std[27] = 157; rs_std[28] = 157; rs_std[29] = 157;
    rs_std[30] = 160; rs_std[31] = 160; rs_std[32] = 160; rs_std[33] = 163; rs_std[34] = 163;
    rs_std[35] = 163; rs_std[36] = 166; rs_std[37] = 166; rs_std[38] = 166; rs_std[39] = 169;
    rs_std[40] = 169; rs_std[41] = 171; rs_std[42] = 171; rs_std[43] = 171; rs_std[44] = 171;
    rs_std[45] = 174; rs_std[46] = 174; rs_std[47] = 176; rs_std[48] = 175; rs_std[49] = 178;*/

    //高度  前瞻
/*    ls_std[0] = 66; ls_std[1] = 65; ls_std[2] = 65; ls_std[3] = 64; ls_std[4] = 63;
    ls_std[5] = 63; ls_std[6] = 63; ls_std[7] = 62; ls_std[8] = 61; ls_std[9] = 60;
    ls_std[10] = 59; ls_std[11] = 59; ls_std[12] = 58; ls_std[13] = 57; ls_std[14] = 57;
    ls_std[15] = 56; ls_std[16] = 56; ls_std[17] = 55; ls_std[18] = 54; ls_std[19] = 54;
    ls_std[20] = 54; ls_std[21] = 53; ls_std[22] = 52; ls_std[23] = 50; ls_std[24] = 50;
    ls_std[25] = 49; ls_std[26] = 48; ls_std[27] = 47; ls_std[28] = 46; ls_std[29] = 46;
    ls_std[30] = 45; ls_std[31] = 44; ls_std[32] = 43; ls_std[33] = 43; ls_std[34] = 33;
    ls_std[35] = 42; ls_std[36] = 41; ls_std[37] = 41; ls_std[38] = 40; ls_std[39] = 39;
    ls_std[40] = 38; ls_std[41] = 37; ls_std[42] = 37; ls_std[43] = 36; ls_std[44] = 35;
    ls_std[45] = 34; ls_std[46] = 34; ls_std[47] = 33; ls_std[48] = 33; ls_std[49] = 32;

    rs_std[0] = 124; rs_std[1] = 125; rs_std[2] = 158; rs_std[3] = 126; rs_std[4] = 127;
    rs_std[5] = 128; rs_std[6] = 129; rs_std[7] = 130; rs_std[8] = 130; rs_std[9] = 131;
    rs_std[10] = 131; rs_std[11] = 132; rs_std[12] = 133; rs_std[13] = 133; rs_std[14] = 134;
    rs_std[15] = 134; rs_std[16] = 135; rs_std[17] = 136; rs_std[18] = 137; rs_std[19] = 138;
    rs_std[20] = 138; rs_std[21] = 139; rs_std[22] = 140; rs_std[23] = 140; rs_std[24] = 141;
    rs_std[25] = 141; rs_std[26] = 142; rs_std[27] = 142; rs_std[28] = 143; rs_std[29] = 143;
    rs_std[30] = 144; rs_std[31] = 144; rs_std[32] = 145; rs_std[33] = 145; rs_std[34] = 146;
    rs_std[35] = 147; rs_std[36] = 148; rs_std[37] = 148; rs_std[38] = 148; rs_std[39] = 149;
    rs_std[40] = 150; rs_std[41] = 151; rs_std[42] = 152; rs_std[43] = 152; rs_std[44] = 152;
    rs_std[45] = 153; rs_std[46] = 153; rs_std[47] = 154; rs_std[48] = 154; rs_std[49] = 155;
*/
//    ls_std[0] = 71; ls_std[1] = 71; ls_std[2] = 70; ls_std[3] = 69; ls_std[4] = 69;
//    ls_std[5] = 68; ls_std[6] = 67; ls_std[7] = 66; ls_std[8] = 65; ls_std[9] = 64;
//    ls_std[10] = 64; ls_std[11] = 63; ls_std[12] = 62; ls_std[13] = 62; ls_std[14] = 61;
//    ls_std[15] = 60; ls_std[16] = 59; ls_std[17] = 58; ls_std[18] = 57; ls_std[19] = 56;
//    ls_std[20] = 55; ls_std[21] = 55; ls_std[22] = 55; ls_std[23] = 54; ls_std[24] = 54;
//    ls_std[25] = 53; ls_std[26] = 52; ls_std[27] = 52; ls_std[28] = 51; ls_std[29] = 50;
//    ls_std[30] = 50; ls_std[31] = 49; ls_std[32] = 48; ls_std[33] = 47; ls_std[34] = 46;
//    ls_std[35] = 46; ls_std[36] = 45; ls_std[37] = 44; ls_std[38] = 43; ls_std[39] = 42;
//    ls_std[40] = 41; ls_std[41] = 40; ls_std[42] = 40; ls_std[43] = 39; ls_std[44] = 39;
//    ls_std[45] = 38; ls_std[46] = 37; ls_std[47] = 36; ls_std[48] = 35; ls_std[49] = 34;
//
//    rs_std[0] = 120; rs_std[1] = 120; rs_std[2] = 121; rs_std[3] = 122; rs_std[4] = 123;
//    rs_std[5] = 123; rs_std[6] = 124; rs_std[7] = 125; rs_std[8] = 126; rs_std[9] = 127;
//    rs_std[10] = 128; rs_std[11] = 128; rs_std[12] = 129; rs_std[13] = 129; rs_std[14] = 130;
//    rs_std[15] = 130; rs_std[16] = 131; rs_std[17] = 131; rs_std[18] = 132; rs_std[19] = 132;
//    rs_std[20] = 133; rs_std[21] = 134; rs_std[22] = 135; rs_std[23] = 135; rs_std[24] = 136;
//    rs_std[25] = 136; rs_std[26] = 137; rs_std[27] = 138; rs_std[28] = 139; rs_std[29] = 139;
//    rs_std[30] = 140; rs_std[31] = 140; rs_std[32] = 141; rs_std[33] = 142; rs_std[34] = 142;
//    rs_std[35] = 143; rs_std[36] = 143; rs_std[37] = 144; rs_std[38] = 144; rs_std[39] = 145;
//    rs_std[40] = 145; rs_std[41] = 146; rs_std[42] = 147; rs_std[43] = 148; rs_std[44] = 149;
//    rs_std[45] = 149; rs_std[46] = 148; rs_std[47] = 149; rs_std[48] = 150; rs_std[49] = 151;
    //32/30cm  102cm
    ls_std[0] = 71; ls_std[1] = 71; ls_std[2] = 72; ls_std[3] = 72; ls_std[4] = 71;
    ls_std[5] = 71; ls_std[6] = 71; ls_std[7] = 71; ls_std[8] = 70; ls_std[9] = 67;
    ls_std[10] = 67; ls_std[11] = 67; ls_std[12] = 66; ls_std[13] = 65; ls_std[14] = 65;
    ls_std[15] = 65; ls_std[16] = 65; ls_std[17] = 64; ls_std[18] = 61; ls_std[19] = 61;
    ls_std[20] = 61; ls_std[21] = 61; ls_std[22] = 58; ls_std[23] = 58; ls_std[24] = 58;
    ls_std[25] = 58; ls_std[26] = 58; ls_std[27] = 55; ls_std[28] = 55; ls_std[29] = 55;
    ls_std[30] = 55; ls_std[31] = 52; ls_std[32] = 54; ls_std[33] = 51; ls_std[34] = 50;
    ls_std[35] = 49; ls_std[36] = 49; ls_std[37] = 49; ls_std[38] = 49; ls_std[39] = 49;
    ls_std[40] = 48; ls_std[41] = 45; ls_std[42] = 45; ls_std[43] = 44; ls_std[44] = 43;
    ls_std[45] = 45; ls_std[46] = 42; ls_std[47] = 42; ls_std[48] = 41; ls_std[49] = 40;

    rs_std[0] = 112; rs_std[1] = 111; rs_std[2] = 110; rs_std[3] = 110; rs_std[4] = 113;
    rs_std[5] = 113; rs_std[6] = 113; rs_std[7] = 113; rs_std[8] = 115; rs_std[9] = 115;
    rs_std[10] = 115; rs_std[11] = 115; rs_std[12] = 117; rs_std[13] = 118; rs_std[14] = 119;
    rs_std[15] = 119; rs_std[16] = 119; rs_std[17] = 121; rs_std[18] = 121; rs_std[19] = 121;
    rs_std[20] = 121; rs_std[21] = 121; rs_std[22] = 124; rs_std[23] = 124; rs_std[24] = 124;
    rs_std[25] = 124; rs_std[26] = 124; rs_std[27] = 127; rs_std[28] = 127; rs_std[29] = 127;
    rs_std[30] = 127; rs_std[31] = 130; rs_std[32] = 129; rs_std[33] = 129; rs_std[34] = 130;
    rs_std[35] = 130; rs_std[36] = 133; rs_std[37] = 133; rs_std[38] = 133; rs_std[39] = 135;
    rs_std[40] = 135; rs_std[41] = 135; rs_std[42] = 135; rs_std[43] = 137; rs_std[44] = 136;
    rs_std[45] = 138; rs_std[46] = 138; rs_std[47] = 138; rs_std[48] = 140; rs_std[49] = 139;
    //图像行对应的现实距离
    real_distans[0] = 107; real_distans[1] = 104; real_distans[2] = 101; real_distans[3] = 98; real_distans[4] = 95;
    real_distans[5] = 93; real_distans[6] = 90; real_distans[7] = 87; real_distans[8] = 85; real_distans[9] = 82;
    real_distans[10] = 80; real_distans[11] = 77; real_distans[12] = 75; real_distans[13] = 72; real_distans[14] = 70;
    real_distans[15] = 68; real_distans[16] = 66; real_distans[17] = 64; real_distans[18] = 62; real_distans[19] = 59;
    real_distans[20] = 57; real_distans[21] = 55; real_distans[22] = 54; real_distans[23] = 52; real_distans[24] = 50;
    real_distans[25] = 48; real_distans[26] = 47; real_distans[27] = 45; real_distans[28] = 43; real_distans[29] = 41;
    real_distans[30] = 40; real_distans[31] = 39; real_distans[32] = 37; real_distans[33] = 36; real_distans[34] = 35;
    real_distans[35] = 34; real_distans[36] = 32; real_distans[37] = 31; real_distans[38] = 30; real_distans[39] = 29;
    real_distans[40] = 28; real_distans[41] = 27; real_distans[42] = 26; real_distans[43] = 25; real_distans[44] = 24;
    real_distans[45] = 23; real_distans[46] = 23; real_distans[47] = 22; real_distans[48] = 22; real_distans[49] = 21;

    //给标准数组固定的偏差，使计算出来的位置偏差多一点，可以控制车模在弯道时的位置
    for( int8 i = 0;i <= 49;i ++)
    {
        ls_std[i] += 1;
        rs_std[i] += 5;
    }
    //十字方向初始化
    crs_turn_row[0] = 2;
    crs_turn_row[1] = 2;
    crs_turn_row[3] = 0;
    crs_turn_row[4] = 0;
    //惯导角度初始化
    goal_angle[0] = 0;        //阶段0是往前走一点
    goal_angle[1] = -40;
    goal_angle[2] = -40;
    goal_angle[3] = -40;
    goal_angle[4] = -40;
    //惯导编码初始化,平均1m2740个脉冲
    goal_encode[0] = 300;     //30cm
    goal_encode[1] = 6000;   //（上一阶段30+到赛道边缘距离（应该距离再缩短一点，免得下一阶段没有距离））+拐弯距离=225+？？？（设188）
    goal_encode[2] = 6000;   //上一阶段+30
    goal_encode[3] = 6000;
    goal_encode[4] = 6000;   //最后面距离要小于到真正要进入的赛道边缘的距离
    return 0;

}


//-------------------------------------------------------------------------------------------------------------------
//  函数功能：图像处理，用于计算图像边界，并进行进一步的信息提取
//  参数：void
//  返回值：void
//-------------------------------------------------------------------------------------------------------------------
void image_boundary_process(void)
{
    int16 i, j, k;                          //循环变量
    int16 init_lj = MT9V03X_DVP_W/2;        //搜索左边界的起始点
    int16 init_rj = MT9V03X_DVP_W/2;        //搜索右边界的起始点
    int16 flag_startpoint_to_center = 0;    //起始点归中标志，用于处理十字路口时起始点错误的可能情况
    int16 black_cen = 0;                    //中线旁边黑的数，用于判断中线是否正常
    int16 w_flag = 0;                       //弯道标志
    img_raw.data = mt9v03x_image[0];
    thres = GetOSTU(mt9v03x_image);
//    for(i = MT9V03X_DVP_H-1; i >= END_I; i -= 1)
//    {
//        for(j = MT9V03X_DVP_W-1; j >= END_I; j -= 1)
//        {
//            if(mt9v03x_image[i][j] > thres+10)
//            {
//                if(thres+10 < 256)
//                    mt9v03x_image[i][j]  = thres+10;
//            }
//        }
//    }
    flag_cen_false = 0;                     //错误位置零
    cen_black = 0;
    ave_distain = 107;
    //计算左右边界
    for(i = MT9V03X_DVP_H-1; i >= END_I; i -= 1)
    {
        //模式0与模式1的边界搜索
        if(img_mode == 0 || img_mode == 1 || img_mode == 3 ||img_mode == 4 || img_mode == 6 || img_mode == 7 || image_para.imgtype == CIR_W_ADC_1)
        {
            //边界搜索起点设置，选用上一行的中点作为下一行的起始点
            if(i != MT9V03X_DVP_H-1)
                init_lj = init_rj = centerline[i+1];
            else//开始行
            {
                j = MT9V03X_DVP_H/2;
                k = 1;
                //当所选择的起始点是黑点时，需要左右两边探索，找到最近的白点作为起始点
                while(mt9v03x_image_dvp[MT9V03X_DVP_H-1][j] < WHITE_TH)
                {
                    if(j >= MT9V03X_DVP_W/2)
                    {
                        j -= k;
                        k += 1;
                    }
                    else
                    {
                        j += k;
                        k += 1;
                    }
                }
                if(j < MT9V03X_DVP_W-1)
                    init_lj = init_rj = j;
            }

            //如果出现3、5，3、6则起始点归中（其中3、5，3、6表示上一帧的图像进行分段算法处理之后连续出现的阶段）
            if(i <= MT9V03X_DVP_H-3)
            {
                if(leftside[i+2] <= 1 && rightside[i+2] >= 186 &&(leftside[i+1] >= 10 || rightside[i+1] <=176))
                {
                    flag_startpoint_to_center = 1;
                    init_lj = init_rj = MT9V03X_DVP_W/2;
                }
                if(flag_startpoint_to_center == 1)
                    init_lj = init_rj = MT9V03X_DVP_W/2;
            }
            //计算此行左右边界
            row_boundary_process(i, init_lj, init_rj, 0);

            //计算此行中线
            centerline[i] = (leftside[i]+rightside[i])/2;
        }
        //计算左环岛某些阶段的左右边界
        else if(image_para.imgtype == LCIR_W_S2 || image_para.imgtype == LCIR_W_S2_5 || image_para.imgtype == LCIR_W_S3 || image_para.imgtype == LCIR_W_S6 || image_para.imgtype == LCIR_W_S7 || image_para.imgtype == RCIR_W_S4 || image_para.imgtype == RCIR_W_S5)
        {
            //边界搜索起点设置
            if(i != MT9V03X_DVP_H-1)
                init_lj = init_rj = rightside[i+1]-10;
            else
                init_lj = init_rj = MT9V03X_DVP_W/2;

            //计算此行左右边界
            row_boundary_process(i, init_lj, init_rj, 0);

            //计算此行中线
            centerline[i] = (leftside[i]+rightside[i])/2;
        }
        //计算右环岛某些阶段的左右边界
        else if(image_para.imgtype == RCIR_W_S2 || image_para.imgtype == RCIR_W_S2_5 || image_para.imgtype == RCIR_W_S3 || image_para.imgtype == RCIR_W_S6 || image_para.imgtype == RCIR_W_S7 || image_para.imgtype == LCIR_W_S4 || image_para.imgtype == LCIR_W_S5)
        {
            //边界搜索起点设置
            if((i) != MT9V03X_DVP_H-1)
                init_lj = init_rj = leftside[i+1]+10;
            else
                init_lj = init_rj = MT9V03X_DVP_W/2;

            //计算此行左右边界
            row_boundary_process(i, init_lj, init_rj, 0);

            //计算此行中线
            centerline[i] = (leftside[i]+rightside[i])/2;
        }
        else if(img_mode == 5)   //断路从上往下搜索
        {
            //边界搜索起点设置，选用上一行的中点作为下一行的起始点
            if((MT9V03X_DVP_H - i - 1) != 0)
                init_lj = init_rj = centerline[(MT9V03X_DVP_H - i - 1) - 1];
            else//开始行
            {
                j = MT9V03X_DVP_H/2;
                k = 1;
                //当所选择的起始点是黑点时，需要左右两边探索，找到最近的白点作为起始点
                while(mt9v03x_image_dvp[0][j] < thres)
                {
                    if(j >= MT9V03X_DVP_W/2)
                    {
                        j -= k;
                        k += 1;
                    }
                    else
                    {
                        j += k;
                        k += 1;
                    }
                }
                if(j < MT9V03X_DVP_W-1)
                    init_lj = init_rj = j;
            }

            //如果出现3、5，3、6则起始点归中（其中3、5，3、6表示上一帧的图像进行分段算法处理之后连续出现的阶段）
            if((MT9V03X_DVP_H - i - 1) >= 2)
            {
                if(leftside[(MT9V03X_DVP_H - i - 1) - 2] <= 1 && rightside[(MT9V03X_DVP_H - i - 1) - 2] >= 186 &&(leftside[(MT9V03X_DVP_H - i - 1) - 1] >= 10 || rightside[(MT9V03X_DVP_H - i - 1) - 1] <=176))
                {
                    flag_startpoint_to_center = 1;
                    init_lj = init_rj = MT9V03X_DVP_W/2;
                }
                if(flag_startpoint_to_center == 1)
                    init_lj = init_rj = MT9V03X_DVP_W/2;
            }
            //计算此行左右边界
            row_boundary_process((MT9V03X_DVP_H - i - 1), init_lj, init_rj, 0);

            //计算此行中线
            centerline[(MT9V03X_DVP_H - i - 1)] = (leftside[(MT9V03X_DVP_H - i - 1)]+rightside[(MT9V03X_DVP_H - i - 1)])/2;
        }
        else
        {
            //边界搜索起点设置，选用上一行的中点作为下一行的起始点
            if(i != MT9V03X_DVP_H-1)
                init_lj = init_rj = centerline[i+1];
            else//开始行
            {
                j = MT9V03X_DVP_H/2;
                k = 1;
                //当所选择的起始点是黑点时，需要左右两边探索，找到最近的白点作为起始点
                while(mt9v03x_image_dvp[MT9V03X_DVP_H-1][j] < WHITE_TH)
                {
                    if(j >= MT9V03X_DVP_W/2)
                    {
                        j -= k;
                        k += 1;
                    }
                    else
                    {
                        j += k;
                        k += 1;
                    }
                }
                if(j < MT9V03X_DVP_W-1)
                    init_lj = init_rj = j;
            }

            //如果出现3、5，3、6则起始点归中（其中3、5，3、6表示上一帧的图像进行分段算法处理之后连续出现的阶段）
            if(i <= MT9V03X_DVP_H-3)
            {
                if(leftside[i+2] <= 1 && rightside[i+2] >= 186 &&(leftside[i+1] >= 10 || rightside[i+1] <=176))
                {
                    flag_startpoint_to_center = 1;
                    init_lj = init_rj = MT9V03X_DVP_W/2;
                }
                if(flag_startpoint_to_center == 1)
                    init_lj = init_rj = MT9V03X_DVP_W/2;
            }
            //计算此行左右边界
            row_boundary_process(i, init_lj, init_rj, 0);

            //计算此行中线
            centerline[i] = (leftside[i]+rightside[i])/2;
        }
        if(mt9v03x_image_dvp[i][centerline[i]] < thres && i < MT9V03X_DVP_H/2) //下端中线位置为黑
        {
            black_cen += 1;
        }
        if(mt9v03x_image_dvp[i][centerline[i]] < thres && img_mode != 5)    //中线位置为黑,不是断路模式
        {
            roweff_flag[i] = -1;   //行无效
            cen_black += 1;
        }
        else if(img_mode == 5 && i > MT9V03X_DVP_H/2 && mt9v03x_image_dvp[i][centerline[i]] < thres)                   //断路，看底下黑
        {
            cen_black += 1;
        }
        if(roweff_flag[i] != 0 && w_flag ==0) //计算弯道对应的现实距离
        {
            w_flag = 1;
            straight_distain[2] = straight_distain[1];
            straight_distain[1] = straight_distain[0];
            straight_distain[0] = real_distans[i];
            ave_distain = (straight_distain[0]+straight_distain[1]+straight_distain[2])/3;
        }
//        else if(i <= END_I)
//        {
//            w_flag = 1;
//            straight_distain[2] = straight_distain[1];
//            straight_distain[1] = straight_distain[0];
//            straight_distain[0] = real_distans[i];
//            ave_distain = (straight_distain[0]+straight_distain[1]+straight_distain[2])/3;
//        }
    }
    if( w_flag == 0) //计算弯道对应的现实距离
    {
        w_flag = 1;
        straight_distain[2] = straight_distain[1];
        straight_distain[1] = straight_distain[0];
        straight_distain[0] = real_distans[2];
        ave_distain = (straight_distain[0]+straight_distain[1]+straight_distain[2])/3;
    }
    //搜索中线错误
    if(black_cen >= MT9V03X_DVP_H/2 -6)
    {
        flag_cen_false  = 1;
    }
    //分段数组清零
    memset(part_type,0,sizeof(part_type));
    memset(part_index,0,sizeof(part_index));
    memset(part_sum,0,sizeof(part_sum));
    //边界分段算法
    boundary_part();
    //计算边界cos值
    cur_line_cos();
    //拐点寻找
    find_rightdown_point(15,45);
    find_leftdown_point(15,45);
    //图像参数处理
    image_para_process();

}

//-------------------------------------------------------------------------------------------------------------------
//  函数功能：给定第i行和初始j值，计算该行的左右边界
//  参数：i（第i行），init_lj（初始左边界j值），init_rj（初始右边界j值），mode（计算模式，暂时只有0模式）
//  返回值：void
//-------------------------------------------------------------------------------------------------------------------
void row_boundary_process(int16 i, int16 init_lj, int16 init_rj, int16 mode)
{
    int16 j;                    //列循环变量
    int32 gain;                 //中间变量
    int16 flagl = 0;            //左边界查找标志位，1表示左边界查到
    int16 flagr = 0;            //右边界查找标志位，1表示右边界查到
    int16 cnt_white = 0;        //白色点计数，用于判断此行是否为无效类型（-1）
    float x, y, sar;            //x和y为中间变量，sar为差和比值

    if(mode == 0)
    {
        //计算第i行左边界
        leftside[i] = 0;                                //当左边界找不到时默认为0
        for(j = init_lj; j >= SAR_DW+1; j -= ROW_DW)    //此处终止值选用SAR_DW+1是因为要保证j-SAR_DW要大于0，因为我们摄像头采集到的数据最左边一列全为0，数据是错的，各种宏见h文件注释
        {
            //此处选用这种方法是因为可以不用浮点数乘法，加快运算速度
            gain = (mt9v03x_image[i][j]-mt9v03x_image[i][j-SAR_DW])<<8;
            x = (float)gain;
            y = (float)(mt9v03x_image[i][j]+mt9v03x_image[i][j-SAR_DW]+1);
            sar = abs_f(x/y);
            if(sar > SAR_TH)
            {
                leftside[i] = j-SAR_DW;
                flagl = 1;
                break;
            }
        }
        //计算第i行的右边界
        rightside[i] = MT9V03X_DVP_W-1;                     //当右边界找不到时默认为MT9V03X_W-1
        for(j = init_rj; j < MT9V03X_DVP_W-SAR_DW-1; j += ROW_DW)
        {
            gain = (mt9v03x_image[i][j]-mt9v03x_image[i][j+SAR_DW])<<8;
            x = (float)gain;
            y = (float)(mt9v03x_image[i][j]+mt9v03x_image[i][j+SAR_DW]+1);
            sar = abs_f(x/y);
            if(sar > SAR_TH)
            {
                rightside[i] = j+SAR_DW;
                flagr = 1;
                break;
            }
        }
        //行宽度计算与类型判断(初步，只用0，1，2，3，-1表示，还需要进一步细化才能作为最终的类型)
        row_width[i] = rightside[i]-leftside[i];

        if(flagl == 1 && flagr == 1)
            roweff_flag[i] = 0;
        else if(flagl == 0 && flagr == 1)
            roweff_flag[i] = 1;
        else if(flagl == 1 && flagr == 0)
            roweff_flag[i] = 2;
        else if(flagl == 0 && flagr == 0)
            roweff_flag[i] = 3;

        if(leftside[i] >= rightside[i])
            roweff_flag[i] = -1;

        if(roweff_flag[i] == 3)
        {
            for(j = 0; j < 5; j += 1)
            {
                if(mt9v03x_image_dvp[i][MT9V03X_DVP_W/2+j] > WHITE_TH)
                    cnt_white += 1;
                if(mt9v03x_image_dvp[i][MT9V03X_DVP_W/2-j] > WHITE_TH)
                    cnt_white += 1;
            }
            if(cnt_white < 5)
                roweff_flag[i] = -1;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  函数功能：图像数组记录中心线
//  参数：void
//  返回值：void
//-------------------------------------------------------------------------------------------------------------------
void row_centerline_process(void)
{
    int16 i;

    for(i = MT9V03X_DVP_H-1; i >= 0; i -= 1)
    {
        //画赛道中线
        if(centerline[i] < MT9V03X_DVP_W && centerline[i] >= 0)
            mt9v03x_image_dvp[i][centerline[i]] = 0;
        if(leftside[i] < MT9V03X_DVP_W && leftside[i] >= 0)
            mt9v03x_image_dvp[i][leftside[i]] = 0;
        if(rightside[i] < MT9V03X_DVP_W && rightside[i] >= 0)
            mt9v03x_image_dvp[i][rightside[i]] = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  函数功能：图像参数提取以及逻辑处理
//  参数：void
//  返回值：void
//-------------------------------------------------------------------------------------------------------------------
void image_para_process(void)
{
#ifdef Have  //库有电磁
    if(go_strat == 1 && start == 1)  //判断出库
    {
        if(ADC_Filter[0] >= 10 && ADC_Filter[0] - ADC_Filter[2] >= 5 && go_turn == 1 && Go_Encode > GO_END_ENC)  //右出库
        {
            go_turn = 1;
            gara_turn = 2;
        }
        else if(ADC_Filter[2] >= 10 && ADC_Filter[2] - ADC_Filter[0] >= 5 && go_turn == -1 && Go_Encode > GO_END_ENC)   //左出库
        {
            go_turn = -1;
            gara_turn = 1;
        }
        if(get_max_num(ADC_Filter,3,1) >= 40 && abs_u16(ADC_Filter[0]-ADC_Filter[2]) < 20 && Go_Encode > GO_END_ENC) //判断出库完成
            go_strat = 0;
        if(go_turn == 1 && ADC_Filter[2] - ADC_Filter[0] >= 0)
            go_strat = 0;
        else if(go_turn == -1 && ADC_Filter[0] - ADC_Filter[2] >= 0)
            go_strat = 0;
    }
#else    //没有库电磁
    if(go_strat == 1 && start == 1)  //判断出库
    {
        go_t += 1;
        if(((ADC_Filter[0] >= 10 && ADC_Filter[0] - ADC_Filter[2] >= 5) || go_turn == 1) && go_t >= 15)  //右出库
        {
            go_turn = 1;
        }
        else if(((ADC_Filter[2] >= 10 && ADC_Filter[2] - ADC_Filter[0] >= 5) || go_turn == -1) && go_t >= 15)   //左出库
        {
            go_turn = -1;
        }
        if(get_max_num(ADC_Filter,3,1) >= 40 && abs_u16(ADC_Filter[0]-ADC_Filter[2]) < 20 && go_t >= 15) //判断出库完成
            go_strat = 0;
        if(go_turn == 1 && ADC_Filter[2] - ADC_Filter[0] >= 0)
            go_strat = 0;
        else if(go_turn == -1 && ADC_Filter[0] - ADC_Filter[2] >= 0)
            go_strat = 0;
    }
#endif
    if(Open_End_Encode > OPEN_END_ENC_TH)  //积分值大，普通断路惯导结束
    {
        flag_end = 0;
        Open_End_Encode = 0;
    }
    //图像类型识别
    if(go_strat == 0) //出完库开始识别
        image_distinguish();

    //图像位置偏差计算
    image_para.offset = offset_process();
    offest_K = regression(25,48);
    //逻辑处理
    crs_turn = crs_turn_row[crs_num];
}

//-------------------------------------------------------------------------------------------------------------------
//  函数功能：识别图像的赛道类型
//  参数：void
//  返回值：void
//-------------------------------------------------------------------------------------------------------------------
void image_distinguish(void)
{
    image_type img_type = none;         //默认种类为none
    if(CIR_ENABLE == 0)   //关闭环岛识别开始计时，计数到一定值开启识别
    {
        CIR_END_T += 1;
        if(CIR_END_T >= 100)
        {
            CIR_ENABLE = 1;
            CIR_END_T = 0;
        }
    }
    if(img_mode == 0 && start == 1 && flag_cen_false == 0)
    {
        //先判断库
        if(Num_slop_jump() == 1)   //先通过图像识别判断库的方向，图像没有识别再通过标志确定
        {
            if(gara_turn == 0 && (part_type[0] == 5 || part_type[1] == 5 || part_type[3] == 5))
            {
                img_type = LGARA_W_S1;
            }
            else if(gara_turn == 0 && (part_type[0] == 6 || part_type[1] == 6 || part_type[3] == 6))
            {
                img_type = RGARA_W_S1;
            }
            else if(gara_turn == 1)
            {
                img_type = LGARA_W_S1;
            }
            else if(gara_turn == 2)
            {
                img_type = RGARA_W_S1;
            }
            else
                img_type = LGARA_W_S1;
            img_mode = 4;
        }
        //读取电感值识别环岛
        else if(CIR_ENABLE == 1 && (get_max_num(ADC_Filter,3,1) >= CIR_MIX_ADC_TH && get_max_num(ADC_Filter,3,2) >= CIR_MIN_ADC_TH))
        {
            img_type = CIR_W_ADC_1;
            img_mode = 2;
        }
        else if(dl1a_distance_mm <= BARR_TH && count_white(3,MT9V03X_DVP_W/3,MT9V03X_DVP_W*2/3) < 10) //测距小于阈值,并且上端为黑，为路障
        {
            Angle_z_temp = 0; //z轴清零
            img_type = BARR_W_S1;
            img_mode = 3;
        }
        else if(cen_black > 15 && part_type[0] == 0)  //识别短路
        {
            img_type = OPEN_W_S1;
            img_mode = 5;
            if(open_in_start == 1)  //惯导z清零
                Angle_z_temp = 0;
        }
        else if(cen_black > 25)  //识别短路
        {
            img_type = OPEN_W_S1;
            img_mode = 5;
        }
         //特殊情况识别十字,1存在两拐点:
        else if(youxia_guai == 1 && zuoxia_guai == 1 && count_white(right_turn_down[0]-4,right_turn_down[1],right_turn_down[1]+20) > 7 && count_white(left_turn_down[0]-4,left_turn_down[1]-20,left_turn_down[1]) > 7) //下端俩拐点存在，为十字
        {
//            if(crs_turn == 0)
//                img_type = CRS_W_S1;
//            else
//            {
                img_type = CRS_W_S2;
                img_mode = 6;
//            }
        }//行类型大部分为-1,十字默认直走
        else if(part_sum__1 > 25)
        {
            img_type = CRS_W_S1;
        }
        //当赛道的分段数为1时
        else if(part_num == 1)
        {
            //出现0种类，说明是直道或小弯或坡道
            //if(part_type[0] == 0  && row_width[45] > 120 && row_width[40] > 115 && row_width[35] > 110 && row_width[20] > 105)
            if(part_type[0] == 0  && row_width[45] > 108 && row_width[35] > 97 && row_width[25] > 93 && abs_f(offest_K) < 0.1) //底部宽，并且中线斜率小
            {
                img_type = RAMP_S1;
                img_mode = 7;
            }
            else if(part_type[0] == 0)
                img_type = STR_W;
        }
        //当赛道的分段数为2时
        else if(part_num == 2)
        {
            if(part_type[0] == 0 && part_type[1] == 5 && cur_circle(1) >= CUR_CIRTH)   //如果出现先0然后左边界突变5并且右边是直的边界，说明是预进入左环岛
            {
                int16 sum_white=0,sum=0;
                for(int16 i=leftside[part_index[0]+1];i>END_I;i-=2)
                {
                    sum++;
                    if(mt9v03x_image[part_index[0]+1][i]>=WHITE_TH)
                        sum_white++;
                }
                if(sum_white>(sum/4) && (get_max_num(ADC_Filter,3,1) >= CIR_MIX_ADC_TH-10 && get_max_num(ADC_Filter,3,2) >= CIR_MIN_ADC_TH-10))
                {
                    img_type = LCIR_W_S1;
                    img_mode = 1;           //预进入环岛模式，修改图像识别模式为1
                }
                else if(Num_slop_jump() == 1) //判断入库，库在左边
                {
                    img_type = LGARA_W_S1;
                    img_mode = 4;
                }
            }
            else if(part_type[0] == 3 && part_type[1] == 0) //出现先空白区域3后直道0，说明前方是出十字的S2阶段，不为0这里是要拐
            {
                if(crs_turn == 0)
                    img_type = CRS_W_S1;
                else
                {
                    img_type = CRS_W_S2;
                    img_mode = 6;
                }
            }
            else if(part_type[0] == 0 && part_type[1] == 6 && cur_circle(0) >= CUR_CIRTH)   //如果出现先0然后右边界突变6并且左边是直的边界，说明是预进入右环岛
            {
                int16 sum_white=0,sum=0;
                for(int16 i=rightside[part_index[0]+1];i<MT9V03X_DVP_W-3;i+=2)
                {
                    sum++;
                    if(mt9v03x_image[part_index[0]+1][i]>=thres)
                        sum_white++;
                }
                if(sum_white>(sum/4) && (get_max_num(ADC_Filter,3,1) >= CIR_MIX_ADC_TH-10 && get_max_num(ADC_Filter,3,2) >= CIR_MIN_ADC_TH-10))
                {
                    img_type = RCIR_W_S1;
                    img_mode = 1;           //预进入环岛模式，修改图像识别模式为1
                }
                else if(Num_slop_jump() == 1) //判断入库，库在右边
                {
                    img_type = RGARA_W_S1;
                    img_mode = 4;
                }
            }
        }
        //当赛道的分段数大于等于3时
        else
        {
            if(part_type[0] == 0 && part_type[1] == 5 && cur_circle(1) >= CUR_CIRTH && (get_max_num(ADC_Filter,3,1) >= CIR_MIX_ADC_TH && get_max_num(ADC_Filter,3,2) >= CIR_MIN_ADC_TH))
            {
                img_type = LCIR_W_S1;
                img_mode = 1;
            }
            else if(part_type[0] == 0 && part_type[1] == 6 && cur_circle(0) >= CUR_CIRTH && (get_max_num(ADC_Filter,3,1) >= CIR_MIX_ADC_TH && get_max_num(ADC_Filter,3,2) >= CIR_MIN_ADC_TH))
            {
                img_type = RCIR_W_S1;
                img_mode = 1;
            }
            else if(part_type[1] == 0 && part_type[2] == 5 && cur_circle(1) >= CUR_CIRTH && (get_max_num(ADC_Filter,3,1) >= CIR_MIX_ADC_TH && get_max_num(ADC_Filter,3,2) >= CIR_MIN_ADC_TH))
            {
                img_type = LCIR_W_S1;
                img_mode = 1;
            }
            else if(part_type[1] == 0 && part_type[2] == 6 && cur_circle(0) >= CUR_CIRTH && (get_max_num(ADC_Filter,3,1) >= CIR_MIX_ADC_TH && get_max_num(ADC_Filter,3,2) >= CIR_MIN_ADC_TH))
            {
                img_type = RCIR_W_S1;
                img_mode = 1;
            }
        }
    }
    else if(img_mode == 1 && start == 1 && flag_cen_false == 0)
    {
        //识别环岛
        if(CIR_ENABLE == 1 && (get_max_num(ADC_Filter,3,1) >= CIR_MIX_ADC_TH && get_max_num(ADC_Filter,3,2) >= CIR_MIN_ADC_TH)) //电感判断环岛
        {
            if(image_para.imgtype == RCIR_W_S1)
                img_type = RCIR_W_S2;
            else if(image_para.imgtype == LCIR_W_S1)
                img_type = RCIR_W_S2;
            else
                img_type =CIR_W_ADC_1;
            img_mode = 2;
        }
        else if(dl1a_distance_mm <= BARR_TH && count_white(3,MT9V03X_DVP_W/3,MT9V03X_DVP_W*2/3) < 20) //测距小于阈值,并且上端为黑，为路障
        {
            Angle_z_temp = 0; //z轴清零
            img_type = BARR_W_S1;
            img_mode = 3;
        }
        else if(Num_slop_jump() == 1) //判断库
        {
            if(gara_turn == 0 && (part_type[0] == 5 || part_type[1] == 5 || part_type[3] == 5))
            {
                img_type = LGARA_W_S1;
            }
            else if(gara_turn == 0 && (part_type[0] == 6 || part_type[1] == 6 || part_type[3] == 6))
            {
                img_type = RGARA_W_S1;
            }
            else if(gara_turn == 1)
            {
                img_type = LGARA_W_S1;
            }
            else if(gara_turn == 2)
            {
                img_type = RGARA_W_S1;
            }
            else
                img_type = LGARA_W_S1;
            img_mode = 4;
        }
        else if(youxia_guai == 1 && zuoxia_guai == 1)  //两个拐点存在，为十字
        {
//            if(crs_turn == 0)
//                img_type = CRS_W_S1;
//            else
//            {
                img_type = CRS_W_S2;
                img_mode = 6;
//            }
        }
        else if(cen_black > 20)  //识别短路
        {
            img_type = OPEN_W_S1;
            img_mode = 5;
            if(open_in_start == 1)  //惯导z清零
                Angle_z_temp = 0;
        }
        else if(part_num == 1)
        {
            if(part_type[0] == 0)
            {
                img_type = STR_W;
                img_mode = 0;           //表示前方并不是环岛，需要退出1模式，返回0模式
            }
        }
        else if(part_num == 2)
        {
            if(part_type[0] == 0 && part_type[1] == 5)
            {
                img_type = LCIR_W_S1;
                uint8 Grara_flag=0;
                Grara_flag=Garage_Judge(-1);
                if(Grara_flag==1) //判断入库，库在左边
                {
                    img_type = LGARA_W_S1;
                    img_mode = 4;
                }
            }
            else if(part_type[0] == 0 && part_type[1] == 6)
            {
                img_type = RCIR_W_S1;
                uint8 Grara_flag = 0;
                Grara_flag = Garage_Judge(1);
                if(Grara_flag == 1) //判断入库，库在右边
                {
                    img_type = RGARA_W_S1;
                    img_mode = 4;
                }
            }
            else if(part_type[1] == 0 && part_type[2] == 5)
            {
                img_type = LCIR_W_S1;
                uint8 Grara_flag=0;
                Grara_flag=Garage_Judge(-1);
                if(Grara_flag==1) //判断入库，库在左边
                {
                    img_type = LGARA_W_S1;
                    img_mode = 4;
                }
            }
            else if(part_type[1] == 0 && part_type[2] == 6)
            {
                img_type = RCIR_W_S1;
                uint8 Grara_flag = 0;
                Grara_flag = Garage_Judge(1);
                if(Grara_flag == 1) //判断入库，库在右边
                {
                    img_type = RGARA_W_S1;
                    img_mode = 4;
                }
            }
            else if(last_type == LCIR_W_S1 && (part_type[0] == 1 || part_type[1] == 1))
            {
                img_type = LCIR_W_S2;                 //上一次判断为环岛,如果这一次还为左缺矢，则还是环岛第二阶段
                img_mode = 2;
                cirsud_i = 0;
            }
            else if(last_type == RCIR_W_S1 && (part_type[0] == 2 || part_type[1] == 2))
            {
                img_type = RCIR_W_S2;                 //上一次判断为环岛,如果这一次还为右缺矢，则还是环岛第二阶段
                img_mode = 2;
                cirsud_i = 0;
            }
            else if(part_type[0] == 1 && part_type[1] == 0) //这是入环的一个必经阶段，当在1模式下看到此阶段，说明确认进入环岛
            {
                img_type = LCIR_W_S2;
                img_mode = 2;
                cirsud_i = 0;       //进入环岛S2阶段后将特殊点清零，进行S3阶段的搜索（期间还会经过一个S2_5阶段，为防止错误进入S3阶段）
            }
            else if(part_type[0] == 5 && part_type[1] == 0)
            {
                img_type = LCIR_W_S2;
                img_mode = 2;
                cirsud_i = 0;
            }
            else if(part_type[0] == 2 && part_type[1] == 0)
            {
                img_type = RCIR_W_S2;
                img_mode = 2;
                cirsud_i = 0;
            }
            else if(part_type[0] == 6 && part_type[1] == 0)
            {
                img_type = RCIR_W_S2;
                img_mode = 2;
                cirsud_i = 0;
            }
            else if(part_type[0] == -1)
            {
                img_type = none;
                img_mode = 0;
            }
        }
        else //当赛道的分段数大于等于3时
        {
            if(part_type[0] == 0 && part_type[1] == 5)
            {
                img_type = LCIR_W_S1;
                uint8 Grara_flag=0;
                Grara_flag=Garage_Judge(-1);
                if(Grara_flag==1) //判断入库，库在左边
                {
                    img_type = LGARA_W_S1;
                    img_mode = 4;
                }
            }
            else if(part_type[0] == 0 && part_type[1] == 6)
            {
                img_type = RCIR_W_S1;
                uint8 Grara_flag = 0;
                Grara_flag = Garage_Judge(1);
                if(Grara_flag == 1) //判断入库，库在右边
                {
                    img_type = RGARA_W_S1;
                    img_mode = 4;
                }
            }
            else if(part_type[1] == 0 && part_type[2] == 5)
            {
                img_type = LCIR_W_S1;
                uint8 Grara_flag=0;
                Grara_flag=Garage_Judge(-1);
                if(Grara_flag==1) //判断入库，库在左边
                {
                    img_type = LGARA_W_S1;
                    img_mode = 4;
                }
            }
            else if(part_type[1] == 0 && part_type[2] == 6)
            {
                img_type = RCIR_W_S1;
                uint8 Grara_flag = 0;
                Grara_flag = Garage_Judge(1);
                if(Grara_flag == 1) //判断入库，库在右边
                {
                    img_type = RGARA_W_S1;
                    img_mode = 4;
                }
            }
            else if(last_type == LCIR_W_S1 && (part_type[0] == 1 || part_type[1] == 1 || part_type[2] == 1))
            {
                img_type = LCIR_W_S2;                 //上一次判断为环岛,如果这一次还为左缺矢，则还是环岛第二阶段
                img_mode = 2;
                cirsud_i = 0;
            }
            else if(last_type == RCIR_W_S1 && (part_type[0] == 2 || part_type[1] == 2 || part_type[2] == 2))
            {
                img_type = RCIR_W_S2;                 //上一次判断为环岛,如果这一次还为右缺矢，则还是环岛第二阶段
                img_mode = 2;
                cirsud_i = 0;
            }
            else if(part_type[0] == 1 && part_type[1] == 0)
            {
                img_type = LCIR_W_S2;
                img_mode = 2;
            }
            else if(part_type[0] == 2 && part_type[1] == 0)
            {
                img_type = RCIR_W_S2;
                img_mode = 2;
            }
            else if(part_type[0] == -1)
            {
                img_type = none;
                img_mode = 0;
            }
        }
        if(image_para.imgtype != none)
            last_type=image_para.imgtype;
    }
    else if(img_mode == 2 && start == 1)
    {
        if(image_para.imgtype == CIR_W_ADC_1 && (part_type[part_num-1] == 5 || part_type[part_num-1] == 1 || part_type[part_num-1] == 7)) //电感识别到环岛判断看到左突变或者无左边界
        {
            img_type = LCIR_W_S3;
            CIR_W_S3 = 0; //s3阶段时间清零
        }
        else if(image_para.imgtype == CIR_W_ADC_1 && (part_type[part_num-1] == 6 || part_type[part_num-1] == 2 || part_type[part_num-1] == 8))//电感识别到环岛判断看到右突变或者无右边界
        {
            img_type = RCIR_W_S3;
            CIR_W_S3 = 0; //s3阶段时间清零
        }
        else if(image_para.imgtype == LCIR_W_S2 && part_type[0] == 0)    //当在S2阶段后看到直道，说明行使到环岛直道的中间，可以进入S3阶段了，因此设置了S2_5的过渡阶段
        {
            img_type = LCIR_W_S2_5;
        }
        else if(image_para.imgtype == RCIR_W_S2 && part_type[0] == 0)
        {
            img_type = RCIR_W_S2_5;
        }
        else if(image_para.imgtype == LCIR_W_S3 && ((part_type[1] == 7 && part_index[0] >= 30) || part_type[0] == 7))   //当三阶段特殊点很低时，说明车身即将进入环内，设置为环内S4阶段
        {
            CIR_W_S3 += 1; //s3阶段时间
            img_type = LCIR_W_S4;
            S3S4_flag = 1;     //说明是进入S4阶段的第一帧，需要进行特殊处理
        }
        else if(image_para.imgtype == RCIR_W_S3 && ((part_type[1] == 8 && part_index[0] >= 30) || part_type[0] == 8))
        {
            CIR_W_S3 += 1; //s3阶段时间
            img_type = RCIR_W_S4;
            S3S4_flag = 1;
        }
        //还得加陀螺仪积分值cnt_degree >= CIR_DE_S5
        else if(image_para.imgtype == LCIR_W_S4 && Angle_z_temp >=CIR_DE_S5 && part_num == 2 && (part_type[1] == 3 || part_type[1] == 6))    //利用陀螺仪与图像，当转过一般的角度，且能看到出环的图像时，置为S5阶段
        {
            img_type = LCIR_W_S5;
        }//cnt_degree >= CIR_DE_S5
        else if(image_para.imgtype == LCIR_W_S4 && Angle_z_temp >=CIR_DE_S5 && part_num >= 3 && (part_type[1] == 3 || part_type[1] == 6 || part_type[2] == 3 || part_type[2] == 6))
        {
            img_type = LCIR_W_S5;
        }//cnt_degree<=-CIR_DE_S5
        else if(image_para.imgtype == RCIR_W_S4 && Angle_z_temp <=-CIR_DE_S5 && part_num == 2 && (part_type[1] == 3 || part_type[1] == 5))
        {
            img_type = RCIR_W_S5;
        }//cnt_degree<=-CIR_DE_S5
        else if(image_para.imgtype == RCIR_W_S4 && Angle_z_temp <=-CIR_DE_S5 && part_num >= 3 && (part_type[1] == 3 || part_type[1] == 5 || part_type[2] == 3 || part_type[2] == 5))
        {
            img_type = RCIR_W_S5;
        }//cnt_degree >= CIR_DE
        else if(image_para.imgtype == LCIR_W_S5 && Angle_z_temp >=CIR_DE)
        {
            img_type = LCIR_W_S6;
        }
        else if(image_para.imgtype == RCIR_W_S5 && Angle_z_temp <=-CIR_DE)
        {
            img_type = RCIR_W_S6;
        }
        else if(image_para.imgtype == LCIR_W_S6 && part_num >= 2 && (part_type[0] == 1 || part_type[0] == 5) && part_type[1] == 0)  //当S6阶段行使到第二次入环位置时，不能入环了，需要置为环岛的S7阶段，用来彻底驶出环岛
        {
            img_type = LCIR_W_S7;
        }
        else if(image_para.imgtype == RCIR_W_S6 && part_num >= 2 && (part_type[0] == 2 || part_type[0] == 6) && part_type[1] == 0)
        {
            img_type = RCIR_W_S7;
        }
        else if(image_para.imgtype == LCIR_W_S7 && (part_type[0] == 0 || part_type[0] == 2))    //当环岛S7阶段可以看到非环岛内的图像时，说明已经经过了环岛，此时退出环岛模式
        {
            img_type = STR_W;
            img_mode = 0;
            //清除变量
            offset_S5 = 0;
            weight_S5 = 0;
            S3S4_flag = 0;
            s3_encode = 0;
        }
        else if(image_para.imgtype == RCIR_W_S7 && part_type[0] == 0)
        {
            img_type = STR_W;
            img_mode = 0;
            //清除变量
            offset_S5 = 0;
            weight_S5 = 0;
            S3S4_flag = 0;
            s3_encode = 0;

        }
        //后面保证在进入环岛某一阶段后不能退出，防止环内复杂的图像影响，因此我们也必须经过1模式，来确保不会错误的识别别的图像为环岛
        else if(image_para.imgtype == CIR_W_ADC_1)
        {
            img_type = CIR_W_ADC_1;
        }
        else if(image_para.imgtype == LCIR_W_S2)
        {
            img_type = LCIR_W_S2;
        }
        else if(image_para.imgtype == RCIR_W_S2)
        {
            img_type = RCIR_W_S2;
        }
        else if(image_para.imgtype == LCIR_W_S2_5)
        {
            img_type = LCIR_W_S2_5;
        }
        else if(image_para.imgtype == RCIR_W_S2_5)
        {
            img_type = RCIR_W_S2_5;
        }
        else if(image_para.imgtype == LCIR_W_S3)
        {
            CIR_W_S3 += 1; //s3阶段时间计时
            img_type = LCIR_W_S3;
        }
        else if(image_para.imgtype == RCIR_W_S3)
        {
            CIR_W_S3 += 1; //s3阶段时间计时
            img_type = RCIR_W_S3;
        }
        else if(image_para.imgtype == LCIR_W_S4)
        {
            img_type = LCIR_W_S4;
        }
        else if(image_para.imgtype == RCIR_W_S4)
        {
            img_type = RCIR_W_S4;
        }
        else if(image_para.imgtype == LCIR_W_S5)
        {
            img_type = LCIR_W_S5;
        }
        else if(image_para.imgtype == RCIR_W_S5)
            img_type = RCIR_W_S5;
        else if(image_para.imgtype == LCIR_W_S6)
            img_type = LCIR_W_S6;
        else if(image_para.imgtype == RCIR_W_S6)
            img_type = RCIR_W_S6;
        else if(image_para.imgtype == LCIR_W_S7)
        {
            img_type = LCIR_W_S7;
            CIR_ENABLE = 0;   //关闭环岛识别一段时间
        }
        else if(image_para.imgtype == RCIR_W_S7)
        {
            img_type = RCIR_W_S7;
            CIR_ENABLE = 0;   //关闭环岛识别一段时间
        }
    }
    else if(img_mode == 3 && start == 1) //为障碍
    {
        //方向往左打，距离变大，直走，
        if(image_para.imgtype == BARR_W_S1 && dl1a_distance_mm >= 6500)
        {
            img_type = BARR_W_S2;
            Barr_S2_Z = Angle_z_temp;
            Barr_2_flag = 0;
        }
        else if(image_para.imgtype == BARR_W_S2 && Barr_Encode >= BARR2_ENC_TH)
        {
            img_type = BARR_W_S3;
        }
        else if(image_para.imgtype == BARR_W_S3 && Angle_z_temp <= -BARR_DE4_TH && barr_turn == 1)
        {
            img_type = BARR_W_S4;
        }
        else if(image_para.imgtype == BARR_W_S3 && Angle_z_temp >= BARR_DE4_TH && barr_turn == 2)
        {
            img_type = BARR_W_S4;
        }
        else if(image_para.imgtype == BARR_W_S4 && barr_turn == 2 && ADC_Filter[0] >= 20 && ADC_Filter[2] >= 20 && Barr_Encode >= BARR3_ENC_TH)
        {
            img_mode = 0;
            img_type = STR_W;
        }
        else if(image_para.imgtype == BARR_W_S4 && barr_turn == 1 && ADC_Filter[0] >= 20 && ADC_Filter[2] >= 20 && Barr_Encode >= BARR3_ENC_TH)
        {
            img_mode = 0;
            img_type = STR_W;
        }
        else if(image_para.imgtype == BARR_W_S1)
        {
            img_type = BARR_W_S1;
        }
        else if(image_para.imgtype == BARR_W_S2)
            img_type = BARR_W_S2;
        else if(image_para.imgtype == BARR_W_S3)
            img_type = BARR_W_S3;
        else if(image_para.imgtype == BARR_W_S4)
            img_type = BARR_W_S4;
        else if(img_type == none)
            img_type =image_para.imgtype;
    }
    else if(img_mode == 4 && start == 1)  //为入库
    {
        if(image_para.imgtype == LGARA_W_S1 && (Num_jump(MT9V03X_DVP_H-10,MT9V03X_DVP_W/4,MT9V03X_DVP_W*3/4) > CHANGE_TH-4 || Num_jump(MT9V03X_DVP_H-5,MT9V03X_DVP_W/4,MT9V03X_DVP_W*3/4) > CHANGE_TH-4))
        {
            img_type = LGARA_W_S2;
        }
        else if(image_para.imgtype == RGARA_W_S1 && (Num_jump(MT9V03X_DVP_H-10,MT9V03X_DVP_W/4,MT9V03X_DVP_W*3/4) > CHANGE_TH-4 || Num_jump(MT9V03X_DVP_H-5,MT9V03X_DVP_W/4,MT9V03X_DVP_W*3/4) > CHANGE_TH-4))
        {
            img_type = RGARA_W_S2;
        }
        else if(image_para.imgtype == LGARA_W_S2 && (cen_black >= 30 || Angle_z_temp >= GAR_ED_OUT) && Gara_Encode > GARA_ENC)
        {
            img_type = LGARA_W_S3;
            start = 0;
        }
        else if(image_para.imgtype == RGARA_W_S2 && (cen_black >= 30 || Angle_z_temp <= -GAR_ED_OUT) && Gara_Encode > GARA_ENC)
        {
            img_type = RGARA_W_S3;
            start = 0;
        }
        //后面保证在进入某一阶段后不能退出
        else if(image_para.imgtype == LGARA_W_S1)
        {
            img_type = LGARA_W_S1;
        }
        else if(image_para.imgtype == RGARA_W_S1)
        {
            img_type = RGARA_W_S1;
        }
        else if(image_para.imgtype == LGARA_W_S2)
        {
            img_type = LGARA_W_S2;
        }
        else if(image_para.imgtype == RGARA_W_S2)
        {
            img_type = RGARA_W_S2;
        }
        else if(image_para.imgtype == LGARA_W_S3)
        {
            img_type = LGARA_W_S3;
            start = 0;
        }
        else if(image_para.imgtype == RGARA_W_S3)
        {
            img_type = RGARA_W_S3;
            start = 0;
        }
    }
    else if(img_mode == 5 && start == 1)  //断路
    {
        if(open_in_start == 0) //普通模式
        {
            if(image_para.imgtype == OPEN_W_S1 && cen_black < 5 && Open_Encode > OPEN_ENC_TH)
            {
                img_mode = 0;
                img_type = STR_W;
                flag_end = 1;
            }
            else
                img_type = OPEN_W_S1;
        }
        else if(open_in_start == 1)//惯导
        {
#ifdef Ture
            if(image_para.imgtype == OPEN_W_S1 && Open_Encode > OPEN_ENC_TH && open_t >= 5 && cen_black < 5 && OPpen_Encode > OPEN_S12_ENC) //看到几次白色赛道
            {
                img_type = OPEN_W_S2;
            }
            else if(image_para.imgtype == OPEN_W_S2 && ADC_Filter[0] >= 25 && ADC_Filter[2] >= 25)  //双端都有电磁，进入赛道
            {
                img_mode = 0;
                img_type = STR_W;
            }
            else if(image_para.imgtype == OPEN_W_S2 && ADC_Filter[0] >= 25 && ADC_Filter[2] >= 25)  //双端都有电磁，进入赛道
            {
                img_mode = 0;
                img_type = STR_W;
            }
            else if(image_para.imgtype == OPEN_W_S1)
            {
                img_type = OPEN_W_S1;
                if(cen_black < 5 && Open_Encode > OPEN_ENC_TH)  //黑点数少，看到白道，开始计时
                {
                    open_t += 1;
                }
            }
            else if(image_para.imgtype == OPEN_W_S2)
            {
                img_type = OPEN_W_S2;
            }
#else
            if(image_para.imgtype == OPEN_W_S1 && cen_black <= 5 && OPpen_Encode >= goal_encode[4] && get_max_num(ADC_Filter,3,1) >= 20 && ADC_Filter[0] > ADC_Filter[2])
            {
                img_type = OPEN_W_S2;
                end_turn = 1;
            }
            else if(image_para.imgtype == OPEN_W_S1 && cen_black <= 5 && OPpen_Encode >= goal_encode[4] && get_max_num(ADC_Filter,3,1) >= 20 && ADC_Filter[2] > ADC_Filter[0])
            {
                img_type = OPEN_W_S2;
                end_turn = -1;
            }
            else if(image_para.imgtype == OPEN_W_S2 && end_turn == 1 && ADC_Filter[0] >= 25 && ADC_Filter[2] >= 25) //双端都有电磁，进入赛道
            {
                img_mode = 0;
                img_type = STR_W;
            }
            else if(image_para.imgtype == OPEN_W_S2 && end_turn == -1 && ADC_Filter[0] >= 25 && ADC_Filter[2] >= 25) //双端都有电磁，进入赛道
            {
                img_mode = 0;
                img_type = STR_W;
            }
            else if(image_para.imgtype == OPEN_W_S1)
            {
                img_type = OPEN_W_S1;
                if(cen_black < 5 && OPpen_Encode > goal_encode[4])  //最后一阶段黑点数少，看到白道，开始计时
                {
                    open_t += 1;
                }
            }
            else if(image_para.imgtype == OPEN_W_S2)
            {
                img_type = OPEN_W_S2;
            }
#endif
        }
        else   //其他情况为普通模式
        {
            if(image_para.imgtype == OPEN_W_S1 && cen_black < 5)
            {
                img_mode = 0;
                img_type = STR_W;
            }
            else
                img_type = OPEN_W_S1;
        }
    }
    else if(img_mode == 6 && start == 1) //当图像识别模式为6（十字）且已经出库
    {
        //图像底为白，进入拐弯阶段
        if(image_para.imgtype == CRS_W_S2 && count_white(MT9V03X_H-25,MT9V03X_W/2,MT9V03X_W-2) > 30 && count_white(MT9V03X_H-30,0,MT9V03X_W/2) > 30)
        {
            img_type = CRS_W_S3;
        }
        else if(image_para.imgtype == CRS_W_S2 && count_white(MT9V03X_H-25,0,MT9V03X_W/2) > 30 && count_white(MT9V03X_H-30,MT9V03X_W/2,MT9V03X_W-30) > 35)
        {
            img_type = CRS_W_S3;
        }//退出阶段额定时间=额定值+额定值*归一化的误差
        else if(image_para.imgtype == CRS_W_S3 && abs_f(Angle_z_temp) >= 35 && crs_turn == 2 && ADC_Filter[2]-ADC_Filter[0] >= 0)
        {
            crs_num += 1;   //运行次数加一
            img_type = STR_W;
            img_mode = 0;
        }
        else if(image_para.imgtype == CRS_W_S3 && abs_f(Angle_z_temp) >= 35 && crs_turn == 1 && ADC_Filter[0]-ADC_Filter[1] > 0)
        {
            crs_num += 1;   //运行次数加一
            img_type = STR_W;
            img_mode = 0;
        }
        //else if(image_para.imgtype == CRS_W_S3 && (part_type[0] == 0 || part_type[0] == 1 ||  part_type[0] == 2) && part_num == 1 && crs_t >= 15*(1+abs_f(offest_crs_S3/weight_crs_S3)/45) && abs_u16(ADC_Filter[0]-ADC_Filter[2]) < 30)
        else if(image_para.imgtype == CRS_W_S3 && abs_f(Angle_z_temp) >= 40 && abs_u16(ADC_Filter[0]-ADC_Filter[2]) < 20 && crs_turn != 0)
        {
            crs_num += 1;   //运行次数加一
            img_type = STR_W;
            img_mode = 0;
        }//直走模式
        else if(image_para.imgtype == CRS_W_S3 && crs_turn == 0 && crs_t > 40 && (count_white(MT9V03X_H-5,0,MT9V03X_W/3) < 15 || count_white(MT9V03X_H-5,MT9V03X_W*2/3,MT9V03X_W) < 15))
        {
            crs_num += 1;   //运行次数加一
            img_type = STR_W;
            img_mode = 0;
        }
//        else if(image_para.imgtype == CRS_W_S3 && abs_f(Angle_z_temp) >= CRS_DE) //转过90°退出十字
//        {
//            crs_num += 1;   //运行次数加一
//            img_type = STR_W;
//            img_mode = 0;
//        }
//        else if(image_para.imgtype == CRS_W_S3 && crs_t > 200)  //运行时间长，退出
//        {
//            img_type = STR_W;
//            img_mode = 0;
//        }
        else if(image_para.imgtype == CRS_W_S2)
        {
            img_type = CRS_W_S2;
        }
        else if(image_para.imgtype == CRS_W_S3)
        {
            img_type = CRS_W_S3;
            crs_t += 1;//记录s3十字时间
        }
        else
            img_type = image_para.imgtype;
    }
    else if(img_mode == 7 && start == 1)//坡道模式
    {
        uint16 part_1_row_width = 0;   //类型为0时赛道宽度和
        uint16 part_1_sum = 0;         //类型为0的行累计
        for(uint16 i = MT9V03X_DVP_H-1; i >= MT9V03X_DVP_H/2; i -= 2)
        {
            if(part_type[i] == 0)
            {
                part_1_row_width += row_width[i];
                part_1_sum += 1;
            }
        }
        if(part_1_sum == 0)
        {
            part_1_row_width += row_width[35];
            part_1_row_width += row_width[40];
            part_1_row_width += row_width[45];
            part_1_sum += 3;
        }
        if(image_para.imgtype == RAMP_S1 && part_1_row_width/part_1_sum <= 65)
        {
            img_type = RAMP_S2;
        }
        else if(image_para.imgtype == RAMP_S2 && part_1_row_width/part_1_sum >= 80)
        {
            img_type = STR_W;
            img_mode = 0;
        }
        else if(image_para.imgtype == RAMP_S1)
            img_type = RAMP_S1;
        else if(image_para.imgtype == RAMP_S2)
            img_type = RAMP_S2;
        else if(image_para.imgtype == none)  //错误标志
        {
            img_type = STR_W;
            img_mode = 0;
        }
    }
    image_para.imgtype = img_type;//赋值图像类型
}


//-------------------------------------------------------------------------------------------------------------------
//  函数功能：计算图像位置偏差值
//  参数：void
//  返回值：图像位置偏差值
//-------------------------------------------------------------------------------------------------------------------
float offset_process(void)
{
    int16 i;
    float offset = 0;
    float weight = 0;
    float offest_bottom = 0;
    float weight_bottom = 0;  //底端误差
    int16 road_len=0;       //赛道长度
    int16 offset_si;        //计算偏差值的起始行i值
    int16 offset_ei;        //计算偏差值的终止行i值，注意i从大到小为从下到上
    int16 offset_bottom_si; //计算底部偏差值的起始行i值
    int16 offset_bottom_ei; //计算底部偏差值的终止行i值，注意i从大到小为从下到上
    offset_si = 30;
    offset_ei = 10;
    offset_bottom_si = MT9V03X_DVP_H-2;
    offset_bottom_ei = MT9V03X_DVP_H-12;
    //确定计算的行
    for(i=MT9V03X_DVP_H-1;i>END_I;i-=2)
    {
        if(mt9v03x_image_dvp[i][(leftside[MT9V03X_DVP_H-1]+rightside[MT9V03X_DVP_H-1])/2] >= thres)
        {
            road_len = i;
            break;
        }
    }
    if((MT9V03X_DVP_H - offset_ei) > road_len*2)
    {
        offset_ei = MT9V03X_DVP_H - road_len;
        if(offset_ei+17 > MT9V03X_DVP_H-1)
            offset_si = MT9V03X_DVP_H-1;
        else
            offset_si = offset_ei+17;
    }
    //坡道,处于坡道时行数取低，防止因为远处的图像而在坡顶掉落
    if(img_mode == 7)
    {
        offset_si = 47;
        offset_ei = 42;
    }
    if(flag_cen_false == 1 && img_mode != 3 && img_mode != 5)  //除了障碍s3断路以外的中线错误
    {
        offset = image_para.offset;
        weight = 1;
    }
    //十字路口S1阶段
    else if(image_para.imgtype == CRS_W_S1)
    {
        //全图搜索，用双边界的各行相对于图像中线的偏差作为位置偏差值
        for(i = MT9V03X_DVP_H-1; i >= END_I; i -= 1)
        {
            if(roweff_flag[i] == 0)
            {
                offset += (float)(centerline[i]-MT9V03X_DVP_W/2);
                weight += 1;
            }
        }
        //十字偏差限制
        if(offset > CRS_OFFSET_TH*weight)
            offset = CRS_OFFSET_TH*weight;
        else if(offset < -CRS_OFFSET_TH*weight)
            offset = -CRS_OFFSET_TH*weight;
    }
    //斜入十字路口S1阶段
    else if(image_para.imgtype == CRS_W_S1_1 || image_para.imgtype == CRS_W_S1_2)
    {
        //全图搜索，用双边界的各行相对于图像中线的偏差作为位置偏差值
        for(i = MT9V03X_DVP_H-1; i >= END_I; i -= 1)
        {
            if(roweff_flag[i] == 1 || roweff_flag[i] == 2)
            {
                if(image_para.imgtype == CRS_W_S1_1)
                {
                    offset += (float)(centerline[i]-MT9V03X_DVP_W/2)+10;
                }
                if(image_para.imgtype == CRS_W_S1_2)
                {
                    offset += (float)(centerline[i]-MT9V03X_DVP_W/2)-10;
                }
                weight += 1;
            }
        }
    }
    else if(image_para.imgtype == CRS_W_S2)
    {
        if(crs_turn == 0)  //直走
        {
            //全图搜索，用双边界的各行相对于图像中线的偏差作为位置偏差值
            for(i = MT9V03X_DVP_H-1; i >= END_I; i -= 1)
            {
                if(roweff_flag[i] == 0 || roweff_flag[i] == 4)
                {
                    offset += (float)(centerline[i]-MT9V03X_DVP_W/2);
                    weight += 1;
                }
                else if(roweff_flag[i] == 1 || roweff_flag[i] == 5)
                {
                    offset += (float)(rightside[i]-rs_std[i]);
                    weight += 1;
                }
                else if(roweff_flag[i] == 2 || roweff_flag[i] == 6)
                {
                    offset += (float)(leftside[i]-ls_std[i]);
                    weight += 1;
                }
            }
            if(weight < 4)  //看到的边界太少
            {
                offset = image_para.offset;
                weight = 1;
            }
            if(weight == 0)
                weight = 1;
        }
        else if(abs_f(image_para.offset) <= CRS_OFFSET_TH)
        {
            offset = image_para.offset;
            weight = 1;
        }
        else  //大误差
        {
            //全图搜索，用双边界的各行相对于图像中线的偏差作为位置偏差值
            for(i = MT9V03X_DVP_H-1; i >= END_I; i -= 1)
            {
                if(roweff_flag[i] == 0 || roweff_flag[i] == 4)
                {
                    offset += (float)(centerline[i]-MT9V03X_DVP_W/2);
                    weight += 1;
                }
                else if(roweff_flag[i] == 1 || roweff_flag[i] == 5)
                {
                    offset += (float)(rightside[i]-rs_std[i]);
                    weight += 1;
                }
                else if(roweff_flag[i] == 2 || roweff_flag[i] == 6)
                {
                    offset += (float)(leftside[i]-ls_std[i]);
                    weight += 1;
                }
            }
            if(weight < 4)  //看到的边界太少
            {
                offset = image_para.offset;
                weight = 1;
            }
            if(weight == 0)
                weight = 1;
            if(offest_crs_S3 <= 2000 && weight != 0)
            {
                offest_crs_S3 += offset/weight;
                weight_crs_S3 += 1;
            }
        }
    }
    else if(image_para.imgtype == CRS_W_S3)//十字路口S3
    {
        if(crs_turn == 1) //十字左走,寻左线
        {
            for(i = MT9V03X_DVP_H-1; i >= END_I; i -= 1)
            {
                offset += (float)1.5*(leftside[i]-ls_std[i]);
                weight += 1;
            }
            if(-offest_K < 0)
                offset -= 50*offest_K;
        }
        else if(crs_turn == 2) //十字右走,寻右线
        {
            for(i = MT9V03X_DVP_H-1; i >= END_I; i -= 1)
            {
                offset += (float)1.5*(rightside[i]-rs_std[i]);
                weight += 1;
            }
            if(-offest_K > 0)
            offset -= 50*offest_K;
        }
        else  //其他情况直走
        {
            //全图搜索，用双边界的各行相对于图像中线的偏差作为位置偏差值
            for(i = MT9V03X_DVP_H-1; i >= END_I; i -= 1)
            {
                if(roweff_flag[i] == 0)
                {
                    offset += (float)(centerline[i]-MT9V03X_DVP_W/2);
                    weight += 1;
                }
            }
            //十字偏差限制
            if(offset > CRS_OFFSET_TH*weight)
                offset = CRS_OFFSET_TH*weight;
            else if(offset < -CRS_OFFSET_TH*weight)
                offset = -CRS_OFFSET_TH*weight;
        }
    }
    else if(image_para.imgtype == CIR_W_ADC_1)  //电感识别到，跟普通路段像
    {
        for(i = offset_si; i >= offset_ei; i -= 1)
        {
            if(roweff_flag[i] == 0 || roweff_flag[i] == 4)
            {
                offset += (float)(centerline[i]-MT9V03X_DVP_W/2);
                weight += 1;
            }
            else if(roweff_flag[i] == 1 || roweff_flag[i] == 5)
            {
                offset += (float)(rightside[i]-rs_std[i]);
                weight += 1;
            }
            else if(roweff_flag[i] == 2 || roweff_flag[i] == 6)
            {
                offset += (float)(leftside[i]-ls_std[i]);
                weight += 1;
            }
        }
    }
    else if(image_para.imgtype == LCIR_W_S6 || image_para.imgtype == LCIR_W_S7)  //左环部分阶段按照右边界循迹
    {
        for(i = offset_si; i >= offset_ei; i -= 1)
        {
            offset += (float)(rightside[i]-rs_std[i]);
            weight += 1;
        }
    }
    else if(image_para.imgtype == RCIR_W_S6 || image_para.imgtype == RCIR_W_S7)  //右环部分阶段按照左边界循迹
    {
        for(i = offset_si; i >= offset_ei; i -= 1)
        {
            offset += (float)(leftside[i]-ls_std[i]);
            weight += 1;
        }
    }
    else if(image_para.imgtype == LCIR_W_S3)
    {
        if(s3_encode <= CIR_D_S3_EN)
        {
            if(part_type[1] == 7)
                cirsud_i = part_index[0]-1;
            for(i = offset_si; i >= offset_ei; i -= 1)
            {
                if(i > cirsud_i)
                {
                    if(code_mode == 1)
                    {
                        row_boundary_process(i, leftside[MT9V03X_H-1]+30, leftside[MT9V03X_H-1]+30, 0);
                        offset += (float)(leftside[i]-ls_std[i]);
                        weight += 1;
                    }
                    else if(code_mode == 2)
                    {
                        offset = -40;
                        weight = 1;
                    }
                }
                else
                {
                    row_boundary_process(i, leftside[MT9V03X_H-1]+20, leftside[MT9V03X_H-1]+20, 0);
                    offset += (float)(rightside[i]-rs_std[i]);
                    weight += 1;
                }
            }
            if(CIR_W_S3 <= 15 && code_mode == 1)   //延迟一段时间再拐
            {
                offset = 0;
                weight = 1;
            }
        }
        else
        {
            for(i = offset_si; i >= offset_ei; i -= 1)
            {
                if(part_type[1] == 7)
                    cirsud_i = part_index[0]-1;
                if(i > cirsud_i)
                {
                    if(code_mode == 1)
                    {
                        row_boundary_process(i, leftside[MT9V03X_H-1]+30, leftside[MT9V03X_H-1]+30, 0);
                        offset += (float)(leftside[i]-ls_std[i]);
                        weight += 1;
                    }
                    else if(code_mode == 2)
                    {
                        offset = -40;
                        weight = 1;
                    }
                }
                else
                {
                    row_boundary_process(i, leftside[MT9V03X_H-1]+20, leftside[MT9V03X_H-1]+20, 0);
                    offset += (float)(rightside[i]-rs_std[i]);
                    weight += 1;
                }
            }
        }
    }
    else if(image_para.imgtype == RCIR_W_S3)  //左右不对称
    {
        if(s3_encode <= CIR_D_S3_EN)
        {
            if(part_type[1] == 8)
                cirsud_i = part_index[0]-1;
            for(i = offset_si; i >= offset_ei; i -= 1)
            {
                if(i > cirsud_i)
                {
                    if(code_mode == 1)
                    {
                        row_boundary_process(i, rightside[MT9V03X_H-1]-30, rightside[MT9V03X_H-1]-30, 0);
                        offset += (float)(rightside[i]-rs_std[i]);
                        weight += 1;
                    }
                    else if(code_mode == 2)
                    {
                        offset = 40;
                        weight = 1;
                    }
                }
                else
                {
                    row_boundary_process(i, rightside[MT9V03X_H-1]-20, rightside[MT9V03X_H-1]-20, 0);
                    offset += (float)(leftside[i]-ls_std[i]);
                    weight += 1;
                }
            }
            if(CIR_W_S3 <= 15 && code_mode == 1) //延迟一段时间再拐
            {
                offset = 0;
                weight = 1;
            }
        }
        else
        {
            if(part_type[1] == 8)
                cirsud_i = part_index[0]-1;
            for(i = offset_si; i >= offset_ei; i -= 1)
            {
                if(i > cirsud_i)
                {
                    if(code_mode == 1)
                    {
                        row_boundary_process(i, rightside[MT9V03X_H-1]-30, rightside[MT9V03X_H-1]-30, 0);
                        offset += (float)(rightside[i]-rs_std[i]);
                        weight += 1;
                    }
                    else if(code_mode == 2)
                    {
                        offset = 40;
                        weight = 1;
                    }
                }
                else
                {
                    row_boundary_process(i, rightside[MT9V03X_H-1]-20, rightside[MT9V03X_H-1]-20, 0);
                    offset += (float)(leftside[i]-ls_std[i]);
                    weight += 1;
                }
            }
        }
    }
    else if(image_para.imgtype == LCIR_W_S4)
    {
        if(S3S4_flag == 1)  //当第一次进入S4阶段时，该帧图像的各行类型判断采用了环岛S3阶段的判断方法，因此需要特殊处理
        {
            S3S4_flag = 0;
            for(i = offset_si; i >= offset_ei; i -= 1)
            {
                row_boundary_process(i, leftside[MT9V03X_H-1]+10, leftside[MT9V03X_H-1]+10, 0); //重算一遍边界，防止因为起始点导致边界查找错误
                if(roweff_flag[i] == 0 || roweff_flag[i] == 4)
                {
                    offset += (float)(centerline[i]-MT9V03X_W/2);
                    weight += 1;
                }
                else if(roweff_flag[i] == 1 || roweff_flag[i] == 5)
                {
                    offset += (float)(rightside[i]-rs_std[i]);
                    weight += 1;
                }
                else if(roweff_flag[i] == 2 || roweff_flag[i] == 6)
                {
                    offset += (float)(leftside[i]-ls_std[i]);
                    weight += 1;
                }
            }
            //计算S4阶段的offset的平均值，作为S5阶段的值
            if(weight != 0)
                offset_S5 = offset/weight;
            else
                offset_S5 =-40;
            weight_S5 = 1;
            //防止误差突变
            if(weight == 0)
                weight = 1;
        }
        else
        {
            for(i = offset_si; i >= offset_ei; i -= 1)
            {
                if(roweff_flag[i] == 0 || roweff_flag[i] == 4)
                {
                    offset += (float)(centerline[i]-MT9V03X_W/2);
                    weight += 1;
                }
                else if(roweff_flag[i] == 1 || roweff_flag[i] == 5)
                {
                    offset += (float)(rightside[i]-rs_std[i]);
                    weight += 1;
                }
                else if(roweff_flag[i] == 2 || roweff_flag[i] == 6)
                {
                    offset += (float)(leftside[i]-ls_std[i]);
                    weight += 1;
                }
            }
            for(i = offset_bottom_si; i >= offset_bottom_ei; i -= 1)
            {
                if(roweff_flag[i] == 0 || roweff_flag[i] == 4)
                {
                    offest_bottom += (float)(centerline[i]-MT9V03X_DVP_W/2);
                    weight_bottom += 1;
                }
                else if(roweff_flag[i] == 1 || roweff_flag[i] == 5)
                {
                    offest_bottom += (float)(rightside[i]-rs_std[i]);
                    weight_bottom += 1;
                }
                else if(roweff_flag[i] == 2 || roweff_flag[i] == 6)
                {
                    offest_bottom += (float)(leftside[i]-ls_std[i]);
                    weight_bottom += 1;
                }
            }
            if(weight_S5 <= 2000 && weight != 0 && offset/weight <= -20)
            {
                offset_S5 += offset/weight;
                weight_S5 += 1;
            }
        }

    }
    else if(image_para.imgtype == RCIR_W_S4)
    {
        if(S3S4_flag == 1)
        {
            S3S4_flag = 0;
            for(i = offset_si; i >= offset_ei; i -= 1)
            {
                row_boundary_process(i, rightside[MT9V03X_H-1]-10, rightside[MT9V03X_H-1]-10, 0);
                if(roweff_flag[i] == 0 || roweff_flag[i] == 4)
                {
                    offset += (float)(centerline[i]-MT9V03X_W/2);
                    weight += 1;
                }
                else if(roweff_flag[i] == 1 || roweff_flag[i] == 5)
                {
                    offset += (float)(rightside[i]-rs_std[i]);
                    weight += 1;
                }
                else if(roweff_flag[i] == 2 || roweff_flag[i] == 6)
                {
                    offset += (float)(leftside[i]-ls_std[i]);
                    weight += 1;
                }
            }
            if(weight != 0)
                offset_S5 = offset/weight;
            else
                offset_S5 = 40;
            weight_S5 = 1;
            if(weight == 0)
                weight = 1;
        }
        else
        {
            for(i = offset_si; i >= offset_ei; i -= 1)
            {
                for(i = offset_si; i >= offset_ei; i -= 1)
                {
                    if(roweff_flag[i] == 0 || roweff_flag[i] == 4)
                    {
                        offset += (float)(centerline[i]-MT9V03X_W/2);
                        weight += 1;
                    }
                    else if(roweff_flag[i] == 1 || roweff_flag[i] == 5)
                    {
                        offset += (float)(rightside[i]-rs_std[i]);
                        weight += 1;
                    }
                    else if(roweff_flag[i] == 2 || roweff_flag[i] == 6)
                    {
                        offset += (float)(leftside[i]-ls_std[i]);
                        weight += 1;
                    }
                }
            }
            for(i = offset_bottom_si; i >= offset_bottom_ei; i -= 1)
            {
                if(roweff_flag[i] == 0 || roweff_flag[i] == 4)
                {
                    offest_bottom += (float)(centerline[i]-MT9V03X_DVP_W/2);
                    weight_bottom += 1;
                }
                else if(roweff_flag[i] == 1 || roweff_flag[i] == 5)
                {
                    offest_bottom += (float)(rightside[i]-rs_std[i]);
                    weight_bottom += 1;
                }
                else if(roweff_flag[i] == 2 || roweff_flag[i] == 6)
                {
                    offest_bottom += (float)(leftside[i]-ls_std[i]);
                    weight_bottom += 1;
                }
            }
            if(weight_S5 <= 2000 && weight != 0 && offset/weight >= 15)
            {
                offset_S5 += offset/weight;
                weight_S5 += 1;
            }
        }
    }
    else if(image_para.imgtype == LCIR_W_S5 || image_para.imgtype == RCIR_W_S5) //环岛S5阶段采用S4阶段的平均值作为其开环值
    {
        offset = offset_S5;
        weight = weight_S5;
        if(weight_S5 < 3)
        {
            if(image_para.imgtype == LCIR_W_S5)  //30
                offset = -25;
            else if(image_para.imgtype == RCIR_W_S5)
                offset = 25;
            weight = 1;
        }
        //限制误差
        else if(offset_S5/weight_S5 >= 40 || offset_S5/weight_S5 <= -40)
        {
            if(image_para.imgtype == LCIR_W_S5)
                offset = -40;
            else if(image_para.imgtype == RCIR_W_S5)
                offset = 40;
            weight = 1;
        }
        else if(offset_S5/weight_S5 <= 20 && offset_S5/weight_S5 >= -20)  //30
        {
            if(image_para.imgtype == LCIR_W_S5)
                offset = -20;
            else if(image_para.imgtype == RCIR_W_S5)
                offset = 20;
            weight = 1;
        }
    }
    //障碍误差
    else if(image_para.imgtype == BARR_W_S1)
    {
        if(barr_turn == 1)
        {
            offset = -offest_quike;
            weight = 1;
        }
        else if(barr_turn == 2)
        {
            offset = offest_quike;
            weight = 1;
        }
        else
        {
            offset = -offest_quike;
            weight = 1;
        }
    }
    else if(image_para.imgtype == BARR_W_S2)
    {
        //转的角度刚好偏离障碍，但还是太小了，这里还有再拐一点,然后直走
        if(barr_turn == 1)
        {
            if(Angle_z_temp < Barr_S2_Z+BARR_DE_TH && Barr_2_flag == 0)
            {
                offset = -offest_quike;
                weight = 1;
            }
            else
            {
                Barr_2_flag = 1;
                offset = 0;
                weight = 1;
            }
        }
        else if(barr_turn == 2)
        {
            if(Angle_z_temp > Barr_S2_Z-BARR_DE_TH && Barr_2_flag == 0)
            {
                offset = offest_quike;
                weight = 1;
            }
            else
            {
                Barr_2_flag = 1;
                offset = 0;
                weight = 1;
            }
        }
        else
        {
            if(Angle_z_temp < Barr_S2_Z+BARR_DE_TH)
            {
                offset = -offest_quike;
                weight = 1;
            }
            else
            {
                offset = 0;
                weight = 1;
            }
        }
    }
    else if(image_para.imgtype == BARR_W_S3)
    {
        if(barr_turn == 1)
        {
            offset = offest_smell;
            weight = 1;
        }
        else if(barr_turn == 2)
        {

            offset = -offest_smell;
            weight = 1;
        }
        else
        {
            offset = offest_smell;
            weight = 1;
        }
    }
    else if(image_para.imgtype == BARR_W_S4)
    {
        if(flag_cen_false == 0) //&& get_max_num(ADC_Filter,3,1) >= 10
        {
            //底端误差
            for(i = offset_si; i >= offset_ei; i -= 1)
            {
                if(roweff_flag[i] == 0 || roweff_flag[i] == 4)
                {
                    offset += (float)(centerline[i]-MT9V03X_DVP_W/2);
                    weight += 1;
                }
                else if(roweff_flag[i] == 1 || roweff_flag[i] == 5)
                {
                    offset += (float)(rightside[i]-rs_std[i]);
                    weight += 1;
                }
                else if(roweff_flag[i] == 2 || roweff_flag[i] == 6)
                {
                    offset += (float)(leftside[i]-ls_std[i]);
                    weight += 1;
                }
            }
        }
        else if(flag_cen_false == 1)   //赛道识别失误
        {
            offset = 0;
            weight = 1;
        }
    }
    //看到库，先直走
    else if(image_para.imgtype == LGARA_W_S1 ||image_para.imgtype == RGARA_W_S1)
    {
        offset = Adc_Err;
        weight = 1;
        if(offset > 30)
            offset = 30;
        else if(offset < -30)
            offset = -30;
    }
    //斑马线到底端拐
    else if(image_para.imgtype == LGARA_W_S2)
    {
        //入库误差额定值
        offset = -60;
        weight = 1;
    }
    else if(image_para.imgtype == RGARA_W_S2)
    {
        //入库误差额定值
        offset = 60;
        weight = 1;
    }
    else if(image_para.imgtype == OPEN_W_S1)//断路
    {
        if(open_in_start == 1)  //惯导模式,看到白道之前，用陀螺仪误差
        {
#ifdef Ture
            if(open_t < 5 && Open_Encode >= OPEN_ENC_TH)   //没有看到赛道
            {
                offset = 0.8*(Angle_z_temp - goal_z);
                if(offset > 45)
                    offset = 45;
                else if(offset < -45)
                    offset = -45;
                weight = 1;
            }
            else
            {
                offset = 0;
                weight = 1;
            }
#else
            if(OPpen_Encode <= goal_encode[0])  //一阶段
            {
//                offset = Angle_z_temp - goal_angle[0];
//                weight = 1;
                //大角度误差限制,这里通常角度为零，所以计不计算无所谓,目的是让车稳定
                if(Angle_z_temp - goal_angle[0] > 30)
                {
                    offset = 30;
                    weight = 1;
                }
                else if(Angle_z_temp - goal_angle[0] < -30)
                {
                    offset = -30;
                    weight = 1;
                }
                else
                {
                    offset = Angle_z_temp - goal_angle[0];
                    weight = 1;
                }
            }
            else if(OPpen_Encode > goal_encode[0] && OPpen_Encode <= goal_encode[1]) //二阶段
            {
//                offset = Angle_z_temp - goal_angle[1];
//                weight = 1;
                //大角度误差限制
                if(Angle_z_temp - goal_angle[1] > 30)
                {
                    offset = 30;
                    weight = 1;
                }
                else if(Angle_z_temp - goal_angle[1] < -30)
                {
                    offset = -30;
                    weight = 1;
                }
                else
                {
                    offset = Angle_z_temp - goal_angle[1];
                    weight = 1;
                }
            }
            else if(OPpen_Encode > goal_encode[1] && OPpen_Encode <= goal_encode[2]) //三阶段
            {
//                offset = Angle_z_temp - goal_angle[2];
//                weight = 1;
                //大角度误差限制
                if(Angle_z_temp - goal_angle[2] > 30)
                {
                    offset = 30;
                    weight = 1;
                }
                else if(Angle_z_temp - goal_angle[2] < -30)
                {
                    offset = -30;
                    weight = 1;
                }
                else
                {
                    offset = Angle_z_temp - goal_angle[2];
                    weight = 1;
                }
            }
//            else if(OPpen_Encode > goal_encode[2] && OPpen_Encode <= goal_encode[3])
//            {
//                offset = Angle_z_temp - goal_angle[3];
//                weight = 1;
//            }
            else //其他情况按最后面一个角度来算
            {
                if(Angle_z_temp - goal_angle[4] > 30)
                {
                    offset = 30;
                    weight = 1;
                }
                else if(Angle_z_temp - goal_angle[4] < -30)
                {
                    offset = -30;
                    weight = 1;
                }
                else
                {
                    offset = Angle_z_temp - goal_angle[4];
                    weight = 1;
                }
            }
#endif
        }
        else //普通模式
        {
            offset = 0;
            weight = 1;
        }
    }
    else if(image_para.imgtype ==OPEN_W_S2)
    {
#ifdef Ture
        if(flag_cen_false == 1)   //赛道识别失误
        {
            offset = 0;
            weight = 1;
        }
        else
        {
            for(i = offset_si; i >= offset_ei; i -= 1)
            {
                if(roweff_flag[i] == 0 || roweff_flag[i] == 4)
                {
                    offset += (float)(centerline[i]-MT9V03X_DVP_W/2);
                    weight += 1;
                }
                else if(roweff_flag[i] == 1 || roweff_flag[i] == 5)
                {
                    offset += (float)(rightside[i]-rs_std[i]);
                    weight += 1;
                }
                else if(roweff_flag[i] == 2 || roweff_flag[i] == 6)
                {
                    offset += (float)(leftside[i]-ls_std[i]);
                    weight += 1;
                }
            }
        }
#else
        if(flag_cen_false == 1)   //赛道识别失误
        {
            offset = 0;
            weight = 1;
        }
        else if(end_turn == 1)
        {
            for(i = offset_si; i >= offset_ei; i -= 1)
            {
                if(roweff_flag[i] == 0 || roweff_flag[i] == 4)
                {
                    offset += (float)(centerline[i]-MT9V03X_DVP_W/2);
                    weight += 1;
                }
                else if(roweff_flag[i] == 1 || roweff_flag[i] == 5)
                {
                    offset += (float)(rightside[i]-rs_std[i]);
                    weight += 1;
                }
                else if(roweff_flag[i] == 2 || roweff_flag[i] == 6)
                {
                    offset += (float)(leftside[i]-ls_std[i]);
                    weight += 1;
                }
            }
        }
        else if(end_turn == -1)
        {
            for(i = offset_si; i >= offset_ei; i -= 1)
            {
                if(roweff_flag[i] == 0 || roweff_flag[i] == 4)
                {
                    offset += (float)(centerline[i]-MT9V03X_DVP_W/2);
                    weight += 1;
                }
                else if(roweff_flag[i] == 1 || roweff_flag[i] == 5)
                {
                    offset += (float)(rightside[i]-rs_std[i]);
                    weight += 1;
                }
                else if(roweff_flag[i] == 2 || roweff_flag[i] == 6)
                {
                    offset += (float)(leftside[i]-ls_std[i]);
                    weight += 1;
                }
            }
        }
        else
        {
            offset = 0;
            weight = 1;
        }
#endif
    }
    else//正常普通路段的循迹
    {
        //前端误差
        for(i = offset_si; i >= offset_ei; i -= 1)
        {
            if(roweff_flag[i] == 0 || roweff_flag[i] == 4)
            {
                offset += (float)(centerline[i]-MT9V03X_DVP_W/2);
                weight += 1;
            }
            else if(roweff_flag[i] == 1 || roweff_flag[i] == 5)
            {
                offset += (float)(rightside[i]-rs_std[i]);
                weight += 1;
            }
            else if(roweff_flag[i] == 2 || roweff_flag[i] == 6)
            {
                offset += (float)(leftside[i]-ls_std[i]);
                weight += 1;
            }
        }
        //底端误差
        for(i = offset_bottom_si; i >= offset_bottom_ei; i -= 1)
        {
            if(roweff_flag[i] == 0 || roweff_flag[i] == 4)
            {
                offest_bottom += (float)(centerline[i]-MT9V03X_DVP_W/2);
                weight_bottom += 1;
            }
            else if(roweff_flag[i] == 1 || roweff_flag[i] == 5)
            {
                offest_bottom += (float)(rightside[i]-rs_std[i]);
                weight_bottom += 1;
            }
            else if(roweff_flag[i] == 2 || roweff_flag[i] == 6)
            {
                offest_bottom += (float)(leftside[i]-ls_std[i]);
                weight_bottom += 1;
            }
        }
    }
    //防止除数为0
    if(weight == 0)
        weight = 1;
    if(weight_bottom == 0)
        weight_bottom = 1;
    //计算误差
    if(img_mode == 2)
    {
        offset = 1.5*offset/weight;
    }
    else if(img_mode != 3 && img_mode != 4 && img_mode != 6)
        offset = 2*offset/weight;
    else
        offset = offset/weight;
    offest_bottom = offest_bottom/weight_bottom;
    offest_cam = offset;
    //限制误差
    if(offset > 70)
        offset = 70;
    else if(offset < -70)
        offset = -70;
    if(offest_bottom > 70)
        offest_bottom = 70;
    else if(offest_bottom < -70)
        offest_bottom = -70;

    offset_menb[2] = offset_menb[1];
    offset_menb[1] = offset_menb[0];
    offset_menb[0] = offset;
    offset = 0.5*offset_menb[0] + 0.3*offset_menb[1] + 0.2*offset_menb[2];
#ifdef Have  //库有电磁
    if(go_strat == 1 && start == 1)  //出库
    {
        if(Go_Encode <= GO_STR_ENC)  //先直走出库，再拐弯
        {
            offset = 0;
            weight = 1;
        }
        else
        {
            if((ADC_Filter[0] >= 10 && ADC_Filter[0] - ADC_Filter[2] >= 5) &&  go_turn == 0)
                go_turn = 1;
            else if((ADC_Filter[2] >= 10 && ADC_Filter[2] - ADC_Filter[0] >= 5) && go_turn == 0)
                go_turn = -1;
            if(go_turn == 1)
            {
                //底端误差
                for(i = offset_bottom_si; i >= offset_bottom_ei; i -= 1)
                {
                    if(roweff_flag[i] == 0 || roweff_flag[i] == 4)
                    {
                        offest_bottom += (float)(centerline[i]-MT9V03X_DVP_W/2);
                        weight_bottom += 1;
                    }
                    else if(roweff_flag[i] == 1 || roweff_flag[i] == 5)
                    {
                        offest_bottom += (float)(rightside[i]-rs_std[i]);
                        weight_bottom += 1;
                    }
                    else if(roweff_flag[i] == 2 || roweff_flag[i] == 6)
                    {
                        offest_bottom += (float)(leftside[i]-ls_std[i]);
                        weight_bottom += 1;
                    }
                }
                if(abs_f(offest_K) < 0.4)
                    offset = 1.2*offest_bottom/weight_bottom -20*offest_K ;
                else
                    offset = -20*offest_K;
                weight = 1;
                if(offset < 2)
                    offset = 2;
                if(ADC_Filter[0] >= 30 && ADC_Filter[2] >= 30 && ADC_Filter[0] < ADC_Filter[2]) //左电感大于右电感
                    go_strat = 0;
            }
            else if(go_turn == -1)
            {
                //底端误差
                for(i = offset_bottom_si; i >= offset_bottom_ei; i -= 1)
                {
                    if(roweff_flag[i] == 0 || roweff_flag[i] == 4)
                    {
                        offest_bottom += (float)(centerline[i]-MT9V03X_DVP_W/2);
                        weight_bottom += 1;
                    }
                    else if(roweff_flag[i] == 1 || roweff_flag[i] == 5)
                    {
                        offest_bottom += (float)(rightside[i]-rs_std[i]);
                        weight_bottom += 1;
                    }
                    else if(roweff_flag[i] == 2 || roweff_flag[i] == 6)
                    {
                        offest_bottom += (float)(leftside[i]-ls_std[i]);
                        weight_bottom += 1;
                    }
                }
                if(abs_f(offest_K) < 0.4)
                    offset = 1.2*offest_bottom/weight_bottom -20*offest_K ;
                else
                    offset = -20*offest_K;
                weight = 1;
                if(offset > -2)
                    offset = -2;
                if(ADC_Filter[0] >= 30 && ADC_Filter[2] >= 30 && ADC_Filter[0] > ADC_Filter[2]) //右电感大于左电感
                    go_strat = 0;
            }
            else
            {
                offset = 0;
                weight = 1;
            }
        }
    }
#else    //没有库电磁
    if(go_strat == 1 && start == 1)  //出库
    {
        if(go_turn == 1)
        {
            //底端误差
            for(i = offset_bottom_si; i >= offset_bottom_ei; i -= 1)
            {
                if(roweff_flag[i] == 0 || roweff_flag[i] == 4)
                {
                    offest_bottom += (float)(centerline[i]-MT9V03X_DVP_W/2);
                    weight_bottom += 1;
                }
                else if(roweff_flag[i] == 1 || roweff_flag[i] == 5)
                {
                    offest_bottom += (float)(rightside[i]-rs_std[i]);
                    weight_bottom += 1;
                }
                else if(roweff_flag[i] == 2 || roweff_flag[i] == 6)
                {
                    offest_bottom += (float)(leftside[i]-ls_std[i]);
                    weight_bottom += 1;
                }
            }
            if(abs_f(offest_K) < 0.4)
                offset = 1.2*offest_bottom/weight_bottom -20*offest_K ;
            else
                offset = -20*offest_K;
            weight = 1;
            if(offset < 2)
                offset = 2;
            if(ADC_Filter[0] >= 30 && ADC_Filter[2] >= 30 && ADC_Filter[0] < ADC_Filter[2]) //左电感大于右电感
                go_strat = 0;
        }
        else if(go_turn == -1)
        {
            //底端误差
            for(i = offset_bottom_si; i >= offset_bottom_ei; i -= 1)
            {
                if(roweff_flag[i] == 0 || roweff_flag[i] == 4)
                {
                    offest_bottom += (float)(centerline[i]-MT9V03X_DVP_W/2);
                    weight_bottom += 1;
                }
                else if(roweff_flag[i] == 1 || roweff_flag[i] == 5)
                {
                    offest_bottom += (float)(rightside[i]-rs_std[i]);
                    weight_bottom += 1;
                }
                else if(roweff_flag[i] == 2 || roweff_flag[i] == 6)
                {
                    offest_bottom += (float)(leftside[i]-ls_std[i]);
                    weight_bottom += 1;
                }
            }
            if(abs_f(offest_K) < 0.4)
                offset = 1.2*offest_bottom/weight_bottom -20*offest_K ;
            else
                offset = -20*offest_K;
            weight = 1;
            if(offset > -2)
                offset = -2;
            if(ADC_Filter[0] >= 30 && ADC_Filter[2] >= 30 && ADC_Filter[0] > ADC_Filter[2]) //右电感大于左电感
                go_strat = 0;
        }
        else
        {
            offset = 0;
            weight = 1;
        }
    }
#endif
    return offset;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数功能：判断斑马线跳变次数
//  参数：line判断的行数;
//  返回值：跳变次数
//-------------------------------------------------------------------------------------------------------------------
uint16 Num_jump(uint16 line,uint16 start_i,uint16 end_i)
{
    uint16 i = 0;
    uint16 count = 0; //斑马线数量
    float x, y, sar;            //x和y为中间变量，sar为差和比值
    int32 gain;                 //中间变量
    //计算第line行斑马线跳变数量
    count=0;
    if(start_i <= 2)
        start_i = 2;
    if(end_i >= MT9V03X_W - 2)
        end_i = MT9V03X_W - 2;
    for(i = end_i;i > start_i;i -= 2)
    {
        //此处选用这种方法是因为可以不用浮点数乘法，加快运算速度
        gain = (mt9v03x_image_dvp[line][i]-mt9v03x_image_dvp[line][i-2])<<8;
        x = (float)gain;
        y = (float)(mt9v03x_image_dvp[line][i]+mt9v03x_image_dvp[line][i-2]+1);
        sar = abs_f(x/y);
        if(sar > SAR_TH+20)
        {
            count+=1;
        }
    }
    return count;
}

uint16 Num_slop_jump(void)
{
    uint16 i = 0,re = 0;
    uint16 count,count_max = 0;
    int16 width_sum=0;
    int16 width_weghit=0;
    uint16 start_i = 0;
    uint16 end_i = 0;
    for(i = MT9V03X_DVP_H-10; i >= 10; i -= 2)
    {
        if(roweff_flag[i] == 0)
        {
            width_sum += row_width[i];
            width_weghit += 1;
        }
    }
    if(width_weghit == 0)
        width_weghit = 1;
//    if(part_num >= 2 && part_type[0] == 1 && (part_type[1] == 5 || part_type[1] == 6 || part_type[1] == 0) && width_sum/width_weghit > 45 && width_sum/width_weghit <75)
//    {//斜入库
//        start_i = part_index[1];
//        end_i = part_index[0];
//        if(end_i < 10)
//            end_i = 10;
//        for(i = start_i; i >= end_i; i -= 2)
//        {
//            count = Num_jump(i,20,rs_std[i]);
//            if(count > count_max)
//                count_max = count;
//        }
//        if(count_max >= CHANGE_TH-4)
//            re = 1;
//    }
    if(1 == 1)
    {
        //一般下
//        start_i = MT9V03X_DVP_H*2/3;
//        end_i = MT9V03X_DVP_H/3;
//        for(i = start_i; i >= end_i; i -= 2)
//        {
//            count = Num_jump(i,centerline[i] - MT9V03X_DVP_W/2 + ls_std[i],centerline[i] + rs_std[i] - MT9V03X_DVP_W/2);
//            if(count > count_max)
//                count_max = count;
//        }
//        if(count_max >= CHANGE_TH)
//            re = 1;
        //比赛前一天改的,由于比赛时特殊，所以这么改
        start_i = MT9V03X_DVP_H*2/3;
        end_i = MT9V03X_DVP_H/3;
        for(i = start_i; i >= end_i; i -= 2)
        {
            count = Num_jump(i,leftside[i],rs_std[40]);
            if(count > count_max)
                count_max = count;
        }
        if(count_max >= CHANGE_TH)
            re = 1;
    }
    return re;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数功能：判断入库
//  参数：1 判断入库，库在右边,-1 判断入库，库在边左,0 普通判断入库
//  返回值：是否有库
//-------------------------------------------------------------------------------------------------------------------
uint8 Garage_Judge(int16 dir)
{
    uint16_t i=0,j=0;
    uint16_t count=0; //斑马线数量
    float x, y, sar;            //x和y为中间变量，sar为差和比值
    int32 gain;                 //中间变量
    if(dir == -1)  //判断入库，库在右边
    {
        //计算斑马线跳变数量
        for(j=part_index[0];j>END_I+1;j-=2)
        {
            count=0;
            for(i=rightside[part_index[0]];i>2+1;i-=2)
            {
                //此处选用这种方法是因为可以不用浮点数乘法，加快运算速度
                gain = (mt9v03x_image_dvp[j][i]-mt9v03x_image_dvp[j][i-2])<<8;
                x = (float)gain;
                y = (float)(mt9v03x_image_dvp[j][i]+mt9v03x_image_dvp[j][i-2]+1);
                sar = abs_f(x/y);
                if(sar > SAR_TH + 20)
                {
                    count+=1;
                }
            }
            if(count>CHANGE_TH)
            {
                return 1;
            }
        }
    }
    else if(dir == 1) //判断入库，库在边左
    {
        for(j=part_index[0];j>END_I+1;j-=2)
        {
            count=0;
            for(i=leftside[part_index[0]];i<MT9V03X_DVP_W-3;i+=2)
            {
                //此处选用这种方法是因为可以不用浮点数乘法，加快运算速度
                gain = (mt9v03x_image_dvp[j][i]-mt9v03x_image_dvp[j][i-2])<<8;
                x = (float)gain;
                y = (float)(mt9v03x_image_dvp[j][i]+mt9v03x_image_dvp[j][i-2]+1);
                sar = abs_f(x/y);
                if(sar > SAR_TH+20)
                {
                    count+=1;
                }
            }
            if(count>CHANGE_TH)
            {
                return 1;
            }
        }
    }
    else if(dir == 0)
    {
        //计算斑马线跳变数量
        for(j=part_index[0];j>END_I+1;j-=2)
        {
            count=0;
            for(i=rightside[part_index[0]];i>2+1;i-=2)
            {
                //此处选用这种方法是因为可以不用浮点数乘法，加快运算速度
                gain = (mt9v03x_image_dvp[j][i]-mt9v03x_image_dvp[j][i-2])<<8;
                x = (float)gain;
                y = (float)(mt9v03x_image_dvp[j][i]+mt9v03x_image_dvp[j][i-2]+1);
                sar = abs_f(x/y);
                if(sar > SAR_TH)
                {
                    count+=1;
                }
            }
            if(count>CHANGE_TH)
            {
                return 1;
            }
        }
    }
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数功能：环岛计算左右边界弯曲程度
//  参数：void
//  返回值：uint6,1左弯曲大，2右弯曲大
//-------------------------------------------------------------------------------------------------------------------
uint16 circle_max(void)
{
    uint16 i = 0;
    uint16 j = 0;
    uint16 x = 0;
    uint16 r_high, r_low;
    uint16 l_high, l_low;
    uint16 ave_row_high, ave_row_low;
    uint16 wigth_l=0,wigth_r=0;
    float offest_l, offest_r;
    uint16 re=0;
    float K_r, K_l;
    for(i = END_I; i >= MT9V03X_DVP_H-2; i += 1)
    {
        if(roweff_flag[i] == 0)
        {
            r_high += rightside[i];
            l_high += leftside[i];
            j += 1;
            ave_row_high += i;
        }
        if(j >= 4)
            break;
    }
    ave_row_high = ave_row_high/4;
    j = 0;
    for(i = MT9V03X_DVP_H-2; i >= END_I; i -= 1)
    {
        if(roweff_flag[i] == 0)
        {
            r_low += rightside[i];
            l_low += leftside[i];
            j += 1;
            ave_row_low += i;
        }
        if(j >= 4)
            break;
    }
    ave_row_low = ave_row_low/4;
    K_l = (float)(l_low - l_high) / (ave_row_low - ave_row_high);
    K_r = (float)(r_low - r_high) / (ave_row_low - ave_row_high);
    for(i = END_I; i >= MT9V03X_DVP_H-2; i += 1)
    {
        if(leftside[i] != 0)
        {
            x = (uint16)(K_l * (i - ave_row_high) - l_high);
            offest_l += abs_f(x-leftside[i]);
            wigth_l += 1;
        }
        if(rightside[i] != MT9V03X_W-1)
        {
            x = (uint16)(K_r * (i - ave_row_high) - r_high);
            offest_r += abs_f(x-rightside[i]);
            wigth_r += 1;
        }
    }
    if(wigth_l != 0)
        offest_l = offest_l/wigth_l;
    if(wigth_r != 0)
    if(offest_l > offest_r)
        re = 1;
    else if(offest_r > offest_l)
        re = 2;
    return re;

}

//-------------------------------------------------------------------------------------------------------------------
//  函数功能：计算i行从start到end的白点数,是原来的一半
//  参数：uint16
//  返回值：uint16
//-------------------------------------------------------------------------------------------------------------------
uint16 count_white(uint16 i,uint16 start,uint16 end)
{
    uint16 count_white = 0;
    uint16 end_j,start_j;
    if(end >= MT9V03X_DVP_W-2)
        end_j = MT9V03X_DVP_W-2;
    else if(end <= 2)
        end_j = 2;
    else
        end_j = end;
    if(start >= MT9V03X_DVP_W-2)
        start_j = MT9V03X_DVP_W-2;
    else if(start <= 2)
        start_j = 2;
    else
        start_j = start;
    if(i >= MT9V03X_DVP_H-1)
        i= MT9V03X_DVP_H-1;
    else if(i <= 2)
        i = 2;
    for(uint16 j = start; j <= end_j; j += 2)
    {
        if(mt9v03x_image_dvp[i][j] >= thres + 10)
        {
            count_white += 1;
        }
    }
    return count_white;
}


//-------------------------------------------------------------------------------------------------------------------
//  函数功能：计算浮点数绝对值
//  参数：float
//  返回值：float
//-------------------------------------------------------------------------------------------------------------------
float abs_f(float x)
{
    if(x < 0)
        return -x;
    else
        return x;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数功能：计算浮点数绝对值
//  参数：float
//  返回值：float
//-------------------------------------------------------------------------------------------------------------------
uint16 abs_u16(uint16 x)
{
    if(x < 0)
        return -x;
    else
        return x;
}


//-------------------------------------------------------------------------------------------------------------------
//  函数功能：边界分段算法
//  参数：void
//  返回值：void
//-------------------------------------------------------------------------------------------------------------------
void boundary_part(void)
{
    int16 i;
    int16 part_i, part_flag, part_mi;   //part_i为此时的part_type的下标，part_flag为此行类型，part_mi为和并后的下标
    int16 partcnt;                      //同种类的行计数

    //行类型判断(完整版)
    rowtype_process();
    //统计各种类型的行数
    for(i = MT9V03X_DVP_H-2; i > END_I; i -= 1)
    {
        part_sum[roweff_flag[i]] += 1;
    }
    //分段类型判断
    part_i = 0;
    part_flag = roweff_flag[MT9V03X_DVP_H-1];
    partcnt = 1;

    for(i = MT9V03X_DVP_H-2; i > END_I; i -= 1)
    {
        if(part_flag == roweff_flag[i])
            partcnt += 1;
        else{
            if(partcnt > C_TH)
            {
                part_type[part_i] = part_flag;
                part_index[part_i] = i+1;
                part_i += 1;
                part_flag = roweff_flag[i];
                partcnt = 1;
            }
            else{
                part_flag = roweff_flag[i];
                partcnt = 1;
            }
        }
    }
    if(partcnt > C_TH)
    {
        part_type[part_i] = part_flag;
        part_index[part_i] = i+1;
        part_num = part_i+1;
    }
    else
    {
        part_num = part_i;
        part_index[part_i-1] = i+1;
    }

    //边界分段合并，因为以上计算出的赛道分段可能会出现相同类型的两段连续出现的情况，比如0，0，1，此时要把这合并起来
    part_mi = 1;
    for(part_i = 1; part_i < part_num; part_i += 1)
    {
        if(part_type[part_i] == part_type[part_mi-1])
            part_index[part_mi-1] = part_index[part_i];
        else
        {
            part_type[part_mi] = part_type[part_i];
            part_index[part_mi] = part_index[part_i];
            part_mi += 1;
        }
    }
    part_num = part_mi;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数功能：行类型处理（完整）
//  参数：void
//  返回值：void
//-------------------------------------------------------------------------------------------------------------------
void rowtype_process(void)
{
    int16 i;
    float dl, dr;           //相对于上一行的左右边界变化，注意i值从大到小变化
    int16 w_min;            //之前行的最小宽度
    int16 dw[MT9V03X_DVP_H];    //相对于上一行的行宽度变化
    int16 flag_sud = 0;     //突变的标志

    w_min = WIDTH_MIN;
    part_sum__1 = 0;

    for(i = MT9V03X_DVP_H-2; i >= END_I; i -= 1)
    {
        if(roweff_flag[i] != -1)
        {
            if(row_width[i] < w_min && i > 10)          //更新最小行宽度
                w_min = row_width[i];
            else if(i%2 == 0 )                          //行数越往上，宽度逐渐减小，因此需要手动减少行宽度
                w_min -= 1;

            dw[i] = row_width[i]-row_width[i+1];        //与上一行的宽度差
            dl = leftside[i]-leftside[i+1];             //与上一行的左边界差距
            dr = rightside[i]-rightside[i+1];           //与上一行的左边界差距

            //开始判断类型
            if(image_para.imgtype == LCIR_W_S2_5 || image_para.imgtype == LCIR_W_S3)
            {
                if(flag_sud == 0 && dr <= NSUD_TH && dr >= -SSUD_TH && dl >= 5*SSUD_TH) //7类型表示左边界向内突变，从特殊情况突变为正常左边界，说明突变点为入环的特殊点
                {
                    roweff_flag[i] = 7;
                    flag_sud = 7;
                }
                else if(flag_sud == 7 && dl < SSUD_TH && dl > -SSUD_TH && dr <= NSUD_TH && dr >= -SSUD_TH)
                    roweff_flag[i] = 7;
                else if(flag_sud == 0 && dr <= NSUD_TH && dr >= -SSUD_TH && dl < -SSUD_TH)  //0突变到5说明出现了误判，此时要用flag_sud记录好突变发生过了，当回到正常值时5会变为0，从而消除因为图像某一行的边界错误导致最终的错误
                {
                    roweff_flag[i] = 5;
                    flag_sud = 5;
                }
                else if(flag_sud == 5 && dl < SSUD_TH && dl > -SSUD_TH)
                    roweff_flag[i] = 5;
                else if(flag_sud == 5 && dl > SSUD_TH)
                {
                    roweff_flag[i] = 0;
                    flag_sud = 0;
                }
                else
                {
                    roweff_flag[i] = 0;
                    flag_sud = 0;
                }
            }
            else if(image_para.imgtype == RCIR_W_S2_5 || image_para.imgtype == RCIR_W_S3)
            {
                if(flag_sud == 0 && dl <= SSUD_TH && dl >= -NSUD_TH && dr <= -5*SSUD_TH)
                {
                    roweff_flag[i] = 8;
                    flag_sud = 8;
                }
                else if(flag_sud == 8 && dr < SSUD_TH && dr > -SSUD_TH && dl <= SSUD_TH && dl >= -NSUD_TH)
                    roweff_flag[i] = 8;
                else if(flag_sud == 0 && dl <= SSUD_TH && dl >= -NSUD_TH && dr > SSUD_TH)
                {
                    roweff_flag[i] = 6;
                    flag_sud = 6;
                }
                else if(flag_sud == 6 && dr < SSUD_TH && dr > -SSUD_TH)
                    roweff_flag[i] = 6;
                else if(flag_sud == 6 && dr < -SSUD_TH)
                {
                    roweff_flag[i] = 0;
                    flag_sud = 0;
                }
                else
                {
                    roweff_flag[i] = 0;
                    flag_sud = 0;
                }
            }
            //正常模式下 的图像行类型判断
            else
            {
                if((dw[i] > SUD_TH && dl < -SSUD_TH && dr > NSUD_TH) || (dw[i] > SUD_TH && dl < -NSUD_TH && dr > SSUD_TH))  //表示左边突变了，右边边界还向右走，类似斜入十字，说明此行是3类型
                {
                    roweff_flag[i] = 3;
                    flag_sud = 3;
                }
                else if(roweff_flag[i] == 3 || (roweff_flag[i] == 1 && dr > SSUD_TH && flag_sud == 0) || (roweff_flag[i] == 2 && dl < -SSUD_TH && flag_sud == 0))
                {
                    roweff_flag[i] = 3;
                    flag_sud = 3;
                }
                else if(dl < -SSUD_TH && dr < NSUD_TH && flag_sud == 0)
                {
                    roweff_flag[i] = 5;
                    flag_sud = 5;
                }
                else if(dr > SSUD_TH && dl > -NSUD_TH && flag_sud == 0)
                {
                    roweff_flag[i] = 6;
                    flag_sud = 6;
                }
                else if(flag_sud == 5 && dl < 5 && dl > -5 && dr < NSUD_TH && row_width[i] > w_min)
                {
                    roweff_flag[i] = 5;
                }
                else if(flag_sud == 6 && dr < 5 && dr > -5 && dl > -NSUD_TH && row_width[i] > w_min)
                {
                    roweff_flag[i] = 6;
                }
                else if(flag_sud == 5 && dl < NSUD_TH && dl > -NSUD_TH && dr >= NSUD_TH)
                {
                    roweff_flag[i] = 3;
                    flag_sud = 3;
                }
                else if(flag_sud == 6 && dr < NSUD_TH && dr > -NSUD_TH && dl <= -NSUD_TH)
                {
                    roweff_flag[i] = 3;
                    flag_sud = 3;
                }
                else if(flag_sud == 3 && dr < SSUD_TH && dr > -SSUD_TH && dl < SSUD_TH && dl > -SSUD_TH && row_width[i] > w_min)
                {
                    roweff_flag[i] = 3;
                }
                else if(flag_sud == 3 && (dr < -SSUD_TH || dl > SSUD_TH || row_width[i] < w_min)&&roweff_flag[i] == 0)
                {
                    flag_sud = 0;
                }
                else if((flag_sud == 3) &&(((dl > -NSUD_TH && dl < NSUD_TH)&& dr <= -SSUD_TH) || (roweff_flag[i] == 1 && row_width[i] <= w_min)))
                {
                    roweff_flag[i] = 5;
                    flag_sud = 5;
                }
                else if((flag_sud == 3) &&(((dr > -NSUD_TH && dr < NSUD_TH) && dl >= SSUD_TH) || (roweff_flag[i] == 2 && row_width[i] <= w_min)))
                {
                    roweff_flag[i] = 6;
                    flag_sud = 6;
                }
                else if(flag_sud == 5 && (dl > 15 || row_width[i] < w_min))
                {
                    flag_sud = 0;
                }
                else if(flag_sud == 6 && (dr < -15 || row_width[i] < w_min))
                {
                    flag_sud = 0;
                }
                else if( dl <= 0 && dl >= -SSUD_TH && dr >= 0 && dr <= SSUD_TH && flag_sud == 0 && dw[i] > 0)   //判断为两边边界在发散，但不是突变的情况，说明是三岔类型
                {
                    roweff_flag[i] = 4;
                }
            }
        }
        else
            part_sum__1 += 1;
    }

    //滤波算法，把单行及双行的误判去掉
    for(i = MT9V03X_DVP_H-2; i >= 1; i--)
    {
        if(roweff_flag[i-1] == roweff_flag[i+1] && roweff_flag[i-2] == roweff_flag[i+1] && roweff_flag[i] != roweff_flag[i+1])
        {
            roweff_flag[i] = roweff_flag[i+1];
        }
        if(roweff_flag[i+1] == roweff_flag[i-2] && roweff_flag[i+1] == roweff_flag[i-3] && roweff_flag[i] != roweff_flag[i+1] && roweff_flag[i-1] != roweff_flag[i+1])//若这一行是误判
        {
            roweff_flag[i] = roweff_flag[i+1];
            roweff_flag[i-1] = roweff_flag[i+1];
        }
    }
}


//-------------------------------------------------------------------------------------------------------------------
//  函数功能：获取数值的第几个最大值
//  参数：数组以及数组大小，第几个最大值
//  返回值：uint16
//-------------------------------------------------------------------------------------------------------------------
uint16 get_max_num(uint16 a[],uint16 len,uint16 num)
{
    uint16 i = 0;
    uint16 j = 0;
    uint16 p[len];
    memcpy(p,a,len*2);
    for (i = 0; i < len - 1; i++)//需要进行len-1趟
    {
        uint16 flag = 1;//flag=1，说明已经排好序
        for (j = 0; j < len - 1 - i;j++)//每趟两两比较较未排好序元素个数-1次。
        {
            if (p[j] > p[j+1])
            {
                uint16 tmp = p[j];
                p[j] = p[j + 1];
                p[j + 1] = tmp;
                flag = 0;
             }
        }
        if (flag==1)//判断是否排好序
            break;
    }
    return p[len-num];
}


//-------------------------------------------------------------------------------------------------------------------
//  函数功能：给三点x1，y1，x2，y2，x3，y3,计算x1，y1处的夹角余弦值的平方
//  参数：三点坐标
//  返回值：float
//-------------------------------------------------------------------------------------------------------------------
float angle_caculate(float x1, float y1, float x2, float y2, float x3, float y3)
{
    float cosA2;

    y1 = 2.115*y1;
    y2 = 2.115*y2;
    y3 = 2.115*y3;
    cosA2 = ((x2-x1)*(x3-x1)+(y2-y1)*(y3-y1))*((x2-x1)*(x3-x1)+(y2-y1)*(y3-y1))/(((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))*((x3-x1)*(x3-x1)+(y3-y1)*(y3-y1)));

    return cosA2;
}
//-------------------------------------------------------------------------------------------------------------------
//  函数功能：输入0算左边界弯曲程度，1算右边,2算中线
//  参数：类型m
//  返回值：float
//-------------------------------------------------------------------------------------------------------------------
float cur_circle(int16 m)
{
    int16 i;
    int16 a = 40, b = 25, c = 7;//所选三行的初始值
    float num = 0;
    float cur = 0;

    if(m == 0)
    {
        for(i = 0; i <= 5; i++)
        {
            if((roweff_flag[b+i] == 0 || roweff_flag[b+i] == 2 || roweff_flag[b+i] == 4 || roweff_flag[b+i] == 6)&&(roweff_flag[a+i] == 0 || roweff_flag[a+i] == 2 || roweff_flag[a+i] == 4 || roweff_flag[a+i] == 6)&&(roweff_flag[c+i] == 0 || roweff_flag[c+i] == 2 || roweff_flag[c+i] == 4 || roweff_flag[c+i] == 6))
            {
                num += 1;
                cur += angle_caculate(leftside[b+i], b+i, leftside[a+i], a+i, leftside[c+i], c+i);
            }
        }
    }
    if(m == 1)
    {
        for(i = 0; i <= 5; i++)
        {
            if((roweff_flag[b+i] == 0 || roweff_flag[b+i] == 1 || roweff_flag[b+i] == 4 || roweff_flag[b+i] == 5)&&(roweff_flag[a+i] == 0 || roweff_flag[a+i] == 1 || roweff_flag[a+i] == 4 || roweff_flag[a+i] == 5)&&(roweff_flag[c+i] == 0 || roweff_flag[c+i] == 1 || roweff_flag[c+i] == 4 || roweff_flag[c+i] == 5))
            {
                num += 1;
                cur += angle_caculate(rightside[b+i], b+i, rightside[a+i], a+i, rightside[c+i], c+i);
            }
        }
    }
    if(m == 2)
    {
        for(i = 0; i <= 5; i++)
        {
            if(roweff_flag[b+i] != -1 && roweff_flag[a+i] != -1 && roweff_flag[c+i] != -1)
            {
                num += 1;
                cur += angle_caculate(rightside[b+i], b+i, rightside[a+i], a+i, rightside[c+i], c+i);
            }
        }
    }

    if(num == 0)
        return 0;
    else
        return cur/num;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数功能：计算每一个边界的余闲,以及右边界最小，左边界最大
//  参数：类型m
//  返回值：
//-------------------------------------------------------------------------------------------------------------------
void cur_line_cos(void)
{
    uint16 i = 0;
    uint16 count=0;
    right_line_min[1] = MT9V03X_DVP_W-1;
    left_line_max[1] = 0;
    memset(right_cos,100,sizeof(right_cos));
    memset(left_cos,100,sizeof(left_cos));
    for(i = MT9V03X_DVP_H-6; i >= 6; i--)
    {
        right_cos[i] = (uint16)100*angle_caculate(rightside[i], i, rightside[i-4], i-4, rightside[i+4], i+4);
        left_cos[i] = (uint16)100*angle_caculate(leftside[i], i, leftside[i-4], i-4, leftside[i+4], i+4);
        left_cos_ave += left_cos[i]/10;
        right_cos_ave += right_cos[i]/10;
        if(rightside[i] < right_line_min[1] && rightside[i] > 30)
        {
            right_line_min[0] = i;
            right_line_min[1] = rightside[i];
        }
        if(leftside[i] > left_line_max[1] && leftside[i] < MT9V03X_DVP_W - 30)
        {
            left_line_max[0] = i;
            left_line_max[1] = leftside[i];
        }
    }
    left_cos_ave = left_cos_ave/(MT9V03X_DVP_H-12);
    right_cos_ave = right_cos_ave/(MT9V03X_DVP_H-12);
    //抑制滤波变化
    for(i = MT9V03X_DVP_H-7; i >= 7; i--)
    {
        if(abs_u16(right_cos[i]-right_cos[i-1]) > 40 && abs_u16(right_cos[i]-right_cos[i+1]) >40)
        {
            right_cos[i] = (right_cos[i-1] + right_cos[i+1])/2;
        }
        if(abs_u16(left_cos[i]-left_cos[i-1]) > 40 && abs_u16(left_cos[i]-left_cos[i+1]) >40)
        {
            left_cos[i] = (left_cos[i-1] + left_cos[i+1])/2;
        }
        if(right_cos[i] < 30)
        {
            count += 1;
        }
    }
}


uint16 find_rightdown_point(uint16 start_point, uint16 end_point)
{
    uint16 i;
        for (i = start_point; i <= end_point; i++)
        {
            //找右边突变(下拐点)
            if(right_cos[i] < 15 && abs_u16(rightside[i] - right_line_min[1]) < 5)    //此处行cos小于0.3，并且右边界接近最小右边界列数
            {
                right_turn_down[0] = i ;//用一维数组两个值来记录拐点坐标，为y=j，x=rightline[j]
                right_turn_down[1] = rightside[i];
                youxia_guai = 1;
                break;
            }
            else
                youxia_guai = 0;
        }
//        if(youxia_guai == 1)
//        {
//            ips114_draw_point(right_turn_down[1],right_turn_down[0]-1,RGB565_RED);
//            ips114_draw_point(right_turn_down[1],right_turn_down[0],RGB565_RED);
//            ips114_draw_point(right_turn_down[1],right_turn_down[0]+1,RGB565_RED);
//            ips114_draw_point(right_turn_down[1]-1,right_turn_down[0],RGB565_RED);
//            ips114_draw_point(right_turn_down[1]+1,right_turn_down[0],RGB565_RED);
//        }
        return i;
}
//找左下拐点函数
uint16 find_leftdown_point(uint16 start_point, uint16 end_point)
{
    uint16 i;
    //找十字左下拐点
            for (i = start_point; i <= end_point; i++)
            {
                //找左边突变(下拐点)
                if (left_cos[i] < 15 && abs_u16(leftside[i] - left_line_max[1]) < 5)
                {
                    left_turn_down[0] = i;//同上
                    left_turn_down[1] = leftside[i];
                    zuoxia_guai = 1;
                    break;
                }
                else
                    zuoxia_guai = 0;
            }
//            if(zuoxia_guai == 1 && left_turn_down[0] > 5 && left_turn_down[1] > 5)
//            {
//                ips114_draw_point(left_turn_down[1],left_turn_down[0]-1,RGB565_BLUE);
//                ips114_draw_point(left_turn_down[1],left_turn_down[0]-2,RGB565_BLUE);
//                ips114_draw_point(left_turn_down[1],left_turn_down[0],RGB565_BLUE);
//                ips114_draw_point(left_turn_down[1],left_turn_down[0]+1,RGB565_BLUE);
//                ips114_draw_point(left_turn_down[1],left_turn_down[0]+2,RGB565_BLUE);
//                ips114_draw_point(left_turn_down[1]-1,left_turn_down[0],RGB565_BLUE);
//                ips114_draw_point(left_turn_down[1]-21,left_turn_down[0],RGB565_BLUE);
//                ips114_draw_point(left_turn_down[1]+1,left_turn_down[0],RGB565_BLUE);
//                ips114_draw_point(left_turn_down[1]+2,left_turn_down[0],RGB565_BLUE);
//            }
            return i;
}
//找左上拐点函数
uint16 find_leftup_point(uint16 start_point, uint16 end_point)
{
    uint16 j;
            for (j = start_point; j <= end_point; j++)
            {
                //如果出现了连续的行数超过40行判定为长直道，此时不存在拐点
                if (part_sum[0] >= 40)
                {
                    zuoshang_guai = 0;
                    break;
                }
                //左上拐点
                if ((leftside[j] - leftside[j +1]) > 4 && abs(leftside[j] - leftside[j - 1]) <= 3 && abs(leftside[j -1] - leftside[j-2]) <= 3
                    && leftside[j - 2] == 1 && leftside[j] == 1 && leftside[j-1] == 1&&leftside[j+1] == 0)
                {
                    left_turn_up[0] = j;
                    left_turn_up[1] = leftside[j];
                    zuoshang_guai=1;
                    break;
                }
                else{zuoshang_guai=0;}
            }
            return j;
}
//找右上拐点
uint16 find_rightup_point(uint16 start_point, uint16 end_point)
{
    uint16 j;
            for (j = start_point; j <= end_point; j++)
            {
                //如果出现了连续的行数超过40行判定为长直道，此时不存在拐点
                if (part_sum[0] >= 40) {youshang_guai=0; break;}
                //右上拐点
                if ((rightside[j+1] - rightside[j]) > 4 && abs(rightside[j] - rightside[j - 1]) <= 3 && abs(rightside[j -1] - rightside[j-2]) <= 3
                    && rightside[j - 1] == 1 && rightside[j] == 1 && rightside[j-2] == 1&& rightside[j + 1] == 0)
                {
                    rightside[0] = j;
                    rightside[1] = rightside[j];
                    youshang_guai = 1;
                    break;
                }
                else{youshang_guai=0;}
            }
            return j;
}



uint16 GetOSTU (uint8 tmImage[][MT9V03X_W])
{
    signed short i, j;
    unsigned long Amount = 0;
    unsigned long PixelBack = 0;
    unsigned long PixelshortegralBack = 0;
    unsigned long Pixelshortegral = 0;
    signed long PixelshortegralFore = 0;
    signed long PixelFore = 0;
    float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma;
    signed short MinValue, MaxValue;
    signed short Threshold = 0;
    uint16 HistoGram[256];

    for (j = 0; j < 256; j++)
        HistoGram[j] = 0;

    for (j = 0; j < MT9V03X_H; j++)
    {
        for (i = 0; i < MT9V03X_W; i++)
        {
            HistoGram[tmImage[j][i]]++;
        }
    }

    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++)
        ;
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--)
        ;

    if (MaxValue == MinValue)
        return (unsigned char) MaxValue;
    if (MinValue + 1 == MaxValue)
        return (unsigned char) MinValue;
    for (j = MinValue; j <= MaxValue; j++)
        Amount += HistoGram[j];

    Pixelshortegral = 0;
    for (j = MinValue; j <= MaxValue; j++)
    {
        Pixelshortegral += HistoGram[j] * j;
    }
    SigmaB = -1;
    for (j = MinValue; j < MaxValue; j++)
    {
        PixelBack = PixelBack + HistoGram[j];
        PixelFore = Amount - PixelBack;
        OmegaBack = (float) PixelBack / Amount;
        OmegaFore = (float) PixelFore / Amount;
        PixelshortegralBack += HistoGram[j] * j;
        PixelshortegralFore = Pixelshortegral - PixelshortegralBack;
        MicroBack = (float) PixelshortegralBack / PixelBack;
        MicroFore = (float) PixelshortegralFore / PixelFore;
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);
        if (Sigma > SigmaB)
        {
            SigmaB = Sigma;
            Threshold = j;
        }
    }
    if(Threshold < 40)  //防止断路时阈值过小
        Threshold = 40;
    return Threshold;
}

/************************************线性回归计算中线斜率************************************/
// y = Ax+B
float regression(int startline,int endline)
{

  int i=0,SumX=0,SumY=0,SumLines = 0;
  float SumUp=0,SumDown=0,avrX=0,avrY=0,B,A;
  if(startline <= 2)
      startline = 2;
  if(endline >= MT9V03X_DVP_H-1)
      endline = MT9V03X_DVP_H-1;
  SumLines=endline-startline;   // startline 为开始行， //endline 结束行 //SumLines

  for(i=startline;i<endline;i++)
  {
    SumX+=i;
    SumY+=centerline[i];    //这里centerline为存放中线的数组
  }
  avrX=(float)SumX/SumLines;     //X的平均值
  avrY=(float)SumY/SumLines;     //Y的平均值
  SumUp=0;
  SumDown=0;
  for(i=startline;i<endline;i++)
  {
    SumUp+=(centerline[i]-avrY)*(i-avrX);
    SumDown+=(i-avrX)*(i-avrX);
  }
  if(SumDown==0)
    B=0;
  else
    B=(float)(SumUp/SumDown);
    A=(SumY-B*SumX)/SumLines;  //截距
    return B;  //返回斜率
}
