/************************************
 * 729_CODE
 * 729_camera.h
 * version 1
 ************************************/

#ifndef _729_CAMERA_H
#define _729_CAMERA_H

#define uint16_t uint16
#define uint8_t uint8
#define MT9V03X_DVP_W             MT9V03X_W                 //图像宽度  范围1-188
#define MT9V03X_DVP_H             MT9V03X_H
//图像种类枚举
typedef enum
{
    none = 0,               //无类型   0

    STR_W,                  //直道     1
    CUR_W,                  //弯道（好像没用到）  2

    CRS_W_S1,               //十字阶段1（入十字）  3
    CRS_W_S1_1,             //左斜入十字   4
    CRS_W_S1_2,             //右斜入十字   5
    CRS_W_S2,               //十字阶段2（发现十字）  6
    CRS_W_S3,                //十字阶段3 拐      7

    FOK_W_S1,               //三岔阶段1（入三岔，可以看到三岔下面的直道）  8
    FOK_W_S2,               //三岔阶段2（看不到直道了） 9
    FOK_W_S3,               //三岔阶段3（开始转弯） 10

    //电感识别环岛阶段
    CIR_W_ADC_1,              //电感识别环岛阶段1，不判断左右环岛 11

    //左环
    LCIR_W_S1,              //环岛阶段1（初入左环岛，看到了直道后左边界缺失了但有边界是直线，可能会误判，这个也是判断入库的阶段1）  12
    LCIR_W_S2,              //环岛阶段2（环岛确认，发现S1阶段后出现了最下面一段是左边界没有了）  13
    LCIR_W_S2_5,            //环岛阶段2.5（S3的准备阶段，看到双边界都有时进入，说明在环岛直道的中间）  14
    LCIR_W_S3,              //环岛阶段3（入环阶段，寻找特殊点，补线入环）  15
    LCIR_W_S4,              //环岛阶段4（环内阶段）  16
    LCIR_W_S5,              //环岛阶段5（出环阶段，发现边界出现了类型为3或6的段）   17
    LCIR_W_S6,              //环岛阶段6（出环结束，在圆环的直道上，通过陀螺仪判断）   18
    LCIR_W_S7,              //环岛阶段7（准备彻底驶出环岛，再次走到了S3阶段的位置，但判断条件是，第二段为正常边界）   19

    //右环
    RCIR_W_S1,              //环岛阶段1      20
    RCIR_W_S2,              //环岛阶段2      21
    RCIR_W_S2_5,            //环岛阶段2.5    22
    RCIR_W_S3,              //环岛阶段3      23
    RCIR_W_S4,              //环岛阶段4      24
    RCIR_W_S5,              //环岛阶段5      25
    RCIR_W_S6,              //环岛阶段6      26
    RCIR_W_S7,              //环岛阶段7      27

    //左入库
    LGARA_W_S1,             //入库阶段1（看见斑马线)   28
    LGARA_W_S2,             //入库阶段2  (图像地全白)   29
    LGARA_W_S3,             //入库阶段3 (已经入库)     30

    //右入库
    RGARA_W_S1,             //入库阶段1（看见斑马线）    31
    RGARA_W_S2,             //入库阶段2  (斑马线到下端)   32
    RGARA_W_S3,             //入库阶段3 (已经入库)       33

    //障碍
    BARR_W_S1,              //障碍阶段，发现障碍   34
    BARR_W_S2,              //障碍阶段(距离变大)   35
    BARR_W_S3,              //障碍阶段，刚过障碍   36
    BARR_W_S4,              //障碍阶段，回正       37

    //断路
    OPEN_W_S1,              //断路阶段1   38
    OPEN_W_S2,              //惯导使用阶段 39

    //坡道
    RAMP_S1,                //上坡
    RAMP_S2,                //下坡
} image_type;

//从图像中提取出的信息结构体
typedef struct
{
    image_type imgtype;     //图像类型
    float offset;           //位置偏差
} image_parameter;

typedef struct image {
    uint8 *data;
    uint32 width;
    uint32 height;
    uint32 step;
} image_t;
#define DEF_IMAGE(ptr, w, h)         {.data=ptr, .width=w-1, .height=h, .step=w}



//需要修改的值
#define CIR_DE      320// 23000           //陀螺仪转一周的变化量，环岛s5到s6
#define CIR_DE_S5   200//200            //陀螺仪转一定角度，环岛s4到s5
#define GAR_ED_OUT  70//48000            //出入库转90度陀螺仪积分值
#define CRS_DE      70              // 陀螺仪转90的变化量，判断出十字
#define CIR_D_S3_EN 1500            //环岛s3编码器跳变
#define OFFSET_S3   18              //环岛s3最小误差
#define OFFSET_S4   15              //环岛s4最小误差
#define CRS_S2      15              //十字无效行数

#define WHITE_TH 130                //白色像素判断阈值，比赛赛道阈值150，实验室145,边缘比较暗
#define SAR_TH 60                   //差和比阈值 50

#define SAR_DW 6                    //差和比宽度比较步长，差和比的两个点的距离，行方向
#define SAR_DH 5                    //差和比高度比较步长，列方向
#define ROW_DW 3                    //行边界计算步长，没隔ROW_DW个点计算一次差和比，为了加快边界提取的速度可以取大一点，但不能超过SAR_DW，否则会出现找不到边界的误判情况
#define C_TH 7                      //分段连续性阈值，相同的行类型连续出现C_TH次才可认为是一段，用于滤波，排除单行错误
#define WIDTH_MIN 130               //初始最小赛道宽度

#define CHANGE_TH 12                //入库斑马线像素跳变数量
#define GARA_ENC  12000             //入库编码器积分值
#define GO_ENC    8500              //出库编码器积分
#define GO_STR_ENC    5000          //出库直走编码器积分
#define GO_END_ENC    8000          //出库结束编码器积分

#define CRS_OFFSET_TH 7            //十字偏差阈值，十字路口要直行，偏差不能太大

#define CUR_CIRTH 0.985             //曲率阈值，用来判断是线边界还是曲线，需要实测一下

#define END_I 2                    //图像有效段结束I值

//边界分段判断
#define SUD_TH 18                   //宽度突变阈值
#define SSUD_TH 8                   //单边突变阈值，当dl或dr超过时认为是发生突变了
#define NSUD_TH 3                   //非突变特殊阈值，认为在这个范围内是正常波动

#define DIR_OUT  1                  //出库方向，-1向左出库，1向右出库

#define CIR_ADC1_TH 2000            //左电感环岛阈值
#define CIR_ADC2_TH 3700            //中电感环岛阈值
#define CIR_ADC3_TH 2000            //右电感环岛阈值
#define CIR_MIX_ADC_TH  133         //电感环岛最大值阈值
#define CIR_MIN_ADC_TH  103         //电感环岛第二大值阈值

#define BARR_TH       850           //路障距离阈值
#define BARR_DE_TH    10             //路障2陀螺仪积分阈值,用于2接着拐,直到z轴大于跳变值，加上这个值,速度慢：12，速度快：
#define BARR_DE4_TH   30            //路障陀螺仪积分阈值,用于3到4
#define BARR2_ENC_TH      26000     //路障2编码器积分阈值,用于判断到3
#define BARR3_ENC_TH      45000     //路障3编码器积分阈值,用于出障碍

#define OPEN_ENC_TH       1500     //断路刚开始直走编码器  /100
#define OPEN_END_ENC_TH   10000    //断路结束减速编码器积分
#define OPEN_S12_ENC      2200     //普通惯导结束编码器积分 /1024

extern int16 leftside[MT9V03X_DVP_H];
extern int16 rightside[MT9V03X_DVP_H];
extern int16 centerline[MT9V03X_DVP_H];
extern int16 roweff_flag[MT9V03X_DVP_H];
extern int16 row_width[MT9V03X_DVP_H];
extern image_parameter image_para;
extern int16 img_mode;
extern int16 ls_std[MT9V03X_DVP_H];
extern int16 rs_std[MT9V03X_DVP_H];
extern int16 cirsud_i;
extern int16 upside;
extern uint16 adc_result[3];
extern int16 open_in_start;
extern int16 gara_turn;
extern uint16 cen_black;
extern float offest_K;
extern int16 open_t;
extern int32 Open_Encode;
extern int16 go_strat;
extern uint16 undo;
extern int16 ave_distain;
extern int16 go_turn;
extern int16 flag_end;
extern int32 Open_End_Encode;
extern float offset_S5;
extern float weight_S5;
extern float OPpen_Encode;
extern float s3_encode;
extern int32 Go_Encode;
extern uint16 cir_num ;
extern int16 crs_turn;
extern int16 part_num;
extern uint16 flag_cen_false;
extern uint16 right_cos_ave;
extern uint16 left_cos_ave ;

int16 image_process_init(void);
void image_boundary_process(void);
void row_boundary_process(int16 i, int16 init_lj, int16 init_rj, int16 mode);
void row_centerline_process(void);
void image_para_process(void);
float offset_process(void);
void image_distinguish(void);
uint16 count_white(uint16 i,uint16 start,uint16 end);
float abs_f(float x);
uint16 abs_u16(uint16 x);
uint16 Num_jump(uint16 line,uint16 start_i,uint16 end_i);
uint16 Num_slop_jump(void);
uint8 Garage_Judge(int16 dir);
uint16 circle_max(void);
void boundary_part(void);
void rowtype_process(void);
float angle_caculate(float x1,float y1,float x2,float y2,float x3,float y3);
float cur_circle(int16 m);
void cur_line_cos(void);
uint16 get_max_num(uint16 a[],uint16 len,uint16 num);
uint16 find_rightdown_point(uint16 start_point, uint16 end_point);
uint16 find_leftdown_point(uint16 start_point, uint16 end_point);
uint16 find_leftup_point(uint16 start_point, uint16 end_point);
uint16 find_rightup_point(uint16 start_point, uint16 end_point);
uint16 GetOSTU (uint8 tmImage[][MT9V03X_W]);
float regression(int startline,int endline);


extern int16 CURSE_OFFSET_1;
extern int16 UPSIDE_M;
extern int16 UPSIDE_S;

#endif
