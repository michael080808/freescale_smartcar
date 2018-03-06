#include "camera.h"

uint16_t H_Cnt = 0;                     //记录行中断数
uint32_t V_Cnt = 0;                     //记录场中断次数

uint8_t image = 0;                   //标定奇偶场
//image为0, 表示正在处理img1, DMA接收存储在img2
//image为1, 表示正在处理img2, DMA接收存储在img1
uint8_t img1[CAMERA_ROW][CAMERA_COL];//奇数场存储位置
uint8_t img2[CAMERA_ROW][CAMERA_COL];//偶数场存储位置
uint8_t *imgadd;                     //当前待处理场起始地址

uint8_t Lx[CAMERA_ROW];                       //左引导线中心点列号
uint8_t Rx[CAMERA_ROW];                      //右引导线中心点列号
uint8_t* L_Start;
uint8_t* L_End;
uint8_t* R_Start;
uint8_t* R_End;

//每一行的扫描偏移量
const uint8_t offset[CAMERA_ROW] = {
        40, 40, 40, 40, 40, 40, 40, 40, 40, 40,
        40, 40, 40, 40, 40, 38, 38, 38, 38, 38,
        35, 35, 35, 35, 35, 29, 28, 27, 25, 25,
        24, 24, 24, 24, 23, 23, 23, 23, 22, 22,
        22, 22, 21, 21, 21, 21, 21, 20, 20, 20,
};

void CAMERA_Init(void) {
    // 摄像头状态灯初始化
    GPIO_QuickInit(HW_GPIOE, 25, kGPIO_Mode_OPP);
    // LED1初始化，PORTE_25，没亮为奇数场处理中，亮为偶数场处理中
    GPIO_QuickInit(HW_GPIOC, 18, kGPIO_Mode_OPP);
    // LED2初始化，PORTC_18，亮表示正在处理当前图像，不亮表示没有处理图像

    //摄像头数据口，下拉
    GPIO_QuickInit(HW_GPIOC, 8, kGPIO_Mode_IPD); //初始化PORT_C,  8端口
    GPIO_QuickInit(HW_GPIOC, 9, kGPIO_Mode_IPD); //初始化PORT_C,  9端口
    GPIO_QuickInit(HW_GPIOC, 10, kGPIO_Mode_IPD); //初始化PORT_C, 10端口
    GPIO_QuickInit(HW_GPIOC, 11, kGPIO_Mode_IPD); //初始化PORT_C, 11端口
    GPIO_QuickInit(HW_GPIOC, 12, kGPIO_Mode_IPD); //初始化PORT_C, 12端口
    GPIO_QuickInit(HW_GPIOC, 13, kGPIO_Mode_IPD); //初始化PORT_C, 13端口
    GPIO_QuickInit(HW_GPIOC, 14, kGPIO_Mode_IPD); //初始化PORT_C, 14端口
    GPIO_QuickInit(HW_GPIOC, 15, kGPIO_Mode_IPD); //初始化PORT_C, 15端口

    //场中断
    GPIO_QuickInit(HW_GPIOC, 6, kGPIO_Mode_IPD);  //初始化PORT_C,  6端口
    GPIO_CallbackInstall(HW_GPIOC, CAMERA_Interrupt_Handler);     //注册中断函数,  PORT_C组触发
    GPIO_ITDMAConfig(HW_GPIOC, 6, kGPIO_IT_RisingEdge, false); //上升沿触发中断, 暂时关闭中断

    //行中断
    GPIO_QuickInit(HW_GPIOC, 7, kGPIO_Mode_IPU);  //初始化PORT_C,  7端口
    GPIO_CallbackInstall(HW_GPIOC, CAMERA_Interrupt_Handler);     //注册中断函数,  PORT_C组触发
    GPIO_ITDMAConfig(HW_GPIOC, 7, kGPIO_IT_RisingEdge, false); //上升沿触发中断, 暂时关闭中断

    //像素点中断
    GPIO_QuickInit(HW_GPIOC, 2, kGPIO_Mode_IPU);
    GPIO_ITDMAConfig(HW_GPIOC, 2, kGPIO_DMA_RisingEdge, false); //上升沿触发DMA,  暂时关闭中断

    //DMA初始化
    DMA_InitTypeDef DMA_InitStruct1 = {0};
    DMA_InitStruct1.chl = HW_DMA_CH0;                   //DMA通道号: 0
    DMA_InitStruct1.chlTriggerSource = PORTC_DMAREQ;                 //DMA触发源选择: GPIO_PORTC
    DMA_InitStruct1.triggerSourceMode = kDMA_TriggerSource_Normal;    //DMA触发方式: 正常触发
    DMA_InitStruct1.minorLoopByteCnt = 1;                            //DMA周期字节数: 1字节
    DMA_InitStruct1.majorLoopCnt = CAMERA_COL;                      //DMA循环采集数: 1像素行(152个)

    DMA_InitStruct1.sAddr = (uint32_t) &(PTC->PDIR) + 1;  //DMA来源地址: PTC8~15(PORT_C起始地址偏移一个字节)
    DMA_InitStruct1.sLastAddrAdj = 0;                            //DMA来源地址循环外偏移: 每次循环后的偏移量
    DMA_InitStruct1.sAddrOffset = 0;                            //DMA来源地址循环内偏移: 每个周期后的偏移量
    DMA_InitStruct1.sDataWidth = kDMA_DataWidthBit_8;          //DMA来源数据宽度: 8位
    DMA_InitStruct1.sMod = kDMA_ModuloDisable;           //DMA来源内存空间模数: 限制来源地址起始的内存空间循环大小范围, 用于防止越界, 不设置

    //DMA_InitStruct1.dAddr = (uint32_t)DestBuffer;                   //DMA目的地址: 暂时不设置, 由场中断处理函数设置
    DMA_InitStruct1.dLastAddrAdj = 0;                            //DMA目的地址循环外偏移: 每次循环后的偏移量
    DMA_InitStruct1.dAddrOffset = 1;                            //DMA目的地址循环内偏移: 每个周期后的偏移量
    DMA_InitStruct1.dDataWidth = kDMA_DataWidthBit_8;          //DMA目的数据宽度: 8位
    DMA_InitStruct1.dMod = kDMA_ModuloDisable;           //DMA目的内存空间模数: 限制目的地址起始的内存空间循环大小范围, 用于防止越界, 不设置

    DMA_Init(&DMA_InitStruct1);                                       //应用DMA通道设置
    DMA_DisableRequest(HW_DMA_CH0);                                   //先关闭DMA传输, 在中断处理函数中处理开关

    GPIO_ITDMAConfig(HW_GPIOC, 6, kGPIO_IT_RisingEdge, true);        //上升沿触发中断, 开启中断
    GPIO_ITDMAConfig(HW_GPIOC, 7, kGPIO_IT_RisingEdge, true);        //上升沿触发中断, 开启中断
    GPIO_ITDMAConfig(HW_GPIOC, 2, kGPIO_DMA_RisingEdge, true);        //上升沿触发DMA, 开启中断

    //配置摄像头寄存器
    uint8_t i = 0;
    GPIO_QuickInit(HW_GPIOC, 0, kGPIO_Mode_OPP);
    GPIO_QuickInit(HW_GPIOC, 3, kGPIO_Mode_OPP);
    while (i == 0)
        i += LPLD_SCCB_WriteReg(0x14, 0x24); //QVGA(320*120)
    while (i == 1)
        i += LPLD_SCCB_WriteReg(0x24, 0x20); //连续采集模式(320*240)
    while (i == 2)
        i += LPLD_SCCB_WriteReg(0x70, 0xc1); //驱动电流增加一倍
    while (i == 3)
        i += LPLD_SCCB_WriteReg(0x06, 0xa0); //亮度控制
}

void CAMERA_Processing(void) {
    int16_t lp1, lp2; //第0行时，lp1,lp2从上一次的线的第零行开始扫描
    int16_t lp3, lp4;
    uint8_t CurL = 0;
    //指向当前的处理的行
    uint8_t LastL = 0;            //上一行
    uint8_t LastR = 0;
    uint8_t isLeftEdge = 0;       //是否看到左右边界
    uint8_t isRightEdge = 0;      //是否看到左右边界
    uint8_t *p;
    uint8_t BlackcntL = 0;      //记录出现几个黑点
    uint8_t BlackcntR = 0;
    uint8_t LRightEdge, LLeftEdge;
    uint8_t i = 0, j = 0;
    uint8_t xx = 0;

    PCout(18) = 1;

    if (Lx[L_Start[0]] != CAMERA_COL && Lx[L_Start[0]] != 0 && L_Start[0] < 15)     //前十五行有非空左线起点
    {
        lp1 = (Lx[L_Start[0]] > 1 + offset[L_Start[0]]) ? Lx[L_Start[0]] - offset[L_Start[0]] : 1;
        lp2 = lp1 + P_WIDTH;
    } else  //前一半都没有找到参考值
    {
        if (Rx[R_Start[0]] != 0 && R_Start[0] < 15) {
            lp1 = ((int16_t)((int16_t) Rx[R_Start[0]] + (int16_t) offset[R_Start[0]]) > CAMERA_COL - P_WIDTH - 10) ?
                  CAMERA_COL - P_WIDTH - 10 : Rx[R_Start[0]] + offset[R_Start[0]];
            lp2 = lp1 + P_WIDTH;
        } else {
            lp1 = CAMERA_CENTER;
            lp2 = lp1 + P_WIDTH;
        }
    }
    if (Rx[R_Start[0]] != 0 && R_Start[0] < 15) {
        lp3 = ((int16_t)((int16_t) Rx[R_Start[0]] + (int16_t) offset[R_Start[0]]) < CAMERA_COL - P_WIDTH - 5) ?
              Rx[R_Start[0]] + offset[R_Start[0]] : CAMERA_COL - P_WIDTH - 5;
        lp4 = lp3 + P_WIDTH;
    } else  //如果前面一直没有找到边，那么就依照找到的左线的位置推测lp1,lp2位置
    {
        if (Lx[L_Start[0]] != CAMERA_COL && L_Start[0] < 15) {
            lp3 = (Lx[L_Start[0]] > 5 + offset[L_Start[0]]) ? Lx[L_Start[0]] - offset[L_Start[0]] : 5;
            lp4 = lp3 + P_WIDTH;
        } else {
            lp3 = CAMERA_CENTER;
            lp4 = lp3 + P_WIDTH;
        }
    }
    for (CurL = 0; CurL < CAMERA_ROW; CurL++) {
        p = imgadd + CurL * CAMERA_COL;                                                            //指向当前行
        if (CurL > 0) {                                                                        //确定左右线扫描起点
            LastL = CurL - 1;                                                           //Lx[]=CAMERA_COL表示没有找到左黑线
            while (LastL > 0 && CurL - LastL < 5 && Lx[LastL] == CAMERA_COL)LastL--;
            if (Lx[LastL] != CAMERA_COL)                                                   //在本行前五行找到上一个线中心
            {
                if (Lx[LastL] > offset[LastL]) {
                    if (Rx[CurL - 1] != 0 &&
                        Lx[LastL] - offset[LastL] < Rx[CurL - 1] + 5)            //如果扫线开始点在上一行右线的右边 则以上一行的右线作为扫线开始
                        lp1 = Rx[CurL - 1] + 5;
                    else {
                        lp1 = Lx[LastL] - (offset[LastL] / 3);                                        //需修改
                    }
                } else {
                    if (Rx[CurL - 1] != 0 && Rx[CurL - 1] + 5 < CAMERA_COL - P_WIDTH)
                        lp1 = Rx[CurL - 1] + 5;
                    else
                        lp1 = 1;
                }
                lp2 = lp1 + P_WIDTH;
            } else                                             //前面一直无可搜寻到的左线
            {
                if (Rx[CurL - 1] != 0)                  //那么如果第零行是用右线确定的，则用右线的推出来值
                {
                    lp1 = ((int16_t)((int16_t) Rx[CurL - 1] + 5) > CAMERA_COL - P_WIDTH - 10) ? CAMERA_COL - P_WIDTH - 10 :
                          Rx[CurL - 1] + 5;
                    lp2 = lp1 + P_WIDTH;
                } else {
                    lp1 = CAMERA_CENTER;
                    lp2 = lp1 + P_WIDTH;
                }
            }
            //确定方向右线扫描起点
            LastR = CurL - 1;
            while (LastR > 0 && CurL - LastR < 5 && Rx[LastR] == 0)LastR--;     //LastL中记录第一个找到的有效左黑线中心所在行数
            if (Rx[LastR] != 0)                                             //找到上一个右线中心
            {
                if ((int16_t)((int16_t) Rx[LastR] + (int16_t) offset[LastR]) < CAMERA_COL - P_WIDTH - 5) {
                    if (Lx[CurL - 1] != CAMERA_COL && (int16_t)(Rx[LastR] + offset[LastR]) > (int16_t)(
                            Lx[CurL - 1] - P_WIDTH - 5))              //如果扫线开始点在上一行左线的左边 则以上一行的左线作为扫线开始
                        lp3 = Lx[CurL - 1] - P_WIDTH - 5;
                    else {
                        lp3 = Rx[LastR];                                        //需修改
                    }
                } else {
                    if (Lx[CurL - 1] == CAMERA_COL) {
                        lp3 = CAMERA_COL - P_WIDTH - 5;
                    } else if (Lx[CurL - 1] > 5 + P_WIDTH)lp3 = Lx[CurL - 1] - P_WIDTH - 5;
                    else lp3 = CAMERA_CENTER;
                }
                lp4 = lp3 + P_WIDTH;
            } else //之前一直没有找到右线
            {
                if (Lx[CurL - 1] != CAMERA_COL)//按照本行左线的位置 确定lp1 lp2
                {
                    lp3 = ((int16_t)(Lx[CurL - 1] > 5 + P_WIDTH + 5)) ? Lx[CurL - 1] - P_WIDTH - 5 : 5;
                    lp4 = lp3 + P_WIDTH;
                } else { //本行左线没有
                    lp3 = CAMERA_CENTER;
                    lp4 = lp3 + P_WIDTH;
                }
            }
            //左右线扫线开始
        }
        if (lp3 > 0) //找左右边界
        {
            //------找线右边界-------------------
            while (lp3 > 0 && !isRightEdge) {
                if ((int16_t)(*(p + lp4)) > BW_DELTA + (int16_t) * (p + lp3))     //利用有符号的来消除噪点
                {
                    while ((int16_t)(*(p + lp4)) > BW_DELTA + (int16_t) * (p + lp3) && lp3 > 0) {
                        if ((int16_t)(*(p + lp4)) < 255)BlackcntR++;
                        lp3--;
                        lp4--;
                        if (BlackcntR >= LINE_EDGE)break;
                    }
                    if (BlackcntR >= LINE_EDGE) //判断找到黑线
                    {
                        LRightEdge = lp3 + LINE_EDGE;
                        BlackcntR = 0;

                        xx = 0;
                        for (i = 0; i < 10; i++) {
                            if (*(p + lp3 + LINE_EDGE + i) > THRESHOLD)xx++;
                        }
                        if (xx > 6)
                            isRightEdge = 1;
                        else
                            BlackcntR = 0;
                    } else {
                        BlackcntR = 0;
                    }
                } else {
                    lp3--;
                    lp4--;
                }
            }
        }
        if (lp2 < CAMERA_COL)                    //找左右边界,从内向外扫描
        {
            //------找线左边界-------------------
            while (lp2 < CAMERA_COL && !isLeftEdge)        //当lp2没有到达列的最大值继续扫描，右扫描模式
            {
                if (((int16_t)(*(p + lp1))) > BW_DELTA + (int16_t) * (p + lp2))           //
                {
                    while ((int16_t)(*(p + lp1)) > BW_DELTA + (int16_t) * (p + lp2) && lp2 < CAMERA_COL) {
                        if ((int16_t)(*(p + lp1)) < 255)BlackcntL++;
                        lp1++;
                        lp2++;
                        if (BlackcntL >= LINE_EDGE)break;
                    }
                    if (BlackcntL >= LINE_EDGE)           //找到左边界退出循环，lp1和lp2间隔设为1,
                    {
                        LLeftEdge = lp2 - LINE_EDGE;
                        BlackcntL = 0;

                        xx = 0;
                        for (j = 0; j < 10; j++) {
                            if (*(p + lp2 - LINE_EDGE - j) > THRESHOLD)xx++;
                        }
                        if (xx > 6)
                            isLeftEdge = 1;
                        else
                            BlackcntL = 0;
                    } else                  //遇到噪点，计数清零
                        BlackcntL = 0;
                } else {
                    lp1++;
                    lp2++;
                }
            }
        }
        if (isLeftEdge) {
            isLeftEdge = 0;
            Lx[CurL] = LLeftEdge;
        } else Lx[CurL] = CAMERA_COL;
        if (isRightEdge) {
            isRightEdge = 0;
            Rx[CurL] = LRightEdge;
        } else Rx[CurL] = 0;
    }

    PCout(18) = 0; // 结束处理，LED关闭
}

void CAMERA_Display_Full(void) {
    int i, j;
    // 将字节逐个写入并映射到OLED上
    for (i = 0; i < CAMERA_ROW; i++)
        for (j = 0; j < CAMERA_COL; j++)
            OLED_DrawPoint(j * OLED_COL / CAMERA_COL, i, img1[i][j] > THRESHOLD);
    // 刷新OLED以显示图像
    OLED_Refresh_Gram();
}

void CAMERA_Display_Edge(void) {
    int i;
    // 清空OLED显示
    OLED_Fill(0, 0, 128, 50, 0);
    // 将字节逐个写入并映射到OLED上
    for (i = 0; i < CAMERA_ROW; i++) {
        if (Lx[i] > 0x00 && Lx[i] < CAMERA_COL)
            OLED_DrawPoint(Lx[i] * OLED_COL / CAMERA_COL, i, 1); // 左边界
        if (Rx[i] > 0x00 && Rx[i] < CAMERA_COL)
            OLED_DrawPoint(Rx[i] * OLED_COL / CAMERA_COL, i, 1); // 右边界
    }
    // 刷新OLED以显示图像
    OLED_Refresh_Gram();
}

void CAMERA_UART_TX_Full(const uint32_t instance) {
    int i, j;

    // 符合山外多功能调试助手，起始帧为0x01 0xFE
    UART_WriteByte(instance, 0x01);
    UART_WriteByte(instance, 0xFE);

    // 逐个图像字节连续发送
    for (i = 0; i < CAMERA_ROW; i++)
        for (j = 0; j < CAMERA_COL; j++)
            UART_WriteByte(instance, img1[i][j]);

    // 符合山外多功能调试助手，结束帧为0xFE 0x01
    UART_WriteByte(instance, 0xFE);
    UART_WriteByte(instance, 0x01);
}

void CAMERA_UART_TX_Edge(const uint32_t instance) {
    int i, j;

    // 符合山外多功能调试助手，起始帧为0x01 0xFE
    UART_WriteByte(instance, 0x01);
    UART_WriteByte(instance, 0xFE);

    // 逐个图像字节连续发送
    for (i = 0; i < CAMERA_ROW; i++)
        for (j = 0; j < CAMERA_COL; j++)
            if (j > 0x00 && j == Lx[i] || j < CAMERA_COL && j == Rx[i])
                UART_WriteByte(instance, 0xFF);
            else
                UART_WriteByte(instance, 0x00);

    // 符合山外多功能调试助手，结束帧为0xFE 0x01
    UART_WriteByte(instance, 0xFE);
    UART_WriteByte(instance, 0x01);
}
