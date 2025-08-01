

/**
 * main.c
 */

#include "DSP28x_Project.h"
#include "parameter.h"
#include "SysVariable.h"
#include "ProtectRelay.h"
#include "BAT_LTC6802.h"
#include "BATAlgorithm.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

/*
 *
 */
void InitGpio(void);

void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr);

/*
 *
 */
void InitECanaGpio(void);
void InitECana(void);
void CANATX(unsigned int ID, unsigned char Length, unsigned int Data0, unsigned int Data1,unsigned int Data2,unsigned int Data3);

/*
 *
 */
void SysTimerINIT(SystemReg *s);
void SysVarINIT(SystemReg *s);
void CANRegVarINIT(CANAReg *P);
void SysDigitalInput(SystemReg *sys);
void SysDigitalOutput(SystemReg *sys);
void MDCalInit(SystemReg *P);
/*
 *
 */

//void ProtectRlySateCheck(PrtectRelayReg *P);

//void ProtectRlyOnInit(PrtectRelayReg *P);
//void ProtectRlyOnHandle(PrtectRelayReg *P);
//void ProtectRlyOffInit(PrtectRelayReg *P);
//void ProtectRlyOffHandle(PrtectRelayReg *P);
//void ProtectRlyEMSHandle(PrtectRelayReg *P);

void ProtectRlyVarINIT(PrtectRelayReg *P);
void ProtectRlyHandle(PrtectRelayReg *P);

/*
 *
 */
void SysCalVoltageHandle(SystemReg *s);
void SysCalCurrentHandle(SystemReg *s);
void SysCalTemperatureHandle(SystemReg *s);
void MDCalVoltandTemsHandle(SystemReg *P);
void CalFarasis52AhRegsInit(SocReg *P);
void CalFarasis52AhSocInit(SocReg *P);
void CalFarasis52AhSocHandle(SocReg *P);
void SysFaultCheck(SystemReg *s);
void SysAlarmtCheck(SystemReg *s);
void SysProtectCheck(SystemReg *s);
/*
 *
 */


//void CalFrey60AhRegsInit(SocReg *P);
//void CalFrey60AhSocInit(SocReg *P);
//void CalFrey60AhSocHandle(SocReg *P);




/*
 *
 */
int float32ToInt(float32 Vaule, Uint32 Num);
/*
 *
 */

/*
 *
 */
void SPI_Write(unsigned int WRData);
unsigned int SPI_Read(void);
//void BAT_InitSPI(void);
void SPI_BATWrite(unsigned int WRData);
/*
 *
 */
int LTC6804_read_cmd(char address, short command, char data[], int len);
int LTC6804_write_cmd(char address, short command, char data[], int len);
void init_PEC15_Table(void);
unsigned short pec15(char *data, int len);
int SlaveBMSIint(SlaveReg *s);
int SlaveBmsBalance(SlaveReg *s);
void SlaveVoltagHandler(SlaveReg *s);
void SlaveVoltagBalaHandler(SlaveReg *s);
void SlaveBMSDigiteldoutOHandler(SlaveReg *P);
void SalveTempsHandler(SlaveReg *s);
void TempTemps(SystemReg *s);
/*
 *  인터럽트 함수 선언
 */
interrupt void cpu_timer0_isr(void);
interrupt void ISR_CANRXINTA(void);
//interrupt void cpu_timer2_isr(void);

SystemReg       SysRegs;
float32 randomCT=0;
float32 randomA=0;
float32 randomC=0;
PrtectRelayReg  PrtectRelayRegs;
SlaveReg        Slave0Regs;
SlaveReg        Slave1Regs;
SlaveReg        Slave2Regs;
SlaveReg        Slave3Regs;
SlaveReg        Slave4Regs;
SlaveReg        Slave5Regs;
SlaveReg        Slave6Regs;
SlaveReg        Slave7Regs;
SlaveReg        Slave8Regs;
SlaveReg        Slave9Regs;
SlaveReg        Slave10Regs;
SlaveReg        Slave11Regs;
SlaveReg        Slave12Regs;
SlaveReg        Slave13Regs;
SlaveReg        Slave14Regs;
SlaveReg        Slave15Regs;
CANAReg         CANARegs;
SocReg          Farasis52AhSocRegs;
//SocReg          Frey60AhSocRegs;
float32         NCMsocTestVoltAGV =3.210;
float32         NUMsocTestVCT =0.0;
float32         LFPsocTestVoltAGV =3.050;
float32         LFPsocTestVCT =0.0;
unsigned int    ProtectRelayCyle=0;
//extern unsigned int    CellVoltUnBalaneFaulCount=0;

void main(void)
{
//    struct ECAN_REGS ECanaShadow;
    InitSysCtrl();
    /*
     * To check the clock status of the C2000 in operation
     */
  //  GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 3; //enable XCLOCKOUT through GPIO mux
  //  SysCtrlRegs.XCLK.bit.XCLKOUTDIV = 0; //XCLOCKOUT = 1/2* SYSCLK

// Step 2. Initalize GPIO:
// This example function is found in the DSP2803x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// For this example use the following configuration:
// Step 3. Clear all interrupts and initialize PIE vector table:
    DINT;
// Initialize PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the DSP2803x_PieCtrl.c file.
    InitPieCtrl();
// Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();
    EALLOW;  // This is needed to write to EALLOW protected registers

    /*
     *  인터럽트 함수 선언
     */
    PieVectTable.TINT0 = &cpu_timer0_isr;
    PieVectTable.ECAN0INTA  = &ISR_CANRXINTA;
//    PieVectTable.TINT2 = &cpu_timer2_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers
    InitGpio();
    //GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 3; //enable XCLOCKOUT through GPIO mux
    //SysCtrlRegs.XCLK.bit.XCLKOUTDIV = 2; //XCLOCKOUT = SYSCLK
    InitSpiGpio();
    InitSpi();
    InitECanGpio();
    InitECan();

    MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
    InitFlash();

    ConfigCpuTimer(&CpuTimer0, 80, 1000);
    //CpuTimer0Regs.PRD.all = 80000;// 90000 is 1msec
    CpuTimer0Regs.PRD.all = 80400;// 90000 is 1msec
    //   ConfigCpuTimer(&CpuTimer1, 80, 1000000);
    //   ConfigCpuTimer(&CpuTimer2, 80, 1000000);
    CpuTimer0Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0
    //  CpuTimer1Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0
    //  CpuTimer2Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0
//    InitAdc();
//    AdcOffsetSelfCal();
    EALLOW;
    EDIS;    // This is needed to disable write to EALLOW protected registers
    IER |= M_INT1;
    IER |= M_INT13;
    IER |= M_INT14;
    IER |= M_INT9;//test
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;      // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER9.bit.INTx5 = 1;      // Enable ECAN-A interrupt of PIE group 9
//  PieCtrlRegs.PIEIER9.bit.INTx1 = 1;      // SCIA RX interrupt of PIE group
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM
    SysRegs.SysMachine = System_STATE_INIT;
    PrtectRelayRegs.StateMachine= PrtctRly_INIT;

    while(1)
    {

        SysRegs.Maincount++;
        switch(SysRegs.SysMachine)
        {
            case System_STATE_INIT:
                 SysRegs.SysDigitalOutPutReg.bit.LEDAlarmOUT=0;
                 SysRegs.SysDigitalOutPutReg.bit.LEDFaultOUT=0;
                 SysRegs.SysDigitalOutPutReg.bit.LEDProtectOUT=0;
                 SysRegs.SysStateReg.bit.SysBalanceMode=0;
                 Farasis52AhSocRegs.SoCStateRegs.bit.CalMeth=0;
                 SysTimerINIT(&SysRegs);
                 SysVarINIT(&SysRegs);
                 CANRegVarINIT(&CANARegs);
                 ProtectRlyVarINIT(&PrtectRelayRegs);
                 MDCalInit(&SysRegs);
                 CalFarasis52AhRegsInit(&Farasis52AhSocRegs);

                 Slave0Regs.ID=BMS_ID_0;
                 Slave0Regs.SlaveCh=C_Slave_ACh;
                 SlaveBMSIint(&Slave0Regs);

                 Slave1Regs.ID=BMS_ID_1;
                 Slave1Regs.SlaveCh=C_Slave_ACh;
                 SlaveBMSIint(&Slave1Regs);

                 Slave2Regs.ID=BMS_ID_2;
                 Slave2Regs.SlaveCh=C_Slave_ACh;
                 SlaveBMSIint(&Slave2Regs);

                 Slave3Regs.ID=BMS_ID_3;
                 Slave3Regs.SlaveCh=C_Slave_ACh;
                 SlaveBMSIint(&Slave3Regs);

                 Slave4Regs.ID=BMS_ID_4;
                 Slave4Regs.SlaveCh=C_Slave_ACh;
                 SlaveBMSIint(&Slave4Regs);

                 Slave5Regs.ID=BMS_ID_5;
                 Slave5Regs.SlaveCh=C_Slave_ACh;
                 SlaveBMSIint(&Slave5Regs);

                 Slave6Regs.ID=BMS_ID_6;
                 Slave6Regs.SlaveCh=C_Slave_ACh;
                 SlaveBMSIint(&Slave6Regs);

                 Slave7Regs.ID=BMS_ID_7;
                 Slave7Regs.SlaveCh=C_Slave_ACh;
                 SlaveBMSIint(&Slave7Regs);

                 Slave8Regs.ID=BMS_ID_0;
                 Slave8Regs.SlaveCh=C_Slave_BCh;
                 SlaveBMSIint(&Slave8Regs);

                 Slave9Regs.ID=BMS_ID_1;
                 Slave9Regs.SlaveCh=C_Slave_BCh;
                 SlaveBMSIint(&Slave9Regs);

                 Slave10Regs.ID=BMS_ID_2;
                 Slave10Regs.SlaveCh=C_Slave_BCh;
                 SlaveBMSIint(&Slave10Regs);

                 Slave11Regs.ID=BMS_ID_3;
                 Slave11Regs.SlaveCh=C_Slave_BCh;
                 SlaveBMSIint(&Slave11Regs);

                 Slave12Regs.ID=BMS_ID_4;
                 Slave12Regs.SlaveCh=C_Slave_BCh;
                 SlaveBMSIint(&Slave12Regs);

                 Slave13Regs.ID=BMS_ID_5;
                 Slave13Regs.SlaveCh=C_Slave_BCh;
                 SlaveBMSIint(&Slave13Regs);

                 Slave14Regs.ID=BMS_ID_6;
                 Slave14Regs.SlaveCh=C_Slave_BCh;
                 SlaveBMSIint(&Slave14Regs);

                 Slave15Regs.ID=BMS_ID_7;
                 Slave15Regs.SlaveCh=C_Slave_BCh;
                 SlaveBMSIint(&Slave15Regs);

                 SysRegs.SysStateReg.bit.SysProtect=0;
                 SysRegs.SysStateReg.bit.SysStatus=0;
                 SysRegs.SysMachine=System_STATE_STANDBY;
            break;
            case System_STATE_STANDBY:// INIT
                 SysRegs.SysStateReg.bit.SysStatus =0;
                 if(SysRegs.SysStateReg.bit.INITOK==1)
                 {
                     Farasis52AhSocRegs.CellAgvVoltageF=SysRegs.SysCellAgvVoltageF;
                     CalFarasis52AhSocInit(&Farasis52AhSocRegs);
                     SysRegs.SysMachine=System_STATE_READY;
                     Farasis52AhSocRegs.state=SOC_STATE_RUNNING;
                     Farasis52AhSocRegs.SoCStateRegs.bit.CalMeth=0;
                 }

            break;
            case System_STATE_READY:
                 SysRegs.SysStateReg.bit.SysStatus =1;
                 if(SysRegs.SysStateReg.bit.SysProtect==0)
                 {
                     if(SysRegs.SysStateReg.bit.HMICOMEnable ==0)
                     {
                         SysRegs.SysStateReg.bit.SysWakeUpEn = CANARegs.PMSCMDRegs.bit.RUNStatus;
                         CANARegs.HMICMDRegs.bit.HMI_RlyEN =0;
                     }
                     if(SysRegs.SysStateReg.bit.HMICOMEnable==1)
                     {
                         SysRegs.SysStateReg.bit.SysWakeUpEn = CANARegs.HMICMDRegs.bit.HMI_RlyEN;
                         CANARegs.PMSCMDRegs.bit.RUNStatus =0;
                     }
                 }
                 if(PrtectRelayRegs.State.bit.WakeuPOnEND==1)
                 {
                     SysRegs.SysMachine=System_STATE_RUNING;
                 }

            break;
            case System_STATE_RUNING:
                 SysRegs.SysStateReg.bit.SysStatus =2;
                 if(SysRegs.SysStateReg.bit.SysProtect==0)
                 {
                    if(SysRegs.SysStateReg.bit.HMICOMEnable ==0)
                    {
                        SysRegs.SysStateReg.bit.SysWakeUpEn = CANARegs.PMSCMDRegs.bit.RUNStatus;
                        CANARegs.HMICMDRegs.bit.HMI_RlyEN =0;
                    }
                    if(SysRegs.SysStateReg.bit.HMICOMEnable==1)
                    {
                        SysRegs.SysStateReg.bit.SysWakeUpEn = CANARegs.HMICMDRegs.bit.HMI_RlyEN;
                        CANARegs.PMSCMDRegs.bit.RUNStatus =0;
                    }
                 }
                 if(PrtectRelayRegs.State.bit.WakeuPOffEND==1)
                 {
                    SysRegs.SysMachine=System_STATE_READY;
                 }


            break;
            case System_STATE_PROTECTER:
                 SysRegs.SysStateReg.bit.SysStatus =4;
                 SysRegs.SysDigitalOutPutReg.bit.LEDAlarmOUT=0;
                 SysRegs.SysDigitalOutPutReg.bit.LEDFaultOUT=1;
                 PrtectRelayRegs.StateMachine=PrtctRly_ProtectpOFF;

            break;
            case System_STATE_CLEAR:

            break;
            default :
            break;
        }
        if(SysRegs.CellVoltsampling>=CellVoltSampleTime)
        {
            /*
             *
             */
            //Balance 위한 전류 조건
            if(SysRegs.SysPackCurrentAsbF<=C_PackBalanCurrent)
            {
                SysRegs.BalanceModeCount++;
                if(SysRegs.BalanceModeCount>=100)
                {
                    SysRegs.BalanceModeCount=101;
                    SysRegs.SysStateReg.bit.SysBalanceMode=1;
                }
            }
            else if(SysRegs.SysPackCurrentAsbF>C_PackBalanCurrent)
            {
                SysRegs.SysStateReg.bit.SysBalanceMode=0;
                SysRegs.SysStateReg.bit.SysBalanceEn=0;
                SysRegs.BalanceModeCount=0;
                SysRegs.BalanceTimeCount=0;
            }
            //Balance 위한 셀 최조 전압 조건
            if(SysRegs.SysCellMinVoltageF<C_CellBalanLimtVolt)
            {
                SysRegs.SysStateReg.bit.SysBalanceMode=0;
                SysRegs.SysStateReg.bit.SysBalanceEn=0;
                SysRegs.BalanceModeCount=0;
                SysRegs.BalanceTimeCount=0;
            }
           // SysRegs.SysStateReg.bit.SysBalanceMode=0;
           // SysRegs.SysStateReg.bit.SysBalanceEn=0;
            //Balance 시간 조정
            if(SysRegs.SysStateReg.bit.SysBalanceMode==1)
            {
                SysRegs.BalanceTimeCount++;
                if(SysRegs.BalanceTimeCount>10)
                {
                   SysRegs.SysStateReg.bit.SysBalanceEn = !  SysRegs.SysStateReg.bit.SysBalanceEn;
                   SysRegs.BalanceTimeCount=0;
                }
            }
            else
            {
                SysRegs.SysStateReg.bit.SysBalanceEn=0;
            }
            // 셀 전압 Balance
            if(SysRegs.SysStateReg.bit.SysBalanceEn==1)
            {
                if(SysRegs.SysStateReg.bit.HMICOMEnable==0)
                {
                    SysRegs.HMICANErrCheck=0;
                    CANARegs.HMICMDRegs.all=0;
                    CANARegs.HMICEllTempsAgv=250;
                    CANARegs.HMICEllVoltMin=4200;
                    SysRegs.BalanceRefVoltageF = SysRegs.SysCellMinVoltageF;
                }
                if(SysRegs.SysStateReg.bit.HMICOMEnable==1)
                {
                    SysRegs.HMICANErrCheck++;
                    if(SysRegs.SysStateReg.bit.HMIBalanceMode==0)
                    {
                        SysRegs.BalanceRefVoltageF = SysRegs.SysCellMinVoltageF;
                    }
                    if(SysRegs.SysStateReg.bit.HMIBalanceMode==1)
                    {
                       SysRegs.BalanceRefVoltageF = (float32)(CANARegs.HMICEllVoltMin*0.001);
                    }
                    if(SysRegs.HMICANErrCheck>200)
                    {
                       SysRegs.HMICANErrCheck=210;
                       CANARegs.HMICEllTempsAgv=250;
                      // CANARegs.HMICEllVoltMin=4200;
                       SysRegs.BalanceRefVoltageF = (float32)(CANARegs.HMICEllVoltMin*0.001);
                       SysRegs.SysStateReg.bit.SysBalanceEn=0;
                    }

                }
                if(SysRegs.SysStateReg.bit.SysBalanceEn==1)
                {
                    Slave0Regs.ID=BMS_ID_0;
                    Slave0Regs.SlaveCh=C_Slave_ACh;
                    Slave0Regs.SysCellMinVoltage = SysRegs.BalanceRefVoltageF;
                    SlaveVoltagBalaHandler(&Slave0Regs);
                    //Slave0Regs.Balance.bit.B_Cell07=0;
                    //Slave0Regs.Balance.bit.B_Cell08=0;
                    //Slave0Regs.Balance.bit.B_Cell09=0;
                    //Slave0Regs.Balance.bit.B_Cell10=0;
                    Slave0Regs.Balance.bit.B_Cell11=0;
                    SlaveBmsBalance(&Slave0Regs);
                    SysRegs.SlaveVoltErrCount[0]=Slave0Regs.ErrorCount;

                    Slave1Regs.ID=BMS_ID_1;
                    Slave1Regs.SlaveCh=C_Slave_ACh;
                    Slave1Regs.SysCellMinVoltage = SysRegs.BalanceRefVoltageF;
                    SlaveVoltagBalaHandler(&Slave1Regs);
                    //Slave1Regs.Balance.bit.B_Cell07=0;
                    //Slave1Regs.Balance.bit.B_Cell08=0;
                    //Slave1Regs.Balance.bit.B_Cell09=0;
                    //Slave1Regs.Balance.bit.B_Cell10=0;
                    Slave1Regs.Balance.bit.B_Cell11=0;
                    SlaveBmsBalance(&Slave1Regs);
                    SysRegs.SlaveVoltErrCount[1]=Slave1Regs.ErrorCount;


                    Slave2Regs.ID=BMS_ID_2;
                    Slave2Regs.SlaveCh=C_Slave_ACh;
                    Slave2Regs.SysCellMinVoltage = SysRegs.BalanceRefVoltageF;
                    SlaveVoltagBalaHandler(&Slave2Regs);
                  //  Slave2Regs.Balance.bit.B_Cell07=0;
                  //  Slave2Regs.Balance.bit.B_Cell08=0;
                  //  Slave2Regs.Balance.bit.B_Cell09=0;
                  //  Slave2Regs.Balance.bit.B_Cell10=0;
                    Slave2Regs.Balance.bit.B_Cell11=0;
                    SlaveBmsBalance(&Slave2Regs);
                    SysRegs.SlaveVoltErrCount[2]=Slave2Regs.ErrorCount;

                    Slave3Regs.ID=BMS_ID_3;
                    Slave3Regs.SlaveCh=C_Slave_ACh;
                    Slave3Regs.SysCellMinVoltage = SysRegs.BalanceRefVoltageF;
                    SlaveVoltagBalaHandler(&Slave3Regs);
                  //  Slave3Regs.Balance.bit.B_Cell07=0;
                  //  Slave3Regs.Balance.bit.B_Cell08=0;
                  //  Slave3Regs.Balance.bit.B_Cell09=0;
                   // Slave3Regs.Balance.bit.B_Cell10=0;
                    Slave3Regs.Balance.bit.B_Cell11=0;
                    SlaveBmsBalance(&Slave3Regs);
                    SysRegs.SlaveVoltErrCount[3]=Slave3Regs.ErrorCount;

                    Slave4Regs.ID=BMS_ID_4;
                    Slave4Regs.SlaveCh=C_Slave_ACh;
                    Slave4Regs.SysCellMinVoltage = SysRegs.BalanceRefVoltageF;
                    SlaveVoltagBalaHandler(&Slave4Regs);
                 //  Slave4Regs.Balance.bit.B_Cell07=0;
                 //  Slave4Regs.Balance.bit.B_Cell08=0;
                 //  Slave4Regs.Balance.bit.B_Cell09=0;
                 //   Slave4Regs.Balance.bit.B_Cell10=0;
                    Slave4Regs.Balance.bit.B_Cell11=0;
                    SlaveBmsBalance(&Slave4Regs);
                    SysRegs.SlaveVoltErrCount[4]=Slave4Regs.ErrorCount;

                    Slave5Regs.ID=BMS_ID_5;
                    Slave5Regs.SlaveCh=C_Slave_ACh;
                    Slave5Regs.SysCellMinVoltage = SysRegs.BalanceRefVoltageF;
                    SlaveVoltagBalaHandler(&Slave5Regs);
                 //   Slave5Regs.Balance.bit.B_Cell07=0;
                 //   Slave5Regs.Balance.bit.B_Cell08=0;
                 //   Slave5Regs.Balance.bit.B_Cell09=0;
                 //   Slave5Regs.Balance.bit.B_Cell10=0;
                    Slave5Regs.Balance.bit.B_Cell11=0;
                    SlaveBmsBalance(&Slave5Regs);
                    SysRegs.SlaveVoltErrCount[5]=Slave5Regs.ErrorCount;

                    Slave6Regs.ID=BMS_ID_6;
                    Slave6Regs.SlaveCh=C_Slave_ACh;
                    Slave6Regs.SysCellMinVoltage = SysRegs.BalanceRefVoltageF;
                    SlaveVoltagBalaHandler(&Slave6Regs);
               //     Slave6Regs.Balance.bit.B_Cell07=0;
               //     Slave6Regs.Balance.bit.B_Cell08=0;
                //    Slave6Regs.Balance.bit.B_Cell09=0;
               //     Slave6Regs.Balance.bit.B_Cell10=0;
                    Slave6Regs.Balance.bit.B_Cell11=0;
                    SlaveBmsBalance(&Slave6Regs);
                    SysRegs.SlaveVoltErrCount[6]=Slave6Regs.ErrorCount;

                    Slave7Regs.ID=BMS_ID_7;
                    Slave7Regs.SlaveCh=C_Slave_ACh;
                    Slave7Regs.SysCellMinVoltage = SysRegs.BalanceRefVoltageF;
                    SlaveVoltagBalaHandler(&Slave7Regs);
                //    Slave7Regs.Balance.bit.B_Cell07=0;
                //    Slave7Regs.Balance.bit.B_Cell08=0;
                //    Slave7Regs.Balance.bit.B_Cell09=0;
                //    Slave7Regs.Balance.bit.B_Cell10=0;
                    Slave7Regs.Balance.bit.B_Cell11=0;
                    SlaveBmsBalance(&Slave7Regs);
                    SysRegs.SlaveVoltErrCount[7]=Slave7Regs.ErrorCount;

                    Slave8Regs.ID=BMS_ID_0;
                    Slave8Regs.SlaveCh=C_Slave_BCh;
                    Slave8Regs.SysCellMinVoltage = SysRegs.BalanceRefVoltageF;
                    SlaveVoltagBalaHandler(&Slave8Regs);
               //     Slave8Regs.Balance.bit.B_Cell07=0;
               //    Slave8Regs.Balance.bit.B_Cell08=0;
               //     Slave8Regs.Balance.bit.B_Cell09=0;
               //     Slave8Regs.Balance.bit.B_Cell10=0;
                    Slave8Regs.Balance.bit.B_Cell11=0;
                    SlaveBmsBalance(&Slave8Regs);
                    SysRegs.SlaveVoltErrCount[8]=Slave8Regs.ErrorCount;

                    Slave9Regs.ID=BMS_ID_1;
                    Slave9Regs.SlaveCh=C_Slave_BCh;
                    Slave9Regs.SysCellMinVoltage = SysRegs.BalanceRefVoltageF;
                    SlaveVoltagBalaHandler(&Slave9Regs);
              //      Slave9Regs.Balance.bit.B_Cell07=0;
              //      Slave9Regs.Balance.bit.B_Cell08=0;
              //      Slave9Regs.Balance.bit.B_Cell09=0;
              //      Slave9Regs.Balance.bit.B_Cell10=0;
                    Slave9Regs.Balance.bit.B_Cell11=0;
                    SlaveBmsBalance(&Slave9Regs);
                    SysRegs.SlaveVoltErrCount[9]=Slave9Regs.ErrorCount;

                    Slave10Regs.ID=BMS_ID_2;
                    Slave10Regs.SlaveCh=C_Slave_BCh;
                    Slave10Regs.SysCellMinVoltage = SysRegs.BalanceRefVoltageF;
                    SlaveVoltagBalaHandler(&Slave10Regs);
              //      Slave10Regs.Balance.bit.B_Cell07=0;
              //      Slave10Regs.Balance.bit.B_Cell08=0;
            //        Slave10Regs.Balance.bit.B_Cell09=0;
              //      Slave10Regs.Balance.bit.B_Cell10=0;
                    Slave10Regs.Balance.bit.B_Cell11=0;
                    SlaveBmsBalance(&Slave10Regs);
                    SysRegs.SlaveVoltErrCount[10]=Slave10Regs.ErrorCount;

                    Slave11Regs.ID=BMS_ID_3;
                    Slave11Regs.SlaveCh=C_Slave_BCh;
                    Slave11Regs.SysCellMinVoltage = SysRegs.BalanceRefVoltageF;
                    SlaveVoltagBalaHandler(&Slave11Regs);
               //     Slave11Regs.Balance.bit.B_Cell07=0;
               //     Slave11Regs.Balance.bit.B_Cell08=0;
               //     Slave11Regs.Balance.bit.B_Cell09=0;
               //     Slave11Regs.Balance.bit.B_Cell10=0;
                    Slave11Regs.Balance.bit.B_Cell11=0;
                    SlaveBmsBalance(&Slave11Regs);
                    SysRegs.SlaveVoltErrCount[11]=Slave11Regs.ErrorCount;

                    Slave12Regs.ID=BMS_ID_4;
                    Slave12Regs.SlaveCh=C_Slave_BCh;
                    Slave12Regs.SysCellMinVoltage = SysRegs.BalanceRefVoltageF;
                    SlaveVoltagBalaHandler(&Slave12Regs);
               //     Slave11Regs.Balance.bit.B_Cell07=0;
               //     Slave11Regs.Balance.bit.B_Cell08=0;
               //     Slave11Regs.Balance.bit.B_Cell09=0;
               //     Slave11Regs.Balance.bit.B_Cell10=0;
                    Slave12Regs.Balance.bit.B_Cell11=0;
                    SlaveBmsBalance(&Slave12Regs);
                    SysRegs.SlaveVoltErrCount[12]=Slave12Regs.ErrorCount;

                    Slave13Regs.ID=BMS_ID_5;
                    Slave13Regs.SlaveCh=C_Slave_BCh;
                    Slave13Regs.SysCellMinVoltage = SysRegs.BalanceRefVoltageF;
                    SlaveVoltagBalaHandler(&Slave13Regs);
               //     Slave11Regs.Balance.bit.B_Cell07=0;
               //     Slave11Regs.Balance.bit.B_Cell08=0;
               //     Slave11Regs.Balance.bit.B_Cell09=0;
               //     Slave11Regs.Balance.bit.B_Cell10=0;
                    Slave13Regs.Balance.bit.B_Cell11=0;
                    SlaveBmsBalance(&Slave13Regs);
                    SysRegs.SlaveVoltErrCount[13]=Slave13Regs.ErrorCount;

                    Slave14Regs.ID=BMS_ID_6;
                    Slave14Regs.SlaveCh=C_Slave_BCh;
                    Slave14Regs.SysCellMinVoltage = SysRegs.BalanceRefVoltageF;
                    SlaveVoltagBalaHandler(&Slave14Regs);
               //     Slave11Regs.Balance.bit.B_Cell07=0;
               //     Slave11Regs.Balance.bit.B_Cell08=0;
               //     Slave11Regs.Balance.bit.B_Cell09=0;
               //     Slave11Regs.Balance.bit.B_Cell10=0;
                    Slave14Regs.Balance.bit.B_Cell11=0;
                    SlaveBmsBalance(&Slave14Regs);
                    SysRegs.SlaveVoltErrCount[14]=Slave14Regs.ErrorCount;

                    Slave15Regs.ID=BMS_ID_7;
                    Slave15Regs.SlaveCh=C_Slave_BCh;
                    Slave15Regs.SysCellMinVoltage = SysRegs.BalanceRefVoltageF;
                    SlaveVoltagBalaHandler(&Slave15Regs);
               //     Slave11Regs.Balance.bit.B_Cell07=0;
               //     Slave11Regs.Balance.bit.B_Cell08=0;
               //     Slave11Regs.Balance.bit.B_Cell09=0;
               //     Slave11Regs.Balance.bit.B_Cell10=0;
                    Slave15Regs.Balance.bit.B_Cell11=0;
                    SlaveBmsBalance(&Slave15Regs);
                    SysRegs.SlaveVoltErrCount[15]=Slave15Regs.ErrorCount;
                }
            }
            if(SysRegs.SysStateReg.bit.SysBalanceEn==0)
            {
                if(SysRegs.SysStateReg.bit.CellVoltOk==0)
                {
                    if(SysRegs.SlaveReadVoltEn.bit.SlaveBMS00==1)
                    {
                        LEDSysState_H;
                        Slave0Regs.ID=BMS_ID_0;
                        Slave0Regs.SlaveCh=C_Slave_ACh;

                        Slave0Regs.Balance.all = 0x0000;
                        SlaveBmsBalance(&Slave0Regs);
                        SysRegs.SlaveBalanErrCount[0]=Slave0Regs.ErrorCount;
                        if(SysRegs.SlaveBalanErrCount[0]>C_ISOSPIPrtectCont)
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS00=1;
                            SysRegs.SlaveBalanErrCount[0]=C_ISOSPIPrtectCont+10;
                        }
                        else
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS00 =0;
                        }

                        Slave0Regs.StateMachine = STATE_BATREAD;
                        SlaveVoltagHandler(&Slave0Regs);
                        SysRegs.SlaveVoltErrCount[0]=Slave0Regs.ErrorCount;
                        if(SysRegs.SlaveVoltErrCount[0]>C_ISOSPIPrtectCont)
                        {
                             SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS00=1;
                             SysRegs.SlaveVoltErrCount[0]=C_ISOSPIPrtectCont+10;
                        }
                        else
                        {
                             SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS00 =0;
                        }
                        LEDSysState_L;
                    }
                    if(SysRegs.SlaveReadVoltEn.bit.SlaveBMS01==1)
                    {
                        Slave1Regs.ID=BMS_ID_1;
                        Slave1Regs.SlaveCh=C_Slave_ACh;

                        Slave1Regs.Balance.all = 0x0000;
                        SlaveBmsBalance(&Slave1Regs);
                        SysRegs.SlaveBalanErrCount[1]=Slave1Regs.ErrorCount;
                        if(SysRegs.SlaveBalanErrCount[1]>C_ISOSPIPrtectCont)
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS00=1;
                            SysRegs.SlaveBalanErrCount[1]=C_ISOSPIPrtectCont+10;
                        }
                        else
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS00 =0;
                        }

                        Slave1Regs.StateMachine = STATE_BATREAD;
                        SlaveVoltagHandler(&Slave1Regs);
                        SysRegs.SlaveVoltErrCount[1]=Slave1Regs.ErrorCount;
                        if(SysRegs.SlaveVoltErrCount[1]>C_ISOSPIPrtectCont)
                        {
                             SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS01=1;
                             SysRegs.SlaveVoltErrCount[1]=C_ISOSPIPrtectCont+10;
                        }
                        else
                        {
                             SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS01 =0;
                        }
                    }
                    if(SysRegs.SlaveReadVoltEn.bit.SlaveBMS02==1)
                    {
                        Slave2Regs.ID=BMS_ID_2;
                        Slave2Regs.SlaveCh=C_Slave_ACh;

                        Slave2Regs.Balance.all = 0x0000;
                        SlaveBmsBalance(&Slave2Regs);
                        SysRegs.SlaveBalanErrCount[2]=Slave2Regs.ErrorCount;
                        if(SysRegs.SlaveBalanErrCount[2]>C_ISOSPIPrtectCont)
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS01=1;
                            SysRegs.SlaveBalanErrCount[2]=C_ISOSPIPrtectCont+10;
                        }
                        else
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS01 =0;
                        }

                        Slave2Regs.StateMachine = STATE_BATREAD;
                        SlaveVoltagHandler(&Slave2Regs);
                        SysRegs.SlaveVoltErrCount[2]=Slave2Regs.ErrorCount;
                        if(SysRegs.SlaveVoltErrCount[2]>C_ISOSPIPrtectCont)
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS02=1;
                            SysRegs.SlaveVoltErrCount[2]=C_ISOSPIPrtectCont+10;
                        }
                        else
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS02=0;
                        }
                    }
                    if(SysRegs.SlaveReadVoltEn.bit.SlaveBMS03==1)
                    {
                        Slave3Regs.ID=BMS_ID_3;
                        Slave3Regs.SlaveCh=C_Slave_ACh;
                        Slave3Regs.Balance.all = 0x0000;
                        SlaveBmsBalance(&Slave3Regs);
                        SysRegs.SlaveVoltErrCount[3]=Slave3Regs.ErrorCount;
                        if(SysRegs.SlaveVoltErrCount[3]>C_ISOSPIPrtectCont)
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS03=1;
                            SysRegs.SlaveVoltErrCount[3]=C_ISOSPIPrtectCont+10;
                        }
                        else
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS03 =0;
                        }

                        Slave3Regs.StateMachine = STATE_BATREAD;
                        Slave3Regs.ID=BMS_ID_3;
                        Slave3Regs.SlaveCh=C_Slave_ACh;
                        SlaveVoltagHandler(&Slave3Regs);
                        SysRegs.SlaveVoltErrCount[3]=Slave3Regs.ErrorCount;
                        if(SysRegs.SlaveVoltErrCount[3]>C_ISOSPIPrtectCont)
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS03=1;
                            SysRegs.SlaveVoltErrCount[3]=C_ISOSPIPrtectCont+10;
                        }
                        else
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS03=0;
                        }
                    }
                    if(SysRegs.SlaveReadVoltEn.bit.SlaveBMS04==1)
                    {
                        Slave4Regs.ID=BMS_ID_4;
                        Slave4Regs.SlaveCh=C_Slave_ACh;
                        Slave4Regs.Balance.all = 0x0000;
                        SlaveBmsBalance(&Slave4Regs);
                        SysRegs.SlaveVoltErrCount[4]=Slave3Regs.ErrorCount;
                        if(SysRegs.SlaveVoltErrCount[4]>C_ISOSPIPrtectCont)
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS04=1;
                            SysRegs.SlaveVoltErrCount[4]=C_ISOSPIPrtectCont+10;
                        }
                        else
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS04=0;
                        }
                        Slave4Regs.StateMachine = STATE_BATREAD;
                        SlaveVoltagHandler(&Slave4Regs);
                        SysRegs.SlaveVoltErrCount[4]=Slave4Regs.ErrorCount;
                        if(SysRegs.SlaveVoltErrCount[4]>C_ISOSPIPrtectCont)
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS04=1;
                            SysRegs.SlaveVoltErrCount[4]=C_ISOSPIPrtectCont+10;
                        }
                        else
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS04=0;
                        }
                    }
                    if(SysRegs.SlaveReadVoltEn.bit.SlaveBMS05==1)
                    {
                        Slave5Regs.ID=BMS_ID_5;
                        Slave5Regs.SlaveCh=C_Slave_ACh;
                        Slave5Regs.Balance.all = 0x0000;
                        SlaveBmsBalance(&Slave5Regs);
                        SysRegs.SlaveVoltErrCount[5]=Slave5Regs.ErrorCount;
                        if(SysRegs.SlaveVoltErrCount[5]>C_ISOSPIPrtectCont)
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS05=1;
                            SysRegs.SlaveVoltErrCount[5]=C_ISOSPIPrtectCont+10;
                        }
                        else
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS05=0;
                        }
                        Slave5Regs.StateMachine = STATE_BATREAD;
                        SlaveVoltagHandler(&Slave5Regs);
                        SysRegs.SlaveVoltErrCount[5]=Slave5Regs.ErrorCount;
                        if(SysRegs.SlaveVoltErrCount[5]>C_ISOSPIPrtectCont)
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS05=1;
                            SysRegs.SlaveVoltErrCount[5]=C_ISOSPIPrtectCont+10;
                        }
                        else
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS05=0;
                        }
                    }
                    if(SysRegs.SlaveReadVoltEn.bit.SlaveBMS06==1)
                    {
                        Slave6Regs.ID=BMS_ID_6;
                        Slave6Regs.SlaveCh=C_Slave_ACh;
                        Slave6Regs.Balance.all = 0x0000;
                        SlaveBmsBalance(&Slave6Regs);
                        SysRegs.SlaveVoltErrCount[6]=Slave6Regs.ErrorCount;
                        if(SysRegs.SlaveVoltErrCount[6]>C_ISOSPIPrtectCont)
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS06=1;
                            SysRegs.SlaveVoltErrCount[6]=C_ISOSPIPrtectCont+10;
                        }
                        else
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS06=0;
                        }
                        Slave6Regs.StateMachine = STATE_BATREAD;
                        SlaveVoltagHandler(&Slave6Regs);
                        SysRegs.SlaveVoltErrCount[6]=Slave6Regs.ErrorCount;
                        if(SysRegs.SlaveVoltErrCount[6]>C_ISOSPIPrtectCont)
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS06=1;
                            SysRegs.SlaveVoltErrCount[6]=C_ISOSPIPrtectCont+10;
                        }
                        else
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS06=0;
                        }
                    }
                    if(SysRegs.SlaveReadVoltEn.bit.SlaveBMS07==1)
                    {
                        Slave7Regs.ID=BMS_ID_7;
                        Slave7Regs.SlaveCh=C_Slave_ACh;
                        Slave7Regs.Balance.all = 0x0000;
                        SlaveBmsBalance(&Slave7Regs);
                        SysRegs.SlaveVoltErrCount[7]=Slave7Regs.ErrorCount;
                        if(SysRegs.SlaveVoltErrCount[7]>C_ISOSPIPrtectCont)
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS07=1;
                            SysRegs.SlaveVoltErrCount[7]=C_ISOSPIPrtectCont+10;
                        }
                        else
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS07=0;
                        }
                        Slave7Regs.StateMachine = STATE_BATREAD;
                        SlaveVoltagHandler(&Slave7Regs);
                        SysRegs.SlaveVoltErrCount[7]=Slave7Regs.ErrorCount;
                        if(SysRegs.SlaveVoltErrCount[7]>C_ISOSPIPrtectCont)
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS07=1;
                            SysRegs.SlaveVoltErrCount[7]=C_ISOSPIPrtectCont+10;
                        }
                        else
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS07=0;
                        }
                    }
                    if(SysRegs.SlaveReadVoltEn.bit.SlaveBMS08==1)
                    {
                        Slave8Regs.ID=BMS_ID_0;
                        Slave8Regs.SlaveCh=C_Slave_BCh;
                        Slave8Regs.Balance.all = 0x0000;
                        SlaveBmsBalance(&Slave8Regs);
                        SysRegs.SlaveVoltErrCount[8]=Slave8Regs.ErrorCount;
                        if(SysRegs.SlaveVoltErrCount[8]>C_ISOSPIPrtectCont)
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS08=1;
                            SysRegs.SlaveVoltErrCount[8]=C_ISOSPIPrtectCont+10;
                        }
                        else
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS08=0;
                        }
                        Slave8Regs.StateMachine = STATE_BATREAD;
                        SlaveVoltagHandler(&Slave8Regs);
                        SysRegs.SlaveVoltErrCount[8]=Slave8Regs.ErrorCount;
                        if(SysRegs.SlaveVoltErrCount[8]>C_ISOSPIPrtectCont)
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS08=1;
                            SysRegs.SlaveVoltErrCount[8]=C_ISOSPIPrtectCont+10;
                        }
                        else
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS08=0;
                        }
                    }
                    if(SysRegs.SlaveReadVoltEn.bit.SlaveBMS09==1)
                    {
                        Slave9Regs.ID=BMS_ID_1;
                        Slave9Regs.SlaveCh=C_Slave_BCh;
                        Slave9Regs.Balance.all = 0x0000;
                        SlaveBmsBalance(&Slave9Regs);
                        SysRegs.SlaveVoltErrCount[9]=Slave8Regs.ErrorCount;
                        if(SysRegs.SlaveVoltErrCount[9]>C_ISOSPIPrtectCont)
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS09=1;
                            SysRegs.SlaveVoltErrCount[9]=C_ISOSPIPrtectCont+10;
                        }
                        else
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS09=0;
                        }
                        Slave9Regs.StateMachine = STATE_BATREAD;
                        SlaveVoltagHandler(&Slave9Regs);
                        SysRegs.SlaveVoltErrCount[9]=Slave9Regs.ErrorCount;
                        if(SysRegs.SlaveVoltErrCount[9]>C_ISOSPIPrtectCont)
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS09=1;
                            SysRegs.SlaveVoltErrCount[9]=C_ISOSPIPrtectCont+10;
                        }
                        else
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS09=0;
                        }
                    }
                    if(SysRegs.SlaveReadVoltEn.bit.SlaveBMS10==1)
                    {
                        Slave10Regs.ID=BMS_ID_2;
                        Slave10Regs.SlaveCh=C_Slave_BCh;
                        Slave10Regs.Balance.all = 0x0000;
                        SlaveBmsBalance(&Slave10Regs);
                        SysRegs.SlaveVoltErrCount[10]=Slave10Regs.ErrorCount;
                        if(SysRegs.SlaveVoltErrCount[10]>C_ISOSPIPrtectCont)
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS10=1;
                            SysRegs.SlaveVoltErrCount[10]=C_ISOSPIPrtectCont+10;
                        }
                        else
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS10=0;
                        }
                        Slave10Regs.StateMachine = STATE_BATREAD;
                        SlaveVoltagHandler(&Slave10Regs);
                        SysRegs.SlaveVoltErrCount[10]=Slave10Regs.ErrorCount;
                        if(SysRegs.SlaveVoltErrCount[10]>C_ISOSPIPrtectCont)
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS10=1;
                            SysRegs.SlaveVoltErrCount[10]=C_ISOSPIPrtectCont+10;
                        }
                        else
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS10=0;
                        }
                    }
                    if(SysRegs.SlaveReadVoltEn.bit.SlaveBMS11==1)
                    {
                        Slave11Regs.ID=BMS_ID_3;
                        Slave11Regs.SlaveCh=C_Slave_BCh;
                        Slave11Regs.Balance.all = 0x0000;
                        SlaveBmsBalance(&Slave11Regs);
                        SysRegs.SlaveVoltErrCount[11]=Slave10Regs.ErrorCount;
                        if(SysRegs.SlaveVoltErrCount[11]>C_ISOSPIPrtectCont)
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS11=1;
                            SysRegs.SlaveVoltErrCount[11]=C_ISOSPIPrtectCont+10;
                        }
                        else
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS11=0;
                        }
                        Slave11Regs.StateMachine = STATE_BATREAD;
                        SlaveVoltagHandler(&Slave11Regs);
                        SysRegs.SlaveVoltErrCount[11]=Slave11Regs.ErrorCount;
                        if(SysRegs.SlaveVoltErrCount[11]>C_ISOSPIPrtectCont)
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS11=1;
                            SysRegs.SlaveVoltErrCount[11]=C_ISOSPIPrtectCont+10;
                        }
                        else
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS11=0;
                        }
                    }
                    SysRegs.VoltTempsReadCount++;
                    SysRegs.SysStateReg.bit.CellVoltOk=1;
                }
                if(SysRegs.SlaveReadVoltEn.bit.SlaveBMS12==1)
                {
                    Slave12Regs.ID=BMS_ID_4;
                    Slave12Regs.SlaveCh=C_Slave_BCh;
                    Slave12Regs.Balance.all = 0x0000;
                    SlaveBmsBalance(&Slave12Regs);
                    SysRegs.SlaveVoltErrCount[12]=Slave10Regs.ErrorCount;
                    if(SysRegs.SlaveVoltErrCount[12]>C_ISOSPIPrtectCont)
                    {
                        SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS12=1;
                        SysRegs.SlaveVoltErrCount[12]=C_ISOSPIPrtectCont+10;
                    }
                    else
                    {
                        SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS12=0;
                    }
                    Slave12Regs.StateMachine = STATE_BATREAD;
                    SlaveVoltagHandler(&Slave12Regs);
                    SysRegs.SlaveVoltErrCount[12]=Slave11Regs.ErrorCount;
                    if(SysRegs.SlaveVoltErrCount[12]>C_ISOSPIPrtectCont)
                    {
                        SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS12=1;
                        SysRegs.SlaveVoltErrCount[12]=C_ISOSPIPrtectCont+10;
                    }
                    else
                    {
                        SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS12=0;
                    }
                 }
                if(SysRegs.SlaveReadVoltEn.bit.SlaveBMS13==1)
                {
                    Slave13Regs.ID=BMS_ID_5;
                    Slave13Regs.SlaveCh=C_Slave_BCh;
                    Slave13Regs.Balance.all = 0x0000;
                    SlaveBmsBalance(&Slave13Regs);
                    SysRegs.SlaveVoltErrCount[13]=Slave10Regs.ErrorCount;
                    if(SysRegs.SlaveVoltErrCount[13]>C_ISOSPIPrtectCont)
                    {
                        SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS13=1;
                        SysRegs.SlaveVoltErrCount[13]=C_ISOSPIPrtectCont+10;
                    }
                    else
                    {
                        SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS13=0;
                    }
                    Slave13Regs.StateMachine = STATE_BATREAD;
                    SlaveVoltagHandler(&Slave13Regs);
                    SysRegs.SlaveVoltErrCount[13]=Slave11Regs.ErrorCount;
                    if(SysRegs.SlaveVoltErrCount[13]>C_ISOSPIPrtectCont)
                    {
                        SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS13=1;
                        SysRegs.SlaveVoltErrCount[13]=C_ISOSPIPrtectCont+10;
                    }
                    else
                    {
                        SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS13=0;
                    }
                }
                if(SysRegs.SlaveReadVoltEn.bit.SlaveBMS14==1)
                {
                    Slave14Regs.ID=BMS_ID_6;
                    Slave14Regs.SlaveCh=C_Slave_BCh;
                    Slave14Regs.Balance.all = 0x0000;
                    SlaveBmsBalance(&Slave14Regs);
                    SysRegs.SlaveVoltErrCount[14]=Slave10Regs.ErrorCount;
                    if(SysRegs.SlaveVoltErrCount[14]>C_ISOSPIPrtectCont)
                    {
                        SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS14=1;
                        SysRegs.SlaveVoltErrCount[14]=C_ISOSPIPrtectCont+10;
                    }
                    else
                    {
                        SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS14=0;
                    }
                    Slave14Regs.StateMachine = STATE_BATREAD;
                    SlaveVoltagHandler(&Slave14Regs);
                    SysRegs.SlaveVoltErrCount[14]=Slave11Regs.ErrorCount;
                    if(SysRegs.SlaveVoltErrCount[14]>C_ISOSPIPrtectCont)
                    {
                        SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS14=1;
                        SysRegs.SlaveVoltErrCount[14]=C_ISOSPIPrtectCont+10;
                    }
                    else
                    {
                        SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS14=0;
                    }
                }
                if(SysRegs.SlaveReadVoltEn.bit.SlaveBMS15==1)
                {
                    Slave15Regs.ID=BMS_ID_7;
                    Slave15Regs.SlaveCh=C_Slave_BCh;
                    Slave15Regs.Balance.all = 0x0000;
                    SlaveBmsBalance(&Slave15Regs);
                    SysRegs.SlaveVoltErrCount[15]=Slave10Regs.ErrorCount;
                    if(SysRegs.SlaveVoltErrCount[15]>C_ISOSPIPrtectCont)
                    {
                        SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS15=1;
                        SysRegs.SlaveVoltErrCount[15]=C_ISOSPIPrtectCont+10;
                    }
                    else
                    {
                        SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS15=0;
                    }
                    Slave15Regs.StateMachine = STATE_BATREAD;
                    SlaveVoltagHandler(&Slave15Regs);
                    SysRegs.SlaveVoltErrCount[15]=Slave11Regs.ErrorCount;
                    if(SysRegs.SlaveVoltErrCount[15]>C_ISOSPIPrtectCont)
                    {
                        SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS15=1;
                        SysRegs.SlaveVoltErrCount[15]=C_ISOSPIPrtectCont+10;
                    }
                    else
                    {
                        SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS15=0;
                    }
                }
                SysRegs.VoltTempsReadCount++;
                SysRegs.SysStateReg.bit.CellVoltOk=1;
            }
            SysRegs.CellVoltsampling=0;
        }
       if(SysRegs.CellTempssampling>CellTempSampleTime)
       {
           if(SysRegs.SysStateReg.bit.CellTempsOk==0)
           {
               if(SysRegs.SlaveReadTempsEn.bit.SlaveBMS00==1)
               {
                   Slave0Regs.ID=BMS_ID_0;
                   Slave0Regs.SlaveCh=C_Slave_ACh;

                   Slave0Regs.BATICDO.bit.GPIO1=1;
                   SlaveBMSDigiteldoutOHandler(&Slave0Regs);

                   SysRegs.SlaveTempsErrCount[0]=Slave0Regs.ErrorCount;
                   if(SysRegs.SlaveTempsErrCount[0]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS00=1;
                       SysRegs.SlaveTempsErrCount[0]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS00=0;
                   }

                   SalveTempsHandler(&Slave0Regs);
                   if(SysRegs.SlaveTempsErrCount[0]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS00=1;
                       SysRegs.SlaveTempsErrCount[0]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS00=0;
                   }

               }
               if(SysRegs.SlaveReadTempsEn.bit.SlaveBMS01==1)
               {
                   Slave1Regs.ID=BMS_ID_1;
                   Slave1Regs.SlaveCh=C_Slave_ACh;

                   Slave1Regs.BATICDO.bit.GPIO1=1;
                   SlaveBMSDigiteldoutOHandler(&Slave1Regs);

                   SysRegs.SlaveTempsErrCount[1]=Slave1Regs.ErrorCount;
                   if(SysRegs.SlaveTempsErrCount[1]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS01=1;
                       SysRegs.SlaveTempsErrCount[1]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS01=0;
                   }

                   SalveTempsHandler(&Slave1Regs);
                   SysRegs.SlaveTempsErrCount[1]=Slave1Regs.ErrorCount;
                   if(SysRegs.SlaveTempsErrCount[1]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS01=1;
                       SysRegs.SlaveTempsErrCount[1]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS01=0;
                   }
               }
               if(SysRegs.SlaveReadTempsEn.bit.SlaveBMS02==1)
               {
                   Slave2Regs.ID=BMS_ID_2;
                   Slave2Regs.SlaveCh=C_Slave_ACh;

                   Slave2Regs.BATICDO.bit.GPIO1=1;
                   SlaveBMSDigiteldoutOHandler(&Slave2Regs);

                   SysRegs.SlaveTempsErrCount[2]=Slave2Regs.ErrorCount;
                   if(SysRegs.SlaveTempsErrCount[2]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS02=1;
                       SysRegs.SlaveTempsErrCount[2]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS02=0;
                   }

                   SalveTempsHandler(&Slave2Regs);
                   SysRegs.SlaveTempsErrCount[2]=Slave2Regs.ErrorCount;
                   if(SysRegs.SlaveTempsErrCount[2]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS02=1;
                       SysRegs.SlaveTempsErrCount[2]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS02=0;
                   }

               }
               if(SysRegs.SlaveReadTempsEn.bit.SlaveBMS03==1)
               {
                   Slave3Regs.ID=BMS_ID_3;
                   Slave3Regs.SlaveCh=C_Slave_ACh;

                   Slave3Regs.BATICDO.bit.GPIO1=1;
                   SlaveBMSDigiteldoutOHandler(&Slave3Regs);
                   SysRegs.SlaveTempsErrCount[3]=Slave3Regs.ErrorCount;
                   if(SysRegs.SlaveTempsErrCount[3]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS03=1;
                       SysRegs.SlaveTempsErrCount[3]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS03=0;
                   }

                   SalveTempsHandler(&Slave3Regs);
                   SysRegs.SlaveTempsErrCount[3]=Slave3Regs.ErrorCount;
                   if(SysRegs.SlaveTempsErrCount[3]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS03=1;
                       SysRegs.SlaveTempsErrCount[3]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS03=0;
                   }
               }
               if(SysRegs.SlaveReadTempsEn.bit.SlaveBMS04==1)
               {
                   Slave4Regs.ID=BMS_ID_4;
                   Slave4Regs.SlaveCh=C_Slave_ACh;

                   Slave4Regs.BATICDO.bit.GPIO1=1;
                   SlaveBMSDigiteldoutOHandler(&Slave4Regs);

                   SysRegs.SlaveTempsErrCount[4]=Slave4Regs.ErrorCount;
                   if(SysRegs.SlaveTempsErrCount[4]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS04=1;
                       SysRegs.SlaveTempsErrCount[4]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS04=0;
                   }

                   SalveTempsHandler(&Slave4Regs);
                   SysRegs.SlaveTempsErrCount[4]=Slave4Regs.ErrorCount;
                   if(SysRegs.SlaveTempsErrCount[4]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS04=1;
                       SysRegs.SlaveTempsErrCount[4]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS04=0;
                   }
               }
               if(SysRegs.SlaveReadTempsEn.bit.SlaveBMS05==1)
               {
                   Slave5Regs.ID=BMS_ID_5;
                   Slave5Regs.SlaveCh=C_Slave_ACh;

                   Slave5Regs.BATICDO.bit.GPIO1=1;
                   SlaveBMSDigiteldoutOHandler(&Slave5Regs);
                   SysRegs.SlaveTempsErrCount[5]=Slave5Regs.ErrorCount;
                   if(SysRegs.SlaveTempsErrCount[5]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS05=1;
                       SysRegs.SlaveTempsErrCount[5]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS05=0;
                   }

                   SalveTempsHandler(&Slave5Regs);
                   SysRegs.SlaveTempsErrCount[5]=Slave5Regs.ErrorCount;
                   if(SysRegs.SlaveTempsErrCount[5]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS05=1;
                       SysRegs.SlaveTempsErrCount[5]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS05=0;
                   }
               }
               if(SysRegs.SlaveReadTempsEn.bit.SlaveBMS06==1)
               {
                   Slave6Regs.ID=BMS_ID_6;
                   Slave6Regs.SlaveCh=C_Slave_ACh;

                   Slave6Regs.BATICDO.bit.GPIO1=1;
                   SlaveBMSDigiteldoutOHandler(&Slave6Regs);
                   SysRegs.SlaveTempsErrCount[6]=Slave6Regs.ErrorCount;
                   if(SysRegs.SlaveTempsErrCount[6]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS06=1;
                       SysRegs.SlaveTempsErrCount[6]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS06=0;
                   }

                   SalveTempsHandler(&Slave6Regs);
                   SysRegs.SlaveTempsErrCount[6]=Slave6Regs.ErrorCount;
                   if(SysRegs.SlaveTempsErrCount[6]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS06=1;
                       SysRegs.SlaveTempsErrCount[6]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS06=0;
                   }
               }
               if(SysRegs.SlaveReadTempsEn.bit.SlaveBMS07==1)
               {
                   Slave7Regs.ID=BMS_ID_7;
                   Slave7Regs.SlaveCh=C_Slave_ACh;

                   Slave7Regs.BATICDO.bit.GPIO1=1;
                   SlaveBMSDigiteldoutOHandler(&Slave7Regs);
                   SysRegs.SlaveTempsErrCount[7]=Slave7Regs.ErrorCount;
                   if(SysRegs.SlaveTempsErrCount[7]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS07=1;
                       SysRegs.SlaveTempsErrCount[7]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS07=0;
                   }

                   SalveTempsHandler(&Slave7Regs);
                   SysRegs.SlaveTempsErrCount[7]=Slave7Regs.ErrorCount;
                   if(SysRegs.SlaveTempsErrCount[7]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS07=1;
                       SysRegs.SlaveTempsErrCount[7]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS07=0;
                   }
               }
               if(SysRegs.SlaveReadTempsEn.bit.SlaveBMS08==1)
               {
                   Slave8Regs.ID=BMS_ID_0;
                   Slave8Regs.SlaveCh=C_Slave_BCh;

                   Slave8Regs.BATICDO.bit.GPIO1=1;
                   SlaveBMSDigiteldoutOHandler(&Slave8Regs);
                   SysRegs.SlaveTempsErrCount[8]=Slave8Regs.ErrorCount;
                   if(SysRegs.SlaveTempsErrCount[8]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS07=1;
                       SysRegs.SlaveTempsErrCount[8]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS07=0;
                   }

                   SalveTempsHandler(&Slave8Regs);
                   SysRegs.SlaveTempsErrCount[8]=Slave8Regs.ErrorCount;
                   if(SysRegs.SlaveTempsErrCount[8]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS07=1;
                       SysRegs.SlaveTempsErrCount[8]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS07=0;
                   }
               }
               if(SysRegs.SlaveReadTempsEn.bit.SlaveBMS09==1)
               {
                   Slave9Regs.ID=BMS_ID_1;
                   Slave9Regs.SlaveCh=C_Slave_BCh;

                   Slave9Regs.BATICDO.bit.GPIO1=1;
                   SlaveBMSDigiteldoutOHandler(&Slave9Regs);
                   SysRegs.SlaveTempsErrCount[9]=Slave9Regs.ErrorCount;
                   if(SysRegs.SlaveTempsErrCount[9]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS09=1;
                       SysRegs.SlaveTempsErrCount[9]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS09=0;
                   }
                   SalveTempsHandler(&Slave9Regs);
                   SysRegs.SlaveTempsErrCount[9]=Slave9Regs.ErrorCount;
                   if(SysRegs.SlaveTempsErrCount[9]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS09=1;
                       SysRegs.SlaveTempsErrCount[9]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS09=0;
                   }


               }
               if(SysRegs.SlaveReadTempsEn.bit.SlaveBMS10==1)
               {
                   Slave10Regs.ID=BMS_ID_2;
                   Slave10Regs.SlaveCh=C_Slave_BCh;

                   Slave10Regs.BATICDO.bit.GPIO1=0;
                   SlaveBMSDigiteldoutOHandler(&Slave10Regs);
                   SysRegs.SlaveTempsErrCount[10]=Slave10Regs.ErrorCount;
                   if(SysRegs.SlaveTempsErrCount[10]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS10=1;
                       SysRegs.SlaveTempsErrCount[10]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS10=0;
                   }
                   SalveTempsHandler(&Slave10Regs);
                   SysRegs.SlaveTempsErrCount[10]=Slave10Regs.ErrorCount;
                   if(SysRegs.SlaveTempsErrCount[10]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS10=1;
                       SysRegs.SlaveTempsErrCount[10]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS10=0;
                   }
               }
               if(SysRegs.SlaveReadTempsEn.bit.SlaveBMS11==1)
               {
                   Slave11Regs.ID=BMS_ID_3;
                   Slave11Regs.SlaveCh=C_Slave_BCh;

                   Slave11Regs.BATICDO.bit.GPIO1=0;
                   SlaveBMSDigiteldoutOHandler(&Slave11Regs);
                   SysRegs.SlaveTempsErrCount[11]=Slave11Regs.ErrorCount;
                   if(SysRegs.SlaveTempsErrCount[11]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS11=1;
                       SysRegs.SlaveTempsErrCount[11]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS11=0;
                   }

                   SalveTempsHandler(&Slave11Regs);
                   SysRegs.SlaveTempsErrCount[11]=Slave11Regs.ErrorCount;
                   if(SysRegs.SlaveTempsErrCount[11]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS11=1;
                       SysRegs.SlaveTempsErrCount[11]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS11=0;
                   }

               }
               if(SysRegs.SlaveReadTempsEn.bit.SlaveBMS12==1)
               {
                   Slave12Regs.ID=BMS_ID_4;
                   Slave12Regs.SlaveCh=C_Slave_BCh;

                   Slave12Regs.BATICDO.bit.GPIO1=0;
                   SlaveBMSDigiteldoutOHandler(&Slave12Regs);
                   SysRegs.SlaveTempsErrCount[12]=Slave12Regs.ErrorCount;
                   if(SysRegs.SlaveTempsErrCount[12]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS12=1;
                       SysRegs.SlaveTempsErrCount[12]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS12=0;
                   }

                   SalveTempsHandler(&Slave12Regs);
                   SysRegs.SlaveTempsErrCount[12]=Slave12Regs.ErrorCount;
                   if(SysRegs.SlaveTempsErrCount[12]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS12=1;
                       SysRegs.SlaveTempsErrCount[12]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS12=0;
                   }
               }
               if(SysRegs.SlaveReadTempsEn.bit.SlaveBMS13==1)
               {
                   Slave13Regs.ID=BMS_ID_5;
                   Slave13Regs.SlaveCh=C_Slave_BCh;

                   Slave13Regs.BATICDO.bit.GPIO1=0;
                   SlaveBMSDigiteldoutOHandler(&Slave13Regs);
                   SysRegs.SlaveTempsErrCount[13]=Slave13Regs.ErrorCount;
                   if(SysRegs.SlaveTempsErrCount[13]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS13=1;
                       SysRegs.SlaveTempsErrCount[13]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS13=0;
                   }

                   SalveTempsHandler(&Slave13Regs);
                   SysRegs.SlaveTempsErrCount[12]=Slave13Regs.ErrorCount;
                   if(SysRegs.SlaveTempsErrCount[13]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS13=1;
                       SysRegs.SlaveTempsErrCount[13]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS13=0;
                   }
               }
               if(SysRegs.SlaveReadTempsEn.bit.SlaveBMS14==1)
               {
                   Slave14Regs.ID=BMS_ID_6;
                   Slave14Regs.SlaveCh=C_Slave_BCh;

                   Slave14Regs.BATICDO.bit.GPIO1=0;
                   SlaveBMSDigiteldoutOHandler(&Slave14Regs);
                   SysRegs.SlaveTempsErrCount[14]=Slave14Regs.ErrorCount;
                   if(SysRegs.SlaveTempsErrCount[14]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS14=1;
                       SysRegs.SlaveTempsErrCount[14]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS14=0;
                   }

                   SalveTempsHandler(&Slave14Regs);
                   SysRegs.SlaveTempsErrCount[14]=Slave14Regs.ErrorCount;
                   if(SysRegs.SlaveTempsErrCount[14]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS14=1;
                       SysRegs.SlaveTempsErrCount[14]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS14=0;
                   }
               }
               if(SysRegs.SlaveReadTempsEn.bit.SlaveBMS15==1)
               {
                   Slave15Regs.ID=BMS_ID_7;
                   Slave15Regs.SlaveCh=C_Slave_BCh;

                   Slave15Regs.BATICDO.bit.GPIO1=0;
                   SlaveBMSDigiteldoutOHandler(&Slave15Regs);
                   SysRegs.SlaveTempsErrCount[15]=Slave15Regs.ErrorCount;
                   if(SysRegs.SlaveTempsErrCount[15]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS15=1;
                       SysRegs.SlaveTempsErrCount[15]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS15=0;
                   }

                   SalveTempsHandler(&Slave15Regs);
                   SysRegs.SlaveTempsErrCount[15]=Slave15Regs.ErrorCount;
                   if(SysRegs.SlaveTempsErrCount[15]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS15=1;
                       SysRegs.SlaveTempsErrCount[15]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS15=0;
                   }
               }
               SysRegs.VoltTempsReadCount++;
               SysRegs.SysStateReg.bit.CellTempsOk=1;
           }
           SysRegs.CellTempssampling=0;

       }
       if(Slave0Regs.ErrorCount>200) {SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS00=1;}
       if(Slave1Regs.ErrorCount>200) {SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS01=1;}
       if(Slave2Regs.ErrorCount>200) {SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS02=1;}
       if(Slave3Regs.ErrorCount>200) {SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS03=1;}
       if(Slave4Regs.ErrorCount>200) {SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS04=1;}
       if(Slave5Regs.ErrorCount>200) {SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS05=1;}
       if(Slave6Regs.ErrorCount>200) {SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS06=1;}
       if(Slave7Regs.ErrorCount>200) {SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS07=1;}
       if(Slave8Regs.ErrorCount>200) {SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS07=1;}
       if(Slave9Regs.ErrorCount>200) {SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS09=1;}
       if(Slave10Regs.ErrorCount>200){SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS10=1;}
       if(Slave11Regs.ErrorCount>200){SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS11=1;}
       if(Slave12Regs.ErrorCount>200){SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS12=1;}
       if(Slave13Regs.ErrorCount>200){SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS13=1;}
       if(Slave14Regs.ErrorCount>200){SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS14=1;}
       if(Slave15Regs.ErrorCount>200){SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS15=1;}
       if(CANARegs.HMICMDRegs.bit.HMI_MODE==1)
       {

        }
        if(CANARegs.HMICMDRegs.bit.HMI_MODE==1)
        {

        }
        if(SysRegs.Maincount>3000){SysRegs.Maincount=0;}
        memcpy(&CANARegs.SysCellVoltageF[(C_SlaveMEAEa*0)],        &Slave1Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);
        memcpy(&CANARegs.SysCellVoltageF[(C_SlaveMEAEa*1)],        &Slave1Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);
        memcpy(&CANARegs.SysCellVoltageF[(C_SlaveMEAEa*2)],        &Slave2Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);
        memcpy(&CANARegs.SysCellVoltageF[(C_SlaveMEAEa*3)],        &Slave3Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);
        memcpy(&CANARegs.SysCellVoltageF[(C_SlaveMEAEa*4)],        &Slave4Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);
        memcpy(&CANARegs.SysCellVoltageF[(C_SlaveMEAEa*5)],        &Slave5Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);
        memcpy(&CANARegs.SysCellVoltageF[(C_SlaveMEAEa*6)],        &Slave6Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);
        memcpy(&CANARegs.SysCellVoltageF[(C_SlaveMEAEa*7)],        &Slave7Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);
        memcpy(&CANARegs.SysCellVoltageF[(C_SlaveMEAEa*8)],        &Slave9Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);
        memcpy(&CANARegs.SysCellVoltageF[(C_SlaveMEAEa*9)],        &Slave9Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);
        memcpy(&CANARegs.SysCellVoltageF[(C_SlaveMEAEa*10)],       &Slave10Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);
        memcpy(&CANARegs.SysCellVoltageF[(C_SlaveMEAEa*11)],       &Slave11Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);
        memcpy(&CANARegs.SysCellVoltageF[(C_SlaveMEAEa*12)],       &Slave12Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);
        memcpy(&CANARegs.SysCellVoltageF[(C_SlaveMEAEa*13)],       &Slave13Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);
        memcpy(&CANARegs.SysCellVoltageF[(C_SlaveMEAEa*14)],       &Slave14Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);
        memcpy(&CANARegs.SysCellVoltageF[(C_SlaveMEAEa*15)],       &Slave15Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);



        memcpy(&CANARegs.SysCelltemperatureF[(C_SlaveMEAEa*0)],    &Slave0Regs.CellTemperatureF[0],sizeof(float32)*C_SlaveMEAEa);
        memcpy(&CANARegs.SysCelltemperatureF[(C_SlaveMEAEa*1)],    &Slave1Regs.CellTemperatureF[0],sizeof(float32)*C_SlaveMEAEa);
        memcpy(&CANARegs.SysCelltemperatureF[(C_SlaveMEAEa*2)],    &Slave2Regs.CellTemperatureF[0],sizeof(float32)*C_SlaveMEAEa);
        memcpy(&CANARegs.SysCelltemperatureF[(C_SlaveMEAEa*3)],    &Slave3Regs.CellTemperatureF[0],sizeof(float32)*C_SlaveMEAEa);
        memcpy(&CANARegs.SysCelltemperatureF[(C_SlaveMEAEa*4)],    &Slave4Regs.CellTemperatureF[0],sizeof(float32)*C_SlaveMEAEa);
        memcpy(&CANARegs.SysCelltemperatureF[(C_SlaveMEAEa*5)],    &Slave5Regs.CellTemperatureF[0],sizeof(float32)*C_SlaveMEAEa);
        memcpy(&CANARegs.SysCelltemperatureF[(C_SlaveMEAEa*6)],    &Slave6Regs.CellTemperatureF[0],sizeof(float32)*C_SlaveMEAEa);
        memcpy(&CANARegs.SysCelltemperatureF[(C_SlaveMEAEa*7)],    &Slave7Regs.CellTemperatureF[0],sizeof(float32)*C_SlaveMEAEa);
        memcpy(&CANARegs.SysCelltemperatureF[(C_SlaveMEAEa*8)],    &Slave8Regs.CellTemperatureF[0],sizeof(float32)*C_SlaveMEAEa);
        memcpy(&CANARegs.SysCelltemperatureF[(C_SlaveMEAEa*9)],    &Slave9Regs.CellTemperatureF[0],sizeof(float32)*C_SlaveMEAEa);
        memcpy(&CANARegs.SysCelltemperatureF[(C_SlaveMEAEa*10)],   &Slave10Regs.CellTemperatureF[0],sizeof(float32)*C_SlaveMEAEa);
        memcpy(&CANARegs.SysCelltemperatureF[(C_SlaveMEAEa*11)],   &Slave11Regs.CellTemperatureF[0],sizeof(float32)*C_SlaveMEAEa);
        memcpy(&CANARegs.SysCelltemperatureF[(C_SlaveMEAEa*12)],   &Slave12Regs.CellTemperatureF[0],sizeof(float32)*C_SlaveMEAEa);
        memcpy(&CANARegs.SysCelltemperatureF[(C_SlaveMEAEa*13)],   &Slave13Regs.CellTemperatureF[0],sizeof(float32)*C_SlaveMEAEa);
        memcpy(&CANARegs.SysCelltemperatureF[(C_SlaveMEAEa*14)],   &Slave14Regs.CellTemperatureF[0],sizeof(float32)*C_SlaveMEAEa);
        memcpy(&CANARegs.SysCelltemperatureF[(C_SlaveMEAEa*15)],   &Slave15Regs.CellTemperatureF[0],sizeof(float32)*C_SlaveMEAEa);

    }

}

interrupt void cpu_timer0_isr(void)
{
   //LEDSysState_T;
   SysRegs.MainIsr1++;
   SysRegs.SysRegTimer5msecCount++;
   SysRegs.SysRegTimer10msecCount++;
   SysRegs.SysRegTimer50msecCount++;
   SysRegs.SysRegTimer100msecCount++;
   SysRegs.SysRegTimer300msecCount++;
   SysRegs.SysRegTimer500msecCount++;
   SysRegs.SysRegTimer1000msecCount++;
   SysRegs.CellVoltsampling++;
   SysRegs.CellTempssampling++;
   if(SysRegs.SysRegTimer5msecCount   >SysRegTimer5msec)    {SysRegs.SysRegTimer5msecCount=0;}
   if(SysRegs.SysRegTimer10msecCount  >SysRegTimer10msec)   {SysRegs.SysRegTimer10msecCount=0;}
   if(SysRegs.SysRegTimer50msecCount  >SysRegTimer50msec)   {SysRegs.SysRegTimer50msecCount=0;}
   if(SysRegs.SysRegTimer100msecCount >SysRegTimer100msec)  {SysRegs.SysRegTimer100msecCount=0;}
   if(SysRegs.SysRegTimer300msecCount >SysRegTimer300msec)   {SysRegs.SysRegTimer300msecCount=0;}
   if(SysRegs.SysRegTimer1000msecCount>SysRegTimer1000msec)  {SysRegs.SysRegTimer1000msecCount=0;}

  // SysRegs.VCUCANErrCheck++;
  // SysRegs.HMICANErrCheck++;
   //SysRegs.CTCANErrCheck++;

   // HMI_MODE = 0
   if(CANARegs.HMICMDRegs.bit.HMI_MODE==0)
   {
       if(CANARegs.PMSCMDRegs.bit.PrtctReset==1)
       {
        /*
           CANARegs.HMICMDRegs.all=0;
           CANARegs.PMSCMDRegs.all=0;
           SysRegs.SysAlarmReg.all=0;
           SysRegs.SysFaultReg.all=0;
           SysRegs.SysProtectReg.all=0;
           SysRegs.SysStateReg.all=0;
           SysTimerINIT(&SysRegs);
           SysVarINIT(&SysRegs);
           CANRegVarINIT(&CANARegs);
           ProtectRlyVarINIT(&PrtectRelayRegs);
           MDCalInit(&SysRegs);
           delay_ms(200);
           if(SysRegs.SysStateReg.bit.SysProtect==0)
           {
              // SysRegs.SysMachine=System_STATE_INIT;
           }
           */
       }
       if(SysRegs.SysStateReg.bit.SysAalarm==1)
       {
           SysRegs.SysStateReg.bit.SysStatus = 3;
           SysRegs.SysDigitalOutPutReg.bit.LEDAlarmOUT=1;
       }
       if(SysRegs.SysStateReg.bit.SysProtect==1)
       {
           SysRegs.SysStateReg.bit.SysStatus = 4;
           SysRegs.SysStateReg.bit.SysWakeUpEn=0;
           SysRegs.SysMachine=System_STATE_PROTECTER;
       }
   }
   // HMI_MODE = 1 이고, HMI에서 Reset
   if(CANARegs.HMICMDRegs.bit.HMI_MODE==1)
   {
       if(CANARegs.HMICMDRegs.bit.HMI_BSAReset==1)
       {
           CANARegs.HMICMDRegs.all=0;
           CANARegs.PMSCMDRegs.all=0;
           SysRegs.SysAlarmReg.all=0;
           SysRegs.SysFaultReg.all=0;
           SysRegs.SysProtectReg.all=0;
           SysRegs.SysStateReg.all=0;
           SysTimerINIT(&SysRegs);
           SysVarINIT(&SysRegs);
           CANRegVarINIT(&CANARegs);
           ProtectRlyVarINIT(&PrtectRelayRegs);
           MDCalInit(&SysRegs);
           delay_ms(200);
           if(SysRegs.SysStateReg.bit.SysProtect==0)
           {
               SysRegs.SysMachine=System_STATE_INIT;
           }
       }
       if(SysRegs.SysStateReg.bit.SysAalarm==1)
       {
           SysRegs.SysStateReg.bit.SysStatus = 3;
           SysRegs.SysDigitalOutPutReg.bit.LEDAlarmOUT=1;
       }
       if(SysRegs.SysStateReg.bit.SysProtect==1)
       {
           SysRegs.SysStateReg.bit.SysStatus = 4;
           SysRegs.SysStateReg.bit.SysWakeUpEn=0;
           SysRegs.SysMachine=System_STATE_PROTECTER;
       }
   }

   /*
    * DigitalInput detection
    */
   SysDigitalInput(&SysRegs);


  /*
   * current sensing detection
  */
    SysCalCurrentHandle(&SysRegs);

   /*
    *  Farasis52AhSocRegs.CellAgvVoltageF = NCMsocTestVoltAGV;
    *  Farasis52AhSocRegs.SysSoCCTF       = NUMsocTestVCT;
    *  Farasis52AhSocRegs.SysSoCCTAbsF    = NUMsocTestVCT;
    */
   Farasis52AhSocRegs.CellAgvVoltageF = SysRegs.SysCellAgvVoltageF;
   Farasis52AhSocRegs.SysSoCCTF       = SysRegs.SysPackCurrentF;
   Farasis52AhSocRegs.SysSoCCTAbsF    = SysRegs.SysPackCurrentAsbF;
   CalFarasis52AhSocHandle(&Farasis52AhSocRegs);

   if(Farasis52AhSocRegs.SoCStateRegs.bit.CalMeth==0)
   {
       SysRegs.SysSOCF=Farasis52AhSocRegs.SysSocInitF;
   }
   if(Farasis52AhSocRegs.SoCStateRegs.bit.CalMeth==1)
   {
       SysRegs.SysSOCF=Farasis52AhSocRegs.SysPackSOCF;
   }

   /*
    * Frey60AhSocRegs.CellAgvVoltageF = LFPsocTestVoltAGV;
    * Frey60AhSocRegs.SysSoCCTF       = LFPsocTestVCT;
    * Frey60AhSocRegs.SysSoCCTAbsF    = LFPsocTestVCT;
    */

   /*
    * Battery Alarm & Fault & Protect Check
    */
   //SysAlarmtCheck(&SysRegs);
   if((SysRegs.SysAlarmReg.all != 0)&&(SysRegs.SysStateReg.bit.INITOK==1))
   {
       SysRegs.SysStateReg.bit.SysAalarm=1;
   }
   else
   {
       SysRegs.SysStateReg.bit.SysAalarm=0;
   }
   //SysFaultCheck(&SysRegs);
   if((SysRegs.SysFaultReg.all != 0)&&(SysRegs.SysStateReg.bit.INITOK==1))
   {

       SysRegs.SysStateReg.bit.SysFault=1;
   }
   else
   {
       SysRegs.SysStateReg.bit.SysFault=0;
   }
   //SysProtectCheck(&SysRegs)
   if((SysRegs.SysProtectReg.all !=0)&&(SysRegs.SysStateReg.bit.INITOK==1))
   {
       SysRegs.SysStateReg.bit.SysProtect=1;
   }
   switch(SysRegs.SysRegTimer5msecCount)
   {
       case 1:
               SysRegs.SysStateReg.bit.SysProtectStatus=0;
               if(SysRegs.SysStateReg.bit.SysAalarm==1)
               {
                   SysRegs.SysStateReg.bit.SysProtectStatus=1;
                   SysRegs.SysDigitalOutPutReg.bit.LEDAlarmOUT=1;
               }
               if(SysRegs.SysStateReg.bit.SysProtect==1)
               {
                   SysRegs.SysStateReg.bit.SysProtectStatus=2;
                   SysRegs.SysDigitalOutPutReg.bit.LEDProtectOUT=1;
                   SysRegs.SysMachine=System_STATE_PROTECTER;
               }
       break;
       default :
       break;

   }
   switch(SysRegs.SysRegTimer10msecCount)
   {
       case 1:
               if(SysRegs.SysStateReg.bit.CellVoltOk==1)
               {
                   memcpy(&SysRegs.SysCellVoltageF[(C_SlaveMEAEa*0)],        &Slave1Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);
                   memcpy(&SysRegs.SysCellVoltageF[(C_SlaveMEAEa*1)],        &Slave1Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);
                   memcpy(&SysRegs.SysCellVoltageF[(C_SlaveMEAEa*2)],        &Slave2Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);
                   memcpy(&SysRegs.SysCellVoltageF[(C_SlaveMEAEa*3)],        &Slave3Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);
                   memcpy(&SysRegs.SysCellVoltageF[(C_SlaveMEAEa*4)],        &Slave4Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);
                   memcpy(&SysRegs.SysCellVoltageF[(C_SlaveMEAEa*5)],        &Slave5Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);
                   memcpy(&SysRegs.SysCellVoltageF[(C_SlaveMEAEa*6)],        &Slave6Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);
                   memcpy(&SysRegs.SysCellVoltageF[(C_SlaveMEAEa*7)],        &Slave7Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);
                   memcpy(&SysRegs.SysCellVoltageF[(C_SlaveMEAEa*8)],        &Slave9Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);
                   memcpy(&SysRegs.SysCellVoltageF[(C_SlaveMEAEa*9)],        &Slave9Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);
                   memcpy(&SysRegs.SysCellVoltageF[(C_SlaveMEAEa*10)],       &Slave10Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);
                   memcpy(&SysRegs.SysCellVoltageF[(C_SlaveMEAEa*11)],       &Slave11Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);
                   memcpy(&SysRegs.SysCellVoltageF[(C_SlaveMEAEa*12)],       &Slave12Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);
                   memcpy(&SysRegs.SysCellVoltageF[(C_SlaveMEAEa*13)],       &Slave13Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);
                   memcpy(&SysRegs.SysCellVoltageF[(C_SlaveMEAEa*14)],       &Slave14Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);
                   memcpy(&SysRegs.SysCellVoltageF[(C_SlaveMEAEa*15)],       &Slave15Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);
            //       SysRegs.SysCellVoltageF[175] = Slave15Regs.CellVoltageF[9];
                 //  memcpy(&SysRegs.SysCellVoltageF[(C_SlaveMEAEa*12)],       &Slave12Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);
                 //  memcpy(&SysRegs.SysCellVoltageF[(C_SlaveMEAEa*13)],       &Slave13Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);
                 //  memcpy(&SysRegs.SysCellVoltageF[(C_SlaveMEAEa*14)],       &Slave14Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);
                 //  memcpy(&SysRegs.SysCellVoltageF[(C_SlaveMEAEa*15)],       &Slave15Regs.CellVoltageF[0],sizeof(float32)*C_SlaveMEAEa);
                   SysCalVoltageHandle(&SysRegs);
                   SysRegs.SysStateReg.bit.CellVoltOk=0;
               }
       break;
       case 2:
               if(SysRegs.SysStateReg.bit.CellTempsOk==1)
               {
                   memcpy(&SysRegs.SysCelltemperatureF[(C_SlaveMEAEa*0)],    &Slave9Regs.CellTemperatureF[0],sizeof(float32)*C_SlaveMEAEa);
                   memcpy(&SysRegs.SysCelltemperatureF[(C_SlaveMEAEa*1)],    &Slave9Regs.CellTemperatureF[0],sizeof(float32)*C_SlaveMEAEa);
                   memcpy(&SysRegs.SysCelltemperatureF[(C_SlaveMEAEa*2)],    &Slave9Regs.CellTemperatureF[0],sizeof(float32)*C_SlaveMEAEa);
                   memcpy(&SysRegs.SysCelltemperatureF[(C_SlaveMEAEa*3)],    &Slave9Regs.CellTemperatureF[0],sizeof(float32)*C_SlaveMEAEa);
                   memcpy(&SysRegs.SysCelltemperatureF[(C_SlaveMEAEa*4)],    &Slave9Regs.CellTemperatureF[0],sizeof(float32)*C_SlaveMEAEa);
                   memcpy(&SysRegs.SysCelltemperatureF[(C_SlaveMEAEa*5)],    &Slave9Regs.CellTemperatureF[0],sizeof(float32)*C_SlaveMEAEa);
                   memcpy(&SysRegs.SysCelltemperatureF[(C_SlaveMEAEa*6)],    &Slave9Regs.CellTemperatureF[0],sizeof(float32)*C_SlaveMEAEa);
                   memcpy(&SysRegs.SysCelltemperatureF[(C_SlaveMEAEa*7)],    &Slave9Regs.CellTemperatureF[0],sizeof(float32)*C_SlaveMEAEa);
                   memcpy(&SysRegs.SysCelltemperatureF[(C_SlaveMEAEa*8)],    &Slave9Regs.CellTemperatureF[0],sizeof(float32)*C_SlaveMEAEa);
                   memcpy(&SysRegs.SysCelltemperatureF[(C_SlaveMEAEa*9)],    &Slave9Regs.CellTemperatureF[0],sizeof(float32)*C_SlaveMEAEa);
                   memcpy(&SysRegs.SysCelltemperatureF[(C_SlaveMEAEa*10)],   &Slave9Regs.CellTemperatureF[0],sizeof(float32)*C_SlaveMEAEa);
                   memcpy(&SysRegs.SysCelltemperatureF[(C_SlaveMEAEa*11)],   &Slave9Regs.CellTemperatureF[0],sizeof(float32)*C_SlaveMEAEa);
                   memcpy(&SysRegs.SysCelltemperatureF[(C_SlaveMEAEa*12)],   &Slave9Regs.CellTemperatureF[0],sizeof(float32)*C_SlaveMEAEa);
                   memcpy(&SysRegs.SysCelltemperatureF[(C_SlaveMEAEa*13)],   &Slave9Regs.CellTemperatureF[0],sizeof(float32)*C_SlaveMEAEa);
                   memcpy(&SysRegs.SysCelltemperatureF[(C_SlaveMEAEa*14)],   &Slave9Regs.CellTemperatureF[0],sizeof(float32)*C_SlaveMEAEa);
                   memcpy(&SysRegs.SysCelltemperatureF[(C_SlaveMEAEa*15)],   &Slave9Regs.CellTemperatureF[0],sizeof(float32)*C_SlaveMEAEa);
                   SysCalTemperatureHandle(&SysRegs);
                   SysRegs.SysStateReg.bit.CellTempsOk=0;
               }
       break;
       case 3:

       break;
       case 4:
               if(SysRegs.SysStateReg.bit.INITOK==1)
               {
                  // SysRegs.MDNumber=C_SysModuleEa;
                   MDCalVoltandTemsHandle(&SysRegs);
               }
       break;
       case 5:

               if(SysRegs.SysStateReg.bit.INITOK==1)
               {
                 //  SysCalTemperatureHandle(&SysRegs);
               }

       break;
       case 6:

       break;
       case 7:
               if(SysRegs.VoltTempsReadCount>=10)
               {
                   SysRegs.VoltTempsReadCount=100;
                   SysRegs.SysStateReg.bit.INITOK=1;
               }

       break;
       default :
       break;
   }
   switch(SysRegs.SysRegTimer50msecCount)
   {
       case 1:

       break;
       case 5:
              //LEDSysState_H;
              //LEDSysState_L;
       break;
       case 10:
               //LEDSysState_H;
               //At 80MHZ, operation time is 33usec
             //  Cal80VSysVoltageHandle(&SysRegs);
               //LEDSysState_L;
       break;
       case 20:
               //LEDSysState_H;
               //At 80MHZ, operation time is 33usec

               //LEDSysState_L;
       break;
       case 30 :


       break;
       default :
       break;
   }

   switch(SysRegs.SysRegTimer100msecCount)
   {
       case 5:
          //     memcpy(&CANARegs.Salve1VoltageCell[0], &Slave1Regs.CellVoltage[0],sizeof(unsigned int)*12);
       break;
       case 8:
               CANARegs.ProductInfro = ComBine(Product_Version,Product_Type);
               CANARegs.SysConFig    = ComBine(Product_SysCellVauleP,Product_SysCellVauleS);
             //  CANATX(0x610,8,CANARegs.ProductInfro,Product_Voltage,Product_Capacity,CANARegs.SysConFig);
               CANATX(0x610,8,CANARegs.ProductInfro,CANARegs.PMSCMDRegs.all,0x0000,CANARegs.SysConFig);
       break;
       case 10:
               SysRegs.SysSOHF=100.0;
             //  CANARegs.SysPackPT  = (unsigned int)(SysRegs.SysPackVoltageF*10);
               CANARegs.SysPackPT  = (unsigned int)(SysRegs.SysPackVoltageF*10);
               CANARegs.SysPackCT  = (int)(SysRegs.SysPackCurrentF*10);
               CANARegs.SysPackSOC = (int)(SysRegs.SysSOCF*10);
               CANARegs.SysPackSOH = (unsigned int)(SysRegs.SysSOHF*10);
               CANATX(0x611,8,CANARegs.SysPackPT,CANARegs.SysPackCT,CANARegs.SysPackSOC,CANARegs.SysPackSOH);
       break;
       case 12:

               SysRegs.SysAhF=Farasis52AhSocRegs.SysPackAhF;
              // CANARegs.SysState = ComBine(SysRegs.SysStateReg.bit.SysProtectStatus,SysRegs.SysStateReg.bit.SysStatus);
               CANARegs.SysState = SysRegs.SysStateReg.bit.SysStatus;
             //  CANARegs.SysStatus.bit.BalanceMode = SysRegs.SysStateReg.bit.SysBalanceMode;
               //CANARegs.SysStatus.bit.PoRly       = SysRegs.SysDigitalInputReg.bit.PAUX;
               //CANARegs.SysStatus.bit.NegRly      = SysRegs.SysDigitalInputReg.bit.NAUX;

               CANARegs.SysStatus.bit.PoRly       = SysRegs.SysDigitalInputReg.bit.PAUX;
               CANARegs.SysStatus.bit.NegRly      = SysRegs.SysDigitalInputReg.bit.NAUX;
               CANARegs.SysStatus.bit.PreCharRly  = SysRegs.SysDigitalOutPutReg.bit.ProRlyOUT;

               CANARegs.SysStatus.bit.MSDAux      = 0;
               CANARegs.SysPackAh                 =(int)(SysRegs.SysAhF*10);
               CANATX(0x612,8,CANARegs.SysState,CANARegs.SysStatus.all,0x0000, CANARegs.SysPackAh);

       break;
       case 14:

               CANATX(0x613,8,SysRegs.SysAlarmReg.Word.DataL,SysRegs.SysAlarmReg.Word.DataH,SysRegs.SysProtectReg.Word.DataL,SysRegs.SysProtectReg.Word.DataH);
       break;
       case 17:
               CANARegs.SysCHARGPWRContinty    = (Uint16)(SysRegs.SysCHARGPWRContintyF*10);
               CANARegs.SysDISCHAPWRContinty   = (Uint16)(SysRegs.SysDISCHAPWRContintyF*10);
               CANARegs.SysCHARGPWRPeak        = (Uint16)(SysRegs.SysCHARGPWRPeakF*10);
               CANARegs.SysDISCHAPWRPeak       = (Uint16)(SysRegs.SysDISCHAPWRPeakF*10);
               CANATX(0x614,8,CANARegs.SysCHARGPWRContinty,CANARegs.SysDISCHAPWRContinty,CANARegs.SysCHARGPWRPeak,CANARegs.SysDISCHAPWRPeak);
       break;
       case 20:
               CANARegs.CellVoltageMax          = (Uint16)(SysRegs.SysCellMaxVoltageF*1000);
               CANARegs.CellVoltageMin          = (Uint16)(SysRegs.SysCellMinVoltageF*1000);
               CANARegs.CellVoltageAgv          = (Uint16)(SysRegs.SysCellAgvVoltageF*1000);
               CANARegs.CellVoltageDiv          = (Uint16)(SysRegs.SysCellDivVoltageF*1000);
               CANATX(0x615,8,CANARegs.CellVoltageMax,CANARegs.CellVoltageMin,CANARegs.CellVoltageAgv,CANARegs.CellVoltageDiv);
       break;
       case 23:
               CANARegs.CellTemperaturelMAX    = (Uint16)(SysRegs.SysCellMaxTemperatureF*10);
               CANARegs.CellTemperaturelMIN    = (Uint16)(SysRegs.SysCellMinTemperatureF*10);
               CANARegs.CellTemperatureAVG     = (Uint16)(SysRegs.SysCellAgvTemperatureF*10);
               CANARegs.CellTemperatureDiv     = (Uint16)(SysRegs.SysCellDivTemperatureF*10);
               CANATX(0x616,8,CANARegs.CellTemperaturelMAX,CANARegs.CellTemperaturelMIN,CANARegs.CellTemperatureAVG,CANARegs.CellTemperatureDiv);
       break;
       case 26:
             //  CANARegs.CellVoltageMaxNum      = SysRegs.SysVoltageMaxNum;
             //  CANARegs.CellVoltageMinNum      = SysRegs.SysVoltageMinNum;
             //  CANARegs.CellTemperatureMaxNum  = SysRegs.SysTemperatureMaxNum;
             //  CANARegs.CellTemperatureMinNUM  = SysRegs.SysTemperatureMinNum;
               CANARegs.CellNum++;
               if(CANARegs.CellNum>175)
               {
                   CANARegs.CellNum=0;
               }
               CANARegs.SysCellVoltage[CANARegs.CellNum]= (Uint16)(SysRegs.SysCellVoltageF[CANARegs.CellNum]*1000); // Salve BMS 데이틀 가공한 데이터
               CANARegs.SysCelltemperature[CANARegs.CellNum]= (int16)(SysRegs.SysCelltemperatureF[CANARegs.CellNum]*10); // Salve BMS 데이틀 가공한 데이터
               CANARegs.SysCellIR[CANARegs.CellSlaveNumOr]=4;
               CANATX(0x617,8,CANARegs.CellNum,CANARegs.SysCellVoltage[CANARegs.CellNum],CANARegs.SysCelltemperature[CANARegs.CellNum],CANARegs.SysCellIR[CANARegs.CellNum]);
       break;
       case 30:
             //
               CANARegs.MoudleNum++;
               if(CANARegs.MoudleNum>7)
               {
                 CANARegs.MoudleNum=0;
               }
               CANARegs.MDVoltage[CANARegs.MoudleNum]         = (Uint16)(SysRegs.MDVoltageF[CANARegs.MoudleNum]*10);
               CANARegs.MDCellVoltAgv[CANARegs.MoudleNum]     = (Uint16)(SysRegs.MDCellVoltAgvF[CANARegs.MoudleNum]*1000);
               CANARegs.MDCellTempsAgv[CANARegs.MoudleNum]    = (Uint16)(SysRegs.MDCellTempsAgvF[CANARegs.MoudleNum]*10);
               CANATX(0x618,8,CANARegs.MoudleNum,CANARegs.MDVoltage[CANARegs.MoudleNum],CANARegs.MDCellVoltAgv[CANARegs.MoudleNum] ,CANARegs.MDCellTempsAgv[CANARegs.MoudleNum]);

       break;
       case 35:
               CANARegs.CellSlaveNumOr++;
               if(CANARegs.CellSlaveNumOr>175)
               {
                   CANARegs.CellSlaveNumOr=0;
               }
               CANARegs.SysCellVoltage[CANARegs.CellSlaveNumOr     ]= (Uint16)(CANARegs.SysCellVoltageF[CANARegs.CellSlaveNumOr]*1000); // Slave BMS 직접 가져 온 데이터
               CANARegs.SysCelltemperature[CANARegs.CellSlaveNumOr]= (int16)(CANARegs.SysCelltemperatureF[CANARegs.CellSlaveNumOr]*10); // Slave BMS 직접 가져 온 데이터
               CANARegs.SysCellIR[CANARegs.CellSlaveNumOr]=4;
               CANATX(0x619,8,CANARegs.CellNum,CANARegs.SysCellVoltage[CANARegs.CellSlaveNumOr],CANARegs.SysCelltemperature[CANARegs.CellSlaveNumOr],CANARegs.SysCellIR[CANARegs.CellSlaveNumOr]);
       break;
       case 40:
               CANARegs.SlaveNum++;
               if(CANARegs.SlaveNum>15)
               {
                   CANARegs.SlaveNum=0;
               }
               CANATX(0x61A,8,CANARegs.SlaveNum,SysRegs.SlaveVoltErrCount[CANARegs.SlaveNum],SysRegs.SlaveTempsErrCount[CANARegs.SlaveNum],CANARegs.MailBox0RxCount);
       break;
       case 45:
               CANATX(0x61B,8,CANARegs.MDVoltage[3],CANARegs.MDCellVoltAgv[3],CANARegs.MDCellTempsAgv[3],0x0000);
       break;
       case 50:
            //   CANARegs.MDVoltage[4]              = (Uint16)(SysRegs.MDVoltageF[4]*10);
            //   CANARegs.MDCellVoltAgv[4]          = (Uint16)(SysRegs.MDCellVoltAgvF[4]*1000);
            //   CANARegs.MDCellTempsAgv[4]         = (Uint16)(SysRegs.MDCellTempsAgvF[4]*10);
            //   CANATX(0x61C,8,CANARegs.MDVoltage[4],CANARegs.MDCellVoltAgv[4],CANARegs.MDCellTempsAgv[4],0x0000);
       break;
       case 55:
            //   CANARegs.MDVoltage[5]              = (Uint16)(SysRegs.MDVoltageF[5]*10);
            //   CANARegs.MDCellVoltAgv[5]          = (Uint16)(SysRegs.MDCellVoltAgvF[5]*1000);
            //   CANARegs.MDCellTempsAgv[5]         = (Uint16)(SysRegs.MDCellTempsAgvF[5]*10);
            //   CANATX(0x61D,8,CANARegs.MDVoltage[5],CANARegs.MDCellVoltAgv[5],CANARegs.MDCellTempsAgv[5],0x0000);
       break;
       case 60:
               if(CANARegs.HMICMDRegs.bit.HMI_MODE==1)
               {
                 //CANATX(0x617,8,SysRegs.SysStateReg.Word.DataL,SysRegs.SysStateReg.Word.DataH,0X000,0x0000);
               }
       break;
       case 65:
               if(CANARegs.HMICMDRegs.bit.HMI_MODE==1)
               {
                 //CANATX(0x618,8,0x0000,0x0000,0X000,0x0000);
               }
       break;
       case 70:
               if((CANARegs.HMICMDRegs.bit.HMI_MODE==1)&&(CANARegs.HMICMDRegs.bit.HMI_CellVoltReq==1))
               {
                 CANARegs.HMICellVoltCout++;
                 if(CANARegs.HMICellVoltCout>=C_HmiCellVoltCount)
                 {
                     CANARegs.HMICellVoltCout=0;
                 }
                 CANARegs.HMICellVoltNum=CANARegs.HMICellVoltCout*3;
                /* CANATX(0x619,8,CANARegs.HMICellVoltNum,
                                CANARegs.SysCellVoltage[CANARegs.HMICellVoltNum],
                                CANARegs.SysCellVoltage[CANARegs.HMICellVoltNum+1],
                                CANARegs.SysCellVoltage[CANARegs.HMICellVoltNum+2]);*/
               }
       break;
       case 75:
               if((CANARegs.HMICMDRegs.bit.HMI_MODE==1)&&(CANARegs.HMICMDRegs.bit.HMI_CellTempsReq==1))
               {
                   CANARegs.HMICellTempsCout++;
                   if(CANARegs.HMICellTempsCout>=C_HmiCellTempCount)
                   {
                       CANARegs.HMICellTempsCout=0;
                   }
                   CANARegs.HMICellTempsNum=CANARegs.HMICellTempsCout*3;
                   /*CANATX(0x61A,8,CANARegs.HMICellTempsNum,
                                  CANARegs.SysCelltemperature[CANARegs.HMICellTempsNum],
                                  CANARegs.SysCelltemperature[CANARegs.HMICellTempsNum+1],
                                  CANARegs.SysCelltemperature[CANARegs.HMICellTempsNum+2]);*/
               }
       break;
       case 80:
               if((CANARegs.HMICMDRegs.bit.HMI_MODE==1)&&(CANARegs.HMICMDRegs.bit.HMI_CANRXReq==1))
               {
                 //  CANATX(0x61B,8,CANARegs.MailBox0RxCount,CANARegs.MailBox1RxCount,CANARegs.MailBox2RxCount,0x000);
               }
       break;
       case 85:
               if((CANARegs.HMICMDRegs.bit.HMI_MODE==1)&&(CANARegs.HMICMDRegs.bit.HMI_ISOErrReq==1))
               {
                   CANARegs.HMIISOSPIErrCount++;
                   if(CANARegs.HMIISOSPIErrCount>=C_HMIISOSPIErrCount)
                   {
                       CANARegs.HMIISOSPIErrCount=0;
                   }
                   CANARegs.HMIISOSPIErrNum=CANARegs.HMIISOSPIErrCount*3;
                   CANATX(0x61C,8,CANARegs.HMIISOSPIErrNum, SysRegs.SlaveVoltErrCount[CANARegs.HMIISOSPIErrNum],
                                                        SysRegs.SlaveVoltErrCount[CANARegs.HMIISOSPIErrNum+1],
                                                        SysRegs.SlaveVoltErrCount[CANARegs.HMIISOSPIErrNum+2]);
               }
       break;
       default:
       break;
   }

   switch(SysRegs.SysRegTimer300msecCount)
   {
       case 1:

       break;
       case 2:

       break;
       default :
       break;
   }
   switch(SysRegs.SysRegTimer1000msecCount)
   {
       case 1:
       break;
       default :
       break;
   }
   /*
    *
    */
   SysRegs.SysStateReg.bit.HMICOMEnable=CANARegs.HMICMDRegs.bit.HMI_MODE;
   SysRegs.SysStateReg.bit.HMIBalanceMode=CANARegs.HMICMDRegs.bit.HMI_CellBalaEn;
   SysRegs.SysStateReg.bit.NRlyDOStatus=SysRegs.SysDigitalInputReg.bit.NAUX;
   SysRegs.SysStateReg.bit.PRlyDOStatus=SysRegs.SysDigitalInputReg.bit.PAUX;
   SysRegs.SysStateReg.bit.PreRlyDOStatus=PrtectRelayRegs.State.bit.ProRlyDI;
   SysRegs.SysStateReg.bit.SysRlyStatus = PrtectRelayRegs.StateMachine;
   SysRegs.SysStateReg.bit.SysSOCStatus = Farasis52AhSocRegs.state;

   /*
   PrtectRelayRegs.State.bit.NRlyDI=SysRegs.SysDigitalInputReg.bit.NAUX;
   PrtectRelayRegs.State.bit.PRlyDI=SysRegs.SysDigitalInputReg.bit.PAUX;
   ProtectRlySateCheck(&PrtectRelayRegs);
   SysRegs.SysDigitalOutPutReg.bit.NRlyOUT=PrtectRelayRegs.State.bit.NRlyDO;
   SysRegs.SysDigitalOutPutReg.bit.PRlyOUT=PrtectRelayRegs.State.bit.PRlyDO;
   SysRegs.SysDigitalOutPutReg.bit.ProRlyOUT=PrtectRelayRegs.State.bit.PreRlyDO;
   SysRegs.SysProtectReg.bit.PackRly_Err=PrtectRelayRegs.State.bit.RlyFaulttSate;
   SysRegs.SysDigitalOutPutReg.bit.PWRHold=1;
   */
  // SysRegs.SysStateReg.bit.SysProtect
   if(CANARegs.HMICMDRegs.bit.HMI_MODE==0)
   {
       PrtectRelayRegs.State.bit.NRlyDI       = SysRegs.SysDigitalInputReg.bit.NAUX;
       PrtectRelayRegs.State.bit.PRlyDI       = SysRegs.SysDigitalInputReg.bit.PAUX;
       PrtectRelayRegs.State.bit.WakeUpEN     = SysRegs.SysStateReg.bit.SysWakeUpEn;
       if(SysRegs.SysStateReg.bit.SysProtect==1)
       {
           CANARegs.PMSCMDRegs.bit.Pos_Rly=0;
           CANARegs.PMSCMDRegs.bit.Neg_Rly=0;
           CANARegs.PMSCMDRegs.bit.PreChar_Rly=0;
           SysRegs.SysStateReg.bit.SysWakeUpEn=0;
       }
       PrtectRelayRegs.State.bit.CANNRlyDO    = CANARegs.PMSCMDRegs.bit.Neg_Rly;
       PrtectRelayRegs.State.bit.CANPRlyDO    = CANARegs.PMSCMDRegs.bit.Pos_Rly;
       PrtectRelayRegs.State.bit.CANPreRlyDO  = CANARegs.PMSCMDRegs.bit.PreChar_Rly;

       //SysRegs.SysDigitalOutPutReg.bit.NRlyOUT=CANARegs.PMSCMDRegs.bit.Neg_Rly;
       //SysRegs.SysDigitalOutPutReg.bit.PRlyOUT=CANARegs.PMSCMDRegs.bit.Pos_Rly;
       //SysRegs.SysDigitalOutPutReg.bit.ProRlyOUT=CANARegs.PMSCMDRegs.bit.PreChar_Rly;


       ProtectRlyHandle(&PrtectRelayRegs);


       SysRegs.SysDigitalOutPutReg.bit.NRlyOUT=PrtectRelayRegs.State.bit.NRlyDO;
       SysRegs.SysDigitalOutPutReg.bit.PRlyOUT=PrtectRelayRegs.State.bit.PRlyDO;
       SysRegs.SysDigitalOutPutReg.bit.ProRlyOUT=PrtectRelayRegs.State.bit.PreRlyDO;
       SysRegs.SysProtectReg.bit.PackRlyTimerout=PrtectRelayRegs.State.bit.WakeupOnTiemrErr|PrtectRelayRegs.State.bit.WakeupOFFTiemrErr;

   }
   if(CANARegs.HMICMDRegs.bit.HMI_MODE==1)
   {
       PrtectRelayRegs.State.bit.NRlyDI       = SysRegs.SysDigitalInputReg.bit.NAUX;
       PrtectRelayRegs.State.bit.PRlyDI       = SysRegs.SysDigitalInputReg.bit.PAUX;
       PrtectRelayRegs.State.bit.WakeUpEN     = SysRegs.SysStateReg.bit.SysWakeUpEn;
       if(SysRegs.SysStateReg.bit.SysProtect==1)
       {
           CANARegs.HMICMDRegs.bit.HMI_BSANRly=0;
           CANARegs.HMICMDRegs.bit.HMI_BSAPRly=0;
           CANARegs.HMICMDRegs.bit.HMI_BSAPreRly=0;
           SysRegs.SysStateReg.bit.SysWakeUpEn=0;
       }//CANPRlyDO
       PrtectRelayRegs.State.bit.CANPRlyDO    = CANARegs.HMICMDRegs.bit.HMI_BSANRly;
       PrtectRelayRegs.State.bit.CANNRlyDO    = CANARegs.HMICMDRegs.bit.HMI_BSAPRly;
       PrtectRelayRegs.State.bit.CANPreRlyDO  = CANARegs.HMICMDRegs.bit.HMI_BSAPreRly;
       ProtectRlyHandle(&PrtectRelayRegs);
       SysRegs.SysDigitalOutPutReg.bit.NRlyOUT=PrtectRelayRegs.State.bit.NRlyDO;
       SysRegs.SysDigitalOutPutReg.bit.PRlyOUT=PrtectRelayRegs.State.bit.PRlyDO;
       SysRegs.SysDigitalOutPutReg.bit.ProRlyOUT=PrtectRelayRegs.State.bit.PreRlyDO;
      // SysRegs.SysProtectReg.bit.PackRlyTimerout=PrtectRelayRegs.State.bit.WakeupOnTiemrErr|PrtectRelayRegs.State.bit.WakeupOFFTiemrErr;
   }
/*
   if((SysRegs.SysStateReg.bit.SysProtect==1)||CANARegs.PMSCMDRegs.bit.PWRHOLD==1)
   {
       SysRegs.SysDigitalOutPutReg.bit.PWRHold=0;
   }
*/
   SysDigitalOutput(&SysRegs);
   // Acknowledge this interrupt to receive more interrupts from group 1

//   LEDSysState_L;
   if(SysRegs.MainIsr1>3000) {SysRegs.MainIsr1=0;}
//


   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
interrupt void ISR_CANRXINTA(void)
{
    struct ECAN_REGS ECanaShadow;
    if(ECanaRegs.CANGIF0.bit.GMIF0 == 1)
    {
        CANARegs.MailBoxRxCount++;
        if(CANARegs.MailBoxRxCount>300){CANARegs.MailBoxRxCount=0;LEDCANState_T;}
        if(ECanaRegs.CANRMP.bit.RMP0==1)
        {
            if(ECanaMboxes.MBOX0.MSGID.bit.STDMSGID==0x3C2)
            {
                SysRegs.CTCANErrCheck=0;
                CANARegs.MailBox0RxCount++;
                if(CANARegs.MailBox0RxCount>100){CANARegs.MailBox0RxCount=0;}
                SysRegs.SysCurrentData.byte.CurrentH   = (ECanaMboxes.MBOX0.MDL.byte.BYTE0<<8)|(ECanaMboxes.MBOX0.MDL.byte.BYTE1);
                SysRegs.SysCurrentData.byte.CurrentL   = (ECanaMboxes.MBOX0.MDL.byte.BYTE2<<8)|(ECanaMboxes.MBOX0.MDL.byte.BYTE3);
            }

        }
        if(ECanaRegs.CANRMP.bit.RMP1==1)
        {
            if(ECanaMboxes.MBOX1.MSGID.bit.STDMSGID==0x700)
            {
                SysRegs.VCUCANErrCheck=0;
                CANARegs.MailBox1RxCount++;
                if(CANARegs.MailBox1RxCount>100){CANARegs.MailBox1RxCount=0;}
                CANARegs.PMSCMDRegs.all      =  (ECanaMboxes.MBOX1.MDL.byte.BYTE1<<8)|(ECanaMboxes.MBOX1.MDL.byte.BYTE0);
             //   CANARegs.PMSCMDRegs.bit.Neg_Rly
            }
            if(CANARegs.PMSCMDRegs.bit.PrtctReset ==1)
            {
            //    CANARegs.PMSCMDRegs.all =0;
            //    CANARegs.PMSCMDRegs.bit.PrtctReset =1;
            }
        }
        if(ECanaRegs.CANRMP.bit.RMP2==1)
        {
            if(ECanaMboxes.MBOX2.MSGID.bit.STDMSGID==0x701)
            {
                SysRegs.HMICANErrCheck=0;
                CANARegs.MailBox2RxCount++;
                if(CANARegs.MailBox2RxCount>100){CANARegs.MailBox2RxCount=0;}
               // CANARegs.HMICMDRegs.all      =  (ECanaMboxes.MBOX2.MDL.byte.BYTE1<<8)|(ECanaMboxes.MBOX2.MDL.byte.BYTE0);
               // if(CANARegs.HMICMDRegs.bit.HMI_BSAReset==1)
               // {
                    CANARegs.HMICMDRegs.bit.HMI_RlyEN=0;
               // }

                //CANRXRegs.WORD700_1         =  (ECanaMboxes.MBOX2.MDL.byte.BYTE2<<8)|(ECanaMboxes.MBOX2.MDL.byte.BYTE3);
               //  CANARegs.HMICEllTempsAgv     =  (ECanaMboxes.MBOX2.MDH.byte.BYTE5<<8)|(ECanaMboxes.MBOX2.MDH.byte.BYTE4);
               //  CANARegs.HMICEllVoltMin      =  (ECanaMboxes.MBOX2.MDH.byte.BYTE7<<8)|(ECanaMboxes.MBOX2.MDH.byte.BYTE6);
            }
        }
       /*
         if(ECanaRegs.CANRMP.bit.RMP3==1)
        {
            if(ECanaMboxes.MBOX3.MSGID.bit.STDMSGID==0x400)
            {
                CANARegs.MailBox3RxCount++;
                if(CANARegs.MailBox3RxCount>3000){CANARegs.MailBox3RxCount=0;}
                //CANRXRegs.VCUCMDCount=0;
                //CANRXRegs.PCCMDRegs.all      =  (ECanaMboxes.MBOX3.MDL.byte.BYTE1<<8)|(ECanaMboxes.MBOX3.MDL.byte.BYTE0);
                //CANRXRegs.WORD700_1          =  (ECanaMboxes.MBOX3.MDL.byte.BYTE2<<8)|(ECanaMboxes.MBOX3.MDL.byte.BYTE3);
                //CANRXRegs.WORD700_2          =  (ECanaMboxes.MBOX3.MDH.byte.BYTE5<<8)|(ECanaMboxes.MBOX3.MDH.byte.BYTE4);
                //CANRXRegs.WORD700_3          =  (ECanaMboxes.MBOX3.MDH.byte.BYTE7<<8)|(ECanaMboxes.MBOX3.MDH.byte.BYTE6);
            }
        }
        */
    }
    ECanaShadow.CANRMP.all =ECanaRegs.CANRMP.all;
    ECanaShadow.CANRMP.all= 0;
    ECanaShadow.CANRMP.bit.RMP1 = 1;
    ECanaShadow.CANRMP.bit.RMP0 = 1;  //interrupt pending clear by writing 1
    ECanaShadow.CANRMP.bit.RMP2 = 1;
    ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all ;
 //   ECanaShadow.CANME.all=ECanaRegs.CANME.all;
 //   ECanaShadow.CANME.bit.ME0=1;    //0x5NA MCU Rx Enable
 //   ECanaShadow.CANME.bit.ME1=1;    //0x5NA MCU Rx Enable
 //   ECanaShadow.CANME.bit.ME2=1;    //0x5NB MCU Rx Enable
 //   ECanaShadow.CANME.bit.ME31=1;   //CAN-A Tx Enable
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;

   // IER |= 0x0100;                  // Enable INT9
   // EINT;

}//EOF
/*
interrupt void cpu_timer2_isr(void)
{  EALLOW;
   CpuTimer2.InterruptCount++;
   // The CPU acknowledges the interrupt.
  // A_OVCHACurrent;
   EDIS;
}
*/
