clc;
clear;
m_NIRunParameters.m_iRepetition=1;
m_NIRunParameters.m_iBscansPerCscan=100;
m_NIActiveParameters.m_dCscanDutyCycle=0.8;
m_NIActiveParameters.m_dBscanDutyCycle=0.8;
m_NIRunParameters.m_dFrameTriggerVpp=5;
m_NIRunParameters.m_dFastAxisUpperVol=3;
m_NIRunParameters.m_dFastAxisLowerVol=-3;
m_NIRunParameters.m_dSlowAxisUpperVol=3;
m_NIRunParameters.m_dSlowAxisLowerVol=-3;
m_NIActiveParameters.m_dFastAxisInitialLocation=0;
m_NIActiveParameters.m_dSlowAxisInitialLocation=0;
m_NIActiveParameters.m_iAnalogOutputSamplingRate=10^5;
m_NIActiveParameters.m_iLineRate=10^5;
m_NIRunParameters.m_iAscansPerBscan=500;
m_NIActiveParameters.NumFastAxisCycSlowAxisFalling=5;
m_NIActiveParameters.DelayFrameTrig=0;
m_NIActiveParameters.SmoothTri=false;
DualFrame=false;

Repetition = m_NIRunParameters.m_iRepetition;
Repetition2 = 1;
BscansPerCscan=m_NIRunParameters.m_iBscansPerCscan;
AscanPerBscanDutyCycle = 1;
%DutyCycle
CscanDutyCycle =  ( (m_NIActiveParameters.m_dCscanDutyCycle * 100)) / 100;
BscanDutyCycle =  ( (m_NIActiveParameters.m_dBscanDutyCycle * 100)) / 100;
FrameTriggerDutyCycle = 0.5;

%Voltage
FrameTriggerVppTrue = m_NIRunParameters.m_dFrameTriggerVpp;
FastAxisVppTrue = m_NIRunParameters.m_dFastAxisUpperVol - m_NIRunParameters.m_dFastAxisLowerVol;
FastAxisVppTrue2 = m_NIRunParameters.m_dSlowAxisUpperVol - m_NIRunParameters.m_dSlowAxisLowerVol;
SlowAxisVppTrue = m_NIRunParameters.m_dSlowAxisUpperVol - m_NIRunParameters.m_dSlowAxisLowerVol;

FastAxisOffsetTrue = (m_NIRunParameters.m_dFastAxisUpperVol + m_NIRunParameters.m_dFastAxisLowerVol) / 2;
FastAxisOffsetTrue2 = (m_NIRunParameters.m_dSlowAxisUpperVol + m_NIRunParameters.m_dSlowAxisLowerVol) / 2;
SlowAxisOffsetTrue = (m_NIRunParameters.m_dSlowAxisUpperVol + m_NIRunParameters.m_dSlowAxisLowerVol) / 2;

FrameTriggerVpp = 0;
FastAxisVpp = 0;
FastAxisVpp2 = 0;
SlowAxisVpp = 0;

FastAxisInitialLocation = m_NIActiveParameters.m_dFastAxisInitialLocation;
SlowAxisInitialLocation = m_NIActiveParameters.m_dSlowAxisInitialLocation;

%Lambda( Dots)

LineTriggerLambda = m_NIActiveParameters.m_iAnalogOutputSamplingRate / m_NIActiveParameters.m_iLineRate;
FastAxisLambda = LineTriggerLambda *(m_NIRunParameters.m_iAscansPerBscan/ AscanPerBscanDutyCycle / BscanDutyCycle);
FrameTriggerLambda = FastAxisLambda;

CounterFrame = 0;
CounterFast = 0;
CounterSlow = 0;
ContSlowAxisTrigFalling = m_NIActiveParameters.NumFastAxisCycSlowAxisFalling*FastAxisLambda;
SlowAxisLambda = FastAxisLambda * BscansPerCscan *Repetition + ContSlowAxisTrigFalling;
%DataBase
DataBase0D =  (5 * FastAxisLambda);
DataBase1D =  (5 * FastAxisLambda);
DataBase2D =  (3 * FastAxisLambda);
DataBase3D =  SlowAxisLambda;
DataBase4D =  (2.0 * FastAxisLambda * Repetition2);
DataBase5D =  (3 * FastAxisLambda + FastAxisLambda*BscanDutyCycle);

%Smooth Triangle wave
v = 0;
SinFreq = 10;
Value_1 = -1.0 / 8;
Theta_1 = acos(Value_1);
Theta_2 = acos(Value_1 / ((BscanDutyCycle - 1) / BscanDutyCycle));
Delta_D = FastAxisLambda / 10;
SmoothData = zeros(1,ceil(Delta_D));
ZeroData = zeros(1,ceil(Delta_D));
Sine_1 = sin(Theta_1);
Sine_2 = sin(Theta_2);
delta_Sine = sin(Theta_1) - sin(Theta_2);
Step = (Theta_2 - Theta_1) / (Delta_D - 1);
D_1 = (1 - BscanDutyCycle) *delta_Sine / (sin(Theta_1 + Step) - sin(Theta_1)) + BscanDutyCycle*(-Delta_D + FastAxisLambda);
SinM = (FastAxisVppTrue / FastAxisLambda / BscanDutyCycle) / (sin(Theta_1 + Step) - sin(Theta_1));
SinShift = D_1*(FastAxisVppTrue / FastAxisLambda / BscanDutyCycle) - sin(Theta_1)*SinM;
for w = 1 : length(SmoothData)
    SmoothData (w) = sin(Theta_1 + w*Step)*SinM + SinShift;
end
Stop=1;
OneD=2;
TwoD=3;
ThreeD=4;
Cross=5;
Wang=6;
%%
clc;
DataCounter = 1;

for k = 1:6

    switch k

        case Stop
            FrameTriggerVpp = 0;
            FastAxisVpp = 0;
            SlowAxisVpp = 0;
            FastAxisOffset = 0;
            SlowAxisOffset = 0;

            DataBase = DataBase0D;
            Data0D = zeros(1,3 * DataBase);
            Data = Data0D;
            Data2 = ZeroData;


        case OneD
            FrameTriggerVpp = FrameTriggerVppTrue;
            FastAxisVpp = 0;
            SlowAxisVpp = 0;

            FastAxisOffset = 0;
            SlowAxisOffset = 0;

            FastAxisInitialLocation = 0;
            SlowAxisInitialLocation = 0;

            DataBase = DataBase1D;
            Data1D = zeros(1,3 * DataBase);
            Data = Data1D;
            Data2 = ZeroData;


        case TwoD
            FrameTriggerVpp = FrameTriggerVppTrue;
            FastAxisVpp = FastAxisVppTrue;
            SlowAxisVpp = 0;

            FastAxisOffset = FastAxisOffsetTrue;
            SlowAxisOffset = SlowAxisOffsetTrue;

            FastAxisInitialLocation = 0;
            SlowAxisInitialLocation = 0;

            DataBase = DataBase2D;
            Data2D = zeros(1,3 * DataBase);
            Data = Data2D;
            Data2 = SmoothData;


        case ThreeD
            FrameTriggerVpp = FrameTriggerVppTrue;
            FastAxisVpp = FastAxisVppTrue;
            SlowAxisVpp = SlowAxisVppTrue;

            FastAxisOffset = FastAxisOffsetTrue;
            SlowAxisOffset = SlowAxisOffsetTrue;

            FastAxisInitialLocation = 0;
            SlowAxisInitialLocation = 0;

            DataBase = DataBase3D;
            Data3D = zeros(1,3 * DataBase);
            Data = Data3D;
            Data2 = SmoothData;


        case Cross
            FrameTriggerVpp = FrameTriggerVppTrue;
            FastAxisVpp = FastAxisVppTrue;
            FastAxisVpp2 = FastAxisVppTrue2;

            FastAxisOffset = FastAxisOffsetTrue;
            FastAxisOffset2 = FastAxisOffsetTrue2;

            FastAxisInitialLocation = 0;
            SlowAxisInitialLocation = 0;

            DataBase = DataBase4D;
            Data4D = zeros(1,3 * DataBase);
            Data = Data4D;
            Data2 = ZeroData;


        case Wang
            FrameTriggerVpp = FrameTriggerVppTrue;
            FastAxisVpp = FastAxisVppTrue;
            FastAxisVpp2 = FastAxisVppTrue2;

            FastAxisOffset = FastAxisOffsetTrue;
            FastAxisOffset2 = FastAxisOffsetTrue2;

            FastAxisInitialLocation = 0;
            SlowAxisInitialLocation = 0;


            DataBase = DataBase5D;
            Data5D = zeros(1,3 * DataBase);
            Data = Data5D;
            Data2 = ZeroData;

    end
    SlowAxisForward = SlowAxisLambda - ContSlowAxisTrigFalling;
    FastAxisForward = FastAxisLambda * BscanDutyCycle;
    FrameAxisForward = FastAxisLambda*0.5;

    FastAxisForward_s = FastAxisLambda*(Repetition - 1) + FastAxisLambda*(0.5 + BscanDutyCycle / 2);
    FastAxisBackward = FastAxisLambda*(1 - BscanDutyCycle);

    FrameTrigLambdaRem_Frame = 0;																							%20190215
    SlowAxisLambdaRem_Frame = 0;
    FastAxisLambdaRem_Fast = 0;
    SlowAxisLambdaRem_Fast = 0;
    FastAxisLambdaRem_Slow = 0;
    SlowAxisLambdaRem_Slow = 0;																								%20190215
    FastAxisLambdaRem = 0;
    FastAxisWangLambdaRem = 0;

    if k == Stop || k == OneD || k == TwoD || k == ThreeD

        u = 0;
        for i = 0:DataBase-1


            CounterFrame = i - m_NIActiveParameters.DelayFrameTrig;
            CounterSlow = i + FastAxisLambda / 4 * (1 - BscanDutyCycle);


            CounterFast = i;
            %CounterSlow = i + FastAxisLambda / 4 * (1 - BscanDutyCycle) - FastAxisLambda*(1 - AscanPerBscanDutyCycle) / 4;



            if CounterFrame >= DataBase
                CounterFrame = CounterFrame - DataBase;
            elseif CounterFrame < 0
                CounterFrame = CounterFrame + DataBase;
            end

            if CounterFast >= DataBase
                CounterFast = CounterFast - DataBase;
            elseif CounterFast < 0
                CounterFast = CounterFast + DataBase;
            end

            if CounterSlow >= DataBase
                CounterSlow =CounterSlow - DataBase;
            elseif CounterSlow < 0
                CounterSlow =CounterSlow + DataBase;
            end

            if DataCounter > 3 * DataBase
                DataCounter = 1;
            end
            FrameTrigLambdaRem_Frame = mod(CounterFrame,  FrameTriggerLambda);
            SlowAxisLambdaRem_Frame = mod(CounterFrame, SlowAxisLambda);

            FastAxisLambdaRem_Fast = mod(CounterFast, FastAxisLambda);
            SlowAxisLambdaRem_Fast = mod(CounterFast, SlowAxisLambda);

            FastAxisLambdaRem_Slow = mod(CounterSlow, FastAxisLambda*Repetition);
            SlowAxisLambdaRem_Slow = mod(CounterSlow, SlowAxisLambda);					%Chamber_20190215_Signal Delay

            if w >= Delta_D
                w = 0;
            end
            if v >= Delta_D
                v = 0;
            end

            %/FastAxis

            if (FastAxisLambdaRem_Fast >= D_1 && mod(i,  FastAxisLambda) < D_1 + Delta_D && m_NIActiveParameters.SmoothTri)
                Data (DataCounter) = Data2 (w) - FastAxisVpp / 2 ;
                w=w+1;
                DataCounter=DataCounter+1;
            elseif (FastAxisLambdaRem_Fast <= FastAxisForward - D_1 || FastAxisLambdaRem_Fast > FastAxisLambda*(1 + BscanDutyCycle) - Delta_D - D_1) && m_NIActiveParameters.SmoothTri

                u = FastAxisForward - D_1 - v  ;
                v=v+1;
                if u >= 0

                    Data (DataCounter) = -Data2 (u) + FastAxisVpp / 2 ;
                    DataCounter=DataCounter+1;
                else
                    Data (DataCounter) = -Data2 (u +  Delta_D) + FastAxisVpp / 2 ;
                    DataCounter=DataCounter+1;
                end
            elseif (FastAxisLambdaRem_Fast < FastAxisForward)
                %if (FastAxisLambdaRem_Fast < FastAxisForward)
                Data (DataCounter) = FastAxisVpp * FastAxisLambdaRem_Fast / FastAxisForward - FastAxisVpp / 2;
                DataCounter=DataCounter+1;
            else
                Data (DataCounter) = FastAxisVpp * (FastAxisLambda - FastAxisLambdaRem_Fast) / (FastAxisLambda - FastAxisForward) - FastAxisVpp / 2 ;
                DataCounter=DataCounter+1;
            end
            %             if (m_NIRunParameters.m_bBFrame == true)
            %                 Data (DataCounter - 1) = -Data(DataCounter - 1);
            %             end
            Data (DataCounter - 1) = Data (DataCounter - 1)+FastAxisOffset + FastAxisInitialLocation;

            %/SlowAxis
            if SlowAxisLambdaRem_Slow < SlowAxisForward
                c=floor(CounterSlow./FastAxisLambda*Repetition);
                if (FastAxisLambdaRem_Slow < FastAxisForward_s || (c) == (BscansPerCscan - 1))

                    %Data (DataCounter) = SlowAxisVpp*(mod(CounterSlow,  FastAxisLambda*Repetition).quot) / (BscansPerCscan - 1) - SlowAxisVpp / 2 + SlowAxisOffset + SlowAxisInitialLocation;
                    %if (mod(CounterSlow,  FastAxisLambda) > FastAxisLambda*BscanDutyCycle && DualFrame)
                    %	Data (DataCounter) += SlowAxisVpp / (BscansPerCscan - 1) / 2;
                    %DataCounter  ;
                    Data (DataCounter) = SlowAxisVpp*(c) / (BscansPerCscan - 1) - SlowAxisVpp / 2 + SlowAxisOffset + SlowAxisInitialLocation;
                    DataCounter=DataCounter+1;
                else
                    Data (DataCounter) = SlowAxisVpp / (BscansPerCscan - 1)*(FastAxisLambdaRem_Slow - FastAxisForward_s) / (FastAxisLambda*Repetition - FastAxisForward_s)+ SlowAxisVpp*(c) / (BscansPerCscan - 1) - SlowAxisVpp / 2 + SlowAxisOffset + SlowAxisInitialLocation;
                    DataCounter=DataCounter+1;

                end
            else
                Data (DataCounter) = SlowAxisVpp*(SlowAxisLambda - SlowAxisLambdaRem_Slow) / ContSlowAxisTrigFalling - SlowAxisVpp / 2 + SlowAxisOffset + SlowAxisInitialLocation;
                DataCounter=DataCounter+1;

            end

            %/FrameTrig
            if SlowAxisLambdaRem_Frame < SlowAxisForward || k == OneD || k == TwoD

                if FrameTrigLambdaRem_Frame < FrameAxisForward*0.5
                    Data (DataCounter) = FrameTriggerVpp;
                    DataCounter=DataCounter+1;

                elseif FrameTrigLambdaRem_Frame >= FrameAxisForward && FrameTrigLambdaRem_Frame < FrameAxisForward + FastAxisBackward*0.5&&DualFrame ==true
                    Data (DataCounter) = FrameTriggerVpp;
                    DataCounter=DataCounter+1;
                else
                    Data (DataCounter) = 0;
                    DataCounter=DataCounter+1;
                end
            else
                Data (DataCounter) = 0;
                DataCounter=DataCounter+1;


            end
        end
    elseif (k == Cross)

        for   i = 0: DataBase-1
            if i==DataBase
                disp('hello');
            end
            if i==1
                disp('hello');
            end
            CounterFrame = i - m_NIActiveParameters.DelayFrameTrig;
            if CounterFrame >= DataBase
                CounterFrame = CounterFrame-DataBase;
            elseif CounterFrame < 0
                CounterFrame = CounterFrame+DataBase;
            end
            FrameTrigLambdaRem_Frame = mod(CounterFrame,  FrameTriggerLambda);


            %Bscan(channel 1)
            if floor(i/FastAxisLambda) < (Repetition2 - 1)
                Data (DataCounter ) = FastAxisOffset;
                DataCounter=DataCounter+1;

            elseif floor(i/FastAxisLambda) == (Repetition2 - 1)

                if mod(i,  FastAxisLambda) < (FastAxisLambda*BscanDutyCycle)
                    Data (DataCounter ) = FastAxisOffset;
                    DataCounter=DataCounter+1;
                else
                    Data (DataCounter ) = (0 - (-FastAxisVpp / 2))*(FastAxisLambda - mod(i,  FastAxisLambda)) / (FastAxisLambda * (1 - BscanDutyCycle)) - FastAxisVpp / 2 + FastAxisOffset;
                    DataCounter=DataCounter+1;
                end
            elseif (Repetition2 <= floor(i/FastAxisLambda)  && floor(i/FastAxisLambda) < (2 * Repetition2 - 1) && Repetition2 ~= 1)

                if (mod(i,  FastAxisLambda) < (FastAxisLambda*BscanDutyCycle))
                    Data (DataCounter ) = FastAxisVpp*mod(i,  FastAxisLambda) / FastAxisLambda / BscanDutyCycle - FastAxisVpp / 2 + FastAxisOffset;
                    DataCounter=DataCounter+1;
                else
                    Data (DataCounter ) = FastAxisVpp*(FastAxisLambda - mod(i,  FastAxisLambda)) / FastAxisLambda / (1 - BscanDutyCycle) - FastAxisVpp / 2 + FastAxisOffset;
                    DataCounter=DataCounter+1;
                end
            elseif (floor(i/FastAxisLambda) == (2 * Repetition2 - 1))

                if (mod(i,  FastAxisLambda) < (FastAxisLambda*BscanDutyCycle))
                    Data (DataCounter ) = FastAxisVpp*mod(i,  FastAxisLambda) / (FastAxisLambda *  BscanDutyCycle) - FastAxisVpp / 2 + FastAxisOffset;
                    DataCounter=DataCounter+1;
                else
                    Data (DataCounter ) = ((FastAxisVpp / 2) - 0)*(FastAxisLambda - mod(i,  FastAxisLambda)) / (FastAxisLambda * (1 - BscanDutyCycle)) + FastAxisOffset;
                    DataCounter=DataCounter+1;
                end
            end
            %Bscan(channel 2)
            if (floor(i/FastAxisLambda) < (Repetition2 - 1))

                if (mod(i,  FastAxisLambda) < (FastAxisLambda*BscanDutyCycle))
                    Data (DataCounter ) = FastAxisVpp2*mod(i,  FastAxisLambda) / FastAxisLambda / BscanDutyCycle - FastAxisVpp2 / 2 + FastAxisOffset2;
                    DataCounter=DataCounter+1;
                else
                    Data (DataCounter ) = FastAxisVpp2*(FastAxisLambda - mod(i,  FastAxisLambda)) / FastAxisLambda / (1 - BscanDutyCycle) - FastAxisVpp2 / 2 + FastAxisOffset2;
                    DataCounter=DataCounter+1;
                end

            elseif (floor(i/FastAxisLambda) == (Repetition2 - 1))

                if (mod(i,  FastAxisLambda) < (FastAxisLambda*  BscanDutyCycle))
                    Data (DataCounter ) = FastAxisVpp2*mod(i,  FastAxisLambda) / (FastAxisLambda *  BscanDutyCycle) - FastAxisVpp2 / 2 + FastAxisOffset2;
                    DataCounter=DataCounter+1;
                else
                    Data (DataCounter ) = ((FastAxisVpp2 / 2) - 0)*(FastAxisLambda - mod(i,  FastAxisLambda)) / (FastAxisLambda * (1 - BscanDutyCycle)) + +FastAxisOffset2;
                    DataCounter=DataCounter+1;
                end
            elseif (Repetition2 <= floor(i/FastAxisLambda)  && floor(i/FastAxisLambda) < (2 * Repetition2 - 1) && Repetition2 ~= 1)



                Data (DataCounter ) = FastAxisOffset2;
                DataCounter=DataCounter+1;


            elseif (floor(i/FastAxisLambda) == (2 * Repetition2 - 1))

                if (mod(i,  FastAxisLambda) < (FastAxisLambda *  BscanDutyCycle))
                    Data (DataCounter ) = FastAxisOffset2;
                    DataCounter=DataCounter+1;
                else
                    Data (DataCounter ) = (0 - (-FastAxisVpp2 / 2))*(FastAxisLambda - mod(i,  FastAxisLambda)) / (FastAxisLambda * (1 - BscanDutyCycle)) - FastAxisVpp2 / 2 + FastAxisOffset2;
                    DataCounter=DataCounter+1;
                end
            end
            %Trig(channel 3)
            if (floor(CounterFrame/FastAxisLambda) < (Repetition2 - 1))

                if (FrameTrigLambdaRem_Frame < FrameAxisForward)
                    Data (DataCounter ) = FrameTriggerVpp;
                    DataCounter=DataCounter+1;
                else
                    Data (DataCounter ) = 0;
                    DataCounter=DataCounter+1;
                end

            elseif (floor(CounterFrame/FastAxisLambda) == (Repetition2 - 1))

                if (FrameTrigLambdaRem_Frame < FrameAxisForward)
                    Data (DataCounter ) = FrameTriggerVpp;
                    DataCounter=DataCounter+1;
                else
                    Data (DataCounter ) = 0;
                    DataCounter=DataCounter+1;
                end
            elseif (Repetition2 <= floor(CounterFrame/FastAxisLambda)  && floor(CounterFrame/FastAxisLambda) < (2 * Repetition2 - 1) && Repetition2 ~= 1)


                if (FrameTrigLambdaRem_Frame < FrameAxisForward)
                    Data (DataCounter ) = FrameTriggerVpp;																													%/
                    DataCounter=DataCounter+1;
                else
                    Data (DataCounter ) = 0;
                    DataCounter=DataCounter+1;
                end

            elseif (floor(CounterFrame/FastAxisLambda) == (2 * Repetition2 - 1))

                if (FrameTrigLambdaRem_Frame < FrameAxisForward)
                    Data (DataCounter ) = FrameTriggerVpp;																													%/
                    DataCounter=DataCounter+1;
                else
                    Data (DataCounter ) = 0;
                    DataCounter=DataCounter+1;
                end
            end

        end

    elseif (k == Wang)

        for i =0: DataBase-1

            CounterFrame = i - m_NIActiveParameters.DelayFrameTrig;
            if (CounterFrame >= DataBase)
                CounterFrame = CounterFrame-DataBase;
            elseif (CounterFrame < 0)
                CounterFrame = CounterFrame+DataBase;
            end

            %Bscan(channel 1)
            FastAxisWangLambdaRem = mod(i, (FastAxisLambda * 3 + FastAxisForward));
            FastAxisLambdaRem = mod(mod(i, (FastAxisLambda * 3 + FastAxisForward)), FastAxisLambda);
            if (mod(i, FastAxisLambda * 3 + FastAxisForward) > (FastAxisLambda * 2 + FastAxisForward))

                if ((mod(i, (FastAxisLambda * 3 + FastAxisForward)) - (FastAxisLambda * 2 + FastAxisForward)) > ((FastAxisLambda - FastAxisForward) / 2) && (mod(i, (FastAxisLambda * 3 + FastAxisForward)) - (FastAxisLambda * 2 + FastAxisForward)) <= (FastAxisForward + (FastAxisLambda - FastAxisForward) / 2))
                    Data (DataCounter) = FastAxisOffset;
                    DataCounter=DataCounter+1;
                elseif ((mod(i, (FastAxisLambda * 3 + FastAxisForward)) - (FastAxisLambda * 2 + FastAxisForward)) <= ((FastAxisLambda - FastAxisForward) / 2))
                    Data (DataCounter) = FastAxisVpp * (FastAxisLambda - (mod(i, (FastAxisLambda * 3 + FastAxisForward)) - FastAxisLambda * 2)) / (FastAxisLambda - FastAxisForward) - FastAxisVpp2 / 2 + FastAxisOffset;
                    DataCounter=DataCounter+1;
                else
                    Data (DataCounter) = FastAxisVpp * (FastAxisLambda - mod(FastAxisWangLambdaRem - FastAxisForward,  FastAxisLambda)) / (FastAxisLambda - FastAxisForward) - FastAxisVpp / 2 + FastAxisOffset;
                    DataCounter=DataCounter+1;
                end
            elseif (FastAxisLambdaRem <= FastAxisForward)
                Data (DataCounter) = FastAxisVpp*FastAxisLambdaRem / FastAxisForward - FastAxisVpp / 2 + FastAxisOffset;
                DataCounter=DataCounter+1;
            else
                Data (DataCounter) = FastAxisVpp*(FastAxisLambda - FastAxisLambdaRem) / (FastAxisLambda - FastAxisForward) - FastAxisVpp / 2 + FastAxisOffset;
                DataCounter=DataCounter+1;
            end

            %Bscan(channel 2)
            if (mod(i, FastAxisLambda * 3 + FastAxisForward) > (FastAxisLambda * 2 + FastAxisForward))
                if ((mod(i, FastAxisLambda * 3 + FastAxisForward)  - (FastAxisLambda * 2 + FastAxisForward)) > ((FastAxisLambda - FastAxisForward) / 2) && (mod(i, FastAxisLambda * 3 + FastAxisForward)  - (FastAxisLambda * 2 + FastAxisForward)) <= (FastAxisForward + (FastAxisLambda - FastAxisForward) / 2))
                    Data( DataCounter) = FastAxisVpp2 * (mod(i, FastAxisLambda * 3 + FastAxisForward)  - (FastAxisLambda * 2 + FastAxisForward) - ((FastAxisLambda - FastAxisForward) / 2)) / FastAxisForward - FastAxisVpp2 / 2 + FastAxisOffset2;
                    DataCounter=DataCounter+1;
                elseif ((mod(i, FastAxisLambda * 3 + FastAxisForward)  - (FastAxisLambda * 2 + FastAxisForward)) <= ((FastAxisLambda - FastAxisForward) / 2))
                    Data( DataCounter) = -FastAxisVpp2 / 2 + FastAxisOffset2;
                    DataCounter=DataCounter+1;
                else
                    Data( DataCounter) = FastAxisVpp2 - FastAxisVpp2 / 2 + FastAxisOffset2;
                    DataCounter=DataCounter+1;
                end
            elseif (FastAxisLambdaRem <= FastAxisForward)
                Data( DataCounter) = FastAxisVpp2 / 2 * (2 - floor((mod(i, (FastAxisLambda * 3 + FastAxisForward))  / FastAxisLambda))) - FastAxisVpp2 / 2 + FastAxisOffset2;
                DataCounter=DataCounter+1;
            else
                Data( DataCounter) = FastAxisVpp2/2 * (FastAxisLambda - FastAxisLambdaRem) / (FastAxisLambda - FastAxisForward) + FastAxisVpp2/2 * (1 - floor(mod(i, FastAxisLambda * 3 + FastAxisForward)  / FastAxisLambda)) - FastAxisVpp2 / 2 + FastAxisOffset2;
                DataCounter=DataCounter+1;
                %//if (Data( DataCounter - 1) > 10)
            end



            %Trig(channel 3)
            if (mod(CounterFrame, FastAxisLambda * 3 + FastAxisForward) > (FastAxisLambda * 2 + FastAxisForward))

                if ((mod(CounterFrame, FastAxisLambda * 3 + FastAxisForward) - (FastAxisLambda * 2 + FastAxisForward)) > ((FastAxisLambda - FastAxisForward) / 2) && (mod(CounterFrame, FastAxisLambda * 3 + FastAxisForward) - (FastAxisLambda * 2 + FastAxisForward)) <= (FastAxisForward + (FastAxisLambda - FastAxisForward) / 2))
                    Data (DataCounter) = FrameTriggerVpp;
                    DataCounter=DataCounter+1;
                elseif ((mod(CounterFrame, FastAxisLambda * 3 + FastAxisForward) - (FastAxisLambda * 2 + FastAxisForward)) <= ((FastAxisLambda - FastAxisForward) / 2))
                    Data (DataCounter) = 0;
                    DataCounter=DataCounter+1;
                else
                    Data (DataCounter) = 0;
                    DataCounter=DataCounter+1;
                end

            elseif FastAxisLambdaRem <= FastAxisForward
                Data (DataCounter) = FrameTriggerVpp;
                DataCounter=DataCounter+1;
            else
                Data (DataCounter) = 0;
                DataCounter=DataCounter+1;
            end


        end
    end
    switch k

        case Stop
            Data0D=Data;
        case OneD
            Data1D=Data;
        case TwoD
            Data2D=Data;
        case ThreeD
            Data3D=Data;
        case Cross
            Data4D=Data;
        case Wang
            Data5D=Data;
    end
    DataCounter = 1;
    w = 0;
    v = 0;
end




%%
for i=1:6
    switch i

        case Stop
            Data=Data0D;
        case OneD
            Data=Data1D;
        case TwoD
            Data=Data2D;
        case ThreeD
            Data=Data3D;
        case Cross
            Data=Data4D;
        case Wang
            Data=Data5D;
    end
    figure(i)
    subplot(3,1,1)
    plot(Data(1:3:end))
    subplot(3,1,2)
    plot(Data(2:3:end))
    subplot(3,1,3)
    plot(Data(3:3:end))
end

