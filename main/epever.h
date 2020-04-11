#include "mbcontroller.h"


typedef struct 
{
    uint16_t ChargingRatedInputVoltage;
    uint16_t ChargingRatedInputCurrent;
    uint32_t ChargingRatedInputPower;

    uint16_t ChargingRatedOutputVoltage;
    uint16_t ChargingRatedOutputCurrent;
    uint32_t ChargingRatedOutputPower;

    uint16_t ChargingMode;
    uint16_t OutputRatedLoadCurrent;

} epever_rated_t;

typedef struct 
{
    uint16_t ChargingInputVoltage;
    uint16_t ChargingInputCurrent;
    uint32_t ChargingInputPower;

    uint16_t ChargingOutputVoltage;
    uint16_t ChargingOutputCurrent;
    uint32_t ChargingOutputPower;

    uint16_t DischargingOutputVoltage;
    uint16_t DischargingOutputCurrent;
    uint32_t DischargingOutputPower;

    uint16_t BatteryTemperature;
    uint16_t ChargerCaseTemperature;
    uint16_t ChargerSiliconTemperature;

    uint16_t BatterySOC;
    uint16_t RemoteBatteryTemperature;
    uint16_t BatteryRealRatedVoltage;

    uint16_t BatteryStatus;
    uint16_t ChargerStatus;
} epever_realtime_t;

typedef struct 
{
    uint16_t TodayMaxInputVoltage;
    uint16_t TodayMinInputVoltage;
    uint16_t TodayMaxBatteryVoltage;
    uint16_t TodayMinBatteryVoltage;

    uint32_t TodayConsumedEnergy;
    uint32_t ThisMonthConsumedEnergy;
    uint32_t ThisYearConsumedEnergy;
    uint32_t TotalConsumedEnergy;

    uint32_t TodayGeneratedEnergy;
    uint32_t ThisMonthGeneratedEnergy;
    uint32_t ThisYearGeneratedEnergy;
    uint32_t TotalGeneratedEnergy;

    uint32_t CarbonDioxideReduction;

    uint32_t BatteryCurrent;

    uint16_t BatteryTemperature;
    uint16_t AmbientTemperature;

} epever_statistical_t;

typedef struct
{
    uint16_t BatteryType;
    uint16_t BatteryCapacity;
    uint16_t TemperatureCompensationCoefficient;
    uint16_t HighVoltDisconnect;
    uint16_t ChargingLimitVoltage;
    uint16_t OverVoltageReconnect;
    uint16_t EqualizationVoltage;
    uint16_t BoostVoltage;
    uint16_t FloatVoltage;
    uint16_t BoostReconnectVoltage;
    uint16_t LowVoltageReconnect;
    uint16_t UnderVoltageRecover;
    uint16_t UnderVoltageWarning;
    uint16_t LowVoltageDisconnect;
    uint16_t DischargingLimitVoltage;
    uint16_t RealTimeClock1;
    uint16_t RealTimeClock2;
    uint16_t RealTimeClock3;
    uint16_t EqualizationChargingCycle;
    uint16_t BatteryTemperatureWarningUpperLimit;
    uint16_t BatteryTemperatureWarningLowerLimit;
    uint16_t ControllerInnerTemperatureUpperLimit;
    uint16_t ControllerInnerTemperatureUpperLimitRecover;
    uint16_t PowerComponentTemperatureUpperLimit;
    uint16_t PowerComponentTemperatureUpperLimitRecover;
    uint16_t LineImpedance;
    uint16_t NightTimeThresholdVolt;
    uint16_t LightSignalStartupDelayTime;
    uint16_t DayTimeThresholdVolt;
    uint16_t LightSignalTurnOffDelayTime;
    uint16_t LoadControlingMode;
    uint16_t WorkingTimeLength1;
    uint16_t WorkingTimeLength2;
    uint16_t TurnOnTiming1Second;
    uint16_t TurnOnTiming1Minute;
    uint16_t TurnOnTiming1Hour;
    uint16_t TurnOffTiming1Second;
    uint16_t TurnOffTiming1Minute;
    uint16_t TurnOffTiming1Hour;
    uint16_t TurnOnTiming2Second;
    uint16_t TurnOnTiming2Minute;
    uint16_t TurnOnTiming2Hour;
    uint16_t TurnOffTiming2Second;
    uint16_t TurnOffTiming2Minute;
    uint16_t TurnOffTiming2Hour;
    uint16_t LengthOfNight;
    uint16_t BatteryRatedVoltageCode;
    uint16_t LoadTimingControlSelection;
    uint16_t DefaultLoadOnOffInManualMode;
    uint16_t EqualizeDuration;
    uint16_t BoostDuration;
    uint16_t DischargingPercentage;
    uint16_t ChargingPercentage;
    uint16_t ManagementModeOfBatteryChargingAndDischarging;
}epever_setting_t;

typedef struct 
{
    uint16_t ManualControlTheLoad;
    uint16_t EnableLoadTestMode;
    uint16_t ForceTheLoadOnOff;
} epever_coil_t;

typedef struct 
{
    uint16_t OverTemperatureInsideTheDevice;
    uint16_t DayNight;
} epever_discrete_t;


esp_err_t master_init(void);
void master_read_rated(epever_rated_t *epever);
void master_read_realtime(epever_realtime_t *epever);
void master_read_statistical(epever_statistical_t *epever);
void master_read_setting(epever_setting_t *epever);
void master_read_coil(epever_coil_t *epever);
void master_read_discrete(epever_discrete_t *epever);

void master_write_coil_ManualControlTheLoad(bool value);


