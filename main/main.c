// Copyright 2016-2019 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "string.h"
#include "esp_log.h"
//#include "sdkconfig.h"
#include "epever.h"


void print_rated(epever_rated_t *epever_rated)
{
    ESP_LOGI("-------- print_rated ", "--------");
    ESP_LOGI("ChargingRatedInputVoltage", "%d", epever_rated->ChargingRatedInputVoltage);
    ESP_LOGI("ChargingRatedInputCurrent", "%d", epever_rated->ChargingRatedInputCurrent);
    ESP_LOGI("ChargingRatedOutputPower", "%d", epever_rated->ChargingRatedOutputPower);

    ESP_LOGI("ChargingMode", "%d", epever_rated->ChargingMode);
    ESP_LOGI("OutputRatedLoadCurrent", "%d", epever_rated->OutputRatedLoadCurrent);
}

void print_setting(epever_setting_t *epever_setting)
{
    ESP_LOGI("-------- print_setting ", "--------");
    ESP_LOGI("BatteryType", "%d", epever_setting->BatteryType);
    ESP_LOGI("BatteryCapacity", "%d", epever_setting->BatteryCapacity);
    ESP_LOGI("TemperatureCompensationCoefficient", "%d", epever_setting->TemperatureCompensationCoefficient);
    ESP_LOGI("HighVoltDisconnect", "%d", epever_setting->HighVoltDisconnect);
    ESP_LOGI("ChargingLimitVoltage", "%d", epever_setting->ChargingLimitVoltage);
    ESP_LOGI("OverVoltageReconnect", "%d", epever_setting->OverVoltageReconnect);
    ESP_LOGI("EqualizationVoltage", "%d", epever_setting->EqualizationVoltage);
    ESP_LOGI("BoostVoltage", "%d", epever_setting->BoostVoltage);
    ESP_LOGI("FloatVoltage", "%d", epever_setting->FloatVoltage);
    ESP_LOGI("BoostReconnectVoltage", "%d", epever_setting->BoostReconnectVoltage);
    ESP_LOGI("LowVoltageReconnect", "%d", epever_setting->LowVoltageReconnect);
    ESP_LOGI("UnderVoltageRecover", "%d", epever_setting->UnderVoltageRecover);
    ESP_LOGI("UnderVoltageWarning", "%d", epever_setting->UnderVoltageWarning);
    ESP_LOGI("LowVoltageDisconnect", "%d", epever_setting->LowVoltageDisconnect);
    ESP_LOGI("DischargingLimitVoltage", "%d", epever_setting->DischargingLimitVoltage);
    ESP_LOGI("RealTimeClock1", "%d", epever_setting->RealTimeClock1);
    ESP_LOGI("RealTimeClock2", "%d", epever_setting->RealTimeClock2);
    ESP_LOGI("RealTimeClock3", "%d", epever_setting->RealTimeClock3);
    ESP_LOGI("EqualizationChargingCycle", "%d", epever_setting->EqualizationChargingCycle);
    ESP_LOGI("BatteryTemperatureWarningUpperLimit", "%d", epever_setting->BatteryTemperatureWarningUpperLimit);
    ESP_LOGI("BatteryTemperatureWarningLowerLimit", "%d", epever_setting->BatteryTemperatureWarningLowerLimit);
    ESP_LOGI("ControllerInnerTemperatureUpperLimit", "%d", epever_setting->ControllerInnerTemperatureUpperLimit);
    ESP_LOGI("ControllerInnerTemperatureUpperLimitRecover", "%d", epever_setting->ControllerInnerTemperatureUpperLimitRecover);
    ESP_LOGI("PowerComponentTemperatureUpperLimit", "%d", epever_setting->PowerComponentTemperatureUpperLimit);
    ESP_LOGI("PowerComponentTemperatureUpperLimitRecover", "%d", epever_setting->PowerComponentTemperatureUpperLimitRecover);
    ESP_LOGI("LineImpedance", "%d", epever_setting->LineImpedance);
    ESP_LOGI("NightTimeThresholdVolt", "%d", epever_setting->NightTimeThresholdVolt);
    ESP_LOGI("LightSignalStartupDelayTime", "%d", epever_setting->LightSignalStartupDelayTime);
    ESP_LOGI("DayTimeThresholdVolt", "%d", epever_setting->DayTimeThresholdVolt);
    ESP_LOGI("LightSignalTurnOffDelayTime", "%d", epever_setting->LightSignalTurnOffDelayTime);
    ESP_LOGI("LoadControlingMode", "%d", epever_setting->LoadControlingMode);
    ESP_LOGI("WorkingTimeLength1", "%d", epever_setting->WorkingTimeLength1);
    ESP_LOGI("WorkingTimeLength2", "%d", epever_setting->WorkingTimeLength2);
    ESP_LOGI("TurnOnTiming1Second", "%d", epever_setting->TurnOnTiming1Second);
    ESP_LOGI("TurnOnTiming1Minute", "%d", epever_setting->TurnOnTiming1Minute);
    ESP_LOGI("TurnOnTiming1Hour", "%d", epever_setting->TurnOnTiming1Hour);
    ESP_LOGI("TurnOffTiming1Second", "%d", epever_setting->TurnOffTiming1Second);
    ESP_LOGI("TurnOffTiming1Minute", "%d", epever_setting->TurnOffTiming1Minute);
    ESP_LOGI("TurnOffTiming1Hour", "%d", epever_setting->TurnOffTiming1Hour);
    ESP_LOGI("TurnOnTiming2Second", "%d", epever_setting->TurnOnTiming2Second);
    ESP_LOGI("TurnOnTiming2Minute", "%d", epever_setting->TurnOnTiming2Minute);
    ESP_LOGI("TurnOnTiming2Hour", "%d", epever_setting->TurnOnTiming2Hour);
    ESP_LOGI("TurnOffTiming2Second", "%d", epever_setting->TurnOffTiming2Second);
    ESP_LOGI("TurnOffTiming2Minute", "%d", epever_setting->TurnOffTiming2Minute);
    ESP_LOGI("TurnOffTiming2Hour", "%d", epever_setting->TurnOffTiming2Hour);
    ESP_LOGI("LengthOfNight", "%d", epever_setting->LengthOfNight);
    ESP_LOGI("BatteryRatedVoltageCode", "%d", epever_setting->BatteryRatedVoltageCode);
    ESP_LOGI("LoadTimingControlSelection", "%d", epever_setting->LoadTimingControlSelection);
    ESP_LOGI("DefaultLoadOnOffInManualMode", "%d", epever_setting->DefaultLoadOnOffInManualMode);
    ESP_LOGI("EqualizeDuration", "%d", epever_setting->EqualizeDuration);
    ESP_LOGI("BoostDuration", "%d", epever_setting->BoostDuration);
    ESP_LOGI("DischargingPercentage", "%d", epever_setting->DischargingPercentage);
    ESP_LOGI("ChargingPercentage", "%d", epever_setting->ChargingPercentage);
    ESP_LOGI("ManagementModeOfBatteryChargingAndDischarging", "%d", epever_setting->ManagementModeOfBatteryChargingAndDischarging);
}

void print_realtime(epever_realtime_t *epever_realtime )
{
    ESP_LOGI("-------- print_realtime ", "--------");
    ESP_LOGI("ChargingInputVoltage", "%d", epever_realtime->ChargingInputVoltage);
    ESP_LOGI("ChargingInputCurrent", "%d", epever_realtime->ChargingInputCurrent);
    ESP_LOGI("ChargingInputPower", "%d", epever_realtime->ChargingInputPower);

    ESP_LOGI("ChargingOutputVoltage", "%d", epever_realtime->ChargingOutputVoltage);
    ESP_LOGI("ChargingOutputCurrent", "%d", epever_realtime->ChargingOutputCurrent);
    ESP_LOGI("ChargingOutputPower", "%d", epever_realtime->ChargingOutputPower);

    ESP_LOGI("DischargingOutputVoltage", "%d", epever_realtime->DischargingOutputVoltage);
    ESP_LOGI("DischargingOutputCurrent", "%d", epever_realtime->DischargingOutputCurrent);
    ESP_LOGI("DischargingOutputPower", "%d", epever_realtime->DischargingOutputPower);

    ESP_LOGI("BatteryTemperature", "%d", epever_realtime->BatteryTemperature);
    ESP_LOGI("ChargerCaseTemperature", "%d", epever_realtime->ChargerCaseTemperature);
    ESP_LOGI("ChargerSiliconTemperature", "%d", epever_realtime->ChargerSiliconTemperature);

    ESP_LOGI("BatterySOC", "%d", epever_realtime->BatterySOC);
    ESP_LOGI("RemoteBatteryTemperature", "%d", epever_realtime->RemoteBatteryTemperature);
    ESP_LOGI("BatteryRealRatedVoltage", "%d", epever_realtime->BatteryRealRatedVoltage);

    ESP_LOGI("BatteryStatus", "%d", epever_realtime->BatteryStatus);
    ESP_LOGI("ChargerStatus", "%d", epever_realtime->ChargerStatus);
}

void print_statistical(epever_statistical_t *epever_statistical)
{
    ESP_LOGI("-------- print_statistical ", "--------");
    ESP_LOGI("TodayMaxInputVoltage", "%d", epever_statistical->TodayMaxInputVoltage);
    ESP_LOGI("TodayMinInputVoltage", "%d", epever_statistical->TodayMinInputVoltage);
    ESP_LOGI("TodayMaxBatteryVoltage", "%d", epever_statistical->TodayMaxBatteryVoltage);
    ESP_LOGI("TodayMinBatteryVoltage", "%d", epever_statistical->TodayMinBatteryVoltage);

    ESP_LOGI("TodayConsumedEnergy", "%d", epever_statistical->TodayConsumedEnergy);
    ESP_LOGI("ThisMonthConsumedEnergy", "%d", epever_statistical->ThisMonthConsumedEnergy);
    ESP_LOGI("ThisYearConsumedEnergy", "%d", epever_statistical->ThisYearConsumedEnergy);
    ESP_LOGI("TotalConsumedEnergy", "%d", epever_statistical->TotalConsumedEnergy);

    ESP_LOGI("TodayGeneratedEnergy", "%d", epever_statistical->TodayGeneratedEnergy);
    ESP_LOGI("ThisMonthGeneratedEnergy", "%d", epever_statistical->ThisMonthGeneratedEnergy);
    ESP_LOGI("ThisYearGeneratedEnergy", "%d", epever_statistical->ThisYearGeneratedEnergy);
    ESP_LOGI("TotalGeneratedEnergy", "%d", epever_statistical->TotalGeneratedEnergy);

    ESP_LOGI("CarbonDioxideReduction", "%d", epever_statistical->CarbonDioxideReduction);

    ESP_LOGI("BatteryCurrent", "%d", epever_statistical->BatteryCurrent);

    ESP_LOGI("BatteryTemperature", "%d", epever_statistical->BatteryTemperature);
    ESP_LOGI("AmbientTemperature", "%d", epever_statistical->AmbientTemperature);
}

void print_coil(epever_coil_t *epever_coil)
{
    ESP_LOGI("-------- print_coil ", "--------");
    ESP_LOGI("ManualControlTheLoad", "%d", epever_coil->ManualControlTheLoad);
    ESP_LOGI("EnableLoadTestMode", "%d", epever_coil->EnableLoadTestMode);
    ESP_LOGI("ForceTheLoadOnOff", "%d", epever_coil->ForceTheLoadOnOff);
}

void print_discrete(epever_discrete_t *epever_discrete)
{
    ESP_LOGI("-------- print_discrete ", "--------");
    ESP_LOGI("OverTemperatureInsideTheDevice", "%d", epever_discrete->OverTemperatureInsideTheDevice);
    ESP_LOGI("DayNight", "%d", epever_discrete->DayNight);
}

void app_main(void)
{

    // Initialization of device peripheral and objects
    ESP_ERROR_CHECK(master_init());
    vTaskDelay(10);


    epever_rated_t epever_rated = {};
    epever_realtime_t epever_realtime = {};
    epever_statistical_t epever_statistical = {};
    epever_setting_t epever_setting = {};

    epever_coil_t epever_coil = {};
    epever_discrete_t epever_discrete = {};

    uint32_t loop = 0;
    
    master_read_rated(&epever_rated);
    master_read_setting(&epever_setting);

    print_rated(&epever_rated);
    print_setting(&epever_setting);


    master_read_coil(&epever_coil);
    master_read_discrete(&epever_discrete);

    print_coil(&epever_coil);
    print_discrete(&epever_discrete);


    vTaskDelay(500);
    
    master_write_coil_ManualControlTheLoad(1);
    vTaskDelay(500);
    master_write_coil_ManualControlTheLoad(0);


    while(1)
    {
        ESP_LOGI("LOOP ", "%d", loop++);
        master_read_realtime(&epever_realtime);
        print_realtime(&epever_realtime);
        
        master_read_statistical(&epever_statistical);
        print_statistical(&epever_statistical);


        ESP_LOGI("--------- ","---------");

        vTaskDelay(1000);
}
    }

   