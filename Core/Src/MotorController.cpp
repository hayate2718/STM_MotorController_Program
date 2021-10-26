/*
 * MotorController.cpp
 *
 *  Created on: Oct 23, 2021
 *      Author: 0_hayate
 */

#include "main.h"
#include "CAN.hpp"
#include "MotorController.hpp"

MotorController::MotorController(CAN_HandleTypeDef *_hcan, uint8_t id):
use_can(_hcan)
{
	use_can.set_id_CAN(id);
	use_can.filter_set();

	HAL_CAN_Start(_hcan);
	HAL_CAN_ActivateNotification(_hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
}

void MotorController::set_cmd(uint32_t cmd ,float data){
	use_can.use_tx_CAN(cmd, data);
	HAL_Delay(1);
}

float MotorController::get_cmd(uint32_t cmd){
	use_can.use_tx_CAN(cmd, 0);
	use_data.all_data_raw = use_can.use_read_CAN(cmd);
	return use_data.low_data;
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){ //受信割り込みコールバック
	//STM_MotorSystem *ms = STM_MotorSystem::_ms;
	//ms->use_can.use_rx_CAN(hcan);
	_use_mc->use_can.use_rx_CAN(hcan);
}

