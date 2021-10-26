/*
 * CAN.cpp
 *
 *  Created on: 2021/09/11
 *      Author: 0_hayate
 */

#include <MotorController.hpp>
#include <CAN.hpp>


USER_CAN::USER_CAN(CAN_HandleTypeDef * _use_hcan){

	this->_use_hcan = _use_hcan;

	filter.FilterActivation = 1; //filter enable
	filter.FilterBank = 0; //used filterbank 0
 	filter.FilterFIFOAssignment = 0; //rxdata to fifo0
	filter.FilterMode = 0; //filter mode is mask mode
	filter.FilterScale = 0; //filterscale is dual 16bits
	filter.FilterIdHigh = 0;
	filter.FilterMaskIdHigh = 0xf << 5;
	HAL_CAN_ConfigFilter(this->_use_hcan, &filter);

	TxHeader.DLC = 4; //データ長（4byte）
	TxHeader.IDE = 0; //標準識別子
	TxHeader.RTR = 0; //データフレーム
	TxHeader.TransmitGlobalTime = DISABLE; //タイムスタンプ無効

	can_id = 0b0000;

	HAL_CAN_Start(this->_use_hcan);
	HAL_CAN_ActivateNotification(this->_use_hcan,CAN_IT_RX_FIFO0_MSG_PENDING);


}

void USER_CAN::use_tx_CAN(uint32_t cmd,float data){
	can_data tx;

	uint32_t mailbox;

	tx.low_data = data;
	TxHeader.StdId = cmd+can_id;

	HAL_CAN_AddTxMessage(_use_hcan,&TxHeader, tx.low_data_raw,&mailbox);
}

void USER_CAN::set_id_CAN(uint8_t can_id){ //4bits
	this->can_id = can_id;

}


void USER_CAN::filter_set(){
	filter.FilterIdHigh = can_id << 5;
	HAL_CAN_ConfigFilter(this->_use_hcan, &filter);

}

void USER_CAN::use_rx_CAN(CAN_HandleTypeDef *_hcan){
	if(_hcan != _use_hcan){
		return ;
	}

	CAN_RxHeaderTypeDef RxHeader;
	can_data buf;

	if(HAL_CAN_GetRxMessage(_use_hcan, CAN_RX_FIFO0, &RxHeader, buf.all_data) == HAL_OK){
		rx.all_data_raw = buf.all_data_raw;
		read_f = RxHeader.StdId;
	}
}

uint64_t USER_CAN::use_read_CAN(uint32_t cmd){ //モータコントロール用
	while(cmd+id != read_f);
	read_f = 0;
	return rx.all_data_raw;
}




