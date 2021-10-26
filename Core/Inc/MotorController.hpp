/*
 * MotorController.hpp
 *
 *  Created on: Oct 23, 2021
 *      Author: 0_hayate
 */

#include "main.h"
#include "CAN.hpp"

#ifndef INC_MOTORCONTROLLER_HPP_
#define INC_MOTORCONTROLLER_HPP_


class MotorController{
private:

	uint8_t id;
	can_data use_data;

public:
	MotorController(CAN_HandleTypeDef * _hcan, uint8_t id);

	void set_cmd(uint32_t cmd,float data);
	float get_cmd(uint32_t cmd);


	//set_cmd関数で下記すべてできるけど簡単のために関数与えとく

	void init_system(); //最初に必ず実行する（電流センサのキャリブレーション、エンコーダカウントの初期化を行う）

	void default_palameterset( //この関数のあとは必ずsystem_start関数を実行すること
			float v_p, //速度制御pidゲイン
			float v_i,
			float v_d,
			float t_p, //トルク制御PIDゲイン
			float t_i,
			float t_d,
			float c_g, //電流センサゲイン
			float ppr, //エンコーダ分解能
			float volt, //モータ電源電圧
			float kt); //モータ逆起電力定数[v*sec/rad]

	void set_current_limit(float limit);

	//system_start実行直近のモードが行われる

	void set_vel(float vel); //system_startの前に実行すると速度制御モードになる

	void set_torque(float t); //system_startの前に実行するとトルク制御モードになる

	void set_coast(); //coastモードになる。

	void set_stop(); //stopモードになる（速度制御、トルク制御モードと違いADCが停止するので省電力

	void system_start(); //制御モードを変更するにはこの関数を実行する（stop,coastは必要なし）

	void mode_set(uint32_t mode); //モードセット用関数　SET_VELOCITY か　SET_TORQUEを入れる。

	//モータ回転中でのモード変更はmode_set関数を使ってモードセットを行いsystem_start関数を実行してください。

	USER_CAN use_can;

};

inline void MotorController::init_system(){
	set_stop();
	HAL_Delay(500);
	this->get_cmd(cmd::SYSTEM_INIT);
}

inline void MotorController::default_palameterset( //この関数のあとは必ずsystem_start関数を実行すること
		float v_p, //速度制御pidゲイン
		float v_i,
		float v_d,
		float t_p, //トルク制御PIDゲイン
		float t_i,
		float t_d,
		float c_g, //電流センサゲイン
		float ppr, //エンコーダ分解能
		float volt, //モータ電源電圧
		float kt){
	this->set_cmd(cmd::SET_VELOCITY_P, v_p);
	this->set_cmd(cmd::SET_VELOCITY_I, v_i);
	this->set_cmd(cmd::SET_VELOCITY_D, v_d);
	this->set_cmd(cmd::SET_TORQUE_P, t_p);
	this->set_cmd(cmd::SET_TORQUE_I, t_i);
	this->set_cmd(cmd::SET_TORQUE_D, t_d);
	this->set_cmd(cmd::SET_ADC_GAIN, c_g);
	this->set_cmd(cmd::SET_PPR, ppr);
	this->set_cmd(cmd::SET_VOLTAGE, volt);
	this->set_cmd(cmd::SET_KT, kt);
}

inline void MotorController::system_start(){
	this->set_cmd(cmd::SYSTEM_START, 0);
}

inline void MotorController::set_stop(){
	this->set_cmd(cmd::MOTOR_SYSTEM_STOP, 0);
}

inline void MotorController::set_coast(){
	this->set_cmd(cmd::SET_COAST,0);
}

inline void MotorController::set_vel(float vel){
	this->set_cmd(cmd::SET_VELOCITY, vel);
}

inline void MotorController::set_torque(float t){
	this->set_cmd(cmd::SET_TORQUE, t);
}

inline void MotorController::set_current_limit(float limit){
	this->set_cmd(cmd::SET_CURRENT_LIMIT, limit);
}

inline void MotorController::mode_set(uint32_t mode){
	this->set_stop();
	HAL_Delay(500);
	set_cmd(mode,0);
}

//モータコントローラインタスタンスのポインタ宣言
extern MotorController * _use_mc;


#endif /* INC_MOTORCONTROLLER_HPP_ */
