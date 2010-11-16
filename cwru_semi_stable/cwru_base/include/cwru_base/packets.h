/*
 * File:   packets.h
 * Author: Eric Perko
 *
 * Created on August 31, 2009, 12:36 PM
 */

#ifndef _PACKETS_H
#define	_PACKETS_H

namespace cwru_base {
	const int BUF_SIZE = 1024;

	/*
	 * These types are defined to numbers that match the type code in the packets
	 * themselves. This is why they are defined to numbers
	 *
	 */

	enum SendEnums {
		HEADINGSPEEDCOMMAND_t = 1,
		REBOOTCOMMAND_t = 0,
		WAYPOINTCOMMAND_t = 2,
		STARTCOMMAND_t = 3,
		STOPCOMMAND_t = 4,
		QUERYSYSTEMSTATUS_t = 5,
		ESTOP_t = 6,
		ANGULARRATESPEEDCOMMAND_t = 7

	};

	enum ReceiveEnums {
		POSE_t = 0,
		DIAGNOSTICS_t = 1,
		SONARS_t = 2
	};

	typedef struct posePacket_t {
		int8_t type;
		int8_t data1; /* padding */
		int8_t data2;
		int8_t data3;
		float x;
		float y;
		float theta;
		float vel;
		float omega;
		float x_variance;
		float y_variance;
		float theta_variance;
		float vel_variance;
		float omega_variance;
		float sonar_ping_1;
		float sonar_ping_2;
		float sonar_ping_3;
		float sonar_ping_4;
		float sonar_ping_5;
	} CRIOPosePacket;

	typedef struct diagnosticsPacket_t {
		int8_t type;
		int8_t status;
		uint8_t eStopTriggered;
		uint8_t RCOn;
		int16_t FPGAVersion;
		int16_t VMonitor_cRIO_mV;

		int32_t LWheelTicks;
		int32_t RWheelTicks;
		int32_t LMotorTicks;
		int32_t RMotorTicks;

		int16_t VMonitor_24V_mV;
		int16_t VMonitor_13V_mv;
		int16_t VMonitor_5V_mV;
		int16_t VMonitor_eStop_mV;

		int16_t YawRate_mV;
		int16_t YawSwing_mV;
		int16_t YawTemp_mV;
		int16_t YawRef_mV;

		uint16_t C1Steering;
		uint16_t C2Throttle;
		uint16_t C3Mode;
		uint8_t RCeStop;
	} CRIODiagnosticsPacket;

	typedef struct Command_t {
		int8_t type;
		char commandData[BUF_SIZE - sizeof(int8_t)];
	} CRIOCommand;

};
#endif	/* _PACKETS_H */

