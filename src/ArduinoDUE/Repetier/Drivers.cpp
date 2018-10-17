#include "Repetier.h"

#if defined(NUM_MOTOR_DRIVERS) && NUM_MOTOR_DRIVERS > 0
MOTOR_DRIVER_1(motorDriver1);
#if NUM_MOTOR_DRIVERS > 1
MOTOR_DRIVER_2(motorDriver2);
#endif
#if NUM_MOTOR_DRIVERS > 2
MOTOR_DRIVER_3(motorDriver3);
#endif
#if NUM_MOTOR_DRIVERS > 3
MOTOR_DRIVER_4(motorDriver4);
#endif
#if NUM_MOTOR_DRIVERS > 4
MOTOR_DRIVER_5(motorDriver5);
#endif
#if NUM_MOTOR_DRIVERS > 5
MOTOR_DRIVER_6(motorDriver6);
#endif

MotorDriverInterface *motorDrivers[NUM_MOTOR_DRIVERS] =
{
    &motorDriver1
#if NUM_MOTOR_DRIVERS > 1
    , &motorDriver2
#endif
#if NUM_MOTOR_DRIVERS > 2
    , &motorDriver3
#endif
#if NUM_MOTOR_DRIVERS > 3
    , &motorDriver4
#endif
#if NUM_MOTOR_DRIVERS > 4
    , &motorDriver5
#endif
#if NUM_MOTOR_DRIVERS > 5
    , &motorDriver6
#endif
};

MotorDriverInterface *getMotorDriver(int idx)
{
    return motorDrivers[idx];
}

/**
Run motor P until it is at position X
*/
void commandG201(GCode &code)
{
    int id = 0;
    if(code.hasP())
        id = code.P;
    if(id < 0) id = 0;
    if(id >= NUM_MOTOR_DRIVERS) id = 0;
    if(!code.hasX()) return;
    motorDrivers[id]->gotoPosition(code.X);
}

//G202 P<motorId> X<setpos>  - Mark current position as X
void commandG202(GCode &code)
{
    int id = 0;
    if(code.hasP())
        id = code.P;
    if(id < 0) id = 0;
    if(id >= NUM_MOTOR_DRIVERS) id = 0;
    if(!code.hasX()) return;
    motorDrivers[id]->setCurrentAs(code.X);
}
//G203 P<motorId>            - Report current motor position
void commandG203(GCode &code)
{
    int id = 0;
    if(code.hasP())
        id = code.P;
    if(id < 0) id = 0;
    if(id >= NUM_MOTOR_DRIVERS) id = 0;
    Com::printF(PSTR("Motor"),id);
    Com::printFLN(PSTR(" Pos:"),motorDrivers[id]->getPosition());
}
//G204 P<motorId> S<0/1>     - Enable/disable motor
void commandG204(GCode &code)
{
    int id = 0;
    if(code.hasP())
        id = code.P;
    if(id < 0) id = 0;
    if(id >= NUM_MOTOR_DRIVERS) id = 0;
    if(!code.hasS()) return;
    if(code.S)
        motorDrivers[id]->enable();
    else
        motorDrivers[id]->disable();
}
// G205 P<motorId> S<0/1> E<0/1> - Home motor, S1 = go back to stored position, E1 = home only if endstop was never met, meaning it was never homed with motor.
void commandG205(GCode &code)
{
	int id = 0;
	if(code.hasP())
		id = code.P;
	if(id < 0) id = 0;
	if(id >= NUM_MOTOR_DRIVERS) id = 0;
	motorDrivers[id]->home(code.hasS() && code.S != 0, code.hasE() && code.E != 0);
}

void disableAllMotorDrivers()
{
    for(int i = 0; i < NUM_MOTOR_DRIVERS; i++)
        motorDrivers[i]->disable();
}
void initializeAllMotorDrivers()
{
    for(int i = 0; i < NUM_MOTOR_DRIVERS; i++)
        motorDrivers[i]->initialize();
}

#endif // NUM_MOTOR_DRIVERS

#if defined(SUPPORT_LASER) && SUPPORT_LASER

secondspeed_t LaserDriver::intensity = LASER_PWM_MAX; // Intensity to use for next move queued if we want lasers. This is NOT the current value!
secondspeed_t LaserDriver::intens = 0;

bool LaserDriver::laserOn = false;
bool LaserDriver::firstMove = true;

void LaserDriver::initialize()
{
    if(EVENT_INITIALIZE_LASER)
    {
#if LASER_PIN > -1
        SET_OUTPUT(LASER_PIN);
#endif
    }
    changeIntensity(0);
}

void LaserDriver::changeIntensity(secondspeed_t newIntensity)
{
#if defined(DOOR_PIN) && DOOR_PIN > -1
    if(Printer::isDoorOpen()) {
        newIntensity = 0; // force laser off if door is open
    }
#endif
    if(EVENT_SET_LASER(newIntensity))
    {
        // Default implementation
#if LASER_PIN > -1
        WRITE(LASER_PIN,(LASER_ON_HIGH ? newIntensity > 199 : newIntensity < 200));
#endif
    }
    intens=newIntensity;//for "Transfer" Status Page
}
#endif // SUPPORT_LASER

#if defined(SUPPORT_CNC) && SUPPORT_CNC
/**
The CNC driver differs a bit from laser driver. Here only M3,M4,M5 have an influence on the spindle.
The motor also keeps running for G0 moves. M3 and M4 wait for old moves to be finished and then enables
the motor. It then waits CNC_WAIT_ON_ENABLE milliseconds for the spindle to reach target speed.
*/

int8_t CNCDriver::direction = 0;
secondspeed_t CNCDriver::spindleSpeed= 0;
uint16_t CNCDriver::spindleRpm= 0;


/** Initialize cnc pins. EVENT_INITIALIZE_CNC should return false to prevent default initialization.*/
void CNCDriver::initialize()
{
    if(EVENT_INITIALIZE_CNC)
    {
#if CNC_ENABLE_PIN > -1
        SET_OUTPUT(CNC_ENABLE_PIN);
        WRITE(CNC_ENABLE_PIN,!CNC_ENABLE_WITH);
#endif
#if CNC_DIRECTION_PIN > -1
        SET_OUTPUT(CNC_DIRECTION_PIN);
#endif
    }
}
/** Turns off spindle. For event override implement
EVENT_SPINDLE_OFF
returning false.
*/
void CNCDriver::spindleOff()
{
    spindleRpm=0;
    if(direction == 0) return; // already off
    if(EVENT_SPINDLE_OFF)
    {
#if CNC_ENABLE_PIN > -1
        WRITE(CNC_ENABLE_PIN,!CNC_ENABLE_WITH);
#endif
    }
    HAL::delayMilliseconds(CNC_WAIT_ON_DISABLE);
	direction = 0;
}
/** Turns spindle on. Default implementation uses a enable pin CNC_ENABLE_PIN. If
CNC_DIRECTION_PIN is not -1 it sets direction to CNC_DIRECTION_CW. rpm is ignored.
To override with event system, return false for the event
EVENT_SPINDLE_CW(rpm)
*/
void CNCDriver::spindleOnCW(int32_t rpm)
{
  spindleSpeed=map(rpm,0,CNC_RPM_MAX,0,CNC_PWM_MAX);// linear interpolation

  
    if(direction == 1 && spindleRpm == rpm)
        return;
    if(direction == -1) {
	    spindleOff();
    }
    spindleRpm = rpm;// for display
    direction = 1;
    if(EVENT_SPINDLE_CW(rpm)) {
#if CNC_DIRECTION_PIN > -1
        WRITE(CNC_DIRECTION_PIN, CNC_DIRECTION_CW);
#endif
#if CNC_ENABLE_PIN > -1
        WRITE(CNC_ENABLE_PIN, CNC_ENABLE_WITH);
#endif
    }
    HAL::delayMilliseconds(CNC_WAIT_ON_ENABLE);
}
/** Turns spindle on. Default implementation uses a enable pin CNC_ENABLE_PIN. If
CNC_DIRECTION_PIN is not -1 it sets direction to !CNC_DIRECTION_CW. rpm is ignored.
To override with event system, return false for the event
EVENT_SPINDLE_CCW(rpm)
*/
void CNCDriver::spindleOnCCW(int32_t rpm)
{
        spindleSpeed=map(rpm,0,CNC_RPM_MAX,0,CNC_PWM_MAX);// linear interpolation
   
    if(direction == -1 && spindleRpm == rpm)
        return;
    if(direction == 1) {
		spindleOff();
	}
    spindleRpm = rpm;// for display
    direction = -1;
    if(EVENT_SPINDLE_CCW(rpm)) {
#if CNC_DIRECTION_PIN > -1
        WRITE(CNC_DIRECTION_PIN, !CNC_DIRECTION_CW);
#endif
#if CNC_ENABLE_PIN > -1
        WRITE(CNC_ENABLE_PIN, CNC_ENABLE_WITH);
#endif
    }
    HAL::delayMilliseconds(CNC_WAIT_ON_ENABLE);
}
#endif

//////////////////////////////////////////////////////////////////////////////////
//                         INVENTOR BOARD TMC2130 Driver Support				//
//////////////////////////////////////////////////////////////////////////////////
#if (USES_TMC2130_DRIVERS)

// Initialize each stepper object with the correct CS pin, and provide functions
// to get/set registers from the driver (for interactive tuning from M codes 920, 921)
#if (X_IS_TMC2130)
Trinamic_TMC2130 Printer::stepperX(X_SPI_CS);

void Printer::WriteTMC_X(int reg_index, int32_t value) {
#if MOTHERBOARD == INVENTOR_BOARD	
	HAL::RouteSPITo(SPI_X);			// For Inventor board, we must first route the SPI
#endif
	switch (reg_index) {
	case 1:
		stepperX.set_mres(value);
		Com::printFLN("M920: X MRES set to ", value);
		break;
	case 2:
		stepperX.set_intpol(value);
		Com::printFLN("M920: X INTPOL set to ", value);
		break;
	case 3:
		stepperX.set_tbl(value); // ([0-3]) set comparator blank time to 16, 24, 36 or 54 clocks, 1 or 2 is recommended
		Com::printFLN("M920: X TBL set to ", value);
		break;
	case 4:
		stepperX.set_toff(value); // ([0-15]) 0: driver disable, 1: use only with TBL>2, 2-15: off time setting during slow decay phase
		Com::printFLN("M920: X TOFF set to ", value);
		break;
	case 5:
		stepperX.set_hstrt(value);
		Com::printFLN("M920: X HSTRT set to ", value);
		break;
	case 6:		
		stepperX.set_hend(value);
		Com::printFLN("M920: X HEND set to ", value);
		break;
	case 7:
		stepperX.set_chm(value);
		Com::printFLN("M920: X CHM set to ", value);
		break;
	case 8:
		stepperX.set_TPOWERDOWN(value);
		Com::printFLN("M920: X TPOWERDOWN set to ", value);
		break;
	case 9:
		stepperX.set_en_pwm_mode(value);
		Com::printFLN("M920: X EN_PWM_MODE set to ", value);
		break;
	case 10:
		stepperX.set_TCOOLTHRS(value);
		Com::printFLN("M920: X TCOOLTHRS set to ", value);
		break;
	case 11:
		stepperX.set_sfilt(value);
		Com::printFLN("M920: X SFILT set to ", value);
		break;
	case 12:
		stepperX.set_sgt((uint8_t)value);									// managed to confuse myself - can't tell if this will properly set the 7bit signed value that is passed as uint8_t
		Com::printFLN("M920: X SGT set to (signed) ", (int8_t)value);		// again - not sure if this will accurately reflect what was set or not?
		Com::printFLN("M920: X SGT set to (unsigned) ", (uint8_t)value);	// again - not sure if this will accurately reflect what was set or not?
		break;
	case 13:
		stepperX.set_seimin(value);
		Com::printFLN("M920: X SEIMIN set to ", value);
		break;
	case 14:
		stepperX.set_semin(value);
		Com::printFLN("M920: X SEMIN set to ", value);
		break;
	case 15:
		stepperX.set_semax(value);
		Com::printFLN("M920: X SEMAX set to ", value);
		break;
	case 16:
		stepperX.set_sedn(value);
		Com::printFLN("M920: X SEDN set to ", value);
		break;
	case 17:
		stepperX.set_seup(value);
		Com::printFLN("M920: X SEUP set to ", value);
		break;
	default:
		Com::printFLN("M920 error - unknown register index - Try M922 to list available register indicies.");
		break;
	}
#if MOTHERBOARD == INVENTOR_BOARD	
	HAL::RouteSPITo(SPI_NONE);		// Disconnect from all slaves
#endif
}
void Printer::ReadTMC_X(int reg_index) {
	Com::printFLN("M921 error - register reads not implimented yet");
}

#endif
#if (Y_IS_TMC2130)
Trinamic_TMC2130 Printer::stepperY(Y_SPI_CS);
void Printer::WriteTMC_Y(int reg_index, int32_t value) {
#if MOTHERBOARD == INVENTOR_BOARD	
	HAL::RouteSPITo(SPI_Y);			// For Inventor board, we must first route the SPI
#endif
	switch (reg_index) {
	case 1:
		stepperY.set_mres(value);
		Com::printFLN("M920: Y MRES set to ", value);
		break;
	case 2:
		stepperY.set_intpol(value);
		Com::printFLN("M920: Y INTPOL set to ", value);
		break;
	case 3:
		stepperY.set_tbl(value); // ([0-3]) set comparator blank time to 16, 24, 36 or 54 clocks, 1 or 2 is recommended
		Com::printFLN("M920: Y TBL set to ", value);
		break;
	case 4:
		stepperY.set_toff(value); // ([0-15]) 0: driver disable, 1: use only with TBL>2, 2-15: off time setting during slow decay phase
		Com::printFLN("M920: Y TOFF set to ", value);
		break;
	case 5:
		stepperY.set_hstrt(value);
		Com::printFLN("M920: Y HSTRT set to ", value);
		break;
	case 6:
		stepperY.set_hend(value);
		Com::printFLN("M920: Y HEND set to ", value);
		break;
	case 7:
		stepperY.set_chm(value);
		Com::printFLN("M920: Y CHM set to ", value);
		break;
	case 8:
		stepperY.set_TPOWERDOWN(value);
		Com::printFLN("M920: Y TPOWERDOWN set to ", value);
		break;
	case 9:
		stepperY.set_en_pwm_mode(value);
		Com::printFLN("M920: Y EN_PWM_MODE set to ", value);
		break;
	case 10:
		stepperY.set_TCOOLTHRS(value);
		Com::printFLN("M920: Y TCOOLTHRS set to ", value);
		break;
	case 11:
		stepperY.set_sfilt(value);
		Com::printFLN("M920: Y SFILT set to ", value);
		break;
	case 12:
		stepperY.set_sgt((uint8_t)value);									// managed to confuse myself - can't tell if this will properly set the 7bit signed value that is passed as uint8_t		
		Com::printFLN("M920: Y SGT set to (signed) ", (int8_t)value);		// again - not sure if this will accurately reflect what was set or not?
		Com::printFLN("M920: Y SGT set to (unsigned) ", (uint8_t)value);	// again - not sure if this will accurately reflect what was set or not?
		break;
	case 13:
		stepperY.set_seimin(value);
		Com::printFLN("M920: Y SEIMIN set to ", value);
		break;
	case 14:
		stepperY.set_semin(value);
		Com::printFLN("M920: Y SEMIN set to ", value);
		break;
	case 15:
		stepperY.set_semax(value);
		Com::printFLN("M920: Y SEMAX set to ", value);
		break;
	case 16:
		stepperY.set_sedn(value);
		Com::printFLN("M920: Y SEDN set to ", value);
		break;
	case 17:
		stepperY.set_seup(value);
		Com::printFLN("M920: Y SEUP set to ", value);
		break;
	default:
		Com::printFLN("M920 error - unknown register index - Try M922 to list available register indicies.");
		break;
	}
#if MOTHERBOARD == INVENTOR_BOARD	
	HAL::RouteSPITo(SPI_NONE);		// Disconnect from all slaves
#endif
}
void Printer::ReadTMC_Y(int reg_index) {
	Com::printFLN("M921 error - register reads not implimented yet");
}
#endif
#if (Z_IS_TMC2130)
Trinamic_TMC2130 Printer::stepperZ(Z_SPI_CS);
void Printer::WriteTMC_Z(int reg_index, int32_t value) {
#if MOTHERBOARD == INVENTOR_BOARD	
	HAL::RouteSPITo(SPI_Z);			// For Inventor board, we must first route the SPI
#endif
	switch (reg_index) {
	case 1:
		stepperZ.set_mres(value);
		Com::printFLN("M920: Z MRES set to ", value);
		break;
	case 2:
		stepperZ.set_intpol(value);
		Com::printFLN("M920: Z INTPOL set to ", value);
		break;
	case 3:
		stepperZ.set_tbl(value); // ([0-3]) set comparator blank time to 16, 24, 36 or 54 clocks, 1 or 2 is recommended
		Com::printFLN("M920: Z TBL set to ", value);
		break;
	case 4:
		stepperZ.set_toff(value); // ([0-15]) 0: driver disable, 1: use only with TBL>2, 2-15: off time setting during slow decay phase
		Com::printFLN("M920: Z TOFF set to ", value);
		break;
	case 5:
		stepperZ.set_hstrt(value);
		Com::printFLN("M920: Z HSTRT set to ", value);
		break;
	case 6:
		stepperZ.set_hend(value);
		Com::printFLN("M920: Z HEND set to ", value);
		break;
	case 7:
		stepperZ.set_chm(value);
		Com::printFLN("M920: Z CHM set to ", value);
		break;
	case 8:
		stepperZ.set_TPOWERDOWN(value);
		Com::printFLN("M920: Z TPOWERDOWN set to ", value);
		break;
	case 9:
		stepperZ.set_en_pwm_mode(value);
		Com::printFLN("M920: Z EN_PWM_MODE set to ", value);
		break;
	case 10:
		stepperZ.set_TCOOLTHRS(value);
		Com::printFLN("M920: Z TCOOLTHRS set to ", value);
		break;
	case 11:
		stepperZ.set_sfilt(value);
		Com::printFLN("M920: Z SFILT set to ", value);
		break;
	case 12:
		stepperZ.set_sgt((uint8_t)value);						// managed to confuse myself - can't tell if this will properly set the 7bit signed value that is passed as uint8_t
		Com::printFLN("M920: Z SGT set to (signed) ", (int8_t)value);	// again - not sure if this will accurately reflect what was set or not?
		Com::printFLN("M920: Z SGT set to (unsigned) ", (uint8_t)value);	// again - not sure if this will accurately reflect what was set or not?
		break;
	case 13:
		stepperZ.set_seimin(value);
		Com::printFLN("M920: Z SEIMIN set to ", value);
		break;
	case 14:
		stepperZ.set_semin(value);
		Com::printFLN("M920: Z SEMIN set to ", value);
		break;
	case 15:
		stepperZ.set_semax(value);
		Com::printFLN("M920: Z SEMAX set to ", value);
		break;
	case 16:
		stepperZ.set_sedn(value);
		Com::printFLN("M920: Z SEDN set to ", value);
		break;
	case 17:
		stepperZ.set_seup(value);
		Com::printFLN("M920: Z SEUP set to ", value);
		break;
	case 18:
		stepperZ.set_diag1_stall(value);
		Com::printFLN("M920: Z diag1_stall set to ", value);
		break;
	default:
		Com::printFLN("M920 error - unknown register index - Try M922 to list available register indicies.");
		break;
}
#if MOTHERBOARD == INVENTOR_BOARD	
	HAL::RouteSPITo(SPI_NONE);		// Disconnect from all slaves
#endif
}
void Printer::ReadTMC_Z(int reg_index) {
	Com::printFLN("M921 error - register reads not implimented yet");
}
#endif
#if (E0_IS_TMC2130)
Trinamic_TMC2130 Printer::stepperE0(E0_SPI_CS);
#endif
#if (E1_IS_TMC2130)
Trinamic_TMC2130 Printer::stepperE1(E1_SPI_CS);
#endif
#if (E2_IS_TMC2130)
Trinamic_TMC2130 Printer::stepperE2(E2_SPI_CS);
#endif

// Initalize all drivers with the settings from Configuration.h
// NOTE: This currently uses 
void Printer::tmc2130_init() {

#if (X_IS_TMC2130)
#if MOTHERBOARD == INVENTOR_BOARD	
	HAL::RouteSPITo(SPI_X);			// For Inventor board, we must first route the SPI
#endif

	stepperX.init();
	stepperX.set_mres(X_MICROSTEPS);
	stepperX.set_IHOLD_IRUN(X_IHOLD, X_IRUN, X_IHOLDDELAY);
	stepperX.set_I_scale_analog(X_ISCALE);
	stepperX.set_intpol(1);
	stepperX.set_tbl(2); // ([0-3]) set comparator blank time to 16, 24, 36 or 54 clocks, 1 or 2 is recommended
	stepperX.set_toff(3); // ([0-15]) 0: driver disable, 1: use only with TBL>2, 2-15: off time setting during slow decay phase
	stepperX.set_hstrt(6);  // 6 for Hbot system
	stepperX.set_hend(10);  // 0 for Hbot system
	stepperX.set_chm(0);
	stepperX.set_TPOWERDOWN(10);
	stepperX.set_en_pwm_mode(0);	// Disable stealthchop
	stepperX.set_TCOOLTHRS(250);
	stepperX.set_sfilt(1);  // 0 for Hbot system
	stepperX.set_sgt(0);
	// Coolstep
	stepperX.set_seimin(1);
	stepperX.set_semin(2);
	stepperX.set_semax(1);
	stepperX.set_sedn(1);
	stepperX.set_seup(9);
	//	
	stepperX.set_diag1_int_pushpull(0);
	stepperX.set_diag1_index(0);
	stepperX.set_diag1_onstate(0);
	stepperX.set_diag1_steps_skipped(0);
	stepperX.set_diag1_stall(1);

#if MOTHERBOARD == INVENTOR_BOARD	
	HAL::RouteSPITo(SPI_NONE);		// Disconnect from all slaves
#endif
#endif

#if (Y_IS_TMC2130)
#if MOTHERBOARD == INVENTOR_BOARD	
	HAL::RouteSPITo(SPI_Y);			// For Inventor board, we must first route the SPI
#endif

	stepperY.init();
	stepperY.set_mres(Y_MICROSTEPS);
	stepperY.set_IHOLD_IRUN(Y_IHOLD, Y_IRUN, Y_IHOLDDELAY);
	stepperY.set_I_scale_analog(Y_ISCALE);
	stepperY.set_intpol(1);
	stepperY.set_tbl(2);
	stepperY.set_toff(3);
	stepperY.set_hstrt(6);  // 6 for Hbot system
	stepperY.set_hend(10);  // 0 for Hbot system
	stepperY.set_chm(0);
	stepperY.set_TPOWERDOWN(10);
	stepperY.set_en_pwm_mode(0);
	stepperY.set_TCOOLTHRS(250);
	stepperY.set_sfilt(1);  // 0 for Hbot system
	stepperY.set_sgt(0);
	// Coolstep
	stepperY.set_seimin(1);
	stepperY.set_semin(2);
	stepperY.set_semax(1);
	stepperY.set_sedn(1);
	stepperY.set_seup(9);
	//
	stepperY.set_diag1_int_pushpull(0);
	stepperY.set_diag1_index(0);
	stepperY.set_diag1_onstate(0);
	stepperY.set_diag1_steps_skipped(0);
	stepperY.set_diag1_stall(1);

#if MOTHERBOARD == INVENTOR_BOARD	
	HAL::RouteSPITo(SPI_NONE);		// Disconnect from all slaves
#endif
#endif

#if (Z_IS_TMC2130)
#if MOTHERBOARD == INVENTOR_BOARD	
	HAL::RouteSPITo(SPI_Z);			// For Inventor board, we must first route the SPI
#endif

	stepperZ.init();
	stepperZ.set_mres(Z_MICROSTEPS);
	stepperZ.set_IHOLD_IRUN(Z_IHOLD, Z_IRUN, Z_IHOLDDELAY);
	stepperZ.set_I_scale_analog(Z_ISCALE);
	stepperZ.set_intpol(1);
	stepperZ.set_tbl(2);
	stepperZ.set_toff(3);
	stepperZ.set_hstrt(6); // 2 for Hbot system
	stepperZ.set_hend(10);  // 10 for Hbot system
	stepperZ.set_chm(0);
	stepperZ.set_TPOWERDOWN(10);
	stepperZ.set_en_pwm_mode(0);
	stepperZ.set_TCOOLTHRS(250);
	stepperZ.set_sfilt(1);  // 1 for Hbot system
	stepperZ.set_sgt(0);
	// Coolstep
	stepperZ.set_seimin(1);
	stepperZ.set_semin(2);
	stepperZ.set_semax(1);
	stepperZ.set_sedn(1);
	stepperZ.set_seup(9);
	//
	stepperZ.set_diag1_int_pushpull(0);
	stepperZ.set_diag1_index(0);
	stepperZ.set_diag1_onstate(0);
	stepperZ.set_diag1_steps_skipped(0);
	stepperZ.set_diag1_stall(1);

#if MOTHERBOARD == INVENTOR_BOARD	
	HAL::RouteSPITo(SPI_NONE);		// Disconnect from all slaves
#endif
#endif

#if (E0_IS_TMC2130)
#if MOTHERBOARD == INVENTOR_BOARD	
	HAL::RouteSPITo(SPI_E0);			// For Inventor board, we must first route the SPI
#endif
	stepperE0.init();
	stepperE0.set_mres(E0_MICROSTEPS);
	stepperE0.set_IHOLD_IRUN(E0_IHOLD, E0_IRUN, E0_IHOLDDELAY);
	stepperE0.set_I_scale_analog(E0_ISCALE);
	stepperE0.set_tbl(2);
	stepperE0.set_toff(3);
	stepperE0.set_hstrt(4);
	stepperE0.set_hend(1);
	stepperE0.set_chm(0);
	stepperE0.set_TPOWERDOWN(10);
	stepperE0.set_en_pwm_mode(0);

#if MOTHERBOARD == INVENTOR_BOARD	
	HAL::RouteSPITo(SPI_NONE);		// Disconnect from all slaves
#endif
#endif

#if (E1_IS_TMC2130)
#if MOTHERBOARD == INVENTOR_BOARD	
	HAL::RouteSPITo(SPI_E1);			// For Inventor board, we must first route the SPI
#endif
	stepperE1.init();
	stepperE1.set_mres(E1_MICROSTEPS);
	stepperE1.set_IHOLD_IRUN(E1_IHOLD, E1_IRUN, E1_IHOLDDELAY);
	stepperE1.set_I_scale_analog(E1_ISCALE);

#if MOTHERBOARD == INVENTOR_BOARD	
	HAL::RouteSPITo(SPI_NONE);		// Disconnect from all slaves
#endif
#endif

#if (E2_IS_TMC2130)
#if MOTHERBOARD == INVENTOR_BOARD	
	HAL::RouteSPITo(SPI_E2);			// For Inventor board, we must first route the SPI
#endif
	stepperE2.init();
	stepperE2.set_mres(E2_MICROSTEPS);
	stepperE2.set_IHOLD_IRUN(E2_IHOLD, E2_IRUN, E2_IHOLDDELAY);
	stepperE2.set_I_scale_analog(E2_ISCALE);

#if MOTHERBOARD == INVENTOR_BOARD	
	HAL::RouteSPITo(SPI_NONE);		// Disconnect from all slaves
#endif
#endif
}
#endif
