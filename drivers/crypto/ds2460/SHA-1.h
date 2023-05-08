/****************************************Copyright (c)**************************************************
**                               Guangzou ZLG-MCU Development Co.,LTD.
**                                      graduate school
**                                 http://www.zlgmcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			sha-1.h
** Last modified Date:	2005-11-21
** Last Version:		1.01
** Descriptions:		The software realization of the calculate way SHA-1
**
**------------------------------------------------------------------------------------------------------
** Created by:			zouchao
** Created date:		2005-10-28
** Version:				1.0
** Descriptions:		The original version
**
**------------------------------------------------------------------------------------------------------
** Modified by:			Chenmingji
** Modified date:		2005-11-21
** Version:				1.01
** Descriptions:		Change to one function
**
**------------------------------------------------------------------------------------------------------
** Modified by: 
** Modified date:
** Version:	
** Descriptions: 
**
********************************************************************************************************/
#ifndef  __SHA_1_H
#define  __SHA_1_H

extern uint8 MyMacComputation(uint8 *Rt, uint8 *Set, uint8 *Secret);
/*********************************************************************************************************
** Function name:			MacComputation
** Descriptions:			The software realization of the calculate way SHA-1
** input parameters:		Handle:Not use
**                          Rt:    Mac Data
**                          Set:   In Put Message
**                          Secret:Password
** Returned value:			TRUE:  OK
**                          FALSE: Not OK
********************************************************************************************************/

#endif

/*********************************************************************************************************
**                            End Of File
********************************************************************************************************/
