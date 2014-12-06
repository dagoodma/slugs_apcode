/*
The MIT License

Copyright (c) 2009 UCSC Autonomous Systems Lab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

#include "eepLoader.h"

int8_t EEPInit(void) {
    int8_t result = SUCCESS;
    result = (int8_t)DataEEInit();

    return result;
}

/**
 * Loads parameters, waypoints, and mid-level commands from EEPROM.
 * @return SUCCESS of FAILURE
 */
int8_t loadAllEEPData(void) {
    int8_t result = readParamsInEeprom();
    if (readWaypointsInEeprom() == FAILURE)
        result = FAILURE;
    if (readMidLevelCommandsInEeprom() == FAILURE)
        result = FAILURE;

    return result;
}

/**
 * Erases all waypoints after the specified waypoint index.
 * @param startingWp index to clear from
 * @return SUCCESS or ERROR
 */
int8_t eraseWaypointsInEepromFrom(uint8_t startingWp) {
    int8_t writeSuccess = SUCCESS;
    uint8_t indx, indexOffset;
    tFloatToChar tempFloat;

    // erase the flash values in EEPROM emulation
    for (indx = startingWp; indx < MAX_NUM_WPS; indx++) {
        // Compute the adecuate index offset
        indexOffset = indx * 8;

        // Clear the data from the EEPROM
        tempFloat.flData = 0.0;
        writeSuccess += (int8_t)DataEEWrite(tempFloat.shData[0], WPS_OFFSET + indexOffset);
        writeSuccess += (int8_t)DataEEWrite(tempFloat.shData[1], WPS_OFFSET + indexOffset + 1);

        tempFloat.flData = 0.0;
        writeSuccess += (int8_t)DataEEWrite(tempFloat.shData[0], WPS_OFFSET + indexOffset + 2);
        writeSuccess += (int8_t)DataEEWrite(tempFloat.shData[1], WPS_OFFSET + indexOffset + 3);

        tempFloat.flData = 0.0;
        writeSuccess += (int8_t)DataEEWrite(tempFloat.shData[0], WPS_OFFSET + indexOffset + 4);
        writeSuccess += (int8_t)DataEEWrite(tempFloat.shData[1], WPS_OFFSET + indexOffset + 5);

        writeSuccess += (int8_t)DataEEWrite((unsigned short) 0, WPS_OFFSET + indexOffset + 6);
        writeSuccess += (int8_t)DataEEWrite((unsigned short) 0, WPS_OFFSET + indexOffset + 7);
    }

    return writeSuccess;
}

/**
 * Stores a waypoint item to EEPROM.
 *
 * @param mlSingleWp Pointer to mission
 * @return Zero on success, or an error code.
 * @todo Check if all mission parameters are recorded.
 */
int8_t storeWaypointInEeprom(mavlink_mission_item_t* mlSingleWp) {
    uint8_t indexOffset = 0, indx= 0;
    int8_t writeSuccess = SUCCESS;
    tFloatToChar tempFloat;

    // get the WP index
    indx = (uint8_t)mlSingleWp->seq;

    // Compute the adecuate index offset
    indexOffset = indx*WP_SIZE_IN_EEPROM;

    // Save the data to the EEPROM
    tempFloat.flData = mlSingleWp->x;
    writeSuccess += (int8_t)DataEEWrite(tempFloat.shData[0], WPS_OFFSET+indexOffset);
    writeSuccess += (int8_t)DataEEWrite(tempFloat.shData[1], WPS_OFFSET+indexOffset+1);

    tempFloat.flData = mlSingleWp->y;
    writeSuccess += (int8_t)DataEEWrite(tempFloat.shData[0], WPS_OFFSET+indexOffset+2);
    writeSuccess += (int8_t)DataEEWrite(tempFloat.shData[1], WPS_OFFSET+indexOffset+3);

    tempFloat.flData = mlSingleWp->z;
    writeSuccess += (int8_t)DataEEWrite(tempFloat.shData[0], WPS_OFFSET+indexOffset+4);
    writeSuccess += (int8_t)DataEEWrite(tempFloat.shData[1], WPS_OFFSET+indexOffset+5);

    writeSuccess += (int8_t)DataEEWrite((unsigned short)mlSingleWp->command, WPS_OFFSET+indexOffset+6);

    writeSuccess += (int8_t)DataEEWrite((unsigned short)mlSingleWp->param3, WPS_OFFSET+indexOffset+7);

    return writeSuccess;
}

/**
 * @brief Writes a parameter to EEPROM.
 * @param parameter Value of parameter.
 * @param pmIndex Index of paramter.
 * @return Zero on success, and non-zero on failure
 */
int8_t storeParameterInEeprom (float parameter, uint8_t pmIndex){
    uint8_t indexOffset = 0;/* indx= 0 ,*/
    int8_t writeSuccess = SUCCESS;
    tFloatToChar tempFloat;
    // Save the data to the EEPROM
    indexOffset = pmIndex*2;

    tempFloat.flData = parameter;
    writeSuccess += (int8_t)DataEEWrite(tempFloat.shData[0], PARAM_OFFSET+indexOffset);
    writeSuccess += (int8_t)DataEEWrite(tempFloat.shData[1], PARAM_OFFSET+indexOffset+1);


    return writeSuccess;
}

/**
 * @brief Saves all parameters to EEPROM.
 * @return Zero for success, or non-zero error code.
 */
int8_t storeAllParamsInEeprom(void){
    uint8_t indx= 0;
    int8_t writeSuccess = SUCCESS;

    for (indx = 0; indx < PAR_PARAM_COUNT; indx ++){
            writeSuccess += storeParameterInEeprom(mlParamInterface.param[indx], indx);
    }

    return writeSuccess;
}


/**
 * @brief Stores the specified mid-level command values to EEPROM.
 * @note Mid-level commands are read from the mlMidLevelCommands struct.
 * @return SUCCESS or FAILURE.
 */
int8_t storeMidLevelCommandsInEeprom (void) {
    uint8_t indexOffset = 0;
    int8_t writeSuccess = SUCCESS;
    tFloatToChar tempFloat;

    // Save the data to the EEPROM
    tempFloat.flData = mlMidLevelCommands.hCommand;
    writeSuccess += (int8_t)DataEEWrite(tempFloat.shData[0], MIDLEVEL_OFFSET + indexOffset++);
    writeSuccess += (int8_t)DataEEWrite(tempFloat.shData[1], MIDLEVEL_OFFSET + indexOffset++);

    tempFloat.flData = mlMidLevelCommands.uCommand;
    writeSuccess += (int8_t)DataEEWrite(tempFloat.shData[0], MIDLEVEL_OFFSET + indexOffset++);
    writeSuccess += (int8_t)DataEEWrite(tempFloat.shData[1], MIDLEVEL_OFFSET + indexOffset++);

    tempFloat.flData = mlMidLevelCommands.rCommand;
    writeSuccess += (int8_t)DataEEWrite(tempFloat.shData[0], MIDLEVEL_OFFSET + indexOffset++);
    writeSuccess += (int8_t)DataEEWrite(tempFloat.shData[1], MIDLEVEL_OFFSET + indexOffset);

    return writeSuccess;
}

/**
 * @brief Reads all paramters from EEPROM to RAM.
 * @return Zero on success, or non-zero error code.
 */
int8_t readParamsInEeprom (void){
    uint8_t indx= 0, i=0;
    tFloatToChar tempFloat;
    int8_t readSuccess = SUCCESS;

    for(indx = 0; indx < (PAR_PARAM_COUNT*2); indx+=2 ){
        tempFloat.shData[0]= DataEERead(PARAM_OFFSET+indx);
        tempFloat.shData[1]= DataEERead(PARAM_OFFSET+indx+1);

        // This doesn't seem to indicate success or failure.
        //if (tempFloat.shData[0] == 0xFFFF || tempFloat.shData[1] == 0xFFFF)
        //    readSuccess = FAILURE;

        // If the value read from memory is finite assign it, else assign 0
        mlParamInterface.param[i++]	= isFinite(tempFloat.flData)? tempFloat.flData : 0.0;
    }

    return readSuccess;
}

/**
 * @brief Reads mid-level command values from EEPROM and loads into
 *  mlMidLevelCommand struct.
 */
int8_t readMidLevelCommandsInEeprom (void) {
    uint8_t indx= 0, i=0;
    tFloatToChar tempFloat;
    uint8_t readSuccess = SUCCESS;

    tempFloat.shData[0]= DataEERead(MIDLEVEL_OFFSET + indx++);
    tempFloat.shData[1]= DataEERead(MIDLEVEL_OFFSET + indx++);
    // If the value read from memory is finite assign it, else assign 0
    mlMidLevelCommands.hCommand = isFinite(tempFloat.flData)? tempFloat.flData : 0.0f;

    tempFloat.shData[0]= DataEERead(MIDLEVEL_OFFSET + indx++);
    tempFloat.shData[1]= DataEERead(MIDLEVEL_OFFSET + indx++);
    // If the value read from memory is finite assign it, else assign 0
    mlMidLevelCommands.uCommand = isFinite(tempFloat.flData)? tempFloat.flData : 0.0f;

    tempFloat.shData[0]= DataEERead(MIDLEVEL_OFFSET + indx++);
    tempFloat.shData[1]= DataEERead(MIDLEVEL_OFFSET + indx);
    // If the value read from memory is finite assign it, else assign 0
    mlMidLevelCommands.rCommand = isFinite(tempFloat.flData)? tempFloat.flData : 0.0f;

    return readSuccess;
}



/**
 * Reads all paramters from EEPROM to RAM.
 * @return ERROR or the number of waypoints read.
 */
int8_t readWaypointsInEeprom (void) {
    uint8_t i;
    tFloatToChar tempShData;

    for(i = 0; i < MAX_NUM_WPS; i++ ) {
        // Way Points
        tempShData.shData[0]= DataEERead(WPS_OFFSET+i*WP_SIZE_IN_EEPROM);
        tempShData.shData[1]= DataEERead(WPS_OFFSET+i*WP_SIZE_IN_EEPROM+1);
        mlWpValues.lat[i] = isFinite(tempShData.flData)? tempShData.flData : 0.0;

        tempShData.shData[0]= DataEERead(WPS_OFFSET+i*WP_SIZE_IN_EEPROM+2);
        tempShData.shData[1]= DataEERead(WPS_OFFSET+i*WP_SIZE_IN_EEPROM+3);
        mlWpValues.lon[i] = isFinite(tempShData.flData)? tempShData.flData : 0.0;

        tempShData.shData[0]= DataEERead(WPS_OFFSET+i*WP_SIZE_IN_EEPROM+4);
        tempShData.shData[1]= DataEERead(WPS_OFFSET+i*WP_SIZE_IN_EEPROM+5);
        mlWpValues.alt[i] = isFinite(tempShData.flData)? tempShData.flData : 0.0;

        mlWpValues.type[i] = (uint8_t)DataEERead(WPS_OFFSET+i*WP_SIZE_IN_EEPROM+6);

        mlWpValues.orbit[i]   = DataEERead(WPS_OFFSET+i*WP_SIZE_IN_EEPROM +7);
    }

    // Compute the waypoint count
    mlWpValues.wpCount = 0;
    while ((int)(mlWpValues.lat[mlWpValues.wpCount]) != 0 && (mlWpValues.wpCount < ORIGIN_WP_INDEX) ){
        mlWpValues.wpCount++;
    }

    mlPending.miTotalMissions = mlWpValues.wpCount; // must set this or ignores missions
    mlPending.miCurrentMission = 0;
    
    // Don't overwrite a pending message
    if (!mlPending.statustext) {
        mlStatustext.severity = MAV_SEVERITY_INFO;
        sprintf(mlStatustext.text, "Read %d waypoints from EEPROM.",mlWpValues.wpCount);
        mlPending.statustext++;
    }

    return mlWpValues.wpCount;
}
