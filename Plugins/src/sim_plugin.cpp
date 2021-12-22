#include <iostream>
#include "/home/hao/CoppeliaSim_Edu_V4_2_0_Ubuntu18_04/programming/include/simLib.h"
#include <unistd.h>

#include "sim_plugin.h"
#include "sim_main.h"
#include "sim_data.h"

#define strConCat(x, y, z) x y z
#define PLUGIN_NAME "iBot_v1"

static LIBRARY simLib;

#define LUA_CREATE_COMMAND "iBot.create"
const int inArgs_CREATE[] = {1, sim_script_arg_int32 | sim_script_arg_table, 0};
void LUA_CREATE_CALLBACK(SScriptCallBack *cb)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(cb->stackID, inArgs_CREATE, inArgs_CREATE[0], LUA_CREATE_COMMAND))
    {
        std::vector<CScriptFunctionDataItem> *inData = D.getInDataPtr();
        getObjectHandles(inData->at(0).int32Data);
    }
    D.pushOutData(CScriptFunctionDataItem(0));
    D.writeDataToStack(cb->stackID);
}

#define LUA_VISION_COMMAND "iBot.sendVisionInfo"
const int inArgs_VISION[] = {2, sim_script_arg_float, 0, sim_script_arg_float, 0};
void LUA_VISION_CALLBACK(SScriptCallBack *cb)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(cb->stackID, inArgs_VISION, inArgs_VISION[0], LUA_VISION_COMMAND))
    {
        std::vector<CScriptFunctionDataItem> *inData = D.getInDataPtr();
        head.x = inData->at(0).floatData[0];
        head.y = inData->at(1).floatData[0];
    }
    D.pushOutData(CScriptFunctionDataItem(0));
    D.writeDataToStack(cb->stackID);
}

#define LUA_START_COMMAND "iBot.start"
const int inArgs_START[] = {1, sim_script_arg_int32, 0};
void LUA_START_CALLBACK(SScriptCallBack *cb)
{
    CScriptFunctionData D;
    D.pushOutData(CScriptFunctionDataItem(true));
    D.writeDataToStack(cb->stackID);
}

SIM_DLLEXPORT unsigned char simStart(void *, int)
{
    char curDirAndFile[1024];
    getcwd(curDirAndFile, sizeof(curDirAndFile));
    std::string currentDirAndPath(curDirAndFile);
    std::string temp(currentDirAndPath);
    temp += "/libcoppeliaSim.so";

    simLib = loadSimLibrary(temp.c_str());
    if (simLib == NULL)
    {
        printf("Error: could not find or correctly load coppeliaSim.dll. Cannot start the plugin.\n");
        return (0);
    }
    if (getSimProcAddresses(simLib) == 0)
    {
        printf("Error: could not find all required functions in coppeliaSim.dll. Cannot start the plugin.\n");
        unloadSimLibrary(simLib);
        return (0); // Means error, CoppeliaSim will unload this plugin
    }

    // Register new functions
    simRegisterScriptCallbackFunction(strConCat(LUA_CREATE_COMMAND, "@", PLUGIN_NAME), strConCat("", LUA_CREATE_COMMAND, ""), LUA_CREATE_CALLBACK);
    simRegisterScriptCallbackFunction(strConCat(LUA_START_COMMAND, "@", PLUGIN_NAME), strConCat("", LUA_START_COMMAND, ""), LUA_START_CALLBACK);
    simRegisterScriptCallbackFunction(strConCat(LUA_VISION_COMMAND, "@", PLUGIN_NAME), strConCat("", LUA_VISION_COMMAND, ""), LUA_VISION_CALLBACK);

    return (11); // Return the version number of this plugin. 11 is for CoppeliaSim versions after CoppeliaSim 4.2.0
}

SIM_DLLEXPORT void simEnd()
{
    unloadSimLibrary(simLib);
}

SIM_DLLEXPORT void *simMessage(int message, int *auxiliaryData, void *customData, int *replyData)
{
    void *retVal = NULL;

    mainSimulation();

    return (retVal);
}
