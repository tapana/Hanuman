/*
  Lua module to provide process dynamixel packets
*/

#ifdef __cplusplus 
extern "C" {
#endif
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
#ifdef __cplusplus
}
#endif

#include "IMU.h"


IMU imu;

int lua_init(lua_State *L) {
    printf("ivy imu gy-85 calibrating ...\n");
    imu.init();
    return 0;
}

int lua_update(lua_State *L){
    imu.update();
    return 0;
}



int lua_get_quaternion(lua_State *L){

    lua_newtable(L);

    for(int i=0;i<4;i++){
        lua_pushnumber(L,i+1);        
        lua_pushnumber(L,imu.ahrs.quaternion[i]);
        lua_settable(L, -3);
    }

    return 1;
}

int lua_get_euler(lua_State* L){
    lua_newtable(L);

    for(int i=0;i<3;i++){
        lua_pushnumber(L,i+1);        
        lua_pushnumber(L,imu.euler[i]);
        lua_settable(L, -3);
    }
    return 1;
}

int lua_get_raw(lua_State *L){

    lua_newtable(L);
    int count =1;
    //acc
    for(int i=0; i<3; i++){
        lua_pushnumber(L,count++);        
        lua_pushnumber(L,imu.acc_val[i]);
        lua_settable(L, -3);
    }

    //gyro
    for(int i=0; i<3; i++){
        lua_pushnumber(L,count++);        
        lua_pushnumber(L,imu.gyro_val[i]);
        lua_settable(L, -3);
    }

    //mag
    for(int i=0; i<3; i++){
        lua_pushnumber(L,count++);        
        lua_pushnumber(L,imu.mag_val[i]);
        lua_settable(L, -3);
    }        

    return 1;
}



static const struct luaL_reg imuLib[] = {
    {"init", lua_init},
    {"update", lua_update},
    {"get_quaternion",lua_get_quaternion},
    {"get_euler",lua_get_euler},
    {"get_raw",lua_get_raw},
    {NULL, NULL}
};

extern "C"
int luaopen_IMU (lua_State *L) {
 luaL_register(L, "IMU", imuLib);
 return 1;
}


