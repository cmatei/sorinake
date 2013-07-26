#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>

#include <lua.h>
#include <lauxlib.h>
#include <lualib.h>

#include "tcr-i2c.h"

#define T_DOUBLE 1
#define T_UINT   2
#define T_BOOL   3

/* simplified data definition for the slave_data struct */
static struct {
	char *name;
	int type;
	size_t offset;
} slave_data_ffi[] = {
	{ "ds1",    T_DOUBLE, offsetof(struct slave_data, t1[0])       },
	{ "ds2",    T_DOUBLE, offsetof(struct slave_data, t1[1])       },
	{ "ds3",    T_DOUBLE, offsetof(struct slave_data, t1[2])       },
	{ "ds4",    T_DOUBLE, offsetof(struct slave_data, t1[3])       },
	{ "pt1",    T_DOUBLE, offsetof(struct slave_data, t2[0])       },
	{ "pt2",    T_DOUBLE, offsetof(struct slave_data, t2[1])       },
	{ "pt3",    T_DOUBLE, offsetof(struct slave_data, t2[2])       },
	{ "pt4",    T_DOUBLE, offsetof(struct slave_data, t2[3])       },
	{ "r1",     T_BOOL,   offsetof(struct slave_data, relays[0])   },
	{ "r2",     T_BOOL,   offsetof(struct slave_data, relays[1])   },
	{ "r3",     T_BOOL,   offsetof(struct slave_data, relays[2])   },
	{ "r4",     T_BOOL,   offsetof(struct slave_data, relays[3])   },
	{ "uptime", T_UINT,   offsetof(struct slave_data, uptime)      },
	{ "errcnt", T_UINT,   offsetof(struct slave_data, errcnt)      },
	{ "ds0cnt", T_UINT,   offsetof(struct slave_data, t1_count[0]) },
	{ "ds1cnt", T_UINT,   offsetof(struct slave_data, t1_count[1]) },
	{ "ds2cnt", T_UINT,   offsetof(struct slave_data, t1_count[2]) },
	{ "ds3cnt", T_UINT,   offsetof(struct slave_data, t1_count[3]) },
	{ "pt0cnt", T_UINT,   offsetof(struct slave_data, t2_count[0]) },
	{ "pt1cnt", T_UINT,   offsetof(struct slave_data, t2_count[1]) },
	{ "pt2cnt", T_UINT,   offsetof(struct slave_data, t2_count[2]) },
	{ "pt3cnt", T_UINT,   offsetof(struct slave_data, t2_count[3]) },

	{ NULL, 0, 0 }
};

lua_State *L;
static char *conf_filename = "tcr.conf";


void error (lua_State *L, const char *fmt, ...)
{
	va_list argp;
	va_start(argp, fmt);
	vfprintf(stderr, fmt, argp);
	va_end(argp);
	lua_close(L);
	exit(EXIT_FAILURE);
}

static int c_slave_data_read(lua_State *L)
{
	struct slave_data *data = NULL;
	int bus, address;
	int i, r;

	lua_getfield(L, 1, "bus");
	bus = luaL_checknumber(L, -1);

	lua_getfield(L, 1, "address");
	address = luaL_checknumber(L, -1);

	lua_getfield(L, 1, "_data");
	data = lua_touserdata(L, -1);

	/* do at most 3 attempts */
	for (r = 1, i = 0; r && (i < 3); i++) {
		r = slave_get_data(bus, address, data);
		if (r)
			data->errcnt++;
	}

	lua_pushboolean(L, r ? 1 : 0);

	return 1;
}

static int c_slave_data_write(lua_State *L)
{
	struct slave_data *data = NULL;
	int bus, address;
	unsigned relays = 0;

	lua_getfield(L, 1, "bus");
	bus = luaL_checknumber(L, -1);

	lua_getfield(L, 1, "address");
	address = luaL_checknumber(L, -1);

	lua_getfield(L, 1, "_data");
	data = lua_touserdata(L, -1);

	relays = (data->relays[0] ? 0x01 : 0x00) |
		 (data->relays[1] ? 0x02 : 0x00) |
		 (data->relays[2] ? 0x04 : 0x00) |
		 (data->relays[3] ? 0x08 : 0x00);

	slave_set_relays(bus, address, relays);

	return 0;
}

static int c_slave_data_index(lua_State *L)
{
	void *data;
	const char *key;
	int i;

	key = luaL_checkstring(L, 2);

	lua_getfield(L, 1, "_data");
	data = lua_touserdata(L, -1);

	for (i = 0; slave_data_ffi[i].name; i++) {
		if (!strcmp(slave_data_ffi[i].name, key)) {
			switch (slave_data_ffi[i].type) {
			case T_DOUBLE:
				lua_pushnumber(L, *(double *)(data + slave_data_ffi[i].offset));
				break;

			case T_UINT:
				lua_pushnumber(L, *(unsigned int *)(data + slave_data_ffi[i].offset));
				break;

			case T_BOOL:
				lua_pushboolean(L, *(unsigned char *)(data + slave_data_ffi[i].offset));
				break;
			}

			return 1;
		}
	}

	/* not found */
	lua_pushnil(L);
	return 1;
}

static int c_slave_data_newindex(lua_State *L)
{
	struct slave_data *data;
	const char *key;
	int i;

	key = luaL_checkstring(L, 2);

	lua_getfield(L, 1, "_data");
	data = lua_touserdata(L, -1);

	if (!strcmp(key, "r1") || !strcmp(key, "r2") ||
	    !strcmp(key, "r3") || !strcmp(key, "r4")) {

		if (!lua_isboolean(L, 3))
			return luaL_error(L, "expecting a boolean value");

		data->relays[key[1] - '1'] = lua_toboolean(L, 3);
		return 0;
	}

	// deny setting other ffi data
	for (i = 0; slave_data_ffi[i].name; i++) {
		if (!strcmp(slave_data_ffi[i].name, key))
			return luaL_error(L, "cannot set %s\n", key);
	}

	return 0;
}


static int c_slave_data_alloc(lua_State *L)
{
	struct slave_data *data;

	// allocate userdata
	data = lua_newuserdata(L, sizeof(struct slave_data));
	memset(data, 0, sizeof(struct slave_data));

	return 1;
}

int main(int argc, char **argv)
{
	L = lua_open();
	luaL_openlibs(L);

	lua_pushcfunction(L, c_slave_data_alloc);
	lua_setglobal(L, "slave_data_alloc");

	lua_pushcfunction(L, c_slave_data_index);
	lua_setglobal(L, "slave_data_index");

	lua_pushcfunction(L, c_slave_data_newindex);
	lua_setglobal(L, "slave_data_newindex");

	lua_pushcfunction(L, c_slave_data_read);
	lua_setglobal(L, "slave_data_read");

	lua_pushcfunction(L, c_slave_data_write);
	lua_setglobal(L, "slave_data_write");

	if (luaL_loadfile(L, "tcr.lua") || lua_pcall(L, 0, 0, 0))
		error(L, "%s", lua_tostring(L, -1));

	if (luaL_loadfile(L, conf_filename) || lua_pcall(L, 0, 0, 0))
		error(L, "%s", lua_tostring(L, -1));

	while (1) {
		if (luaL_loadstring(L, "step()") || lua_pcall(L, 0, 0, 0))
			error(L, "%s", lua_tostring(L, -1));

		sleep(1);
	}

	return 0;
}
